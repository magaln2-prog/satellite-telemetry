#!/usr/bin/env python3
"""
flight.py

Flight-side telemetry script.

Main idea:
- Build a full record locally and log it (JSONL).
- Build a compact packet for radio.
- Send compact packet using TM-framed fragments (robust against fragmentation).
"""

# -------------------------
# Standard libraries
# -------------------------
import json
import shutil
import subprocess
from pathlib import Path
from typing import Any, Dict
import time
from datetime import datetime, timezone

# -------------------------
# Hardware / sensor libraries
# -------------------------
import board
import busio

from adafruit_bme280 import basic as adafruit_bme280
import adafruit_ltr390
import adafruit_tsl2591
from adafruit_icm20x import ICM20948

# UPS power monitoring
from smbus2 import SMBus

# LoRa driver
import sx126x

# Shared telemetry protocol (MUST match ground exactly)
from protocol_tm import build_frame as _tm_build_frame, HDR_LEN as _TM_HDR_LEN

# -------------------------
# UPS (Waveshare UPS HAT E) constants
# -------------------------
UPS_I2C_ADDR = 0x2D
LOW_BATT_PCT = 20
CRIT_BATT_PCT = 10
LOW_BATT_V = 3.55

# -------------------------
# Camera settings
# -------------------------
IMAGE_PERIOD_S = 25
IMG_W, IMG_H = 640, 480
IMG_Q = 35
CAMERA_TIMEOUT_S = 8

# -------------------------
# LoRa settings
# -------------------------
LORA_PORT = "/dev/serial0"
LORA_FREQ_MHZ = 915
LORA_SRC_ADDR = 1
LORA_DEST_ADDR = 65535
LORA_POWER_DBM = 22
LORA_AIR_SPEED = 2400
LORA_NET_ID = 0
LORA_CRYPT = 0
LORA_RSSI = True

# Module UART buffer size
LORA_BUFFER_SIZE = 240

# TX behavior
LORA_TX_RETRIES = 3
LORA_TX_GAP_S = 0.20

# For 900MHz modules: offset = freq_mhz - 850
LORA_BASE_FREQ_MHZ = 850


# --------------------------------------------------------------------------------------------------
# Time helpers
# --------------------------------------------------------------------------------------------------
def utc_timestamp_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


# --------------------------------------------------------------------------------------------------
# Local logging (JSONL)
# --------------------------------------------------------------------------------------------------
def append_jsonl_safe(path: Path, record: Dict[str, Any]) -> str | None:
    try:
        with path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, separators=(",", ":"), ensure_ascii=False) + "\n")
        return None
    except Exception as e:
        return repr(e)


# --------------------------------------------------------------------------------------------------
# Loop pacing (avoid overheating / busy-wait)
# --------------------------------------------------------------------------------------------------
def sleep_to_rate(loop_start_time_s: float, telemetry_hz: float) -> None:
    period_s = 1.0 / telemetry_hz
    elapsed_s = time.time() - loop_start_time_s
    time.sleep(max(0.0, period_s - elapsed_s))


# --------------------------------------------------------------------------------------------------
# Storage cleanup
# --------------------------------------------------------------------------------------------------
def cleanup_folder_to_mb(folder: Path, max_mb: int) -> int:
    if not folder.exists():
        return 0

    max_bytes = max_mb * 1024 * 1024
    files = [p for p in folder.rglob("*") if p.is_file()]
    files.sort(key=lambda p: p.stat().st_mtime)

    total_bytes = sum(p.stat().st_size for p in files)
    if total_bytes <= max_bytes:
        return 0

    deleted = 0
    for p in files:
        try:
            size = p.stat().st_size
            p.unlink()
            total_bytes -= size
            deleted += 1
            if total_bytes <= max_bytes:
                break
        except Exception:
            pass

    return deleted


# --------------------------------------------------------------------------------------------------
# UPS helpers
# --------------------------------------------------------------------------------------------------
def ups_read_u16(ups_bus: SMBus, reg: int) -> int:
    lo = ups_bus.read_byte_data(UPS_I2C_ADDR, reg)
    hi = ups_bus.read_byte_data(UPS_I2C_ADDR, reg + 1)
    return (hi << 8) | lo


def ups_read_i16(ups_bus: SMBus, reg: int) -> int:
    v = ups_read_u16(ups_bus, reg)
    return v - 65536 if v & 0x8000 else v


def read_ups_status(ups_bus: SMBus) -> tuple[Dict[str, Any], str | None]:
    try:
        batt_mv = ups_read_u16(ups_bus, 0x20)
        batt_ma = ups_read_i16(ups_bus, 0x22)
        batt_pct = ups_read_u16(ups_bus, 0x24)
        rem_mah = ups_read_u16(ups_bus, 0x26)
        rem_min = ups_read_u16(ups_bus, 0x28)

        power_status = {
            "battery_v": batt_mv / 1000.0,
            "battery_a": batt_ma / 1000.0,
            "battery_pct": batt_pct,
            "remaining_mah": rem_mah,
            "remaining_min": rem_min,
        }
        return power_status, None
    except Exception as e:
        return {}, repr(e)


def compute_battery_alerts(power_status: Dict[str, Any]) -> list[str]:
    alerts: list[str] = []
    pct = power_status.get("battery_pct")
    v = power_status.get("battery_v")

    if isinstance(pct, (int, float)):
        if pct <= CRIT_BATT_PCT:
            alerts.append("BATT_CRIT_PCT")
        elif pct <= LOW_BATT_PCT:
            alerts.append("BATT_LOW_PCT")

    if isinstance(v, (int, float)) and v <= LOW_BATT_V:
        alerts.append("BATT_LOW_V")

    return alerts


# --------------------------------------------------------------------------------------------------
# Pi health helpers
# --------------------------------------------------------------------------------------------------
def _run_cmd(cmd: list[str]) -> tuple[str, str | None]:
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
        return out.strip(), None
    except Exception as e:
        return "", repr(e)


def read_pi_throttle_flags() -> tuple[Dict[str, Any], str | None]:
    out, err = _run_cmd(["vcgencmd", "get_throttled"])
    if err:
        return {}, err

    try:
        raw_hex = out.split("=")[1]
        v = int(raw_hex, 16)
        flags = {
            "raw": raw_hex,
            "undervoltage_now": bool(v & (1 << 0)),
            "freq_capped_now": bool(v & (1 << 1)),
            "throttled_now": bool(v & (1 << 2)),
            "undervoltage_has": bool(v & (1 << 16)),
            "freq_capped_has": bool(v & (1 << 17)),
            "throttled_has": bool(v & (1 << 18)),
        }
        return flags, None
    except Exception as e:
        return {}, repr(e)


def read_cpu_temp_c() -> tuple[float | None, str | None]:
    out, err = _run_cmd(["vcgencmd", "measure_temp"])
    if err:
        return None, err

    try:
        temp_c = float(out.split("=")[1].split("'")[0])
        return temp_c, None
    except Exception as e:
        return None, repr(e)


def read_uptime_s() -> tuple[float | None, str | None]:
    try:
        with open("/proc/uptime", "r", encoding="utf-8") as f:
            uptime_s = float(f.read().split()[0])
        return uptime_s, None
    except Exception as e:
        return None, repr(e)


def read_mem_info_mb() -> tuple[Dict[str, Any], str | None]:
    try:
        wanted = {"MemTotal": None, "MemFree": None, "MemAvailable": None}
        with open("/proc/meminfo", "r", encoding="utf-8") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 2:
                    key = parts[0].rstrip(":")
                    if key in wanted:
                        wanted[key] = int(parts[1]) / 1024.0

        return {
            "mem_total_mb": wanted["MemTotal"],
            "mem_free_mb": wanted["MemFree"],
            "mem_available_mb": wanted["MemAvailable"],
        }, None
    except Exception as e:
        return {}, repr(e)


def read_disk_free_mb(path: Path) -> tuple[float | None, str | None]:
    try:
        du = shutil.disk_usage(str(path))
        return du.free / (1024 * 1024), None
    except Exception as e:
        return None, repr(e)


def read_pi_health_status(data_root: Path) -> tuple[Dict[str, Any], Dict[str, str]]:
    health: Dict[str, Any] = {}
    errors: Dict[str, str] = {}

    throttle, err = read_pi_throttle_flags()
    if err:
        errors["throttle"] = err
    else:
        health["pi_power"] = throttle

    cpu_temp_c, err = read_cpu_temp_c()
    if err:
        errors["cpu_temp"] = err
    else:
        health["cpu_temp_c"] = cpu_temp_c

    uptime_s, err = read_uptime_s()
    if err:
        errors["uptime"] = err
    else:
        health["uptime_s"] = uptime_s

    mem, err = read_mem_info_mb()
    if err:
        errors["meminfo"] = err
    else:
        health.update(mem)

    disk_free_mb, err = read_disk_free_mb(data_root)
    if err:
        errors["disk_free"] = err
    else:
        health["disk_free_mb"] = disk_free_mb

    return health, errors


# --------------------------------------------------------------------------------------------------
# Camera capture
# --------------------------------------------------------------------------------------------------
def capture_image_jpeg(images_dir: Path, sequence_id: int) -> tuple[Dict[str, Any], str | None]:
    try:
        images_dir.mkdir(parents=True, exist_ok=True)
        img_path = images_dir / f"img_{sequence_id:06d}.jpg"

        cmd = [
            "rpicam-still",
            "-n",
            "-t", "300",
            "--width", str(IMG_W),
            "--height", str(IMG_H),
            "-q", str(IMG_Q),
            "-o", str(img_path),
        ]

        subprocess.run(
            cmd,
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=CAMERA_TIMEOUT_S,
        )

        return {"path": str(img_path), "bytes": int(img_path.stat().st_size)}, None
    except Exception as e:
        return {}, repr(e)


# --------------------------------------------------------------------------------------------------
# LoRa helpers
# --------------------------------------------------------------------------------------------------
def init_lora_radio() -> tuple[Any | None, str | None]:
    try:
        lora = sx126x.sx126x(
            serial_num=LORA_PORT,
            freq=LORA_FREQ_MHZ,
            addr=LORA_SRC_ADDR,
            power=LORA_POWER_DBM,
            rssi=LORA_RSSI,
            air_speed=LORA_AIR_SPEED,
            net_id=LORA_NET_ID,
            buffer_size=LORA_BUFFER_SIZE,
            crypt=LORA_CRYPT,
            relay=False,
            lbt=False,
            wor=False,
        )
        return lora, None
    except Exception as e:
        return None, repr(e)


def compute_freq_off(freq_mhz: int) -> int:
    off = int(freq_mhz) - int(LORA_BASE_FREQ_MHZ)
    if not (0 <= off <= 255):
        raise ValueError(f"Frequency offset out of range: freq={freq_mhz} base={LORA_BASE_FREQ_MHZ} off={off}")
    return off


def build_fixed_frame(dest_addr: int, payload: bytes) -> bytes:
    freq_off = compute_freq_off(LORA_FREQ_MHZ)
    return bytes([
        (dest_addr >> 8) & 0xFF,
        dest_addr & 0xFF,
        freq_off & 0xFF,
    ]) + payload


# --------------------------------------------------------------------------------------------------
# Compact packet build + TX (TM FRAMED FRAGMENTS)
# --------------------------------------------------------------------------------------------------
def build_compact_radio_packet(record: Dict[str, Any]) -> Dict[str, Any]:
    env = record.get("environment", {}) or {}
    power = record.get("power", {}) or {}
    alerts = record.get("alerts", []) or []

    pkt: Dict[str, Any] = {
        "ts": record.get("timestamp_utc"),
        "seq": record.get("sequence_id"),
        "t_c": env.get("temperature_c"),
        "p_hpa": env.get("pressure_hpa"),
        "rh": env.get("humidity_pct"),
        "bv": power.get("battery_v"),
        "bp": power.get("battery_pct"),
        "img": 1 if (record.get("image", {}) or {}).get("captured") else 0,
    }

    if alerts:
        pkt["alerts"] = alerts

    return pkt


def send_compact_radio_packet(lora: Any, packet: Dict[str, Any]) -> Dict[str, Any]:
    """
    Send compact telemetry over LoRa using TM-framed fragments.
    Uses shared protocol_tm.py (must match ground exactly).
    """
    if lora is None:
        return {"enabled": False, "tx_success": False, "tx_error": "lora_none"}

    # Serialize JSON compactly
    blob = json.dumps(
        packet,
        separators=(",", ":"),
        ensure_ascii=False
    ).encode("utf-8")

    # Reserve 3 bytes for fixed-address header [DEST_H][DEST_L][FREQ_OFF]
    max_app = lora.max_app_payload_bytes(header_len=3)

    # Hard cap app payload to 64 bytes (smaller TM fragments)
    TARGET_MAX_APP = 64
    max_app = min(max_app, TARGET_MAX_APP)

    max_chunk = max_app - _TM_HDR_LEN

    if max_chunk <= 0:
        return {
            "enabled": True,
            "tx_success": False,
            "tx_error": "buffer_too_small_for_tm",
            "max_app": max_app,
        }

    # Split into TM fragments
    chunks = [blob[i:i + max_chunk] for i in range(0, len(blob), max_chunk)]
    frag_tot = len(chunks)

    # IMPORTANT: msg_id must be the SAME for all fragments of this message
    msg_id = int(packet.get("seq") or 0) & 0xFFFF

    sent = 0
    last_err = None

    # Send each fragment multiple times (no ACK on LoRa)
    FRAG_TX_COPIES = 2

    for frag_idx, chunk in enumerate(chunks):
        try:
            tm_payload = _tm_build_frame(
                msg_id,
                frag_idx,
                frag_tot,
                chunk
            )
        except Exception as e:
            return {
                "enabled": True,
                "tx_success": False,
                "tx_error": f"tm_build_failed:{e}",
            }

        # DO NOT CHANGE ADDRESSES
        frame = build_fixed_frame(LORA_DEST_ADDR, tm_payload)

        ok = False
        for _ in range(FRAG_TX_COPIES):
            try:
                lora.send(frame)
                sent += 1
                ok = True
            except Exception as e:
                last_err = repr(e)
                ok = False

            # Always gap between copies AND between fragments
            time.sleep(LORA_TX_GAP_S)

        if not ok:
            return {
                "enabled": True,
                "tx_success": False,
                "tx_error": last_err or "tx_failed",
                "msg_id": msg_id,
                "frags_total": frag_tot,
                "frags_sent": sent,
            }

    return {
        "enabled": True,
        "tx_success": True,
        "msg_id": msg_id,
        "frags_total": frag_tot,
        "frags_sent": sent,
    }



# --------------------------------------------------------------------------------------------------
# Sensor initialization + reading
# --------------------------------------------------------------------------------------------------
def create_run_directories(base_dir: Path) -> Dict[str, Path]:
    run_id_utc = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    run_dir = base_dir / "runs" / run_id_utc
    logs_dir = run_dir / "logs"
    images_dir = run_dir / "images"

    logs_dir.mkdir(parents=True, exist_ok=True)
    images_dir.mkdir(parents=True, exist_ok=True)

    return {"run_dir": run_dir, "logs_dir": logs_dir, "images_dir": images_dir}


def init_i2c_bus() -> busio.I2C:
    return busio.I2C(board.SCL, board.SDA)


def init_sensors(i2c: busio.I2C) -> Dict[str, Any]:
    """Initialize all I2C sensors.

    Notes
    -----
    - We *always* record init failures into sensors["_init_errors"] so downstream telemetry
      can surface the real reason fields show up as null.
    - BME280 address is commonly 0x76 or 0x77. We try 0x76 first (matches your bus scan),
      then fall back to 0x77.
    """
    sensors: Dict[str, Any] = {
        "bme280_sensor": None,
        "ltr390_sensor": None,
        "tsl2591_sensor": None,
        "imu_sensor": None,
        "_init_errors": {},  # type: ignore[dict-item]
    }

    init_errors: Dict[str, str] = {}

    # --- BME280 (Temp/Pressure/Humidity) ---
    try:
        sensors["bme280_sensor"] = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
    except Exception as e76:
        try:
            sensors["bme280_sensor"] = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x77)
        except Exception as e77:
            sensors["bme280_sensor"] = None
            init_errors["bme280_init"] = f"0x76 failed: {type(e76).__name__}: {e76}; 0x77 failed: {type(e77).__name__}: {e77}"

    # --- LTR390 (UV/ALS) ---
    try:
        sensors["ltr390_sensor"] = adafruit_ltr390.LTR390(i2c)
    except Exception as e:
        sensors["ltr390_sensor"] = None
        init_errors["ltr390_init"] = f"{type(e).__name__}: {e}"

    # --- TSL2591 (High range lux) ---
    try:
        sensors["tsl2591_sensor"] = adafruit_tsl2591.TSL2591(i2c)
    except Exception as e:
        sensors["tsl2591_sensor"] = None
        init_errors["tsl2591_init"] = f"{type(e).__name__}: {e}"

    # --- IMU (ICM20948) ---
    try:
        sensors["imu_sensor"] = ICM20948(i2c, address=0x68)
    except Exception as e:
        sensors["imu_sensor"] = None
        init_errors["imu_init"] = f"{type(e).__name__}: {e}"

    sensors["_init_errors"] = init_errors  # type: ignore[index]
    return sensors


def read_sensors(sensors: Dict[str, Any]) -> tuple[Dict[str, Any], Dict[str, str]]:
    data: Dict[str, Any] = {}
    errors: Dict[str, str] = {}

    bme280_sensor = sensors.get("bme280_sensor")
    ltr390_sensor = sensors.get("ltr390_sensor")
    tsl2591_sensor = sensors.get("tsl2591_sensor")
    imu_sensor = sensors.get("imu_sensor")

    if bme280_sensor is not None:
        try:
            data["environment"] = {
                "temperature_c": float(bme280_sensor.temperature),
                "humidity_pct": float(bme280_sensor.humidity),
                "pressure_hpa": float(bme280_sensor.pressure),
            }
        except Exception as e:
            errors["bme280"] = repr(e)

    if ltr390_sensor is not None:
        try:
            data["uv_light"] = {
                "uv_raw": float(ltr390_sensor.uvs),
                "ambient_light_raw": float(ltr390_sensor.light),
            }
        except Exception as e:
            errors["ltr390"] = repr(e)

    if tsl2591_sensor is not None:
        try:
            data["light"] = {
                "lux": float(tsl2591_sensor.lux),
                "infrared": float(tsl2591_sensor.infrared),
                "visible": float(tsl2591_sensor.visible),
            }
        except Exception as e:
            errors["tsl2591"] = repr(e)

    if imu_sensor is not None:
        try:
            accel_x, accel_y, accel_z = imu_sensor.acceleration
            gyro_x, gyro_y, gyro_z = imu_sensor.gyro
            data["imu"] = {
                "accel_mps2": [float(accel_x), float(accel_y), float(accel_z)],
                "gyro_rps": [float(gyro_x), float(gyro_y), float(gyro_z)],
            }
        except Exception as e:
            errors["icm20948"] = repr(e)

    return data, errors


# --------------------------------------------------------------------------------------------------
# Main loop
# --------------------------------------------------------------------------------------------------
def main() -> None:
    telemetry_hz = 1.0
    base_dir = Path(__file__).resolve().parent

    max_log_mb = 200
    max_image_mb = 500

    dirs = create_run_directories(base_dir)
    logs_dir = dirs["logs_dir"]
    images_dir = dirs["images_dir"]
    telemetry_log_path = logs_dir / "telemetry.jsonl"

    i2c = init_i2c_bus()
    sensors = init_sensors(i2c)

    try:
        ups_bus = SMBus(1)
    except Exception:
        ups_bus = None

    lora, lora_err = init_lora_radio()
    if lora_err:
        print(f"LoRa init error: {lora_err}")

    sequence_id = 0
    next_cleanup_time_s = time.time() + 60
    last_image_time_s = 0.0

    try:
        while True:
            loop_start_time_s = time.time()
            now_time_s = loop_start_time_s
            sequence_id += 1

            record: Dict[str, Any] = {
                "timestamp_utc": utc_timestamp_iso(),
                "sequence_id": sequence_id,
            }

            # Surface one-time sensor init failures in the telemetry stream.
            init_errs = sensors.get("_init_errors") or {}
            if init_errs:
                record.setdefault("errors", {})
                for k, v in init_errs.items():
                    record["errors"][k] = v

            record["image"] = {
                "captured": False,
                "seq": None,
                "path": None,
                "bytes": None,
                "error": None,
            }

            sensor_data, sensor_errors = read_sensors(sensors)
            record.update(sensor_data)
            if sensor_errors:
                record.setdefault("errors", {})
                record["errors"].update(sensor_errors)

            if ups_bus is not None:
                power_status, ups_err = read_ups_status(ups_bus)
            else:
                power_status, ups_err = ({}, "ups_bus_init_failed")

            record["power"] = power_status
            record["alerts"] = compute_battery_alerts(power_status)
            if ups_err:
                record.setdefault("errors", {})
                record["errors"]["ups"] = ups_err

            pi_health, pi_errs = read_pi_health_status(base_dir)
            record["pi_health"] = pi_health
            if pi_errs:
                record.setdefault("errors", {})
                for k, v in pi_errs.items():
                    record["errors"][f"pi_{k}"] = v

            image_period_s = 60 if "BATT_CRIT_PCT" in record.get("alerts", []) else IMAGE_PERIOD_S
            if (now_time_s - last_image_time_s) >= image_period_s:
                image_info, img_err = capture_image_jpeg(images_dir, sequence_id)

                record["image"].update({
                    "seq": sequence_id,
                    "path": image_info.get("path"),
                    "bytes": image_info.get("bytes"),
                    "captured": (img_err is None) and bool(image_info.get("path")),
                    "error": img_err,
                })

                if img_err:
                    record.setdefault("errors", {})
                    record["errors"]["camera"] = img_err

                last_image_time_s = now_time_s

            # ---- LoRa TX ----
            radio_packet = build_compact_radio_packet(record)
            record["radio"] = send_compact_radio_packet(lora, radio_packet)

            # ---- JSONL log ----
            log_error = append_jsonl_safe(telemetry_log_path, record)
            if log_error:
                record.setdefault("errors", {})
                record["errors"]["jsonl_log"] = log_error

            # ---- Periodic cleanup ----
            if time.time() >= next_cleanup_time_s:
                cleanup_folder_to_mb(logs_dir, max_log_mb)
                cleanup_folder_to_mb(images_dir, max_image_mb)
                next_cleanup_time_s = time.time() + 60

            # ---- Console status line ----
            error_keys = list(record.get("errors", {}).keys())
            batt_v = (record.get("power", {}) or {}).get("battery_v")
            batt_pct = (record.get("power", {}) or {}).get("battery_pct")
            tx_ok = (record.get("radio", {}) or {}).get("tx_success")
            img_cap = (record.get("image", {}) or {}).get("captured")

            print(
                f"seq={sequence_id} time={record['timestamp_utc']} "
                f"bat={batt_v}V {batt_pct}% img={img_cap} "
                f"radio_ok={tx_ok} errors={error_keys}"
            )

            sleep_to_rate(loop_start_time_s, telemetry_hz)

    finally:
        try:
            if ups_bus is not None:
                ups_bus.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()