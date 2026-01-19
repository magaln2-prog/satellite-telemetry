#!/usr/bin/env python3

"""flight_fixed.py

Flight-side telemetry script (Raspberry Pi payload).

What this script does (high-level):
  1) Reads sensors (BME280, LTR390, TSL2591, ICM20948) over I2C.
  2) Reads UPS battery status over I2C (Waveshare UPS HAT E).
  3) Optionally captures a JPEG using rpicam-still (stores *metadata* only).
  4) Builds TWO telemetry records:
       - A full record written locally as JSONL (one JSON per line).
       - A compact record sent over LoRa (newline-delimited JSON).
  5) Runs forever at a fixed telemetry rate without busy-waiting.

Important reliability rules used here:
  - **Never truncate JSON bytes**. Truncating causes broken JSON on the ground.
    If a packet is too large, we send a tiny fallback packet instead.
  - **Use one UART frame per LoRa packet**. The sx126x driver is patched to
    raise if the frame is too large, instead of silently chunking.
  - **Receiver reassembly happens on the ground** (rx_to_latest.py buffers
    until it sees '\n'). The flight side always appends '\n' to each packet.
"""

# -------------------------
# Standard libraries
# -------------------------
import json  # JSON encode/decode
import shutil  # disk usage + deleting old files
import subprocess  # run system commands like vcgencmd and rpicam-still
from pathlib import Path  # clean path handling
from typing import Any, Dict
import time  # sleep + timing
from datetime import datetime, timezone  # timestamps in UTC

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

# LoRa driver (patched to be telemetry-safe)
import sx126x

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
LORA_PORT = "/dev/serial0"      # UART device
LORA_FREQ_MHZ = 915             # must be within module band (850–930 for 900MHz model)
LORA_SRC_ADDR = 1               # this node's address (configured in the module)
LORA_DEST_ADDR = 65535          # broadcast on many Ebyte/Waveshare modules
LORA_POWER_DBM = 22
LORA_AIR_SPEED = 2400
LORA_NET_ID = 0
LORA_CRYPT = 0
LORA_RSSI = True

# Driver-supported package sizes are typically: 240, 128, 64, 32
# IMPORTANT: This is the module's maximum *UART packet size*.
LORA_BUFFER_SIZE = 240

# TX behavior
LORA_TX_RETRIES = 3
LORA_TX_GAP_S = 0.35

# The 3rd header byte in fixed-address mode is commonly a "frequency offset".
# For 900MHz modules: offset = freq_mhz - 850 (0..80).
LORA_BASE_FREQ_MHZ = 850

# -------------------------
# Time helpers
# -------------------------

def utc_timestamp_iso() -> str:
    """Return current UTC time in ISO format (good for logs + syncing)."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")

# -------------------------
# Local logging (JSONL)
# -------------------------

def append_jsonl_safe(path: Path, record: Dict[str, Any]) -> str | None:
    """Append one telemetry record as a single JSON line.

    Why JSONL?
      - Easy to append (no re-writing the whole file)
      - Easy to stream/parse (one object per line)
      - Survives power loss better than a single giant JSON array

    Returns:
      None on success, or an error string on failure.

    Flight rule:
      - Never crash the main loop due to logging.
    """
    try:
        with path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, separators=(",", ":"), ensure_ascii=False) + "\n")
        return None
    except Exception as e:
        return repr(e)

# -------------------------
# Loop pacing (avoid overheating / busy-wait)
# -------------------------

def sleep_to_rate(loop_start_time_s: float, telemetry_hz: float) -> None:
    """Sleep enough to maintain telemetry_hz, without busy-waiting."""
    period_s = 1.0 / telemetry_hz
    elapsed_s = time.time() - loop_start_time_s
    time.sleep(max(0.0, period_s - elapsed_s))

# -------------------------
# Storage cleanup
# -------------------------

def cleanup_folder_to_mb(folder: Path, max_mb: int) -> int:
    """Delete oldest files until folder is under max_mb. Returns deleted count."""
    if not folder.exists():
        return 0

    max_bytes = max_mb * 1024 * 1024
    files = [p for p in folder.rglob("*") if p.is_file()]

    # Sort oldest first so we delete old runs before new ones.
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
            # Never crash cleanup.
            pass

    return deleted

# --------------------------------------------------------------------------------------------------
# UPS helpers
# --------------------------------------------------------------------------------------------------

def ups_read_u16(ups_bus: SMBus, reg: int) -> int:
    """Read unsigned 16-bit value from UPS registers (little-endian)."""
    lo = ups_bus.read_byte_data(UPS_I2C_ADDR, reg)
    hi = ups_bus.read_byte_data(UPS_I2C_ADDR, reg + 1)
    return (hi << 8) | lo


def ups_read_i16(ups_bus: SMBus, reg: int) -> int:
    """Read signed 16-bit value from UPS registers."""
    v = ups_read_u16(ups_bus, reg)
    return v - 65536 if v & 0x8000 else v


def read_ups_status(ups_bus: SMBus) -> tuple[Dict[str, Any], str | None]:
    """Read UPS battery telemetry.

    Returns:
      (power_status, error)

    power_status example:
      {"battery_v": 3.98, "battery_a": 0.12, "battery_pct": 82, ...}

    Flight rule:
      - never raise; return an error string instead.
    """
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
    """Convert battery readings into human-meaningful alert tags."""
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
    """Run a command and return (stdout, error). Never raises."""
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
        return out.strip(), None
    except Exception as e:
        return "", repr(e)


def read_pi_throttle_flags() -> tuple[Dict[str, Any], str | None]:
    """Read Pi power/throttle flags from 'vcgencmd get_throttled'."""
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
    """Read Pi CPU temperature in Celsius via vcgencmd."""
    out, err = _run_cmd(["vcgencmd", "measure_temp"])
    if err:
        return None, err

    try:
        temp_c = float(out.split("=")[1].split("'")[0])
        return temp_c, None
    except Exception as e:
        return None, repr(e)


def read_uptime_s() -> tuple[float | None, str | None]:
    """Read uptime seconds from /proc/uptime."""
    try:
        with open("/proc/uptime", "r", encoding="utf-8") as f:
            uptime_s = float(f.read().split()[0])
        return uptime_s, None
    except Exception as e:
        return None, repr(e)


def read_mem_info_mb() -> tuple[Dict[str, Any], str | None]:
    """Read basic memory stats from /proc/meminfo in MB."""
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
    """Disk free for a path in MB."""
    try:
        du = shutil.disk_usage(str(path))
        return du.free / (1024 * 1024), None
    except Exception as e:
        return None, repr(e)


def read_pi_health_status(data_root: Path) -> tuple[Dict[str, Any], Dict[str, str]]:
    """Combine Pi health readings into one dict + one errors dict."""
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
    """Capture a JPEG using rpicam-still.

    Returns:
      (image_info, error)

    image_info example:
      {"path": "/home/pi/hab/.../img_000123.jpg", "bytes": 12345}

    We store metadata only (path/size), NOT the image bytes.
    """
    try:
        images_dir.mkdir(parents=True, exist_ok=True)
        img_path = images_dir / f"img_{sequence_id:06d}.jpg"

        # rpicam-still is the modern camera tool on Raspberry Pi OS.
        cmd = [
            "rpicam-still",
            "-n",                # no preview
            "-t", "300",         # capture delay ms
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
    """Initialize LoRa radio using sx126x driver."""
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
    """Compute the fixed-mode frequency offset byte.

    For 900MHz modules, base is typically 850MHz.
    Example: 915MHz -> off = 65.

    This value goes into the 3rd header byte.
    """
    off = int(freq_mhz) - int(LORA_BASE_FREQ_MHZ)
    if not (0 <= off <= 255):
        raise ValueError(f"Frequency offset out of range: freq={freq_mhz} base={LORA_BASE_FREQ_MHZ} off={off}")
    return off


def build_fixed_frame(dest_addr: int, payload: bytes) -> bytes:
    """Build a fixed-addressing UART frame.

    Fixed mode header (3 bytes):
      [DEST_H][DEST_L][FREQ_OFFSET]

    The patched sx126x driver also includes `build_fixed_tx_frame()`. We keep
    a local helper here so the flight script stays readable.
    """
    freq_off = compute_freq_off(LORA_FREQ_MHZ)
    return bytes([
        (dest_addr >> 8) & 0xFF,
        dest_addr & 0xFF,
        freq_off & 0xFF,
    ]) + payload


def build_compact_radio_packet(record: Dict[str, Any]) -> Dict[str, Any]:
    """Build a small radio packet from the full record.

    Keep it tiny:
      - short keys
      - fewer nested fields
      - don't include big arrays unless needed
    """
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

    # Only include alerts if they exist (saves bytes most of the time).
    if alerts:
        pkt["alerts"] = alerts

    return pkt


def send_compact_radio_packet(lora: Any, packet: Dict[str, Any]) -> Dict[str, Any]:
    """Send compact packet over LoRa with retries.

    Important:
      - We never cut JSON bytes.
      - If too big, we send a tiny fallback packet that is guaranteed valid JSON.
    """
    if lora is None:
        return {"enabled": False, "tx_success": False, "tx_error": "lora_none"}

    # Newline-delimited JSON. The receiver buffers until it sees '\n'.
    payload = (json.dumps(packet, separators=(",", ":")) + "\n").encode("utf-8")

    # For fixed header [DEST_H][DEST_L][FREQ_OFF] we reserve 3 bytes.
    # Ask the driver for the exact maximum (it knows buffer_size).
    max_payload = lora.max_app_payload_bytes(header_len=3)

    payload_truncated = False
    if len(payload) > max_payload:
        # Do NOT slice the JSON bytes. That creates broken JSON on the ground.
        # Instead send a tiny packet that still conveys time/sequence.
        tiny = {"ts": packet.get("ts"), "seq": packet.get("seq"), "E": "too_big"}
        payload = (json.dumps(tiny, separators=(",", ":")) + "\n").encode("utf-8")
        payload_truncated = True

    # Build a full UART frame (header + payload).
    frame = build_fixed_frame(LORA_DEST_ADDR, payload)

    last_err: str | None = None
    for attempt in range(1, LORA_TX_RETRIES + 1):
        try:
            lora.send(frame)  # patched driver raises if frame > buffer_size
            time.sleep(LORA_TX_GAP_S)
            return {
                "enabled": True,
                "tx_success": True,
                "attempt": attempt,
                "frame_len_bytes": len(frame),
                "payload_truncated": payload_truncated,
                "max_payload": max_payload,
            }
        except Exception as e:
            last_err = repr(e)
            time.sleep(LORA_TX_GAP_S)

    return {
        "enabled": True,
        "tx_success": False,
        "attempt": LORA_TX_RETRIES,
        "frame_len_bytes": len(frame),
        "payload_truncated": payload_truncated,
        "tx_error": last_err or "unknown",
        "max_payload": max_payload,
    }

# --------------------------------------------------------------------------------------------------
# Sensor initialization + reading
# --------------------------------------------------------------------------------------------------

def create_run_directories(base_dir: Path) -> Dict[str, Path]:
    """Create a unique run directory structure per execution."""
    run_id_utc = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    run_dir = base_dir / "runs" / run_id_utc
    logs_dir = run_dir / "logs"
    images_dir = run_dir / "images"

    logs_dir.mkdir(parents=True, exist_ok=True)
    images_dir.mkdir(parents=True, exist_ok=True)

    return {"run_dir": run_dir, "logs_dir": logs_dir, "images_dir": images_dir}


def init_i2c_bus() -> busio.I2C:
    """Initialize the Raspberry Pi I2C bus (shared by all I2C sensors)."""
    return busio.I2C(board.SCL, board.SDA)


def init_sensors(i2c: busio.I2C) -> Dict[str, Any]:
    """Initialize sensor objects.

    If a sensor fails init, we store None and keep the script running.
    """
    sensors: Dict[str, Any] = {
        "bme280_sensor": None,
        "ltr390_sensor": None,
        "tsl2591_sensor": None,
        "imu_sensor": None,
    }

    # BME280 (temperature/humidity/pressure)
    try:
        sensors["bme280_sensor"] = adafruit_bme280.Adafruit_BME280_I2C(i2c)
    except Exception:
        sensors["bme280_sensor"] = None

    # LTR390 (UV + ambient)
    try:
        sensors["ltr390_sensor"] = adafruit_ltr390.LTR390(i2c)
    except Exception:
        sensors["ltr390_sensor"] = None

    # TSL2591 (lux)
    try:
        sensors["tsl2591_sensor"] = adafruit_tsl2591.TSL2591(i2c)
    except Exception:
        sensors["tsl2591_sensor"] = None

    # ICM20948 IMU (accel/gyro)
    try:
        sensors["imu_sensor"] = ICM20948(i2c)
    except Exception:
        sensors["imu_sensor"] = None

    return sensors


def read_sensors(sensors: Dict[str, Any]) -> tuple[Dict[str, Any], Dict[str, str]]:
    """Read sensor values into a nested dict.

    Returns:
      (data, errors)
    """
    data: Dict[str, Any] = {}
    errors: Dict[str, str] = {}

    bme280_sensor = sensors.get("bme280_sensor")
    ltr390_sensor = sensors.get("ltr390_sensor")
    tsl2591_sensor = sensors.get("tsl2591_sensor")
    imu_sensor = sensors.get("imu_sensor")

    # Environment (BME280)
    if bme280_sensor is not None:
        try:
            data["environment"] = {
                "temperature_c": float(bme280_sensor.temperature),
                "humidity_pct": float(bme280_sensor.humidity),
                "pressure_hpa": float(bme280_sensor.pressure),
            }
        except Exception as e:
            errors["bme280"] = repr(e)

    # UV / ambient (LTR390)
    if ltr390_sensor is not None:
        try:
            data["uv_light"] = {
                "uv_raw": float(ltr390_sensor.uvs),
                "ambient_light_raw": float(ltr390_sensor.light),
            }
        except Exception as e:
            errors["ltr390"] = repr(e)

    # Lux (TSL2591)
    if tsl2591_sensor is not None:
        try:
            data["light"] = {
                "lux": float(tsl2591_sensor.lux),
                "infrared": float(tsl2591_sensor.infrared),
                "visible": float(tsl2591_sensor.visible),
            }
        except Exception as e:
            errors["tsl2591"] = repr(e)

    # IMU (ICM20948)
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
    """Main telemetry loop."""

    telemetry_hz = 1.0

    # Base folder for all runs
    base_dir = Path("/home/pi/hab")

    # Storage caps (MB)
    max_log_mb = 200
    max_image_mb = 500

    # Create a new run folder for this execution
    dirs = create_run_directories(base_dir)
    logs_dir = dirs["logs_dir"]
    images_dir = dirs["images_dir"]
    telemetry_log_path = logs_dir / "telemetry.jsonl"

    # Hardware initialization (done once)
    i2c = init_i2c_bus()
    sensors = init_sensors(i2c)

    # UPS bus init: if it fails, we keep running but battery fields will be empty.
    ups_bus: SMBus | None
    try:
        ups_bus = SMBus(1)
    except Exception:
        ups_bus = None

    # LoRa init: if it fails, we keep running but radio tx will be disabled.
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

            # A full telemetry record (for local JSONL storage)
            record: Dict[str, Any] = {
                "timestamp_utc": utc_timestamp_iso(),
                "sequence_id": sequence_id,
            }

            # Always include an image object so logs have consistent schema.
            record["image"] = {
                "captured": False,
                "seq": None,
                "path": None,
                "bytes": None,
                "error": None,
            }

            # ---- Sensors ----
            sensor_data, sensor_errors = read_sensors(sensors)
            record.update(sensor_data)
            if sensor_errors:
                record.setdefault("errors", {})
                record["errors"].update(sensor_errors)

            # ---- UPS Power ----
            if ups_bus is not None:
                power_status, ups_err = read_ups_status(ups_bus)
            else:
                power_status, ups_err = ({}, "ups_bus_init_failed")

            record["power"] = power_status
            record["alerts"] = compute_battery_alerts(power_status)
            if ups_err:
                record.setdefault("errors", {})
                record["errors"]["ups"] = ups_err

            # ---- Pi Health ----
            pi_health, pi_errs = read_pi_health_status(base_dir)
            record["pi_health"] = pi_health
            if pi_errs:
                record.setdefault("errors", {})
                for k, v in pi_errs.items():
                    record["errors"][f"pi_{k}"] = v

            # ---- Camera (rate controlled; slow down if battery critical) ----
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

            # ---- JSONL log (fail-safe) ----
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
                f"bat={batt_v}V {batt_pct}% img={img_cap} radio_ok={tx_ok} errors={error_keys}"
            )

            # ---- Pace loop ----
            sleep_to_rate(loop_start_time_s, telemetry_hz)

    finally:
        # Close the UPS bus cleanly
        try:
            if ups_bus is not None:
                ups_bus.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
