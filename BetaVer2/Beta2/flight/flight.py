#!/usr/bin/env python3
"""
flight.py (FLIGHT)

Optimizations implemented:
1) Different retry policy:
   - FAST packets: 1 TX copy per fragment
   - FULL packets: 2 TX copies per fragment

3) Threshold-gated FULL:
   - Send FULL only when values change beyond thresholds OR after FULL_MAX_PERIOD_S

Still:
- FAST packet every loop (smooth UI)
- FULL refresh guaranteed (max period)
- Binary payloads + TM fragment framing
"""

import json
import subprocess
from pathlib import Path
from typing import Any, Dict
import time
from datetime import datetime, timezone

import board
import busio

from adafruit_bme280 import basic as adafruit_bme280
import adafruit_ltr390
import adafruit_tsl2591
from adafruit_icm20x import ICM20948

from smbus2 import SMBus
import sx126x

from protocol_tm import (
    build_frame as _tm_build_frame,
    HDR_LEN as _TM_HDR_LEN,
    pack_fast,
    pack_full,
)

# -------------------------
# UPS (Waveshare UPS HAT E)
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

LORA_BUFFER_SIZE = 240
LORA_TX_GAP_S = 0.20

LORA_BASE_FREQ_MHZ = 850


# -------------------------
# TX rates
# -------------------------
FAST_HZ = 2.0                 # FAST rate (packets/sec)
FULL_MAX_PERIOD_S = 5.0       # Guaranteed FULL at least this often

# Thresholds for “changed enough” to trigger FULL early
THRESH = {
    "t_c": 0.10,      # °C
    "rh": 1.0,        # %
    "p_hpa": 0.3,     # hPa
    "lux_rel": 0.10,  # 10% change OR
    "lux_abs": 30.0,  # 30 lux absolute change
    "uvi": 0.5,       # (mapped uv_raw) change
    "cpu_c": 0.3,     # °C
    "bv": 0.02,       # V
    "bp": 1.0,        # %
    "flags": 1,       # any change triggers
}


def utc_timestamp_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def sleep_to_rate(loop_start_time_s: float, hz: float) -> None:
    period_s = 1.0 / max(0.1, hz)
    elapsed_s = time.time() - loop_start_time_s
    time.sleep(max(0.0, period_s - elapsed_s))


def append_jsonl_safe(path: Path, record: Dict[str, Any]) -> str | None:
    try:
        with path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, separators=(",", ":"), ensure_ascii=False) + "\n")
        return None
    except Exception as e:
        return repr(e)


# -------------------------
# UPS helper functions
# -------------------------
def ups_read_u16(bus: SMBus, reg: int) -> int:
    lo = bus.read_byte_data(UPS_I2C_ADDR, reg)
    hi = bus.read_byte_data(UPS_I2C_ADDR, reg + 1)
    return (hi << 8) | lo


def ups_read_i16(bus: SMBus, reg: int) -> int:
    v = ups_read_u16(bus, reg)
    return v - 65536 if v & 0x8000 else v


def init_ups_bus() -> tuple[SMBus | None, str | None]:
    try:
        return SMBus(1), None
    except Exception as e:
        return None, repr(e)


def read_ups_status(ups_bus: SMBus) -> tuple[Dict[str, Any], str | None]:
    try:
        batt_mv = ups_read_u16(ups_bus, 0x20)
        batt_ma = ups_read_i16(ups_bus, 0x22)
        batt_pct = ups_read_u16(ups_bus, 0x24)
        rem_mah = ups_read_u16(ups_bus, 0x26)
        rem_min = ups_read_u16(ups_bus, 0x28)

        return {
            "battery_v": batt_mv / 1000.0,
            "battery_a": batt_ma / 1000.0,
            "battery_pct": batt_pct,
            "remaining_mah": rem_mah,
            "remaining_min": rem_min,
        }, None
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


# -------------------------
# Pi health helpers
# -------------------------
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
        return {
            "raw": raw_hex,
            "undervoltage_now": bool(v & (1 << 0)),
            "freq_capped_now": bool(v & (1 << 1)),
            "throttled_now": bool(v & (1 << 2)),
            "undervoltage_has": bool(v & (1 << 16)),
            "freq_capped_has": bool(v & (1 << 17)),
            "throttled_has": bool(v & (1 << 18)),
        }, None
    except Exception as e:
        return {}, repr(e)


def read_cpu_temp_c() -> tuple[float | None, str | None]:
    out, err = _run_cmd(["vcgencmd", "measure_temp"])
    if err:
        return None, err
    try:
        return float(out.split("=")[1].split("'")[0]), None
    except Exception as e:
        return None, repr(e)


def read_pi_health_status() -> tuple[Dict[str, Any], Dict[str, str]]:
    health: Dict[str, Any] = {}
    errors: Dict[str, str] = {}

    throttle, err = read_pi_throttle_flags()
    if err:
        errors["throttle_flags"] = err
    else:
        health["throttle_flags"] = throttle

    cpu_temp_c, err = read_cpu_temp_c()
    if err:
        errors["cpu_temp"] = err
    else:
        health["cpu_temp_c"] = cpu_temp_c

    return health, errors


# -------------------------
# LoRa init
# -------------------------
def init_lora() -> tuple[Any | None, str | None]:
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


# -------------------------
# Radio TX helpers
# -------------------------
def compute_freq_off(freq_mhz: int) -> int:
    off = int(freq_mhz) - int(LORA_BASE_FREQ_MHZ)
    if not (0 <= off <= 255):
        raise ValueError(f"Frequency offset out of range: freq={freq_mhz} base={LORA_BASE_FREQ_MHZ} off={off}")
    return off


def build_fixed_frame(dest_addr: int, payload: bytes) -> bytes:
    freq_off = compute_freq_off(LORA_FREQ_MHZ)
    return bytes([(dest_addr >> 8) & 0xFF, dest_addr & 0xFF, freq_off & 0xFF]) + payload


def _pack_flags_byte(pi_throttle: Dict[str, Any]) -> int:
    # bit0 undervoltage_now
    # bit1 throttled_now
    # bit2 freq_capped_now
    # bit3 undervoltage_has
    # bit4 throttled_has
    # bit5 freq_capped_has
    b = 0
    if pi_throttle.get("undervoltage_now"): b |= (1 << 0)
    if pi_throttle.get("throttled_now"):    b |= (1 << 1)
    if pi_throttle.get("freq_capped_now"):  b |= (1 << 2)
    if pi_throttle.get("undervoltage_has"): b |= (1 << 3)
    if pi_throttle.get("throttled_has"):    b |= (1 << 4)
    if pi_throttle.get("freq_capped_has"):  b |= (1 << 5)
    return b & 0xFF


def send_blob_over_tm(lora: Any, msg_id: int, blob: bytes, frag_tx_copies: int) -> Dict[str, Any]:
    """Send bytes over TM fragmentation. frag_tx_copies controls reliability."""
    if lora is None:
        return {"enabled": False, "tx_success": False, "tx_error": "lora_none"}

    # Reserve 3 bytes for [DEST_H][DEST_L][FREQ_OFF]
    max_app = lora.max_app_payload_bytes(header_len=3)

    # Conservative fragment sizing
    TARGET_MAX_APP = 64
    max_app = min(max_app, TARGET_MAX_APP)

    max_chunk = max_app - _TM_HDR_LEN
    if max_chunk <= 0:
        return {"enabled": True, "tx_success": False, "tx_error": "buffer_too_small_for_tm", "max_app": max_app}

    chunks = [blob[i:i + max_chunk] for i in range(0, len(blob), max_chunk)]
    frag_tot = len(chunks)

    sent = 0
    last_err = None
    copies = max(1, int(frag_tx_copies))

    for frag_idx, chunk in enumerate(chunks):
        try:
            tm_payload = _tm_build_frame(msg_id & 0xFFFF, frag_idx, frag_tot, chunk)
        except Exception as e:
            return {"enabled": True, "tx_success": False, "tx_error": f"tm_build_failed:{e}"}

        frame = build_fixed_frame(LORA_DEST_ADDR, tm_payload)

        ok = False
        for _ in range(copies):
            try:
                lora.send(frame)
                sent += 1
                ok = True
            except Exception as e:
                last_err = repr(e)
                ok = False
            time.sleep(LORA_TX_GAP_S)

        if not ok:
            return {
                "enabled": True,
                "tx_success": False,
                "tx_error": last_err or "tx_failed",
                "frags_total": frag_tot,
                "frags_sent": sent,
            }

    return {"enabled": True, "tx_success": True, "frags_total": frag_tot, "frags_sent": sent, "copies": copies}


# -------------------------
# Sensors
# -------------------------
def init_i2c_bus() -> busio.I2C:
    return busio.I2C(board.SCL, board.SDA)


def init_sensors(i2c: busio.I2C) -> Dict[str, Any]:
    sensors: Dict[str, Any] = {
        "bme280_sensor": None,
        "ltr390_sensor": None,
        "tsl2591_sensor": None,
        "imu_sensor": None,
        "_init_errors": {},
    }

    init_errors: Dict[str, str] = {}

    try:
        sensors["bme280_sensor"] = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
    except Exception as e76:
        try:
            sensors["bme280_sensor"] = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x77)
        except Exception as e77:
            sensors["bme280_sensor"] = None
            init_errors["bme280_init"] = f"0x76 failed: {type(e76).__name__}: {e76}; 0x77 failed: {type(e77).__name__}: {e77}"

    try:
        sensors["ltr390_sensor"] = adafruit_ltr390.LTR390(i2c)
    except Exception as e:
        sensors["ltr390_sensor"] = None
        init_errors["ltr390_init"] = f"{type(e).__name__}: {e}"

    try:
        sensors["tsl2591_sensor"] = adafruit_tsl2591.TSL2591(i2c)
    except Exception as e:
        sensors["tsl2591_sensor"] = None
        init_errors["tsl2591_init"] = f"{type(e).__name__}: {e}"

    try:
        sensors["imu_sensor"] = ICM20948(i2c, address=0x68)
    except Exception as e:
        sensors["imu_sensor"] = None
        init_errors["imu_init"] = f"{type(e).__name__}: {e}"

    sensors["_init_errors"] = init_errors
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
            ax, ay, az = imu_sensor.acceleration
            gx, gy, gz = imu_sensor.gyro
            data["imu"] = {
                "accel_mps2": [float(ax), float(ay), float(az)],
                "gyro_rps": [float(gx), float(gy), float(gz)],
            }
        except Exception as e:
            errors["icm20948"] = repr(e)

    return data, errors


# -------------------------
# FULL gating helpers
# -------------------------
def _abs_delta(a, b) -> float:
    try:
        return abs(float(a) - float(b))
    except Exception:
        return float("inf")


def _lux_changed(lux_now, lux_prev) -> bool:
    try:
        n = float(lux_now)
        p = float(lux_prev)
        if _abs_delta(n, p) >= THRESH["lux_abs"]:
            return True
        # relative threshold (avoid divide by ~0)
        denom = max(1.0, abs(p))
        return (abs(n - p) / denom) >= THRESH["lux_rel"]
    except Exception:
        return True


def should_send_full(now_vals: dict, prev_vals: dict) -> bool:
    """Return True if FULL should be sent due to threshold changes."""
    if not prev_vals:
        return True

    if _abs_delta(now_vals.get("t_c"), prev_vals.get("t_c")) >= THRESH["t_c"]:
        return True
    if _abs_delta(now_vals.get("rh"), prev_vals.get("rh")) >= THRESH["rh"]:
        return True
    if _abs_delta(now_vals.get("p_hpa"), prev_vals.get("p_hpa")) >= THRESH["p_hpa"]:
        return True
    if _lux_changed(now_vals.get("lux"), prev_vals.get("lux")):
        return True
    if _abs_delta(now_vals.get("uvi"), prev_vals.get("uvi")) >= THRESH["uvi"]:
        return True
    if _abs_delta(now_vals.get("cpu_c"), prev_vals.get("cpu_c")) >= THRESH["cpu_c"]:
        return True
    if _abs_delta(now_vals.get("bv"), prev_vals.get("bv")) >= THRESH["bv"]:
        return True
    if _abs_delta(now_vals.get("bp"), prev_vals.get("bp")) >= THRESH["bp"]:
        return True
    if int(now_vals.get("flags") or 0) != int(prev_vals.get("flags") or 0):
        return True

    return False


# -------------------------
# Main loop
# -------------------------
def main() -> None:
    base_dir = Path(__file__).resolve().parent
    logs_dir = base_dir / "runs" / datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S") / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    telemetry_log_path = logs_dir / "telemetry.jsonl"

    i2c = init_i2c_bus()
    sensors = init_sensors(i2c)

    ups_bus, ups_err0 = init_ups_bus()
    if ups_err0:
        print("UPS init error:", ups_err0)

    lora, lora_err = init_lora()
    if lora_err:
        print("LoRa init error:", lora_err)

    seq = 0
    last_full_tx_s = 0.0
    prev_full_vals: dict = {}

    print("Flight running...")

    try:
        while True:
            loop_start = time.time()
            seq += 1
            now_unix = int(loop_start)

            record: Dict[str, Any] = {
                "timestamp_utc": utc_timestamp_iso(),
                "sequence_id": seq,
                "environment": {},
                "uv_light": {},
                "light": {},
                "imu": {},
                "power": {},
                "alerts": [],
                "pi_health": {},
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

            pi_health, pi_errs = read_pi_health_status()
            record["pi_health"] = pi_health
            if pi_errs:
                record.setdefault("errors", {})
                for k, v in pi_errs.items():
                    record["errors"][f"pi_{k}"] = v

            # ---- Extract values for TX ----
            env = record.get("environment", {}) or {}
            power = record.get("power", {}) or {}
            light = record.get("light", {}) or {}
            uv = record.get("uv_light", {}) or {}
            pi = record.get("pi_health", {}) or {}

            t_c = env.get("temperature_c")
            rh = env.get("humidity_pct")
            p_hpa = env.get("pressure_hpa")
            lux = light.get("lux")
            uvi = uv.get("uv_raw")  # mapped to keep UI populated
            cpu_c = pi.get("cpu_temp_c")
            bv = power.get("battery_v")
            bp = power.get("battery_pct")
            flags = _pack_flags_byte((pi.get("throttle_flags") or {}))

            # FAST always (copies=1)
            fast_blob = pack_fast(seq, now_unix, t_c, rh, p_hpa, lux, uvi)
            tx_fast = send_blob_over_tm(lora, seq, fast_blob, frag_tx_copies=1)

            # FULL: threshold gated OR max period (copies=2)
            now_full_vals = {
                "t_c": t_c, "rh": rh, "p_hpa": p_hpa, "lux": lux, "uvi": uvi,
                "cpu_c": cpu_c, "bv": bv, "bp": bp, "flags": flags
            }

            force_due_to_time = (loop_start - last_full_tx_s) >= FULL_MAX_PERIOD_S
            force_due_to_change = should_send_full(now_full_vals, prev_full_vals)

            if force_due_to_time or force_due_to_change:
                full_blob = pack_full(seq, now_unix, t_c, rh, p_hpa, lux, uvi, cpu_c, bv, bp, flags)
                tx_full = send_blob_over_tm(lora, seq, full_blob, frag_tx_copies=2)
                last_full_tx_s = loop_start
                prev_full_vals = now_full_vals
            else:
                tx_full = None

            record["radio"] = {"fast": tx_fast, "full": tx_full}

            log_error = append_jsonl_safe(telemetry_log_path, record)
            if log_error:
                record.setdefault("errors", {})
                record["errors"]["jsonl_log"] = log_error

            # Print minimal status
            print(
                f"seq={seq} radio_fast={tx_fast.get('tx_success')} "
                f"full_sent={'yes' if tx_full else 'no'} "
                f"copies_fast=1 copies_full=2"
            )

            sleep_to_rate(loop_start, FAST_HZ)

    finally:
        try:
            if ups_bus is not None:
                ups_bus.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
