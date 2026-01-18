#!/usr/bin/env python3

#libraries general
import json #library for json files
import os
import shutil #librsry for deleting files and storage
import subprocess #library for running system commands
from pathlib import Path #to simplify file paths
from typing import Any, Dict #will allow dict to expect different data
import time #time library, starts from 1970
from datetime import datetime, timezone #library timestamps UTC
#hardware libraries
import board 
import busio
#environmental sensors. 
from adafruit_bme280 import basic as adafruit_bme280
import adafruit_ltr390
import adafruit_tsl2591
from adafruit_icm20x import ICM20948

#UPS power monitoring
from smbus2 import SMBus
# LoRa settings (from flight.py.save.py)
import sx126x

# -------------------------
# UPS (Waveshare UPS HAT E) constants 
# -------------------------
UPS_I2C_ADDR = 0x2D

LOW_BATT_PCT = 20
CRIT_BATT_PCT = 10
LOW_BATT_V = 3.55

# Camera settings from flight.py.save.py Idk if they accurate
IMAGE_PERIOD_S = 25
IMG_W, IMG_H = 640, 480
IMG_Q = 35
CAMERA_TIMEOUT_S = 8

#------------------------------
# LORA constants
#-------------------------------
LORA_PORT = "/dev/serial0"
LORA_FREQ_MHZ = 915
LORA_SRC_ADDR = 1
LORA_DEST_ADDR = 65535 #BUG POSSIBILITY IF THIS ADDREss dont match
LORA_POWER_DBM = 22
LORA_AIR_SPEED = 2400
LORA_NET_ID = 0
LORA_CRYPT = 0
LORA_RSSI = True

#must match driver to avoid split frames
LORA_BUFFER_SIZE = 240  #RLLY IMPORTANT. Could cause a bug possibly--------------
LORA_TX_RETRIES = 3
LORA_TX_GAP_S = 0.35

#function to retrieve UTC time
def utc_timestamp_iso() -> str:
    """Return current UTC time in ISO format (good for logs + syncing)."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")

#adds a line to json
def append_jsonl_safe(path: Path, record: Dict[str, Any]) -> str | None:
    """
    Append one telemetry record as a single JSON line.
    Returns:
        None on success, or an error string on failure.
    Flight rule: never crash the loop due to logging.
    """
    try:
        with path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, separators=(",", ":"), ensure_ascii=False) + "\n")
        return None
    except Exception as e: #if something goes wrong hopefully the exception gets caught
        return repr(e)


#function to avoid overheating
def sleep_to_rate(loop_start_time_s: float, telemetry_hz: float) -> None:
    """Sleep enough to maintain telemetry_hz, without busy-waiting."""
    period_s = 1.0 / telemetry_hz
    elapsed_s = time.time() - loop_start_time_s
    time.sleep(max(0.0, period_s - elapsed_s))

#cleans folder
def cleanup_folder_to_mb(folder: Path, max_mb: int) -> int:
    """Delete oldest files until folder is under max_mb. Returns deleted count."""
    if not folder.exists():
        return 0

    max_bytes = max_mb * 1024 * 1024
    files = [p for p in folder.rglob("*") if p.is_file()]
    files.sort(key=lambda p: p.stat().st_mtime)  # oldest first

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

#_______________________________________________________________________________________________________________
#following functions are for UPS

#for unsigned
def ups_read_u16(ups_bus: SMBus, reg: int) -> int:
    """ Function to Read unsigned 16-bit value from UPS registers (little-endian)."""
    lo = ups_bus.read_byte_data(UPS_I2C_ADDR, reg)
    hi = ups_bus.read_byte_data(UPS_I2C_ADDR, reg + 1)
    return (hi << 8) | lo

#for signed
def ups_read_i16(ups_bus: SMBus, reg: int) -> int:
    """Read signed 16-bit value from UPS registers."""
    v = ups_read_u16(ups_bus, reg)
    return v - 65536 if v & 0x8000 else v

def read_ups_status(ups_bus: SMBus) -> tuple[Dict[str, Any], str | None]:
    """
    Read UPS battery telemetry.

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
    """
    Convert battery readings into human-meaningful alert tags.
    - Main loop shouldn’t be full of threshold logic.
    - Alerts can be printed, logged, and transmitted.
    """
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

#------------------------------UPS functions end here-------------------------------------------------

#Following are functions to monitor the Pi health---------------------------------------------------
#makes subprocess calls consistent and safe.
def _run_cmd(cmd: list[str]) -> tuple[str, str | None]:
    """Run a command and return (stdout, error). Never raises."""
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
        return out.strip(), None
    except Exception as e:
        return "", repr(e)

#tells you undervoltage/throttling.
def read_pi_throttle_flags() -> tuple[Dict[str, Any], str | None]:
    """
    Read Pi power/throttle flags from 'vcgencmd get_throttled'.
    """
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

#catches overheating.
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

#the two following functions are not neccessary for script _______________________________________________________________
#meant for debugging.
#if the script restarts, uptime helps prove it.
def read_uptime_s() -> tuple[float | None, str | None]:
    """Read uptime seconds from /proc/uptime."""
    try:
        with open("/proc/uptime", "r", encoding="utf-8") as f:
            uptime_s = float(f.read().split()[0])
        return uptime_s, None
    except Exception as e:
        return None, repr(e)

#/proc/meminfo is a Linux file with RAM statistics
#Catch memory leaks / runaway processes: if available RAM keeps dropping every minute, something is wrong.
#Explain weird behavior: low memory can cause slowdowns, crashes, camera failures, etc.
#Debug after the fact: “radio started failing at seq=900” might correlate with memory exhaustion.
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

#warns before SD fills.
def read_disk_free_mb(path: Path) -> tuple[float | None, str | None]:
    """Disk free for a path in MB."""
    try:
        du = shutil.disk_usage(str(path))
        return du.free / (1024 * 1024), None
    except Exception as e:
        return None, repr(e)

#combines everything into one dict + errors.
def read_pi_health_status(data_root: Path) -> tuple[Dict[str, Any], Dict[str, str]]:
    """
    Combine Pi health readings into one dict + one errors dict.
    """
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

#---------------------------------------------------------------Pi functions end here

#Start of the camera capture function
def capture_image_jpeg(images_dir: Path, sequence_id: int) -> tuple[Dict[str, Any], str | None]:
    """
    Capture a JPEG using rpicam-still.

    Returns:
        (image_info, error)
        image_info example: {"path": "...", "bytes": 12345}
    """
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


#----------------------------------------------------------------------------------------------------------------
#start if Lora functions

#does radio configuration once.
def init_lora_radio() -> tuple[Any | None, str | None]:
    """Initialize LoRa radio using sx126x driver."""
    try:
        lora = sx126x.sx126x(
            #initialized from constants from beginning
            #if those are wrong welp this is cooked
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

#matches Waveshare framing? 
def build_waveshare_frame(dest_addr: int, src_addr: int, payload: bytes) -> bytes:
    base = 850
    #This function expects an offset
    #original Waveshare demo framing, that byte is usually (freq_mhz - 850) for 850–930MHz modules
    #Idk if that applies here. So if its not working uh its here. The offset version is just safer ig
    freq_off = LORA_FREQ_MHZ - base
    if not (0 <= freq_off <= 255):
        raise ValueError("LORA_FREQ_MHZ must be in 850–930 for Waveshare offset framing.")
    return bytes([
        (dest_addr >> 8) & 0xFF, dest_addr & 0xFF, freq_off & 0xFF,
        (src_addr >> 8) & 0xFF,  src_addr & 0xFF,  freq_off & 0xFF,
    ]) + payload


#function ensures LoRa stays small. Dont wanna go crazy
def build_compact_radio_packet(record: Dict[str, Any]) -> Dict[str, Any]:
    """Build a small radio packet from the full record."""
    env = record.get("environment", {}) or {}
    power = record.get("power", {}) or {}

    return {
        "ts": record.get("timestamp_utc"),
        "seq": record.get("sequence_id"),
        "t_c": env.get("temperature_c"),
        "p_hpa": env.get("pressure_hpa"),
        "rh": env.get("humidity_pct"),
        "bv": power.get("battery_v"),
        "bp": power.get("battery_pct"),
        "alerts": record.get("alerts", []),
        "img": 1 if (record.get("image", {}) or {}).get("captured") else 0,
    }


#sends the radio packet 
# handles retries + truncation (no crashes) allegedly we will see
def send_compact_radio_packet(lora: Any, packet: Dict[str, Any]) -> Dict[str, Any]:
    """Send compact packet over LoRa with retries."""
    #should return if for whatever reason lora not accessible
    if lora is None:
        return {"enabled": False, "tx_success": False, "tx_error": "lora_none"}

    payload = (json.dumps(packet, separators=(",", ":")) + "\n").encode("utf-8")

    max_payload = max(0, LORA_BUFFER_SIZE - 6)
    payload_truncated = False
    if len(payload) > max_payload:
        payload = payload[:max_payload]
        payload_truncated = True

    frame = build_waveshare_frame(LORA_DEST_ADDR, LORA_SRC_ADDR, payload)

    last_err = None
    for attempt in range(1, LORA_TX_RETRIES + 1):
        try:
            lora.send(frame)
            time.sleep(LORA_TX_GAP_S)
            return {
                "enabled": True,
                "tx_success": True,
                "attempt": attempt,
                "frame_len_bytes": len(frame),
                "payload_truncated": payload_truncated,
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
    }



#function solely for debugging, ever run will get its own folder. IDk if this one will
#be costly but we will see.
def create_run_directories(base_dir: Path) -> Dict[str, Path]:
    """
    Create a unique run directory structure.
    - Keeps each flight run separated (no overwrites).
    - Makes debugging easier: you can zip one run folder and share it.
    Returns:
        dict with run_dir, logs_dir, images_dir
    """
    run_id_utc = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    run_dir = base_dir / "runs" / run_id_utc
    logs_dir = run_dir / "logs"
    images_dir = run_dir / "images"

    logs_dir.mkdir(parents=True, exist_ok=True)
    images_dir.mkdir(parents=True, exist_ok=True)

    return {"run_dir": run_dir, "logs_dir": logs_dir, "images_dir": images_dir}

#Initialize the Raspberry Pi I2C bus.
def init_i2c_bus() -> busio.I2C:
    """
    - All I2C sensors share this bus.
    - Done once at startup; reused forever.
    """
    return busio.I2C(board.SCL, board.SDA)

#Initialize sensor objects.
def init_sensors(i2c: busio.I2C) -> Dict[str, Any]:
    """
    - Store None for sensors that fail init.

    Returns:
        dict with keys: bme280_sensor, ltr390_sensor, tsl2591_sensor, imu_sensor
    """
    #initialize sensor dictionary
    sensors: Dict[str, Any] = {
        "bme280_sensor": None,
        "ltr390_sensor": None,
        "tsl2591_sensor": None,
        "imu_sensor": None,
    }

#each sensor has its own catch block where if it fails we set it to none
#and we continue. Ik it looks repetitive but we aligning each sensor.

    # BME280
    try:
        sensors["bme280_sensor"] = adafruit_bme280.Adafruit_BME280_I2C(i2c)
    except Exception:
        sensors["bme280_sensor"] = None

    # LTR390
    try:
        sensors["ltr390_sensor"] = adafruit_ltr390.LTR390(i2c)
    except Exception:
        sensors["ltr390_sensor"] = None

    # TSL2591
    try:
        sensors["tsl2591_sensor"] = adafruit_tsl2591.TSL2591(i2c)
    except Exception:
        sensors["tsl2591_sensor"] = None

    # ICM20948 IMU
    try:
        sensors["imu_sensor"] = ICM20948(i2c)
    except Exception:
        sensors["imu_sensor"] = None

    #return the sensor "object"
    return sensors

#function responsible for getting data from sensors
def read_sensors(sensors: Dict[str, Any]) -> tuple[Dict[str, Any], Dict[str, str]]:
    """
    Returns:
        (data, errors)
        - data: nested dict of sensor measurements
        - errors: dict mapping sensor_name -> error string
    """
    #setting lists to empty
    data: Dict[str, Any] = {}
    errors: Dict[str, str] = {}

    #.get function to get what we got from function above
    bme280_sensor = sensors.get("bme280_sensor")
    ltr390_sensor = sensors.get("ltr390_sensor")
    tsl2591_sensor = sensors.get("tsl2591_sensor")
    imu_sensor = sensors.get("imu_sensor")

    #overall the next following functions sort retrieved data from sensors
    #written in a try and catch to avoid crashes

    # Environment
    if bme280_sensor is not None: #only runs if sensor actually got data
        try:
            data["environment"] = {
                "temperature_c": float(bme280_sensor.temperature),
                "humidity_pct": float(bme280_sensor.humidity),
                "pressure_hpa": float(bme280_sensor.pressure),
            }
        except Exception as e:
            errors["bme280"] = repr(e) #possible error

    # UV / ALS
    if ltr390_sensor is not None:
        try:
            data["uv_light"] = {
                "uv_raw": float(ltr390_sensor.uvs),
                "ambient_light_raw": float(ltr390_sensor.light),
            }
        except Exception as e:
            errors["ltr390"] = repr(e)

    # Lux sensor
    if tsl2591_sensor is not None:
        try:
            data["light"] = {
                "lux": float(tsl2591_sensor.lux),
                "infrared": float(tsl2591_sensor.infrared),
                "visible": float(tsl2591_sensor.visible),
            }
        except Exception as e:
            errors["tsl2591"] = repr(e)

    # IMU
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

#main function of the script. 
def main() -> None:
    """
    - create run dirs
    - init sensors + UPS + LoRa
    - each loop: read sensors + UPS + Pi health
    - occasionally: capture image (store only metadata in record["image"])
    - send compact LoRa packet
    - log JSONL (fail-safe)
    - cleanup logs/images periodically
    - pace loop to telemetry_hz
    """
    telemetry_hz = 1.0
    base_dir = Path("/home/pi/hab")

    # Storage caps (MB)
    max_log_mb = 200
    max_image_mb = 500

    # Create a new run folder for this execution
    dirs = create_run_directories(base_dir)
    logs_dir = dirs["logs_dir"]
    images_dir = dirs["images_dir"]
    telemetry_log_path = logs_dir / "telemetry.jsonl"

    # Hardware initialization (do once)
    i2c = init_i2c_bus()
    sensors = init_sensors(i2c)

    ups_bus = SMBus(1)  # UPS power monitor on I2C bus 1

    lora, lora_err = init_lora_radio()
    if lora_err:
        # Don't crash if radio fails; just run without it
        print(f"LoRa init error: {lora_err}")

    sequence_id = 0
    next_cleanup_time_s = time.time() + 60
    last_image_time_s = 0.0  # for image period control

    try:
        while True:
            loop_start_time_s = time.time()
            now_time_s = loop_start_time_s
            sequence_id += 1

            # The telemetry record for this iteration
            record: Dict[str, Any] = {
                "timestamp_utc": utc_timestamp_iso(),
                "sequence_id": sequence_id,
            }

            # Always include an image object so logs have consistent schema
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
            power_status, ups_err = read_ups_status(ups_bus)
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

                # Update image metadata (do NOT store image bytes in JSONL)
                record["image"].update({
                    "seq": sequence_id,
                    "path": image_info.get("path"),
                    "bytes": image_info.get("bytes"),
                    "captured": (img_err is None) and bool(image_info.get("path")),
                    "error": img_err,
                })

                # Keep a top-level error tag too (optional but handy)
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
            ups_bus.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
