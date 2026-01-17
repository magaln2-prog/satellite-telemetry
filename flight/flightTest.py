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


def folder_size_bytes(folder: Path) -> int:
    """Return total size of all files under folder."""
    total = 0
    for p in folder.rglob("*"):
        if p.is_file():
            total += p.stat().st_size
    return total

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
    #setting lists to emptu
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

#main function of the script. Builds the previous.
def main() -> None:
    """
    Minimal flight loop:
    - create run dirs
    - init sensors
    - read sensors each loop
    - log JSONL
    - sleep to rate
    """
    telemetry_hz = 1.0
    base_dir = Path("/home/pi/hab")
    max_log_mb = 200
    max_image_mb = 500  # not used yet (images not added yet)

    dirs = create_run_directories(base_dir)
    logs_dir = dirs["logs_dir"]
    images_dir = dirs["images_dir"]

    telemetry_log_path = logs_dir / "telemetry.jsonl"

    # Hardware init
    i2c = init_i2c_bus()
    sensors = init_sensors(i2c)

    sequence_id = 0
    next_cleanup_time_s = time.time() + 60

    while True:
        loop_start_time_s = time.time()
        sequence_id += 1

        # The telemetry record is the "truth" for this iteration.
        record: Dict[str, Any] = {
            "timestamp_utc": utc_timestamp_iso(),
            "sequence_id": sequence_id,
        }

        sensor_data, sensor_errors = read_sensors(sensors)
        record.update(sensor_data)

        if sensor_errors:
            record["errors"] = sensor_errors

        # Write log (fail-safe)
        log_error = append_jsonl_safe(telemetry_log_path, record)
        if log_error:
            record.setdefault("errors", {})
            record["errors"]["jsonl_log"] = log_error

        # Optional: periodic cleanup (logs only for now)
        if time.time() >= next_cleanup_time_s:
            cleanup_folder_to_mb(logs_dir, max_log_mb)
            # images cleanup will matter once camera is added:
            # cleanup_folder_to_mb(images_dir, max_image_mb)
            next_cleanup_time_s = time.time() + 60

        # Console debug line (simple)
        error_keys = list(record.get("errors", {}).keys())
        print(f"seq={sequence_id} time={record['timestamp_utc']} errors={error_keys}")

        sleep_to_rate(loop_start_time_s, telemetry_hz)


if __name__ == "__main__":
    main()





