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





