#!/usr/bin/env python3
import json
import os
import time
from pathlib import Path
from datetime import datetime, timezone
import sx126x

# --- LoRa config ---
LORA_PORT = "/dev/serial0"
LORA_FREQ_MHZ = 915
LORA_ADDR = 65535          # broadcast receive
LORA_POWER_DBM = 22
LORA_AIR_SPEED = 2400
LORA_NET_ID = 0
LORA_CRYPT = 0
LORA_RSSI = False
LORA_BUFFER_SIZE = 240

# --- Files ---
BASE_DIR = Path(__file__).resolve().parent
OUT_FILE = BASE_DIR / "latest.json"
LOG_DIR = BASE_DIR / "logs"
LOG_DIR.mkdir(exist_ok=True)
HISTORY_FILE = LOG_DIR / "history.jsonl"
RAW_LOG_FILE = LOG_DIR / "rx_raw.log"

# --- Sensor mapping ---
SENSOR_MAP = {
    "t_c": ("Temperature", "C"),
    "p_hpa": ("Pressure", "hPa"),
    "rh": ("Humidity", "%"),
    "lux": ("Illuminance", "lux"),
    "uvi": ("UV_index", ""),
    "bv": ("Battery_V", "V"),
    "bp": ("Battery_percent", "%"),
    "ba": ("Battery_acc", "g"),
    "cpu_c": ("CPU_temp", "C"),
    "uv_now": ("UV_alert", ""),
    "thr_now": ("Thermal_alert", ""),
    "alerts": ("Alerts", ""),
    "img": ("Image_flag", "")
}

def now_iso():
    return datetime.now(timezone.utc).isoformat()

def atomic_write(path: Path, data: str):
    tmp = str(path) + ".tmp"
    with open(tmp, "w") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, str(path))

def format_payload(obj: dict):
    ts = obj.get("ts", now_iso()).split(".")[0]
    parts = [f"Timestamp={ts}"]
    for key, (name, unit) in SENSOR_MAP.items():
        if key in obj:
            val = obj[key]
            val_str = str(val) if isinstance(val, bool) else f"{val} {unit}".strip()
            parts.append(f"{name}={val_str}")
    return " | ".join(parts)

def main():
    print("=== RX -> latest.json ===")
    print(f"Port: {LORA_PORT} | Addr: {LORA_ADDR} | Freq: {LORA_FREQ_MHZ}")

    lora = sx126x.sx126x(
        serial_num=LORA_PORT,
        freq=LORA_FREQ_MHZ,
        addr=LORA_ADDR,
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

    # Reassembly buffer:
    # Serial/LoRa reads are NOT guaranteed to align with application messages.
    # Your flight code sends one JSON object with a trailing "\n".
    # So we buffer bytes until we see a newline, then parse that complete line.
    rx_buf = b""
    MAX_BUF_BYTES = 16 * 1024  # safety: prevent runaway growth if data gets corrupted

    while True:
        chunk = lora.receive()
        if not chunk:
            time.sleep(0.01)
            continue

        # Ensure bytes
        if isinstance(chunk, str):
            chunk = chunk.encode("utf-8", errors="replace")
        elif not isinstance(chunk, (bytes, bytearray)):
            # last resort
            chunk = repr(chunk).encode("utf-8", errors="replace")

        rx_buf += bytes(chunk)

        # Safety cap: if we never see a newline, drop the buffer and log it.
        if len(rx_buf) > MAX_BUF_BYTES:
            with open(RAW_LOG_FILE, "a", buffering=1) as f:
                f.write(f"{now_iso()} | RX buffer overflow (dropping {len(rx_buf)} bytes)\n")
            rx_buf = b""
            continue

        # Process complete lines (JSON messages)
        while b"\n" in rx_buf:
            line, rx_buf = rx_buf.split(b"\n", 1)
            line = line.strip()
            if not line:
                continue

            # Decode once; keep original bytes for logging
            text = line.decode("utf-8", errors="replace")
            try:
                obj = json.loads(text)
                obj["_rx_ts"] = now_iso()

                # Save latest.json
                atomic_write(OUT_FILE, json.dumps(obj, indent=2))

                # Append history
                with open(HISTORY_FILE, "a", buffering=1, encoding="utf-8") as hf:
                    hf.write(json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n")

                print(format_payload(obj), "\n\n")
            except Exception as e:
                # Log the bad line + a short preview of raw bytes
                with open(RAW_LOG_FILE, "a", buffering=1, encoding="utf-8") as f:
                    f.write(f"{now_iso()} | JSON parse error: {e} | {text}\n")
                print("JSON parse error:", e)

if __name__ == "__main__":
    main()
