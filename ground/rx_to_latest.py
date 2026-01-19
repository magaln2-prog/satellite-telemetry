#!/usr/bin/env python3
"""
Ground receiver for flight telemetry.

Responsibilities:
- Receive raw LoRa UART bursts via sx126x.recv_packet()
- Strip radio header + optional RSSI via sx126x.parse_packet()
- Reassemble newline-delimited JSON records
- Write:
    - latest.json (most recent record)  [atomic]
    - logs/history.jsonl (append-only)

Reliability:
- Atomic writes to latest.json (never partial/empty reads)
- Initializes latest.json with a valid JSON object at startup
- Emits NO CONTACT heartbeat if no packets arrive (keeps UI alive)
- Demo mode if sx126x isn't available
"""

import json
import time
from pathlib import Path
from datetime import datetime, timezone

# ------------------------------------------------------------
# Try to import radio driver; allow demo mode if unavailable
# ------------------------------------------------------------
try:
    import sx126x  # type: ignore
    HAVE_RADIO = True
except Exception:
    sx126x = None  # type: ignore
    HAVE_RADIO = False


# ============================================================
# Configuration (MATCHES FLIGHT SETTINGS)
# ============================================================
LORA_PORT = "/dev/serial0"
LORA_FREQ_MHZ = 915
LORA_ADDR = 1
LORA_POWER_DBM = 22
LORA_BUFFER_SIZE = 240
LORA_CRYPT = 0
LORA_RSSI = True
LORA_AIR_SPEED = 2400

RX_TIMEOUT_S = 0.5

# Heartbeat (keeps UI alive even without packets)
NO_CONTACT_AFTER_S = 2.0
HEARTBEAT_EVERY_S = 1.0


# ============================================================
# File locations (LOCAL TO THIS SCRIPT)
# ============================================================
BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR / "logs"

LATEST_PATH = BASE_DIR / "latest.json"
HISTORY_PATH = LOG_DIR / "history.jsonl"

LOG_DIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# Utility helpers
# ============================================================
def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_write_json(path: Path, obj: dict) -> None:
    """
    Write JSON atomically to avoid partial/empty reads.
    IMPORTANT: never open(path, 'w') directly for latest.json.
    """
    tmp = path.parent / (path.name + ".tmp")
    tmp.write_text(
        json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n",
        encoding="utf-8"
    )
    tmp.replace(path)


def append_history(obj: dict) -> None:
    with open(HISTORY_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n")


def ensure_latest_initialized() -> None:
    if (not LATEST_PATH.exists()) or LATEST_PATH.stat().st_size == 0:
        init_obj = {
            "status": "NO DATA YET",
            "_rx_utc": utc_now_iso(),
            "_radio": None,
        }
        atomic_write_json(LATEST_PATH, init_obj)


# ============================================================
# Demo mode (no sx126x / no hardware)
# ============================================================
def demo_loop() -> None:
    print("sx126x not available -> DEMO mode (no radio).")
    print("Writing heartbeat updates to latest.json so the UI can update.\n")

    ensure_latest_initialized()
    last_write = 0.0
    seq = 0

    while True:
        now = time.time()
        if now - last_write >= HEARTBEAT_EVERY_S:
            seq += 1
            demo = {
                "status": "DEMO / NO RADIO",
                "seq": seq,
                "_rx_utc": utc_now_iso(),
                "_radio": None,
            }
            atomic_write_json(LATEST_PATH, demo)
            last_write = now
            print(f"DEMO heartbeat seq={seq}")

        time.sleep(0.05)


# ============================================================
# Main RX logic
# ============================================================
def main() -> None:
    ensure_latest_initialized()

    if not HAVE_RADIO:
        demo_loop()
        return

    print("Starting LoRa RX…")

    # Create driver instance (matches sx126x.py __init__)
    lora = sx126x.sx126x(
        serial_num=LORA_PORT,
        freq=LORA_FREQ_MHZ,
        addr=LORA_ADDR,
        power=LORA_POWER_DBM,
        rssi=LORA_RSSI,
        air_speed=LORA_AIR_SPEED,
        buffer_size=LORA_BUFFER_SIZE,
        crypt=LORA_CRYPT,
        relay=False,
        lbt=False,
        wor=False,
    )

    # Buffer for newline-delimited JSON reassembly
    rx_buf = b""

    print("RX started. Waiting for packets…")

    last_packet_time = 0.0
    last_heartbeat_write = 0.0
    last_good_rx_utc = None

    bad_json = 0

    while True:
        # 1) Read a burst of bytes from UART (may be partial)
        pkt = lora.recv_packet(timeout_s=RX_TIMEOUT_S)
        now = time.time()

        # 1b) Heartbeat if no packet
        if not pkt:
            time_since_pkt = (now - last_packet_time) if last_packet_time > 0 else 9999.0
            if time_since_pkt >= NO_CONTACT_AFTER_S and (now - last_heartbeat_write) >= HEARTBEAT_EVERY_S:
                heartbeat = {
                    "status": "NO CONTACT",
                    "_rx_utc": utc_now_iso(),
                    "last_good_rx_utc": last_good_rx_utc,
                    "_radio": None,
                }
                atomic_write_json(LATEST_PATH, heartbeat)
                last_heartbeat_write = now
            continue

        last_packet_time = now

        # 2) Strip header / RSSI, get payload bytes
        meta, payload = lora.parse_packet(pkt)
        if not payload:
            continue

        # 3) Append payload to reassembly buffer
        rx_buf += payload

        # 4) Extract complete JSON lines
        while b"\n" in rx_buf:
            line, rx_buf = rx_buf.split(b"\n", 1)

            if not line.strip():
                continue

            try:
                record = json.loads(line.decode("utf-8"))
            except Exception:
                bad_json += 1
                if bad_json % 50 == 0:
                    print(f"warning: dropped {bad_json} bad JSON lines")
                continue

            # 5) Annotate record with RX info
            rx_utc = utc_now_iso()
            record["_rx_utc"] = rx_utc
            record["_radio"] = {
                "src_addr": meta.get("src_addr"),
                "freq_mhz": meta.get("freq_mhz"),
                "raw_len": meta.get("raw_len"),
                "packet_rssi_dbm": meta.get("packet_rssi_dbm"),
            }

            # 6) Write latest.json (atomic) + append history
            atomic_write_json(LATEST_PATH, record)
            append_history(record)

            last_good_rx_utc = rx_utc
            last_heartbeat_write = now  # reset heartbeat timer since we got real data

            # 7) Console feedback
            print(
                f"RX OK seq={record.get('seq')} ts={record.get('ts')} "
                f"rssi={meta.get('packet_rssi_dbm')}dBm src={meta.get('src_addr')}"
            )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nRX stopped.")
