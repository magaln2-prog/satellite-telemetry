#!/usr/bin/env python3
"""
Ground receiver for flight telemetry.

Responsibilities:
- Receive raw LoRa packets
- Strip radio header
- Reassemble newline-delimited JSON
- Write:
    - latest.json (most recent record)
    - logs/history.jsonl (append-only)

Key reliability features:
- Atomic writes to latest.json (never partial/empty reads)
- Initializes latest.json with a valid JSON object at startup
- Emits a NO CONTACT heartbeat if no packets arrive (keeps UI alive)
- Optional demo mode if sx126x isn't available
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

RX_TIMEOUT_S = 0.5

# "No contact" heartbeat behavior (helps the UI update even with no packets)
NO_CONTACT_AFTER_S = 2.0      # after this many seconds w/out packets -> NO CONTACT
HEARTBEAT_EVERY_S = 1.0       # how often to refresh latest.json in NO CONTACT state


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
    """Return current UTC time as ISO string."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_write_json(path: Path, obj: dict) -> None:
    tmp = path.parent / (path.name + ".tmp")
    tmp.write_text(json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(path)



def append_history(obj: dict) -> None:
    """Append one JSON line to history."""
    with open(HISTORY_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n")


def ensure_latest_initialized() -> None:
    """Make sure latest.json exists and contains valid JSON."""
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
    print("sx126x not available -> running DEMO mode (no radio).")
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

    # Initialize radio
    lora = sx126x.sx126x(
        serial_num=LORA_PORT,
        freq=LORA_FREQ_MHZ,
        addr=LORA_ADDR,
        power=LORA_POWER_DBM,
        rssi=LORA_RSSI,
        buffer_size=LORA_BUFFER_SIZE,
        crypt=LORA_CRYPT,
        relay=False,
        lbt=False,
        wor=False,
    )

    # RX buffer for newline-based reassembly
    rx_buf = b""

    print("RX started. Waiting for packets…")

    last_packet_time = 0.0
    last_heartbeat_write = 0.0
    last_good_record = None  # type: ignore

    while True:
        # ----------------------------------------------------
        # Step 1: Receive raw packet
        # ----------------------------------------------------
        pkt = lora.recv_packet(timeout_s=RX_TIMEOUT_S)

        now = time.time()

        # ----------------------------------------------------
        # Step 1b: If no packet, maybe write NO CONTACT heartbeat
        # ----------------------------------------------------
        if not pkt:
            # If we've never received anything OR haven't received in a while,
            # refresh latest.json so UI doesn't look "stuck"
            time_since_pkt = (now - last_packet_time) if last_packet_time > 0 else 9999.0
            if time_since_pkt >= NO_CONTACT_AFTER_S and (now - last_heartbeat_write) >= HEARTBEAT_EVERY_S:
                heartbeat = {
                    "status": "NO CONTACT",
                    "_rx_utc": utc_now_iso(),
                    "last_good_rx_utc": (last_good_record or {}).get("_rx_utc"),
                    "_radio": None,
                }
                atomic_write_json(LATEST_PATH, heartbeat)
                last_heartbeat_write = now
            continue

        last_packet_time = now

        # ----------------------------------------------------
        # Step 2: Parse radio frame
        # ----------------------------------------------------
        meta, payload = lora.parse_packet(pkt)
        if not payload:
            continue

        # ----------------------------------------------------
        # Step 3: Append payload to RX buffer
        # ----------------------------------------------------
        rx_buf += payload

        # ----------------------------------------------------
        # Step 4: Extract complete JSON lines
        # ----------------------------------------------------
        while b"\n" in rx_buf:
            line, rx_buf = rx_buf.split(b"\n", 1)

            if not line.strip():
                continue

            try:
                record = json.loads(line.decode("utf-8"))
            except Exception:
                # Corrupt JSON — drop line, continue
                continue

            # ------------------------------------------------
            # Step 5: Annotate with RX metadata
            # ------------------------------------------------
            record["_rx_utc"] = utc_now_iso()
            record["_radio"] = {
                "packet_rssi_dbm": meta.get("packet_rssi_dbm"),
                "src_addr": meta.get("src_addr"),
            }

            # ------------------------------------------------
            # Step 6: Write latest.json (atomic)
            # ------------------------------------------------
            atomic_write_json(LATEST_PATH, record)

            # ------------------------------------------------
            # Step 7: Append to history.jsonl
            # ------------------------------------------------
            append_history(record)

            last_good_record = record
            last_heartbeat_write = now  # reset heartbeat timer since we got real data

            # ------------------------------------------------
            # Step 8: Console feedback
            # ------------------------------------------------
            seq = record.get("seq")
            ts = record.get("ts")
            print(
                f"RX OK seq={seq} ts={ts} "
                f"rssi={meta.get('packet_rssi_dbm')}dBm"
            )


# ============================================================
# Entry point
# ============================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nRX stopped.")
