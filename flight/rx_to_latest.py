#!/usr/bin/env python3
"""
rx_to_latest.py

GROUND RECEIVER SCRIPT

Responsibilities:
- Receive LoRa packets via sx126x driver
- Extract application payload bytes
- Reassemble newline-delimited JSON messages
- Write:
    - latest.json  (for dashboard)
    - history.jsonl (append-only log)

This script MUST be robust to:
- partial packets
- packet concatenation
- dropped packets
"""

import json
import time
from pathlib import Path
from datetime import datetime, timezone

import sx126x

# =========================
# Configuration
# =========================

DATA_ROOT = Path("/home/pi/ground")
LOG_DIR = DATA_ROOT / "logs"

LATEST_PATH = DATA_ROOT / "latest.json"
HISTORY_PATH = LOG_DIR / "history.jsonl"

LOG_DIR.mkdir(parents=True, exist_ok=True)

# RX safety limits
MAX_RX_BUFFER = 8192   # prevent runaway memory if stream corrupts

# =========================
# Utility helpers
# =========================

def utc_now_iso() -> str:
    """UTC timestamp for receive-side metadata."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_write_json(path: Path, obj: dict) -> None:
    """
    Write JSON atomically:
    - write to temp file
    - rename over target

    Prevents dashboard from ever seeing partial JSON.
    """
    tmp = path.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(obj, f, separators=(",", ":"), ensure_ascii=False)
    tmp.replace(path)


def append_jsonl(path: Path, obj: dict) -> None:
    """Append one JSON object as a JSONL line."""
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n")


# =========================
# Main receiver
# =========================

def main() -> None:
    """
    Main RX loop.

    Key idea:
    - LoRa delivers *bytes*, not messages
    - We buffer bytes until we see '\\n'
    - Each '\\n' marks one complete JSON object
    """

    # Initialize LoRa radio
    lora = sx126x.sx126x(
        serial_num="/dev/serial0",
        freq=915,
        addr=1,
        power=22,
        rssi=True,
        buffer_size=240,
        crypt=0,
        relay=False,
        lbt=False,
        wor=False,
    )

    # RX reassembly buffer (bytes)
    rx_buf = b""

    print("RX started, waiting for packets...")

    while True:
        # --------------------------------------------------
        # Step 1: Receive a raw radio packet
        # --------------------------------------------------
        pkt = lora.recv_packet(timeout_s=0.5)
        if not pkt:
            continue

        # --------------------------------------------------
        # Step 2: Parse radio packet
        #   meta   → RSSI, src addr, etc.
        #   payload → application bytes (may be partial JSON)
        # --------------------------------------------------
        meta, payload = lora.parse_packet(pkt)

        if not payload:
            continue

        # --------------------------------------------------
        # Step 3: Append payload to reassembly buffer
        # --------------------------------------------------
        rx_buf += payload

        # Safety: drop buffer if stream goes insane
        if len(rx_buf) > MAX_RX_BUFFER:
            rx_buf = b""
            continue

        # --------------------------------------------------
        # Step 4: Extract complete JSON lines
        # --------------------------------------------------
        while b"\n" in rx_buf:
            line, rx_buf = rx_buf.split(b"\n", 1)
            line = line.strip()

            if not line:
                continue

            # --------------------------------------------------
            # Step 5: Parse JSON (finally safe)
            # --------------------------------------------------
            try:
                msg = json.loads(line.decode("utf-8"))
            except json.JSONDecodeError:
                # Corrupted line → drop it
                continue

            # --------------------------------------------------
            # Step 6: Add RX-side metadata
            # --------------------------------------------------
            msg["_rx_utc"] = utc_now_iso()

            if meta:
                msg["_radio"] = meta

            # --------------------------------------------------
            # Step 7: Write outputs
            # --------------------------------------------------
            atomic_write_json(LATEST_PATH, msg)
            append_jsonl(HISTORY_PATH, msg)

            # Optional console feedback
            seq = msg.get("seq")
            print(f"RX OK seq={seq}")

        # loop continues forever


if __name__ == "__main__":
    main()
