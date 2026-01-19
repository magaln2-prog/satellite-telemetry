#!/usr/bin/env python3
"""
Ground receiver for flight telemetry.

Responsibilities:
- Receive raw LoRa packets
- Strip radio header
- Reassemble newline-delimited JSON
- Write:
    - latest.json (most recent packet)
    - logs/history.jsonl (append-only)
"""

import json
import time
from pathlib import Path
from datetime import datetime, timezone

import sx126x


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
    """Write JSON atomically to avoid partial reads."""
    tmp = path.with_suffix(".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f)
    tmp.replace(path)


# ============================================================
# Main RX logic
# ============================================================

def main() -> None:
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

    while True:
        # ----------------------------------------------------
        # Step 1: Receive raw packet
        # ----------------------------------------------------
        pkt = lora.recv_packet(timeout_s=RX_TIMEOUT_S)
        if not pkt:
            continue

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
            rx_utc = utc_now_iso()

            record["_rx_utc"] = rx_utc     # ← dashboard looks for this
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
            with open(HISTORY_PATH, "a", encoding="utf-8") as f:
                f.write(json.dumps(record, separators=(",", ":")) + "\n")

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
