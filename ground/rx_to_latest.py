#!/usr/bin/env python3
"""
Ground receiver for TM-framed flight telemetry.

This decodes the "TM framed fragments" sent by flight:
- Each LoRa/UART burst contains one or more TM frames
- TM frames include: msg_id, frag_idx, frag_tot, crc16
- Ground reassembles all fragments for a msg_id, then JSON-decodes once.

Writes:
- latest.json (atomic)
- logs/history.jsonl (append-only, only after successful decode)

Also writes NO CONTACT heartbeat when no *decoded* telemetry arrives.
"""

import json
import time
from pathlib import Path
from datetime import datetime, timezone

try:
    import sx126x  # your radio driver
    HAVE_RADIO = True
except Exception:
    sx126x = None
    HAVE_RADIO = False

from protocol_tm import try_parse_one

# ----------------------------
# RADIO CONFIG (must match flight)
# ----------------------------
LORA_PORT = "/dev/serial0"
LORA_FREQ_MHZ = 915
LORA_ADDR = 1
LORA_POWER_DBM = 22
LORA_BUFFER_SIZE = 240
LORA_CRYPT = 0
LORA_RSSI = True
LORA_AIR_SPEED = 2400

RX_TIMEOUT_S = 0.5

# Heartbeat behavior
NO_CONTACT_AFTER_S = 2.0
HEARTBEAT_EVERY_S = 1.0

# Reassembly behavior
REASSEMBLY_TTL_S = 3.0  # drop incomplete messages after this


# ----------------------------
# FILES
# ----------------------------
BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR / "logs"
LATEST_PATH = BASE_DIR / "latest.json"
HISTORY_PATH = LOG_DIR / "history.jsonl"
LOG_DIR.mkdir(parents=True, exist_ok=True)


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_write_json(path: Path, obj: dict):
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(obj, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(path)


def append_history(obj: dict):
    with open(HISTORY_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(obj, ensure_ascii=False) + "\n")


def ensure_latest():
    if not LATEST_PATH.exists() or LATEST_PATH.stat().st_size == 0:
        atomic_write_json(LATEST_PATH, {"status": "NO DATA YET", "_rx_utc": utc_now(), "_radio": None})


def demo_loop():
    print("sx126x not available -> DEMO mode.")
    seq = 0
    while True:
        seq += 1
        atomic_write_json(LATEST_PATH, {"status": "DEMO / NO RADIO", "seq": seq, "_rx_utc": utc_now(), "_radio": None})
        time.sleep(1)


def main():
    ensure_latest()

    if not HAVE_RADIO:
        demo_loop()
        return

    print("Starting LoRa RX (TM-framed)…")

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

    # Stream buffer: holds raw payload bytes extracted from radio frames
    stream = b""

    # Reassembly: msg_id -> {"t0": float, "tot": int, "parts": {idx: bytes}}
    pending = {}

    last_uart_packet_time = 0.0
    last_decoded_time = 0.0
    last_heartbeat = 0.0
    last_good_rx_utc = None

    print("RX running… waiting for TM frames.")

    while True:
        pkt = lora.recv_packet(timeout_s=RX_TIMEOUT_S)
        now = time.time()

        # Expire stale partial messages
        for mid in list(pending.keys()):
            if now - pending[mid]["t0"] > REASSEMBLY_TTL_S:
                del pending[mid]

        # No UART packet received
        if not pkt:
            # Heartbeat: only if we haven't decoded a full telemetry record recently
            if (now - last_decoded_time) >= NO_CONTACT_AFTER_S and (now - last_heartbeat) >= HEARTBEAT_EVERY_S:
                atomic_write_json(LATEST_PATH, {
                    "status": "NO CONTACT",
                    "_rx_utc": utc_now(),
                    "last_good_rx_utc": last_good_rx_utc,
                    "_radio": None,
                })
                last_heartbeat = now
            continue

        last_uart_packet_time = now

        # Normalize raw bytes
        raw = pkt
        if isinstance(raw, (tuple, list)) and raw:
            raw = raw[0]
        if not isinstance(raw, (bytes, bytearray)):
            raw = bytes(raw)

        # Extract payload bytes from driver if possible; otherwise fall back to raw.
        meta = {}
        payload = b""
        try:
            meta, payload = lora.parse_packet(raw)
            if payload is None:
                payload = b""
        except Exception:
            meta, payload = {}, b""

        # IMPORTANT: Some drivers/headers can be inconsistent.
        # We'll feed whichever contains the TM magic most reliably.
        # (Parser resyncs on MAGIC anyway.)
        chunk = payload if payload else raw
        if not chunk:
            continue

        # Append to stream and parse as many TM frames as possible
        stream += chunk

        while True:
            frame, rest = try_parse_one(stream)
            if frame is None:
                stream = rest
                break

            stream = rest
            msg_id = frame["msg_id"]
            frag_idx = frame["frag_idx"]
            frag_tot = frame["frag_tot"]
            frag_payload = frame["payload"]

            st = pending.get(msg_id)
            if st is None:
                st = {"t0": now, "tot": frag_tot, "parts": {}}
                pending[msg_id] = st

            # If total changes, reset that message id
            if st["tot"] != frag_tot:
                st = {"t0": now, "tot": frag_tot, "parts": {}}
                pending[msg_id] = st

            st["parts"][frag_idx] = frag_payload

            # Complete?
            if len(st["parts"]) == st["tot"]:
                full = b"".join(st["parts"][i] for i in range(st["tot"]))
                del pending[msg_id]

                try:
                    record = json.loads(full.decode("utf-8"))
                except Exception:
                    # Bad JSON shouldn't kill the loop
                    continue

                record["_rx_utc"] = utc_now()
                record["_radio"] = meta or None

                atomic_write_json(LATEST_PATH, record)
                append_history(record)

                last_good_rx_utc = record["_rx_utc"]
                last_decoded_time = now
                last_heartbeat = now

                print(
                    f"RX OK msg_id={msg_id} seq={record.get('seq')} "
                    f"rssi={meta.get('packet_rssi_dbm') if meta else None} "
                    f"frags={frag_tot}"
                )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nRX stopped.")
