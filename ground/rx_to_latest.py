#!/usr/bin/env python3
"""
rx_to_latest.py (GROUND) — production RX

What this script does:
- Reads LoRa bursts from sx126x UART module.
- Extracts TM-framed fragments (protocol_tm.py) and reassembles full JSON telemetry.
- Writes:
    latest.json  -> latest decoded telemetry record (or contact status object)
    logs/history.jsonl -> append-only history of decoded records

Why we *trust RAW when it starts with b"TM"*:
- The sx126x UART module sometimes returns bytes that already start with the TM frame.
- The driver parse_packet() assumes a 3-byte header and strips it, which would corrupt TM.
- So if raw starts with MAGIC (b"TM"), we must feed RAW to the TM parser.
"""

import json
import os
import sys
import time
import fcntl
from pathlib import Path
from datetime import datetime, timezone

try:
    import sx126x  # type: ignore
    HAVE_RADIO = True
except Exception:
    sx126x = None  # type: ignore
    HAVE_RADIO = False

from protocol_tm import try_parse_one, HDR_LEN, MAGIC, VER  # must match flight exactly


# ----------------------------
# Single-instance lock
# ----------------------------
# Prevent multiple RX processes from fighting over /dev/serial0 and latest.json
LOCK_PATH = Path("/tmp/rx_to_latest.lock")
_lock_fd = open(LOCK_PATH, "w")
try:
    fcntl.flock(_lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    print("rx_to_latest.py already running (lock held). Exiting.")
    sys.exit(0)


# ----------------------------
# RADIO CONFIG (match flight)
# ----------------------------
LORA_PORT = "/dev/serial0"
LORA_FREQ_MHZ = 915
LORA_ADDR = 1
LORA_POWER_DBM = 22
LORA_BUFFER_SIZE = 240
LORA_CRYPT = 0
LORA_RSSI = True
LORA_AIR_SPEED = 2400
LORA_NET_ID = 0
RX_TIMEOUT_S = 5.0

# ----------------------------
# Status + reassembly tuning
# ----------------------------
# "Contact" means: we have received ANY bytes recently (even if decode hasn't completed).
NO_CONTACT_AFTER_S = 30.0
HEARTBEAT_EVERY_S = 1.0

# Give fragments time to arrive out-of-order / with delays
REASSEMBLY_TTL_S = 60.0

RADIO_SETTLE_S = 2.0
SERIAL_ERROR_BACKOFF_S = 0.5

# How often to write a "CONTACT (RX BYTES...)" status while receiving bytes.
CONTACT_RAW_EVERY_S = 1.0


# ----------------------------
# Files
# ----------------------------
BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR / "logs"
LATEST_PATH = BASE_DIR / "latest.json"
HISTORY_PATH = LOG_DIR / "history.jsonl"
LOG_DIR.mkdir(parents=True, exist_ok=True)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_write_json(path: Path, obj: dict) -> None:
    """
    Atomic write (write temp file then rename) so the web server never reads a half-written JSON.
    """
    tmp = path.with_name(f"{path.name}.{os.getpid()}.tmp")
    data = json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def append_history(obj: dict) -> None:
    """
    Append decoded records for charts/history.
    """
    with open(HISTORY_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n")


def ensure_latest_initialized() -> None:
    """
    Ensure latest.json exists so the dashboard has something to load immediately.
    """
    if (not LATEST_PATH.exists()) or LATEST_PATH.stat().st_size == 0:
        atomic_write_json(LATEST_PATH, {"status": "NO DATA YET", "_rx_utc": utc_now(), "_radio": None})


def pending_summary(pending: dict) -> str:
    """
    Human-readable summary of in-progress reassembly state, e.g. "23:2/3 24:1/3".
    """
    items = []
    for mid, st in pending.items():
        items.append(f"{mid}:{len(st.get('parts', {}))}/{st.get('tot')}")
    return " ".join(items) if items else "(empty)"


def main() -> None:
    ensure_latest_initialized()

    if not HAVE_RADIO:
        print("sx126x not available -> DEMO mode (no radio).")
        while True:
            atomic_write_json(LATEST_PATH, {"status": "DEMO / NO RADIO", "_rx_utc": utc_now(), "_radio": None})
            time.sleep(1.0)

    print("Starting LoRa RX… (TM-framed + legacy newline fallback)")
    print(f"Waiting {RADIO_SETTLE_S:.1f}s for LoRa radio/UART to settle…")
    time.sleep(RADIO_SETTLE_S)

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

    # Buffers:
    # tm_stream: holds a rolling byte stream from which we parse TM frames
    # legacy_buf: supports older newline-delimited JSON (optional fallback)
    tm_stream = b""
    legacy_buf = b""

    # pending[msg_id] = {"t0": time_first_seen, "tot": frag_tot, "parts": {frag_idx: bytes}}
    pending: dict = {}

    last_decoded_time = 0.0
    last_rx_bytes_time = 0.0
    last_heartbeat = 0.0
    last_contact_write = 0.0
    last_good_rx_utc = None

    print("RX running… waiting for telemetry.")
    # print(f"Protocol: MAGIC={MAGIC} VER={VER} HDR_LEN={HDR_LEN}")

    while True:
        # 1) Read a burst from radio/UART
        try:
            pkt = lora.recv_packet(timeout_s=RX_TIMEOUT_S)
        except Exception as e:
            print("Serial RX error, retrying:", e)
            time.sleep(SERIAL_ERROR_BACKOFF_S)
            continue

        now = time.time()

        # 2) Drop stale partial messages (never completed)
        for mid in list(pending.keys()):
            age = now - pending[mid].get("t0", now)
            if age > REASSEMBLY_TTL_S:
                del pending[mid]

        # 3) If no bytes arrived this cycle, write heartbeat status
        if pkt is None:
            if (now - last_heartbeat) >= HEARTBEAT_EVERY_S:
                since_rx = now - last_rx_bytes_time
                status = "CONTACT (RX BYTES, decode pending)" if since_rx < NO_CONTACT_AFTER_S else "NO CONTACT"

                atomic_write_json(
                    LATEST_PATH,
                    {
                        "status": status,
                        "_rx_utc": utc_now(),
                        "last_good_rx_utc": last_good_rx_utc,
                        "pending": pending_summary(pending),
                        "tm_stream_len": len(tm_stream),
                        "_radio": None,
                    },
                )
                last_heartbeat = now
            continue

        # Normalize pkt to raw bytes
        raw = pkt[0] if isinstance(pkt, (tuple, list)) and pkt else pkt
        raw = bytes(raw)

        last_rx_bytes_time = now
        last_good_rx_utc = utc_now()

        # 4) Periodically write a contact-only update so the dashboard shows "contact"
        if (now - last_contact_write) >= CONTACT_RAW_EVERY_S:
            atomic_write_json(
                LATEST_PATH,
                {
                    "status": "CONTACT (RX BYTES, decode pending)",
                    "_rx_utc": utc_now(),
                    "last_good_rx_utc": last_good_rx_utc,
                    "rx_len": len(raw),
                    "rx_hex": raw[:16].hex(),
                    "pending": pending_summary(pending),
                    "tm_stream_len": len(tm_stream),
                    "_radio": None,
                },
            )
            last_contact_write = now
            last_heartbeat = now

        # 5) Extract payload/meta if available, BUT do not corrupt TM frames:
        # If raw already begins with MAGIC, feed raw directly to the TM parser.
        meta, payload = {}, b""
        try:
            meta, payload = lora.parse_packet(raw)
            payload = payload or b""
        except Exception:
            payload = b""
            meta = {}

        if raw.startswith(MAGIC):
            chunk = raw
        elif payload.startswith(MAGIC):
            chunk = payload
        else:
            chunk = payload if payload else raw

        if not chunk:
            continue

        # Feed TM + legacy buffers
        tm_stream += chunk
        legacy_buf += chunk

        # 6) TM-framed decode loop
        while True:
            frame, rest = try_parse_one(tm_stream)
            if frame is None:
                tm_stream = rest
                break

            tm_stream = rest

            msg_id = frame["msg_id"]
            frag_idx = frame["frag_idx"]
            frag_tot = frame["frag_tot"]
            frag_payload = frame["payload"]

            st = pending.get(msg_id)
            if st is None or st.get("tot") != frag_tot:
                st = {"t0": now, "tot": frag_tot, "parts": {}}
                pending[msg_id] = st

            st["parts"][frag_idx] = frag_payload

            # Completed message: assemble JSON and write it out
            if len(st["parts"]) == st["tot"]:
                full = b"".join(st["parts"][i] for i in range(st["tot"]))
                del pending[msg_id]

                try:
                    record = json.loads(full.decode("utf-8"))
                except Exception:
                    # If a decoded frame isn't valid JSON, just ignore it.
                    continue

                record["_rx_utc"] = utc_now()
                record["_radio"] = meta or None

                atomic_write_json(LATEST_PATH, record)
                append_history(record)

                last_decoded_time = now
                last_heartbeat = now

                seq = record.get("seq")
                rssi = meta.get("packet_rssi_dbm") if meta else None
                print(f"RX OK (TM) msg_id={msg_id} seq={seq} frags={frag_tot} rssi={rssi}")

        # 7) Legacy newline-delimited JSON fallback (optional)
        while b"\n" in legacy_buf:
            line, legacy_buf = legacy_buf.split(b"\n", 1)
            if not line.strip():
                continue
            try:
                record = json.loads(line.decode("utf-8"))
            except Exception:
                continue

            record["_rx_utc"] = utc_now()
            record["_radio"] = meta or None
            record.setdefault("status", "LEGACY")

            atomic_write_json(LATEST_PATH, record)
            append_history(record)

            last_decoded_time = now
            last_heartbeat = now

            print(f"RX OK (LEGACY) seq={record.get('seq')} rssi={meta.get('packet_rssi_dbm') if meta else None}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nRX stopped.")

