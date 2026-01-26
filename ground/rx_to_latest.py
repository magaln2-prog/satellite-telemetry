#!/usr/bin/env python3
"""
rx_to_latest.py  (GROUND)

Hardened ground-side receiver.

Fixes:
1) latest.json.tmp collisions when multiple rx_to_latest.py processes run:
   - single-instance lockfile
   - PID-specific temp files for atomic writes

2) SerialException "device reports readiness to read but returned no data":
   - wait a bit at startup for the radio/UART to settle
   - catch/retry around recv_packet() so RX doesn't crash on transient UART issues
"""

import json
import os
import sys
import time
from pathlib import Path
from datetime import datetime, timezone

# ----------------------------
# Single-instance lock (Linux)
# ----------------------------
import fcntl  # Linux only; OK for your Debian Pi

LOCK_PATH = Path("/tmp/rx_to_latest.lock")
_lock_fd = open(LOCK_PATH, "w")
try:
    fcntl.flock(_lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    print("rx_to_latest.py already running (lock held). Exiting.")
    sys.exit(0)

# ----------------------------
# Optional radio driver import
# ----------------------------
try:
    import sx126x  # type: ignore
    HAVE_RADIO = True
except Exception:
    sx126x = None  # type: ignore
    HAVE_RADIO = False

from protocol_tm import try_parse_one  # must match flight's protocol_tm.py

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
LORA_NET_ID = 0
RX_TIMEOUT_S = 0.5

# Heartbeat behavior
NO_CONTACT_AFTER_S = 2.0
HEARTBEAT_EVERY_S = 1.0

# Reassembly behavior
REASSEMBLY_TTL_S = 5.0  # drop incomplete messages after this

# Serial hardening
RADIO_SETTLE_S = 2.0
SERIAL_ERROR_BACKOFF_S = 0.5

# ----------------------------
# FILES (force lowercase)
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
    Collision-proof atomic JSON write.

    - temp file includes PID so two processes can't clobber each other:
      latest.json.<pid>.tmp
    - if replace fails due to rare tmp disappearance, fall back to direct write
      instead of crashing
    """
    path.parent.mkdir(parents=True, exist_ok=True)

    tmp = path.with_name(f"{path.name}.{os.getpid()}.tmp")
    data = json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n"

    with open(tmp, "w", encoding="utf-8") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())

    try:
        os.replace(tmp, path)
    except FileNotFoundError:
        # Extremely rare after PID-temp, but avoids crashing if tmp vanished.
        with open(path, "w", encoding="utf-8") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())


def append_history(obj: dict) -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    with open(HISTORY_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n")


def ensure_latest_initialized() -> None:
    if (not LATEST_PATH.exists()) or LATEST_PATH.stat().st_size == 0:
        atomic_write_json(LATEST_PATH, {"status": "NO DATA YET", "_rx_utc": utc_now(), "_radio": None})


def demo_loop() -> None:
    print("sx126x not available -> DEMO mode (no radio).")
    ensure_latest_initialized()

    seq = 0
    while True:
        seq += 1
        atomic_write_json(
            LATEST_PATH,
            {"status": "DEMO / NO RADIO", "seq": seq, "_rx_utc": utc_now(), "_radio": None},
        )
        time.sleep(1.0)


def main() -> None:
    ensure_latest_initialized()

    if not HAVE_RADIO:
        demo_loop()
        return

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


    tm_stream = b""
    legacy_buf = b""
    pending = {}

    last_decoded_time = 0.0
    last_heartbeat = 0.0
    last_good_rx_utc = None

    print("RX running… waiting for telemetry.")

    while True:
        # ---- HARDENED: don't crash on transient serial issues ----
        try:
            pkt = lora.recv_packet(timeout_s=RX_TIMEOUT_S)
        except Exception as e:
            print("Serial RX error, retrying:", e)
            time.sleep(SERIAL_ERROR_BACKOFF_S)
            continue

        now = time.time()

        # Expire stale partial messages
        for mid in list(pending.keys()):
            if now - pending[mid]["t0"] > REASSEMBLY_TTL_S:
                del pending[mid]

        # No packet -> heartbeat if needed
        if not pkt:
            if (now - last_decoded_time) >= NO_CONTACT_AFTER_S and (now - last_heartbeat) >= HEARTBEAT_EVERY_S:
                atomic_write_json(
                    LATEST_PATH,
                    {"status": "NO CONTACT", "_rx_utc": utc_now(), "last_good_rx_utc": last_good_rx_utc, "_radio": None},
                )
                last_heartbeat = now
            continue

        # Normalize raw bytes
        raw = pkt
        if isinstance(raw, (tuple, list)) and raw:
            raw = raw[0]
        if not isinstance(raw, (bytes, bytearray)):
            raw = bytes(raw)

        # Extract payload via driver if possible
        meta = {}
        payload = b""
        try:
            meta, payload = lora.parse_packet(raw)
            if payload is None:
                payload = b""
        except Exception:
            meta, payload = {}, b""

        chunk = payload if payload else raw
        if not chunk:
            continue

        tm_stream += chunk
        legacy_buf += chunk

        # 1) TM-framed decoding
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

            if len(st["parts"]) == st["tot"]:
                full = b"".join(st["parts"][i] for i in range(st["tot"]))
                del pending[msg_id]

                try:
                    record = json.loads(full.decode("utf-8"))
                except Exception:
                    continue

                record["_rx_utc"] = utc_now()
                record["_radio"] = meta or None

                atomic_write_json(LATEST_PATH, record)
                append_history(record)

                last_good_rx_utc = record["_rx_utc"]
                last_decoded_time = now
                last_heartbeat = now

                print(
                    f"RX OK (TM) msg_id={msg_id} seq={record.get('seq')} frags={frag_tot} "
                    f"rssi={meta.get('packet_rssi_dbm') if meta else None}"
                )

        # 2) Legacy newline JSON fallback
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

            last_good_rx_utc = record["_rx_utc"]
            last_decoded_time = now
            last_heartbeat = now

            print(
                f"RX OK (LEGACY) seq={record.get('seq')} "
                f"rssi={meta.get('packet_rssi_dbm') if meta else None}"
            )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nRX stopped.")

