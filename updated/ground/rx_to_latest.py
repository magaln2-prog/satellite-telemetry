#!/usr/bin/env python3
"""
rx_to_latest_DEBUG.py (GROUND) — TM framing debug (protocol_tm.py)

Key fixes:
- If RAW already starts with b"TM", we trust RAW (do NOT use parse_packet payload).
  This prevents losing bytes when parse_packet strips incorrectly.
- Correct TM header probe:
    MAGIC(2) VER(1) MSG_ID(u16 BE) FRAG_IDX(u8) FRAG_TOT(u8) PAYLEN(u8) CRC(u16)
  need = HDR_LEN + PAYLEN

NEW changes (per latest debugging):
- NO_CONTACT_AFTER_S increased to 30s
- REASSEMBLY_TTL_S increased to 60s
- Heartbeat status is now based on last_rx_bytes (RF contact), not last_decoded (full JSON).
  If we have recent bytes, we report CONTACT even if decode hasn't completed.
"""

import json
import os
import sys
import time
import fcntl
import struct
from pathlib import Path
from datetime import datetime, timezone

try:
    import sx126x  # type: ignore
    HAVE_RADIO = True
except Exception:
    sx126x = None  # type: ignore
    HAVE_RADIO = False

from protocol_tm import try_parse_one, HDR_LEN, MAGIC, VER  # must match flight


# ----------------------------
# Single-instance lock
# ----------------------------
LOCK_PATH = Path("/tmp/rx_to_latest.lock")
_lock_fd = open(LOCK_PATH, "w")
try:
    fcntl.flock(_lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    print("rx_to_latest_DEBUG.py already running (lock held). Exiting.")
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
# Timeouts (UPDATED)
# ----------------------------
# Consider "NO CONTACT" only if we have not received ANY bytes recently.
NO_CONTACT_AFTER_S = 30.0
HEARTBEAT_EVERY_S = 1.0

# Allow plenty of time for out-of-order / delayed fragments to complete.
REASSEMBLY_TTL_S = 60.0

RADIO_SETTLE_S = 2.0
SERIAL_ERROR_BACKOFF_S = 0.5
CONTACT_RAW_EVERY_S = 1.0

# Files
BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR / "logs"
LATEST_PATH = BASE_DIR / "latest.json"
HISTORY_PATH = LOG_DIR / "history.jsonl"
LOG_DIR.mkdir(parents=True, exist_ok=True)

DEBUG = os.environ.get("RX_DEBUG", "0").lower() not in ("0", "", "false", "no", "off")


def dbg(*args: object) -> None:
    if DEBUG:
        print("[DBG]", *args, file=sys.stderr, flush=True)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_write_json(path: Path, obj: dict) -> None:
    tmp = path.with_name(f"{path.name}.{os.getpid()}.tmp")
    data = json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def append_history(obj: dict) -> None:
    with open(HISTORY_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n")


def ensure_latest_initialized() -> None:
    if (not LATEST_PATH.exists()) or LATEST_PATH.stat().st_size == 0:
        atomic_write_json(LATEST_PATH, {"status": "NO DATA YET", "_rx_utc": utc_now(), "_radio": None})


def _hex_preview(b: bytes, n: int = 24) -> str:
    h = b[:n].hex()
    return h + ("…" if len(b) > n else "")


def _pending_summary(pending: dict) -> str:
    items = []
    for mid, st in pending.items():
        items.append(f"{mid}:{len(st.get('parts', {}))}/{st.get('tot')}")
    return " ".join(items) if items else "(empty)"


def tm_header_probe(buf: bytes) -> None:
    if len(buf) < 8:
        return
    if buf[:2] != MAGIC:
        return

    ver = buf[2]
    msg_id = (buf[3] << 8) | buf[4]
    frag_idx = buf[5]
    frag_tot = buf[6]
    paylen = buf[7]
    need = HDR_LEN + paylen

    dbg(
        "TM PROBE",
        f"ver={ver} msg_id={msg_id} frag={frag_idx+1}/{frag_tot} paylen={paylen} need={need} buflen={len(buf)}",
        "head_hex=", _hex_preview(buf, 12),
    )


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

    tm_stream = b""
    legacy_buf = b""
    pending: dict = {}

    last_decoded_time = 0.0
    last_rx_bytes_time = 0.0
    last_heartbeat = 0.0
    last_contact_write = 0.0
    last_good_rx_utc = None

    print("RX running… waiting for telemetry.")
    dbg("CONFIG", "HDR_LEN=", HDR_LEN, "MAGIC=", MAGIC, "VER=", VER)

    while True:
        try:
            pkt = lora.recv_packet(timeout_s=RX_TIMEOUT_S)
        except Exception as e:
            print("Serial RX error, retrying:", e)
            time.sleep(SERIAL_ERROR_BACKOFF_S)
            continue

        now = time.time()

        # Expire stale partial messages
        for mid in list(pending.keys()):
            age = now - pending[mid].get("t0", now)
            if age > REASSEMBLY_TTL_S:
                dbg("REASSEMBLY TTL DROP", "msg_id=", mid, "age_s=", round(age, 3),
                    "state=", f"{len(pending[mid].get('parts', {}))}/{pending[mid].get('tot')}")
                del pending[mid]

        # ------------------------------------------------------------
        # Heartbeat / status when no packet arrives this cycle
        # UPDATED: base contact on last_rx_bytes_time (RF bytes), not last_decoded_time.
        # ------------------------------------------------------------
        if pkt is None:
            if (now - last_heartbeat) >= HEARTBEAT_EVERY_S:
                since_rx = now - last_rx_bytes_time
                status = "CONTACT (RX BYTES, decode pending)" if since_rx < NO_CONTACT_AFTER_S else "NO CONTACT"

                dbg("HEARTBEAT",
                    "status=", status,
                    "since_last_rx_bytes_s=", round(since_rx, 3),
                    "since_last_decoded_s=", round(now - last_decoded_time, 3),
                    "pending=", _pending_summary(pending),
                    "tm_stream_len=", len(tm_stream))

                atomic_write_json(
                    LATEST_PATH,
                    {
                        "status": status,
                        "_rx_utc": utc_now(),
                        "last_good_rx_utc": last_good_rx_utc,
                        "pending": _pending_summary(pending),
                        "tm_stream_len": len(tm_stream),
                        "_radio": None,
                    },
                )
                last_heartbeat = now
            continue

        raw = pkt[0] if isinstance(pkt, (tuple, list)) and pkt else pkt
        raw = bytes(raw)

        last_rx_bytes_time = now
        last_good_rx_utc = utc_now()

        dbg("RX BYTES", "len=", len(raw), "raw_head_ascii=", raw[:8], "raw_head_hex=", raw[:8].hex())

        # Dashboard "raw contact" update
        if (now - last_contact_write) >= CONTACT_RAW_EVERY_S:
            atomic_write_json(
                LATEST_PATH,
                {
                    "status": "CONTACT (RX BYTES, decode pending)",
                    "_rx_utc": utc_now(),
                    "last_good_rx_utc": last_good_rx_utc,
                    "rx_len": len(raw),
                    "rx_hex": raw[:16].hex(),
                    "pending": _pending_summary(pending),
                    "tm_stream_len": len(tm_stream),
                    "_radio": None,
                },
            )
            last_contact_write = now
            last_heartbeat = now

        # Attempt parse_packet, but DO NOT trust it if RAW already begins with TM
        meta, payload = {}, b""
        try:
            meta, payload = lora.parse_packet(raw)
            payload = payload or b""
        except Exception as e:
            dbg("parse_packet FAIL", "err=", repr(e))
            payload = b""

        if raw.startswith(MAGIC):
            chunk = raw
            dbg("CHUNK SOURCE", "RAW (starts with TM)")
        elif payload.startswith(MAGIC):
            chunk = payload
            dbg("CHUNK SOURCE", "PAYLOAD (starts with TM)")
        else:
            chunk = payload if payload else raw
            dbg("CHUNK SOURCE", "FALLBACK", "len=", len(chunk), "head_hex=", chunk[:4].hex())

        dbg("CHUNK", "len=", len(chunk), "head_hex=", chunk[:12].hex())

        if not chunk:
            continue

        tm_header_probe(chunk)

        tm_stream += chunk
        legacy_buf += chunk

        # TM-framed decode
        parsed_any_tm = False
        while True:
            frame, rest = try_parse_one(tm_stream)
            if frame is None:
                tm_stream = rest
                break

            parsed_any_tm = True
            tm_stream = rest

            msg_id = frame["msg_id"]
            frag_idx = frame["frag_idx"]
            frag_tot = frame["frag_tot"]
            frag_payload = frame["payload"]

            dbg("TM FRAME", "msg_id=", msg_id, "frag=", f"{frag_idx+1}/{frag_tot}", "payload_len=", len(frag_payload))

            st = pending.get(msg_id)
            if st is None or st.get("tot") != frag_tot:
                st = {"t0": now, "tot": frag_tot, "parts": {}}
                pending[msg_id] = st

            st["parts"][frag_idx] = frag_payload
            dbg("TM REASSEMBLY", "msg_id=", msg_id, "have=", f"{len(st['parts'])}/{st['tot']}", "pending=", _pending_summary(pending))

            if len(st["parts"]) == st["tot"]:
                full = b"".join(st["parts"][i] for i in range(st["tot"]))
                del pending[msg_id]

                try:
                    record = json.loads(full.decode("utf-8"))
                except Exception as e:
                    dbg("TM JSON decode FAIL", "err=", repr(e), "full_len=", len(full), "full_head_hex=", _hex_preview(full, 48))
                    continue

                record["_rx_utc"] = utc_now()
                record["_radio"] = meta or None

                atomic_write_json(LATEST_PATH, record)
                append_history(record)

                last_decoded_time = now
                last_heartbeat = now

                print(f"RX OK (TM) msg_id={msg_id} seq={record.get('seq')} frags={frag_tot} rssi={meta.get('packet_rssi_dbm') if meta else None}")

        if DEBUG and not parsed_any_tm:
            dbg("TM parse: no complete frame yet", "tm_stream_len=", len(tm_stream), "pending=", _pending_summary(pending))

        # Legacy newline fallback
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