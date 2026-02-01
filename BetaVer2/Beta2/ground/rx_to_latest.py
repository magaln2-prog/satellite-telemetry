#!/usr/bin/env python3
"""
rx_to_latest.py (GROUND)

- Receives LoRa UART bytes
- Parses TM frames (protocol_tm.py)
- Reassembles fragments by msg_id
- Decodes binary payloads (FAST/FULL)
- Merges into a "latched latest" dict so the dashboard always has all categories

Extras for index.html compatibility:
- Adds uv_now + thr_now booleans derived from pwr_flags bits
- Adds ts (ISO string) + _rx_ts
- Builds alerts list from battery thresholds + flags

History:
- Append only FULL packets to history.jsonl (keeps history smaller)
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

from protocol_tm import (
    try_parse_one,
    unpack_payload,
    PKT_FULL,
)

# ----------------------------
# Single-instance lock
# ----------------------------
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
# Timeouts
# ----------------------------
NO_CONTACT_AFTER_S = 30.0
REASSEMBLY_TTL_S = 60.0
RADIO_SETTLE_S = 2.0
SERIAL_ERROR_BACKOFF_S = 0.5

# ----------------------------
# Alerts thresholds (match flight-ish)
# ----------------------------
LOW_BATT_PCT = 20
CRIT_BATT_PCT = 10
LOW_BATT_V = 3.55

# Files
BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR / "logs"
LATEST_PATH = BASE_DIR / "latest.json"
HISTORY_PATH = LOG_DIR / "history.jsonl"
LOG_DIR.mkdir(parents=True, exist_ok=True)

DEBUG = os.environ.get("RX_DEBUG", "0").lower() not in ("0", "", "false", "no", "off")

# Latched merged view (FAST merges into last FULL so UI stays populated)
LAST_LATEST: dict = {}


def dbg(*args: object) -> None:
    if DEBUG:
        print("[DBG]", *args, file=sys.stderr, flush=True)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def unix_to_iso(ts_unix: int | None) -> str | None:
    if ts_unix is None:
        return None
    try:
        return datetime.fromtimestamp(int(ts_unix), tz=timezone.utc).isoformat(timespec="seconds")
    except Exception:
        return None


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
        atomic_write_json(LATEST_PATH, {"status": "NO DATA YET", "_rx_utc": utc_now(), "_rx_ts": utc_now(), "_radio": None})


def _pending_gc(pending: dict, now_s: float) -> None:
    dead = []
    for msg_id, st in pending.items():
        if (now_s - st["t0"]) > REASSEMBLY_TTL_S:
            dead.append(msg_id)
    for msg_id in dead:
        del pending[msg_id]


def init_radio():
    if not HAVE_RADIO:
        return None, "sx126x import failed"

    try:
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
        return lora, None
    except Exception as e:
        return None, repr(e)


def flags_to_bools(pwr_flags: int | None) -> tuple[bool, bool]:
    """
    Matches flight.py _pack_flags_byte layout:
      bit0 undervoltage_now
      bit1 throttled_now
      bit2 freq_capped_now (unused by index.html)
      ...
    index.html expects:
      uv_now, thr_now
    """
    if pwr_flags is None:
        return False, False
    try:
        f = int(pwr_flags) & 0xFF
        uv_now = bool(f & (1 << 0))
        thr_now = bool(f & (1 << 1))
        return uv_now, thr_now
    except Exception:
        return False, False


def compute_alerts(latest: dict) -> list[str]:
    alerts: list[str] = []

    # Battery-based alerts
    try:
        bp = latest.get("bp")
        if bp is not None:
            bp = float(bp)
            if bp <= CRIT_BATT_PCT:
                alerts.append("BATT_CRIT_PCT")
            elif bp <= LOW_BATT_PCT:
                alerts.append("BATT_LOW_PCT")
    except Exception:
        pass

    try:
        bv = latest.get("bv")
        if bv is not None and float(bv) <= LOW_BATT_V:
            alerts.append("BATT_LOW_V")
    except Exception:
        pass

    # Flags alerts (index.html also shows these, but we keep alerts populated)
    uv_now = bool(latest.get("uv_now"))
    thr_now = bool(latest.get("thr_now"))
    if uv_now:
        alerts.append("UNDERVOLT")
    if thr_now:
        alerts.append("THROTTLED")

    return alerts


def main() -> None:
    ensure_latest_initialized()

    lora, err = init_radio()
    if err:
        print("Radio init error:", err)
        sys.exit(1)

    time.sleep(RADIO_SETTLE_S)

    tm_stream = b""
    pending: dict[int, dict] = {}

    last_rx_bytes_time = 0.0

    print("RX running... (set RX_DEBUG=1 for debug)")

    while True:
        now = time.time()

        # Read one chunk
        try:
            payload, meta = lora.recv_packet(timeout=RX_TIMEOUT_S)
            raw = payload if payload else b""
        except Exception as e:
            dbg("Serial/recv error:", repr(e))
            time.sleep(SERIAL_ERROR_BACKOFF_S)
            continue

        if raw:
            last_rx_bytes_time = now

        # Contact heartbeat (do NOT blank UI; keep last values latched)
        if (now - last_rx_bytes_time) > NO_CONTACT_AFTER_S:
            LAST_LATEST.setdefault("status", "NO CONTACT")
            LAST_LATEST["_rx_utc"] = utc_now()
            LAST_LATEST["_rx_ts"] = LAST_LATEST["_rx_utc"]
            LAST_LATEST["_radio"] = meta or None
            atomic_write_json(LATEST_PATH, LAST_LATEST)

        if not raw:
            continue

        tm_stream += raw

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

            # Complete message reassembled
            if len(st["parts"]) == st["tot"]:
                full = b"".join(st["parts"][i] for i in range(st["tot"]))
                del pending[msg_id]

                pkt_type, fields = unpack_payload(full)
                if fields is None:
                    dbg("PAYLOAD decode FAIL", "msg_id=", msg_id, "len=", len(full))
                    continue

                # Merge into latched latest
                LAST_LATEST.update(fields)

                # Add timestamps compatible with index.html
                # Prefer device ts_unix -> ts ISO, but always include _rx_utc/_rx_ts
                ts_iso = unix_to_iso(LAST_LATEST.get("ts_unix"))
                if ts_iso:
                    LAST_LATEST["ts"] = ts_iso
                LAST_LATEST["_rx_utc"] = utc_now()
                LAST_LATEST["_rx_ts"] = LAST_LATEST["_rx_utc"]

                # Decode flags -> booleans expected by index.html
                uv_now, thr_now = flags_to_bools(LAST_LATEST.get("pwr_flags"))
                LAST_LATEST["uv_now"] = uv_now
                LAST_LATEST["thr_now"] = thr_now

                # Build alerts list (optional but nice)
                LAST_LATEST["alerts"] = compute_alerts(LAST_LATEST)

                # Metadata
                LAST_LATEST["_radio"] = meta or None
                LAST_LATEST["_pkt"] = "full" if pkt_type == PKT_FULL else "fast"
                LAST_LATEST["status"] = "CONTACT"

                atomic_write_json(LATEST_PATH, LAST_LATEST)

                # Only append FULL to history
                if pkt_type == PKT_FULL:
                    append_history(LAST_LATEST)

                print(
                    f"RX OK msg_id={msg_id} seq={LAST_LATEST.get('seq')} "
                    f"frags={frag_tot} pkt={LAST_LATEST.get('_pkt')}"
                )

        _pending_gc(pending, now)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nRX stopped.")

