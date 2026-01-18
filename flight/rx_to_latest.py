#!/usr/bin/env python3
"""rx_to_latest.py

Ground-side receiver that matches your **new flight script**.

"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional

import sx126x

# ------------------------------
# Ground LoRa config
# ------------------------------
LORA_PORT = "/dev/serial0"
LORA_FREQ_MHZ = 915
LORA_ADDR = 0xFFFF  # common for RX in fixed mode
LORA_POWER_DBM = 22
LORA_AIR_SPEED = 2400
LORA_NET_ID = 0
LORA_CRYPT = 0
LORA_RSSI = True
LORA_BUFFER_SIZE = 240  # driver supports {240,128,64,32}

# ------------------------------
# Files
# ------------------------------
BASE_DIR = Path(__file__).resolve().parent
OUT_FILE = BASE_DIR / "latest.json"
LOG_DIR = BASE_DIR / "logs"
LOG_DIR.mkdir(exist_ok=True)
HISTORY_FILE = LOG_DIR / "history.jsonl"
RAW_LOG_FILE = LOG_DIR / "rx_raw.log"

# If we haven't received anything in this many seconds, dashboard can treat as "no contact".
NO_CONTACT_S = 10.0

#function that returns time
def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_write(path: Path, text: str) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


@dataclass
class ReassemblyState:
    buf: bytes = b""


def split_lines(state: ReassemblyState, chunk: bytes, max_buf: int = 8192) -> List[bytes]:
    if not chunk:
        return []

    state.buf += chunk

    # Safety cap: if framing is lost, don't let memory grow forever.
    if len(state.buf) > max_buf:
        state.buf = state.buf[-1024:]

    lines: List[bytes] = []
    while b"\n" in state.buf:
        line, state.buf = state.buf.split(b"\n", 1)
        line = line.strip()
        if line:
            lines.append(line)
    return lines


def maybe_strip_to_json(line: bytes) -> bytes:
    """If extra non-JSON header bytes sneak in, strip to first '{'.

    This makes the ground side tolerant during transitions.
    """
    if not line:
        return line
    if line[:1] == b"{":
        return line
    i = line.find(b"{")
    if i >= 0:
        return line[i:]
    return line


def log_raw(reason: str, payload: bytes, meta: Optional[dict] = None) -> None:
    meta_str = "" if not meta else f" meta={meta}"
    with RAW_LOG_FILE.open("a", encoding="utf-8", buffering=1) as f:
        f.write(f"{now_iso()} | {reason}{meta_str} | {payload!r}\n")


def main() -> None:
    print("=== RX -> latest.json (newline reassembly) ===")
    print(f"Port={LORA_PORT} freq={LORA_FREQ_MHZ} addr={LORA_ADDR} buffer={LORA_BUFFER_SIZE} rssi={LORA_RSSI}")

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

    states: Dict[int, ReassemblyState] = {}
    last_rx_time_s = 0.0

    while True:
        pkt = lora.recv_packet(timeout_s=0.5)
        if not pkt:
            # optional: write a heartbeat "no contact" marker without overwriting last good data
            if last_rx_time_s and (time.time() - last_rx_time_s) > NO_CONTACT_S:
                # dashboard can also compute stale from _rx_ts; we keep this minimal.
                pass
            time.sleep(0.01)
            continue

        meta, payload = lora.parse_packet(pkt)
        if not payload:
            continue

        src = int(meta.get("src_addr", 0)) if isinstance(meta, dict) else 0
        state = states.setdefault(src, ReassemblyState())

        for raw_line in split_lines(state, payload):
            line = maybe_strip_to_json(raw_line)
            try:
                obj = json.loads(line.decode("utf-8"))
            except Exception as e:
                log_raw(f"json_decode_error: {e}", raw_line, meta)
                continue

            last_rx_time_s = time.time()

            # Stamp receive time + radio meta
            obj["_rx_ts"] = now_iso()
            obj["_radio"] = meta or {}
            obj["_stale_s"] = 0.0

            # Write latest.json (atomic)
            atomic_write(OUT_FILE, json.dumps(obj, indent=2, ensure_ascii=False))

            # Append history
            with HISTORY_FILE.open("a", encoding="utf-8", buffering=1) as hf:
                hf.write(json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n")

            # Print compact status
            ts = obj.get("ts") or obj.get("timestamp_utc") or obj.get("_rx_ts")
            seq = obj.get("seq") or obj.get("sequence_id")
            print(f"RX src={src} seq={seq} ts={ts}")


if __name__ == "__main__":
    main()
