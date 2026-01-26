#!/usr/bin/env python3
"""
dashboard.py

Flask dashboard backend for the ground station.

CHANGE (Option: "Update only on new valid data"):
- /api/latest returns the LAST VALID (decoded) telemetry object.
- /api/health reports contact/no-contact + age based on raw latest.json status.

Why:
RX sometimes writes status-only objects (NO CONTACT / CONTACT decode pending) into latest.json.
Those objects don't have sensor keys, so the website "blanks".
This server-side latch prevents the UI from ever being overwritten by status-only data.
"""

import json
import os
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

from flask import Flask, jsonify, send_from_directory


# -----------------------------
# Flask app + paths
# -----------------------------
app = Flask(__name__)

BASE_DIR = Path(__file__).resolve().parent
LATEST_FILE = BASE_DIR / "latest.json"
HISTORY_DIR = BASE_DIR / "logs"
HISTORY_FILE = HISTORY_DIR / "history.jsonl"
STATIC_DIR = BASE_DIR / "static"


# -----------------------------
# Server-side latch (in-memory)
# -----------------------------
_LAST_GOOD = None          # last valid telemetry dict
_LAST_GOOD_TS = None       # timestamp string for last valid telemetry


def _load_latest_json():
    if not LATEST_FILE.exists():
        return None, None
    try:
        with open(LATEST_FILE, "r", encoding="utf-8") as f:
            return json.load(f), None
    except Exception as e:
        return None, repr(e)


def _parse_iso_datetime(s: str):
    if not s or not isinstance(s, str):
        return None
    try:
        return datetime.fromisoformat(s.replace("Z", "+00:00"))
    except Exception:
        return None


def _pick_ts(obj: dict | None) -> str | None:
    if not obj:
        return None
    return obj.get("_rx_utc") or obj.get("_rx_ts") or obj.get("ts") or obj.get("timestamp_utc")


def _is_number(x) -> bool:
    try:
        return x is not None and float(x) == float(x)  # not NaN
    except Exception:
        return False


def _looks_like_telemetry(d: dict | None) -> bool:
    """
    Heuristic: treat as "valid telemetry" if it contains at least one real sensor number
    or a sequence number.

    This prevents status-only blobs from overwriting the UI.
    """
    if not d or not isinstance(d, dict):
        return False

    # If it's a status blob, it's not "telemetry"
    status = d.get("status")
    if isinstance(status, str) and ("CONTACT" in status or "NO CONTACT" in status or "pending" in status.lower()):
        # still might include telemetry, so don't hard-block on status alone
        pass

    # seq present usually means it's a real telemetry packet
    if d.get("seq") is not None or d.get("sequence_id") is not None:
        return True

    # numeric sensor keys (add/remove as your schema evolves)
    keys = ["t_c", "rh", "p_hpa", "lux", "uvi", "cpu_c", "bv", "bp", "ba"]
    return any(_is_number(d.get(k)) for k in keys)


def _update_latch_if_valid(candidate: dict | None) -> None:
    """
    Update global last-good telemetry if candidate looks valid and is newer.
    """
    global _LAST_GOOD, _LAST_GOOD_TS

    if not _looks_like_telemetry(candidate):
        return

    cand_ts = _pick_ts(candidate) or ""
    # If we don't have any latched value yet, accept immediately
    if _LAST_GOOD is None:
        _LAST_GOOD = candidate
        _LAST_GOOD_TS = cand_ts or _LAST_GOOD_TS
        return

    # If timestamps exist and differ, prefer the newer one (or just different)
    if cand_ts and cand_ts != _LAST_GOOD_TS:
        _LAST_GOOD = candidate
        _LAST_GOOD_TS = cand_ts
        return

    # No timestamp? still allow replacing if seq increases
    try:
        old_seq = _LAST_GOOD.get("seq")
        new_seq = candidate.get("seq")
        if old_seq is None or new_seq is None:
            return
        if int(new_seq) > int(old_seq):
            _LAST_GOOD = candidate
            _LAST_GOOD_TS = cand_ts or _LAST_GOOD_TS
    except Exception:
        return


# -----------------------------
# Routes
# -----------------------------
@app.route("/")
def index():
    return send_from_directory(STATIC_DIR, "index.html")


@app.route("/api/latest")
def api_latest():
    """
    Returns the last VALID decoded telemetry object (latched).
    Never returns status-only "NO CONTACT" blobs as latest telemetry.
    """
    data, err = _load_latest_json()
    if err:
        # even if latest.json is unreadable, we can still serve last-good if we have it
        if _LAST_GOOD is not None:
            return jsonify({"ok": True, "latest": _LAST_GOOD, "latched": True, "error": err})
        return jsonify({"ok": False, "latest": None, "error": err})

    # If file doesn't exist yet
    if data is None:
        if _LAST_GOOD is not None:
            return jsonify({"ok": True, "latest": _LAST_GOOD, "latched": True})
        return jsonify({"ok": False, "latest": None})

    # Update latch only if this looks like real telemetry
    _update_latch_if_valid(data)

    if _LAST_GOOD is not None:
        return jsonify({"ok": True, "latest": _LAST_GOOD, "latched": True, "latest_ts": _LAST_GOOD_TS})

    # No valid telemetry seen yet
    return jsonify({"ok": False, "latest": None, "latched": False})


@app.route("/api/history")
def api_history():
    n = 300
    if not HISTORY_FILE.exists():
        return jsonify({"ok": True, "records": []})

    last_lines = deque(maxlen=n)
    try:
        with open(HISTORY_FILE, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                last_lines.append(line)
    except Exception as e:
        return jsonify({"ok": False, "records": [], "error": repr(e)})

    records = []
    for line in last_lines:
        line = line.strip()
        if not line:
            continue
        try:
            records.append(json.loads(line))
        except Exception:
            continue

    return jsonify({"ok": True, "records": records})


@app.route("/api/health")
def api_health():
    """
    Health/contact endpoint:
    - Reads raw latest.json (may be status-only) to determine contact state + age.
    - Does NOT affect what /api/latest returns.
    """
    now_dt = datetime.now(timezone.utc)
    now_iso = now_dt.isoformat()

    latest_raw = None
    err = None

    if LATEST_FILE.exists():
        try:
            with open(LATEST_FILE, "r", encoding="utf-8") as f:
                latest_raw = json.load(f)
        except Exception as e:
            latest_raw = None
            err = repr(e)

    latest_ts = _pick_ts(latest_raw)

    latest_age_s = None
    if latest_ts:
        latest_dt = _parse_iso_datetime(latest_ts)
        if latest_dt:
            latest_age_s = (now_dt - latest_dt).total_seconds()

    status = None
    if isinstance(latest_raw, dict):
        status = latest_raw.get("status")

    # A simple contact classification the UI can use:
    # - "contact" if we have received bytes recently OR we have a fresh packet age
    # - "no_contact" if age is very stale or status says no contact
    contact_state = "unknown"
    if isinstance(status, str) and "NO CONTACT" in status:
        contact_state = "no_contact"
    elif latest_age_s is not None:
        # tune thresholds as you like
        contact_state = "contact" if latest_age_s < 10 else "stale"

    return jsonify({
        "ok": True,
        "server_time": now_iso,
        "latest_present": bool(latest_raw),
        "latest_ts": latest_ts,
        "latest_age_s": latest_age_s,
        "latest_error": err,
        "contact_state": contact_state,
        "raw_status": status,
        # also expose what /api/latest will return
        "latched_present": bool(_LAST_GOOD),
        "latched_ts": _LAST_GOOD_TS,
    })


def main():
    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "5001"))
    debug = os.environ.get("DEBUG", "0") == "1"
    app.run(host=host, port=port, debug=debug, use_reloader=False)


if __name__ == "__main__":
    main()

