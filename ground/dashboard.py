#!/usr/bin/env python3
"""
dashboard.py

Flask dashboard backend for the ground station.

This server does three main jobs:
  1) Serves the GUI (static/index.html)
  2) Serves the *latest* received telemetry object (latest.json)
  3) Serves a small window of historical telemetry (logs/history.jsonl)

Important design notes:
  - latest.json is expected to be written atomically by rx_to_latest.py
    (write temp file, then rename). That prevents "half-written JSON".
  - history.jsonl is append-only. Each line is one JSON object.
  - We avoid reading the entire history file into memory (deque tail).
  - /api/health returns a "staleness" age in seconds so the UI can show
    NO CONTACT / stale data conditions.

Expected telemetry keys (may vary depending on whether you are viewing
radio-compact or full records):
  - timestamp keys: _rx_utc (preferred), ts, timestamp_utc
  - sequence keys: seq, sequence_id
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

# Base directory is the folder containing this dashboard.py file.
BASE_DIR = Path(__file__).resolve().parent

# These files are produced by rx_to_latest.py (ground receiver).
LATEST_FILE = BASE_DIR / "latest.json"
HISTORY_DIR = BASE_DIR / "logs"
HISTORY_FILE = HISTORY_DIR / "history.jsonl"

# Front-end assets (index.html + JS/CSS). Keep this in ./static/
STATIC_DIR = BASE_DIR / "static"


# -----------------------------
# Small helper utilities
# -----------------------------
def _load_latest_json():
    """
    Load latest.json safely.

    Returns:
        (data, error_str_or_None)
        - data is a dict (on success) or None
        - error is a string (on failure) or None
    """
    if not LATEST_FILE.exists():
        return None, None

    try:
        with open(LATEST_FILE, "r", encoding="utf-8") as f:
            return json.load(f), None
    except Exception as e:
        # Rare if rx_to_latest writes atomically, but still possible if:
        # - file permissions are wrong
        # - someone edited it mid-run
        # - disk issues, etc.
        return None, repr(e)


def _parse_iso_datetime(s: str):
    """
    Best-effort parse for ISO timestamps.

    We accept strings like:
      - 2026-01-18T12:34:56.123+00:00
      - 2026-01-18T12:34:56.123Z
    """
    if not s or not isinstance(s, str):
        return None
    try:
        return datetime.fromisoformat(s.replace("Z", "+00:00"))
    except Exception:
        return None


def _pick_latest_timestamp(latest: dict | None) -> str | None:
    """
    Choose the best timestamp key available.

    Priority:
      1) _rx_utc        (ground receive time; best for contact/no-contact)
      2) ts             (radio packet timestamp)
      3) timestamp_utc  (full record timestamp)
    """
    if not latest:
        return None
    return (
        latest.get("_rx_utc")
        or latest.get("ts")
        or latest.get("timestamp_utc")
    )


def _pick_latest_seq(latest: dict | None):
    """Choose the best sequence key available (compact vs full record)."""
    if not latest:
        return None
    return latest.get("seq") or latest.get("sequence_id")


# -----------------------------
# Routes
# -----------------------------
@app.route("/")
def index():
    """Serve the GUI HTML."""
    return send_from_directory(STATIC_DIR, "index.html")


@app.route("/api/latest")
def api_latest():
    """
    Return the most recent telemetry object.

    Response:
      { ok: bool, latest: object|null, error?: string }
    """
    data, err = _load_latest_json()
    if data is None and err is None:
        return jsonify({"ok": False, "latest": None})

    if err:
        return jsonify({"ok": False, "latest": None, "error": err})

    return jsonify({"ok": True, "latest": data})


@app.route("/api/history")
def api_history():
    """
    Return last N telemetry objects from history.jsonl.

    We intentionally cap the response so the UI stays fast.
    We do NOT do f.readlines()[-n:] because that loads the entire file.
    """
    n = 300  # max points to return

    if not HISTORY_FILE.exists():
        return jsonify({"ok": True, "records": []})

    # Read file streaming and keep only the last N lines.
    last_lines = deque(maxlen=n)
    try:
        with open(HISTORY_FILE, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                last_lines.append(line)
    except Exception as e:
        return jsonify({"ok": False, "records": [], "error": repr(e)})

    # Parse JSONL lines into objects. Skip malformed lines.
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
    now_dt = datetime.now(timezone.utc)
    now_iso = now_dt.isoformat()

    latest = None
    err = None

    # Try reading latest.json
    if LATEST_FILE.exists():
        try:
            with open(LATEST_FILE, "r", encoding="utf-8") as f:
                latest = json.load(f)
        except Exception as e:
            latest = None
            err = repr(e)

    # Pick the best timestamp key available (works with old + new schemas)
    latest_ts = None
    if latest:
        latest_ts = (
            latest.get("_rx_utc")          # preferred (ground receive time)
            or latest.get("_rx_ts")        # older name (fallback)
            or latest.get("ts")            # radio packet timestamp
            or latest.get("timestamp_utc") # full record timestamp
        )

    # Compute "age" in seconds (how stale the latest packet is)
    latest_age_s = None
    if latest_ts:
        try:
            latest_dt = datetime.fromisoformat(latest_ts.replace("Z", "+00:00"))
            latest_age_s = (now_dt - latest_dt).total_seconds()
        except Exception:
            latest_age_s = None

    return jsonify({
        "ok": True,
        "server_time": now_iso,
        "latest_present": bool(latest),
        "latest_ts": latest_ts,
        "latest_age_s": latest_age_s,
        "latest_error": err,
    })


# -----------------------------
# Main entrypoint
# -----------------------------
def main():
    """
    Run the Flask dev server.

    Suggested environment variables:
      HOST=0.0.0.0
      PORT=5001
      DEBUG=0 or 1
    """
    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "5001"))
    debug = os.environ.get("DEBUG", "0") == "1"

    # use_reloader=False so Flask doesn't start two processes (important on Pi setups)
    app.run(host=host, port=port, debug=debug, use_reloader=False)


if __name__ == "__main__":
    main()
