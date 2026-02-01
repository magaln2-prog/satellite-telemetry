#!/usr/bin/env python3
"""
dashboard.py

Flask dashboard backend for the ground station.

This server does three main jobs:
  1) Serve the GUI (static/index.html)
  2) Serve the *latest* received telemetry object (latest.json)
  3) Serve a bounded window of historical telemetry (logs/history.jsonl)

Why this design:
  - latest.json is written atomically by rx_to_latest.py
    (write temp file → rename). This avoids half-written JSON.
  - history.jsonl is append-only and can grow large.
    We only read the *tail* to keep memory + latency low.
  - /api/health exposes packet staleness so the UI can clearly show
    NO CONTACT / STALE DATA without guessing.

Expected telemetry keys (schema-tolerant):
  - timestamps: _rx_utc (preferred), ts, timestamp_utc
  - sequence:   seq, sequence_id
"""

import json
import os
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

from flask import Flask, jsonify, send_from_directory


# -----------------------------
# Flask app + filesystem paths
# -----------------------------

# Create Flask app instance
app = Flask(__name__)

# Base directory = folder containing dashboard.py
# This makes all paths relative and portable.
BASE_DIR = Path(__file__).resolve().parent

# latest.json is continuously overwritten by rx_to_latest.py
# It always contains the most recent *valid* telemetry object.
LATEST_FILE = BASE_DIR / "latest.json"

# history.jsonl grows over time (append-only)
# Each line = one JSON telemetry record
HISTORY_DIR = BASE_DIR / "logs"
HISTORY_FILE = HISTORY_DIR / "history.jsonl"

# Front-end files (HTML, JS, CSS)
# Flask serves these directly
STATIC_DIR = BASE_DIR / "static"


# -----------------------------
# Small helper utilities
# -----------------------------
def _load_latest_json():
    """
    Safely load latest.json.

    This function isolates file I/O + error handling so routes
    stay clean and predictable.

    Returns:
        (data, error)
          - data: dict if successful, None otherwise
          - error: None on success, string on failure
    """
    # No telemetry has ever been received
    if not LATEST_FILE.exists():
        return None, None

    try:
        with open(LATEST_FILE, "r", encoding="utf-8") as f:
            return json.load(f), None
    except Exception as e:
        # Should be rare due to atomic rename strategy,
        # but still guarded against:
        #  - permission issues
        #  - disk corruption
        #  - manual edits
        return None, repr(e)


def _parse_iso_datetime(s: str):
    """
    Best-effort ISO-8601 timestamp parser.

    Accepts:
      - 2026-01-18T12:34:56.123+00:00
      - 2026-01-18T12:34:56.123Z

    Returns:
        datetime (UTC-aware) or None
    """
    if not s or not isinstance(s, str):
        return None
    try:
        # Normalize trailing 'Z' to explicit UTC offset
        return datetime.fromisoformat(s.replace("Z", "+00:00"))
    except Exception:
        return None


def _pick_latest_timestamp(latest: dict | None) -> str | None:
    """
    Select the most reliable timestamp field.

    Priority order matters:
      1) _rx_utc        → ground receive time (best for contact detection)
      2) ts             → radio packet timestamp
      3) timestamp_utc  → full record timestamp
    """
    if not latest:
        return None

    return (
        latest.get("_rx_utc")
        or latest.get("ts")
        or latest.get("timestamp_utc")
    )


def _pick_latest_seq(latest: dict | None):
    """
    Select the best sequence number field.

    Supports both compact radio packets and full records.
    """
    if not latest:
        return None
    return latest.get("seq") or latest.get("sequence_id")


# -----------------------------
# Routes (HTTP API + GUI)
# -----------------------------
@app.route("/")
def index():
    """
    Serve the main GUI page.

    All JS/CSS assets are loaded relative to index.html.
    """
    return send_from_directory(STATIC_DIR, "index.html")


@app.route("/api/latest")
def api_latest():
    """
    Return the most recent telemetry object.

    This endpoint is polled frequently by the dashboard UI.

    Response format:
      {
        ok: bool,
        latest: object | null,
        error?: string
      }
    """
    data, err = _load_latest_json()

    # No data yet (normal startup condition)
    if data is None and err is None:
        return jsonify({"ok": False, "latest": None})

    # File exists but could not be parsed
    if err:
        return jsonify({"ok": False, "latest": None, "error": err})

    # Success
    return jsonify({"ok": True, "latest": data})


@app.route("/api/history")
def api_history():
    """
    Return the last N telemetry records from history.jsonl.

    Design notes:
      - We cap results to keep the UI responsive.
      - We stream through the file instead of reading it all.
      - Malformed JSON lines are skipped (robust to partial writes).
    """
    n = 300  # maximum number of points sent to the UI

    if not HISTORY_FILE.exists():
        return jsonify({"ok": True, "records": []})

    # deque(maxlen=n) automatically discards old entries
    last_lines = deque(maxlen=n)

    try:
        with open(HISTORY_FILE, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                last_lines.append(line)
    except Exception as e:
        return jsonify({"ok": False, "records": [], "error": repr(e)})

    # Convert JSONL → list of dicts
    records = []
    for line in last_lines:
        line = line.strip()
        if not line:
            continue
        try:
            records.append(json.loads(line))
        except Exception:
            # Skip corrupted or partial lines
            continue

    return jsonify({"ok": True, "records": records})


@app.route("/api/health")
def api_health():
    """
    Health/status endpoint.

    This endpoint exists so the UI does NOT need to guess:
      - Whether telemetry exists
      - How stale the latest packet is
      - Whether parsing failed

    The UI can map age thresholds → ONLINE / STALE / NO CONTACT.
    """
    now_dt = datetime.now(timezone.utc)
    now_iso = now_dt.isoformat()

    latest = None
    err = None

    # Attempt to load latest.json
    if LATEST_FILE.exists():
        try:
            with open(LATEST_FILE, "r", encoding="utf-8") as f:
                latest = json.load(f)
        except Exception as e:
            latest = None
            err = repr(e)

    # Choose best timestamp field available
    latest_ts = None
    if latest:
        latest_ts = (
            latest.get("_rx_utc")          # preferred (ground receive time)
            or latest.get("_rx_ts")        # legacy fallback
            or latest.get("ts")            # radio timestamp
            or latest.get("timestamp_utc") # full record timestamp
        )

    # Compute how old the latest packet is (seconds)
    latest_age_s = None
    if latest_ts:
        try:
            latest_dt = datetime.fromisoformat(
                latest_ts.replace("Z", "+00:00")
            )
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
    Launch the Flask development server.

    Environment variables:
      HOST   → bind address (default 0.0.0.0)
      PORT   → port number (default 5001)
      DEBUG  → "1" enables Flask debug mode

    use_reloader=False is critical on Raspberry Pi:
      - Prevents Flask from spawning two processes
      - Avoids duplicate serial / file access
    """
    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "5001"))
    debug = os.environ.get("DEBUG", "0") == "1"

    app.run(
        host=host,
        port=port,
        debug=debug,
        use_reloader=False
    )


if __name__ == "__main__":
    main()
