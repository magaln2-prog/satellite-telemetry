GROUND STATION FOLDER – README.txt
=================================

Purpose
-------
This folder contains the *ground station* software for the telemetry system.

The ground station is responsible for:
  - Receiving telemetry packets over LoRa
  - Reassembling fragmented packets
  - Writing the most recent valid telemetry to disk
  - Storing a rolling history of received telemetry
  - Serving a live web dashboard for visualization

This side of the system never generates telemetry.
It only receives, validates, stores, and displays data.


Folder Contents
---------------
rx_to_latest.py
  Ground-side receiver script.
  - Reads packets from the SX126x LoRa module
  - Uses protocol_tm.py to parse and validate TM frames
  - Reassembles fragmented messages
  - Writes the most recent valid telemetry to latest.json
  - Appends telemetry to logs/history.jsonl

dashboard.py
  Flask backend for the live dashboard.
  - Serves index.html and static assets
  - Exposes API endpoints:
      /api/latest   → most recent telemetry
      /api/history  → recent telemetry history
      /api/health   → contact + staleness status
  - Designed to be lightweight and safe for Raspberry Pi use

index.html
  Front-end dashboard UI.
  - Displays real-time sensor values
  - Shows charts, tables, and contact status
  - Polls the Flask API at a high rate
  - Does not talk directly to hardware

protocol_tm.py
  Telemetry framing protocol shared with the flight system.
  - Defines TM frame structure
  - Handles fragmentation metadata
  - Verifies CRC16 for data integrity
  - Allows resynchronization on noisy links

sx126x.py
  LoRa UART driver (patched version).
  - Communicates with the SX126x LoRa module
  - Skips runtime configuration when M0/M1 are jumpered
  - Assumes radios are pre-configured with matching settings

logs/
  Directory containing telemetry history.
  - history.jsonl (append-only, one JSON object per line)


High-Level Data Flow
-------------------
1) LoRa packets arrive from the flight system
2) sx126x.py reads raw UART data
3) protocol_tm.py validates and parses TM frames
4) rx_to_latest.py reassembles fragments into full records
5) latest.json is atomically updated with the newest data
6) history.jsonl is appended for long-term storage
7) dashboard.py serves data to index.html via HTTP


Why latest.json Exists
---------------------
The dashboard needs fast access to *only the most recent* telemetry.

Instead of parsing the entire history every time:
  - rx_to_latest.py writes latest.json atomically
  - dashboard.py simply reads one small file
  - The UI never sees half-written or corrupted JSON

This keeps updates fast and reliable.


Why history.jsonl Is Used
-------------------------
history.jsonl is:
  - Append-only
  - Easy to stream
  - Resistant to crashes

Each line is a complete JSON object, making it ideal for:
  - Plotting
  - Post-flight analysis
  - Debugging packet loss


Dashboard API Endpoints
-----------------------
/api/latest
  Returns the most recent telemetry object.

/api/history
  Returns a capped number of recent telemetry records
  (prevents UI slowdowns).

/api/health
  Returns:
    - Server time
    - Whether telemetry exists
    - Timestamp of last packet
    - Age (seconds since last packet)
  Used by the UI to show NO CONTACT / STALE / LIVE states.


Running the Ground Station
--------------------------
Typical startup order:

1) Start the LoRa receiver:
     python3 rx_to_latest.py

2) Start the dashboard server:
     python3 dashboard.py

3) Open a browser:
     http://<ground_pi_ip>:5001


Hardware Assumptions
--------------------
- Raspberry Pi (ground unit)
- SX126x UART LoRa module
- M0 and M1 pins jumpered to NORMAL mode

IMPORTANT:
  Because M0/M1 are jumpered, the radio cannot be reconfigured at runtime.
  Both flight and ground radios must be pre-configured identically.


Troubleshooting Notes
---------------------
- If the dashboard shows NO CONTACT:
    → Check rx_to_latest.py output
    → Verify LoRa wiring and UART device
- If charts are blank but latest.json exists:
    → Check dashboard.py API responses
- If history grows large:
    → This is expected; the dashboard only reads the tail


Design Philosophy
-----------------
The ground station is built to be:
  - Robust to packet loss
  - Safe against partial writes
  - Decoupled from hardware timing
  - Easy to debug and inspect manually

The UI depends only on files + HTTP, never on the radio directly.


End of README
-------------
