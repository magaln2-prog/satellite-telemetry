GROUND STATION – README.txt
===========================

This folder contains the GROUND STATION software for receiving LoRa telemetry,
logging it, and displaying it in a live web dashboard.

The ground system has THREE main components:
1) LoRa receiver (rx_to_latest.py)
2) Web API server (dashboard.py)
3) Web UI (static/index.html)

------------------------------------------------------------
FOLDER STRUCTURE
------------------------------------------------------------

ground/
│
├── rx_to_latest.py        # Receives LoRa packets and writes JSON files
├── dashboard.py           # Flask web server (API + dashboard)
├── sx126x.py              # LoRa driver (shared with flight)
│
├── logs/                  # Auto-created at runtime
│   └── history.jsonl      # Rolling telemetry history
│
├── latest.json            # Auto-created: most recent telemetry packet
│
└── static/
    └── index.html         # Web dashboard UI


NOTE:
- You do NOT need to manually create `logs/` or `latest.json`
- rx_to_latest.py creates them automatically at runtime

------------------------------------------------------------
WHAT EACH SCRIPT DOES
------------------------------------------------------------

1) rx_to_latest.py
------------------
This is the LoRa RECEIVER.

Responsibilities:
- Initialize the SX126x LoRa radio
- Listen continuously for incoming packets
- Parse packets into JSON
- Write:
    - latest.json        (overwritten every packet)
    - logs/history.jsonl (append-only log)
- Add receive-side metadata:
    - receive timestamp (_rx_utc)
    - RSSI
    - link status

This script MUST be running for the dashboard to show live data.

Run it first.


2) dashboard.py
---------------
This is the WEB SERVER.

Responsibilities:
- Serve API endpoints:
    /api/latest   → latest telemetry
    /api/history  → recent history (for charts)
    /api/health   → link status + packet age
- Serve the dashboard UI from /static

This script does NOT talk to the radio.
It only reads files written by rx_to_latest.py.

Run this AFTER rx_to_latest.py.


3) static/index.html
--------------------
This is the WEB DASHBOARD UI.

Responsibilities:
- Fetch data from the Flask API
- Display:
    - Temperature, humidity, pressure
    - Battery voltage and percentage
    - Image capture flag
    - Radio RSSI
    - Link status (LIVE / STALE / NO CONTACT)
    - Time-series charts

This file should NOT be edited unless you are changing the UI.


4) sx126x.py
------------
This is the LoRa driver.

Responsibilities:
- Handle serial communication with the SX126x module
- Send and receive raw packets
- Parse Waveshare-style frames
- Report RSSI and metadata

This file is shared between flight and ground.
Do NOT modify unless you understand the radio protocol.


------------------------------------------------------------
HOW TO RUN (STEP-BY-STEP)
------------------------------------------------------------

1) Connect LoRa hardware
-----------------------
- SX126x module connected via UART
- Correct serial port (e.g. /dev/serial0 or /dev/ttyUSB0)
- Matching frequency, net ID, and addressing with flight unit

2) Install dependencies
-----------------------
Python 3.9+ recommended.

Required packages:
- flask
- pyserial

Example:
    pip install flask pyserial

(Exact dependencies may vary by platform)


3) Start the receiver
---------------------
From inside the ground folder:

    python3 rx_to_latest.py

Expected behavior:
- logs/ folder appears
- latest.json appears after first packet
- Console shows packet receive messages

Leave this running.


4) Start the dashboard
----------------------
Open a NEW terminal:

    python3 dashboard.py

Expected output:
- Flask server starts on port 5001 (default)


5) Open the dashboard
---------------------
Open a browser and go to:

    http://localhost:5001

You should see:
- Live telemetry (if flight is transmitting)
- Charts updating over time
- Link status badge (green/yellow/red)

------------------------------------------------------------
TROUBLESHOOTING
------------------------------------------------------------

Dashboard shows "NO CONTACT":
- rx_to_latest.py is not running
- No packets are being received
- Frequency / net ID mismatch

Dashboard loads but charts are empty:
- latest.json exists but history.jsonl is empty
- Receiver started recently (wait a few packets)

Pe
