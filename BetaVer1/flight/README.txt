FLIGHT FOLDER – README.txt
==========================

Purpose
-------
This folder contains the *flight-side* telemetry code for the project.
It runs on the flight Raspberry Pi and is responsible for:

  - Reading sensors
  - Building a full telemetry record
  - Logging data locally
  - Sending compact telemetry packets over LoRa

The flight system is designed to keep working even if radio contact is
intermittent or lost.


Folder Contents
---------------
flight.py
  Main flight telemetry script.
  - Initializes sensors
  - Collects measurements
  - Builds a full telemetry record
  - Logs data locally (JSONL)
  - Builds compact radio packets
  - Sends packets using TM-framed LoRa messages

protocol_tm.py
  Telemetry framing protocol shared by flight and ground.
  - Defines the TM frame format
  - Handles fragmentation
  - Adds CRC16 for data integrity
  - Allows the receiver to resynchronize if bytes are dropped

sx126x.py
  LoRa radio driver (patched version).
  - Communicates with the SX126x UART LoRa module
  - Modified to skip runtime configuration when M0/M1 are jumpered
  - Assumes radios are pre-configured to matching settings


High-Level Data Flow
-------------------
1) Sensors are read on the flight Raspberry Pi
2) A full telemetry record is created in Python
3) Full record is logged locally as JSONL (for post-flight analysis)
4) A compact version of the data is fragmented
5) Each fragment is wrapped in a TM frame (protocol_tm.py)
6) Frames are sent over LoRa using sx126x.py


Why Fragmentation Is Used
-------------------------
LoRa UART links are unreliable and packet sizes are limited.
Instead of sending one large JSON blob:

  - Telemetry is split into small fragments
  - Each fragment is numbered
  - CRC checks detect corruption
  - The ground station can reassemble safely

This makes the system more robust to packet loss.


Local Logging
-------------
The flight script logs full telemetry locally even if:
  - No radio contact exists
  - Some packets are dropped
  - The ground station is offline

This ensures no data is permanently lost during flight.


Hardware Assumptions
--------------------
- Raspberry Pi (flight unit)
- SX126x UART LoRa module
- Sensors connected via I2C/SPI (depending on configuration)
- M0 and M1 pins on the LoRa module are jumpered to NORMAL mode

IMPORTANT:
  Because M0/M1 are jumpered, the radio cannot enter configuration mode.
  The provided sx126x.py driver is patched to skip runtime configuration.


Running the Flight Script
-------------------------
From the flight Raspberry Pi:

  python3 flight.py

The script is designed to run continuously until stopped.


Common Files Written by flight.py
---------------------------------
- Local telemetry log (JSONL)
- Optional images or additional sensor data (if enabled)

Exact filenames depend on configuration inside flight.py.


Troubleshooting Notes
---------------------
- "No contact" on the ground station does NOT mean flight.py is failing
- Check UART wiring if no packets are received
- Ensure flight and ground radios share the same configuration
- Verify sensors are detected on the I2C bus if values are missing


Design Philosophy
-----------------
This system prioritizes:
  - Robustness over elegance
  - Data preservation over bandwidth
  - Clear separation between flight and ground responsibilities

The flight side never depends on the ground side to function.


End of README
-------------
