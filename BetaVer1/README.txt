GROUND STATION – BETA PROTOTYPE
README.txt
==============================

Version
-------
Name: Ground Station Telemetry System
Stage: BETA (Prototype)
Purpose: Functional end-to-end validation

This version represents a *working prototype* of the ground station.
All major subsystems are present and operational, but performance,
latency, and resource usage have not yet been fully optimized.


What This Beta Version Demonstrates
-----------------------------------
This beta proves that the following pipeline works reliably:

  - LoRa telemetry packets are received over UART
  - Fragmented packets are reassembled correctly
  - CRC-protected frames reject corrupted data
  - Telemetry is stored safely on disk
  - A live dashboard updates in near real time
  - Loss of contact is detected and displayed

The focus of this version is **correctness and robustness**, not speed.


Current Architecture Overview
-----------------------------
Flight System
  → Sends fragmented telemetry frames over LoRa

Ground System
  → sx126x.py
      Reads raw bytes from the LoRa UART
  → protocol_tm.py
      Parses TM frames and validates CRC
  → rx_to_latest.py
      Reassembles messages
      Writes latest.json atomically
      Appends history.jsonl
  → dashboard.py
      Serves telemetry via HTTP
  → index.html
      Displays charts, tables, and contact state


Why This Is Still a Beta
-----------------------
Although functional, this prototype has known performance and design
limitations that are acceptable for a beta but not for final deployment.


Known Limitations (Intentional for Beta)
----------------------------------------

1) File-Based Data Passing
   - latest.json is written to disk and re-read frequently
   - history.jsonl is streamed on demand
   - This is safe and simple, but not the fastest approach

2) High-Frequency Polling
   - The dashboard polls /api/latest and /api/health frequently
   - This keeps the UI responsive but increases CPU usage

3) Fragment Handling
   - Fragment reassembly favors correctness over throughput
   - No aggressive packet batching or compression is used

4) Single-Process Design
   - rx_to_latest.py and dashboard.py run as separate processes
   - Communication occurs only via the filesystem

5) Development-Oriented Flask Server
   - Flask dev server is used instead of a production WSGI server
   - Suitable for testing, not long-term deployment

6) Conservative Safety Checks
   - Extra validation, JSON parsing, and fallbacks are enabled
   - These add overhead but improve stability during development


Why These Tradeoffs Were Chosen
-------------------------------
For a telemetry system, losing data is worse than being slow.

This beta intentionally prioritizes:
  - Data integrity
  - Debug visibility
  - Crash resistance
  - Easy inspection of intermediate files

Optimization was deferred until correctness was proven.


Optimization Goals for the Next Version
---------------------------------------

The next iteration will focus on **speed, efficiency, and scalability**
without sacrificing reliability.

Planned improvements include:


1) Reduce Disk I/O
   - Cache latest telemetry in memory inside dashboard.py
   - Only hit disk when data actually changes
   - Possibly remove latest.json reads from hot paths

2) Smarter Update Triggers
   - Update dashboard only when new telemetry arrives
   - Eliminate redundant polling where possible

3) More Efficient History Handling
   - Maintain a rolling in-memory buffer of recent telemetry
   - Reduce repeated parsing of history.jsonl

4) Protocol-Level Optimization
   - Reduce payload size further
   - Optimize fragment sizing for LoRa throughput
   - Consider optional compression for numeric telemetry

5) UI Rendering Optimization
   - Lower chart redraw frequency
   - Batch UI updates instead of per-field updates
   - Reduce DOM manipulation cost

6) Server Performance Improvements
   - Move away from Flask dev server if needed
   - Consider event-driven or async patterns

7) Clear Versioning + Metrics
   - Add version tags to telemetry
   - Log latency and packet loss statistics
   - Measure performance before and after optimizations


Definition of “Optimized” for This Project
------------------------------------------
An optimized version should:

  - Update the dashboard only on new data
  - Minimize disk access during steady-state operation
  - Maintain low CPU usage on Raspberry Pi
  - Handle packet loss gracefully
  - Scale to higher telemetry rates without UI lag


Status Summary
--------------
✔ End-to-end telemetry pipeline works
✔ Robust against packet loss and corruption
✔ Safe file handling
✔ Live visualization functional

⏳ Performance tuning pending
⏳ Resource usage optimization pending
⏳ Production hardening pending


This README Describes
---------------------
This document describes the **Beta Prototype** only.
Behavior, performance, and architecture may change significantly
in future versions as optimization work proceeds.


End of README
-------------
