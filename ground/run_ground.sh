#!/usr/bin/env bash
set -e

# Start LoRa receiver in background
python3 rx_to_latest.py &

# Start dashboard (foreground)
python3 dashboard.py

