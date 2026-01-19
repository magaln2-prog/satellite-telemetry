#!/usr/bin/env python3
import time
import sx126x

LORA_PORT="/dev/serial0"
LORA_FREQ_MHZ=915
LORA_ADDR=1
LORA_POWER_DBM=22
LORA_BUFFER_SIZE=240
LORA_CRYPT=0
LORA_RSSI=True
LORA_AIR_SPEED=2400

lora = sx126x.sx126x(
    serial_num=LORA_PORT,
    freq=LORA_FREQ_MHZ,
    addr=LORA_ADDR,
    power=LORA_POWER_DBM,
    rssi=LORA_RSSI,
    air_speed=LORA_AIR_SPEED,
    buffer_size=LORA_BUFFER_SIZE,
    crypt=LORA_CRYPT,
    relay=False,
    lbt=False,
    wor=False,
)

print("RX debug running… waiting for ANY packets")
count=0

while True:
    pkt = lora.recv_packet(timeout_s=0.5)
    if not pkt:
        continue

    count += 1
    meta, payload = lora.parse_packet(pkt)
    print(f"\n--- PACKET #{count} ---")
    print("meta:", meta)
    print("payload_len:", 0 if payload is None else len(payload))
    if payload:
        print("payload_preview:", payload[:80])
