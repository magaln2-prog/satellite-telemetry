"""
sx126x.py (TEMP PATCH: skip runtime configuration)

Demo-safe driver for Waveshare/Ebyte SX126x UART LoRa modules when
M0/M1 are strapped with jumpers (Normal Mode).

- Keeps original API
- Skips self.set() at init
- Forces Normal Mode (M0=0, M1=0)
- Adds burst-safe recv_packet() to avoid partial UART slices
"""

import time
from typing import Any, Dict, Optional, Tuple

import RPi.GPIO as GPIO
import serial


class sx126x:
    # GPIO pins (BCM)
    M0 = 22
    M1 = 27

    def __init__(
        self,
        serial_num: str = "/dev/serial0",
        freq: int = 915,
        addr: int = 1,
        power: int = 22,
        rssi: bool = False,
        air_speed: int = 2400,
        net_id: int = 0,
        buffer_size: int = 240,
        crypt: int = 0,
        relay: bool = False,
        lbt: bool = False,
        wor: bool = False,
    ) -> None:
        self.serial_num = serial_num
        self.start_freq = int(freq)
        self.addr = int(addr)
        self.power = int(power)
        self.rssi = bool(rssi)
        self.air_speed = int(air_speed)
        self.net_id = int(net_id)
        self.buffer_size = int(buffer_size)
        self.crypt = int(crypt)
        self.relay = bool(relay)
        self.lbt = bool(lbt)
        self.wor = bool(wor)

        # GPIO init (safe even if not used)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.M0, GPIO.OUT)
        GPIO.setup(self.M1, GPIO.OUT)

        # Force NORMAL mode (M0=0, M1=0)
        GPIO.output(self.M0, GPIO.LOW)
        GPIO.output(self.M1, GPIO.LOW)

        # Serial init
        self.ser = serial.Serial(
            self.serial_num,
            9600,
            timeout=0,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )

        # IMPORTANT: Skip self.set(...) / runtime configuration
        # Assumes both radios were pre-configured to same settings.

    # -------------------------------------------------
    # TX
    # -------------------------------------------------
    def send(self, data: bytes) -> None:
        if isinstance(data, str):
            data = data.encode("utf-8")
        self.ser.write(data)

    # -------------------------------------------------
    # Helpers
    # -------------------------------------------------
    def max_app_payload_bytes(self, header_len: int = 3) -> int:
        """
        Maximum application payload size accounting for UART header bytes.
        """
        return max(0, int(self.buffer_size) - int(header_len))

    # -------------------------------------------------
    # RX helpers
    # -------------------------------------------------
    def recv_packet(self, timeout_s: float = 0.5) -> Optional[bytes]:
        """
        Read one UART *burst* (closest thing to a full packet) instead of an arbitrary slice.

        Behavior:
        - Wait up to timeout_s for the first byte.
        - Once bytes arrive, keep reading and accumulating.
        - Stop when the UART has been idle for a short gap (IDLE_GAP_S).

        This greatly reduces the chance that higher-level code feeds partial packet fragments
        into parse_packet(), which assumes packet-aligned data.
        """
        deadline = time.time() + timeout_s
        buf = bytearray()

        # 30ms of no new bytes -> consider the burst complete
        IDLE_GAP_S = 0.03
        # Safety cap: don't let one read grow unbounded
        MAX_BURST = int(getattr(self, "buffer_size", 240))

        while time.time() < deadline:
            waiting = self.ser.inWaiting()
            if waiting > 0:
                # Read what is currently available
                chunk = self.ser.read(waiting)
                if chunk:
                    buf.extend(chunk)

                # Keep collecting until line goes idle briefly
                idle_start = time.time()
                while True:
                    time.sleep(0.005)
                    waiting2 = self.ser.inWaiting()
                    if waiting2 > 0:
                        chunk2 = self.ser.read(waiting2)
                        if chunk2:
                            buf.extend(chunk2)
                        idle_start = time.time()
                        if len(buf) >= MAX_BURST:
                            break
                    else:
                        if (time.time() - idle_start) >= IDLE_GAP_S:
                            break

                return bytes(buf) if buf else None

            time.sleep(0.005)

        return None

    def parse_packet(self, pkt: bytes) -> Tuple[Dict[str, Any], bytes]:
        """
        Parse module header:

        - First 2 bytes: src address (big-endian)
        - Next 1 byte: freq offset
        - Optional last 1 byte: RSSI (if self.rssi True)
        - Payload in the middle
        """
        meta: Dict[str, Any] = {}
        if not pkt or len(pkt) < 3:
            return meta, b""

        src_addr = (pkt[0] << 8) | pkt[1]
        freq_off = pkt[2]
        freq_mhz = int(self.start_freq + freq_off)

        meta.update({"src_addr": src_addr, "freq_mhz": freq_mhz, "raw_len": len(pkt)})

        if self.rssi and len(pkt) >= 4:
            rssi_raw = pkt[-1]
            meta["packet_rssi_dbm"] = -(256 - int(rssi_raw))
            payload = pkt[3:-1]
        else:
            payload = pkt[3:]

        return meta, payload