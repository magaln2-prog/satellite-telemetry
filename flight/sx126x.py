from __future__ import annotations

import time
from typing import Dict, Tuple

import RPi.GPIO as GPIO
import serial


class sx126x:
    # GPIO pins (BCM numbering) for M0/M1 (Waveshare defaults)
    M0 = 22
    M1 = 27

    # If header is 0xC0, register settings persist across power cycles.
    cfg_reg = [0xC0, 0x00, 0x09, 0x00, 0x00, 0x00, 0x62, 0x00, 0x12, 0x43, 0x00, 0x00]

    # Default module families:
    # E22-400Txx: 410-493MHz
    # E22-900Txx: 850-930MHz
    start_freq = 850
    offset_freq = 18

    SX126X_UART_BAUDRATE_9600 = 0x60

    SX126X_PACKAGE_SIZE_240_BYTE = 0x00
    SX126X_PACKAGE_SIZE_128_BYTE = 0x40
    SX126X_PACKAGE_SIZE_64_BYTE = 0x80
    SX126X_PACKAGE_SIZE_32_BYTE = 0xC0

    lora_air_speed_dic = {1200: 0x01, 2400: 0x02, 4800: 0x03, 9600: 0x04, 19200: 0x05, 38400: 0x06, 62500: 0x07}
    lora_power_dic = {22: 0x00, 17: 0x01, 13: 0x02, 10: 0x03}
    lora_buffer_size_dic = {240: SX126X_PACKAGE_SIZE_240_BYTE, 128: SX126X_PACKAGE_SIZE_128_BYTE, 64: SX126X_PACKAGE_SIZE_64_BYTE, 32: SX126X_PACKAGE_SIZE_32_BYTE}

    def __init__(
        self,
        serial_num: str,
        freq: int,
        addr: int,
        power: int,
        rssi: bool,
        air_speed: int = 2400,
        net_id: int = 0,
        buffer_size: int = 240,
        crypt: int = 0,
        relay: bool = False,
        lbt: bool = False,
        wor: bool = False,
        uart_baud: int = 9600,
    ) -> None:
        self.rssi = bool(rssi)
        self.addr = int(addr) & 0xFFFF
        self.freq = int(freq)
        self.serial_n = str(serial_num)
        self.power = int(power)

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.M0, GPIO.OUT)
        GPIO.setup(self.M1, GPIO.OUT)
        GPIO.output(self.M0, GPIO.LOW)
        GPIO.output(self.M1, GPIO.HIGH)

        self.ser = serial.Serial(serial_num, uart_baud, timeout=0)
        self.ser.flushInput()

        self.set(freq, addr, power, rssi, air_speed, net_id, buffer_size, crypt, relay, lbt, wor)

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    # -------------------------
    # Configuration
    # -------------------------
    def set(
        self,
        freq: int,
        addr: int,
        power: int,
        rssi: bool,
        air_speed: int = 2400,
        net_id: int = 0,
        buffer_size: int = 240,
        crypt: int = 0,
        relay: bool = False,
        lbt: bool = False,
        wor: bool = False,
    ) -> None:
        self.send_to = int(addr) & 0xFFFF
        self.addr = int(addr) & 0xFFFF

        GPIO.output(self.M0, GPIO.LOW)
        GPIO.output(self.M1, GPIO.HIGH)
        time.sleep(0.1)

        low_addr = self.addr & 0xFF
        high_addr = (self.addr >> 8) & 0xFF
        net_id_temp = int(net_id) & 0xFF

        freq = int(freq)
        if freq > 850:
            freq_temp = freq - 850
            self.start_freq = 850
            self.offset_freq = freq_temp
        elif freq > 410:
            freq_temp = freq - 410
            self.start_freq = 410
            self.offset_freq = freq_temp
        else:
            raise ValueError("freq must be in 410–493 or 850–930MHz")

        air_speed_temp = self.lora_air_speed_dic.get(int(air_speed))
        if air_speed_temp is None:
            raise ValueError(f"air_speed must be one of {sorted(self.lora_air_speed_dic.keys())}, got {air_speed}")

        buffer_size = int(buffer_size)
        buffer_size_temp = self.lora_buffer_size_dic.get(buffer_size)
        if buffer_size_temp is None:
            raise ValueError(f"buffer_size must be one of {sorted(self.lora_buffer_size_dic.keys())}, got {buffer_size}")

        # store chosen packet size (bytes) for send() validation
        self._buffer_size_bytes = buffer_size

        power_temp = self.lora_power_dic.get(int(power))
        if power_temp is None:
            raise ValueError(f"power must be one of {sorted(self.lora_power_dic.keys())}, got {power}")

        rssi_temp = 0x80 if rssi else 0x00

        l_crypt = int(crypt) & 0xFF
        h_crypt = (int(crypt) >> 8) & 0xFF

        # NOTE: This driver doesn't implement lbt/wor in cfg_reg (kept for API compatibility)
        _ = (lbt, wor)

        if not relay:
            self.cfg_reg[3] = high_addr
            self.cfg_reg[4] = low_addr
            self.cfg_reg[5] = net_id_temp
            self.cfg_reg[6] = self.SX126X_UART_BAUDRATE_9600 + air_speed_temp
            self.cfg_reg[7] = buffer_size_temp + power_temp + 0x20
            self.cfg_reg[8] = freq_temp
            self.cfg_reg[9] = 0x43 + rssi_temp
            self.cfg_reg[10] = h_crypt
            self.cfg_reg[11] = l_crypt
        else:
            # relay mode demo defaults
            self.cfg_reg[3] = 0x01
            self.cfg_reg[4] = 0x02
            self.cfg_reg[5] = 0x03
            self.cfg_reg[6] = self.SX126X_UART_BAUDRATE_9600 + air_speed_temp
            self.cfg_reg[7] = buffer_size_temp + power_temp + 0x20
            self.cfg_reg[8] = freq_temp
            self.cfg_reg[9] = 0x03 + rssi_temp
            self.cfg_reg[10] = h_crypt
            self.cfg_reg[11] = l_crypt

        self.ser.flushInput()

        # write config twice (common pattern) and accept if we see any response
        for i in range(2):
            self.ser.write(bytes(self.cfg_reg))
            time.sleep(0.2)
            if self.ser.inWaiting() > 0:
                time.sleep(0.1)
                _ = self.ser.read(self.ser.inWaiting())
                break
            if i == 1:
                raise RuntimeError("LoRa module config failed (no response)")

        GPIO.output(self.M0, GPIO.LOW)
        GPIO.output(self.M1, GPIO.LOW)
        time.sleep(0.1)

    # -------------------------
    # TX helpers
    # -------------------------
    def max_packet_bytes(self) -> int:
        """Configured module packet size in bytes (UART write limit)."""
        return int(getattr(self, "_buffer_size_bytes", 240))

    def max_app_payload_bytes_fixed(self) -> int:
        """Maximum app payload bytes in fixed-transmit mode.

        You write 3 bytes of destination+channel header, so app payload max is:
          packet_size - 3
        """
        return max(0, self.max_packet_bytes() - 3)

    def build_fixed_tx_frame(self, dest_addr: int, payload: bytes, freq_mhz: int | None = None) -> bytes:
        """Build the 3-byte fixed transmit header + payload.

        This matches the module's expected TX format in fixed mode:
          [DEST_H][DEST_L][CHAN] + PAYLOAD

        Where CHAN is (freq_mhz - start_freq). If freq_mhz is None, uses self.freq.
        """
        if isinstance(payload, str):
            payload = payload.encode("utf-8")

        freq_mhz = int(self.freq if freq_mhz is None else freq_mhz)
        if freq_mhz >= 850:
            start = 850
        elif freq_mhz >= 410:
            start = 410
        else:
            raise ValueError("freq_mhz must be in 410–493 or 850–930")

        chan = freq_mhz - start
        if not (0 <= chan <= 255):
            raise ValueError("computed channel offset out of range")

        da = int(dest_addr) & 0xFFFF
        header = bytes([(da >> 8) & 0xFF, da & 0xFF, chan & 0xFF])
        return header + payload

    def send(self, data: bytes | str) -> None:
        """Send bytes over the module UART in ONE write.

        No silent chunking.
        """
        if isinstance(data, str):
            data = data.encode("utf-8")

        max_len = self.max_packet_bytes()
        if len(data) > max_len:
            raise ValueError(f"payload too large for module packet size: {len(data)} > {max_len}")

        GPIO.output(self.M1, GPIO.LOW)
        GPIO.output(self.M0, GPIO.LOW)
        time.sleep(0.02)

        self.ser.write(data)
        time.sleep(0.02 + 0.001 * len(data))

    # -------------------------
    # RX helpers
    # -------------------------
    def recv_packet(self, timeout_s: float = 0.5) -> bytes:
        """Read one best-effort UART burst."""
        t0 = time.time()
        while self.ser.inWaiting() <= 0:
            if (time.time() - t0) >= timeout_s:
                return b""
            time.sleep(0.005)

        time.sleep(0.03)  # settle
        n = self.ser.inWaiting()
        if n <= 0:
            return b""
        return self.ser.read(n)

    def parse_packet(self, r_buff: bytes) -> Tuple[Dict, bytes]:
        """Parse RX bytes into (meta, payload).

        RX format (module-fixed):
          [SRC_H][SRC_L][CHAN][PAYLOAD...][RSSI?]
        """
        if not r_buff or len(r_buff) < 4:
            return {}, b""

        src_addr = (r_buff[0] << 8) | r_buff[1]
        freq_mhz = int(r_buff[2] + self.start_freq)
        payload = r_buff[3:]

        packet_rssi_dbm = None
        if self.rssi and len(payload) >= 1:
            rssi_raw = payload[-1]
            packet_rssi_dbm = -(256 - rssi_raw)
            payload = payload[:-1]

        meta: Dict = {"src_addr": src_addr, "freq_mhz": freq_mhz}
        if packet_rssi_dbm is not None:
            meta["packet_rssi_dbm"] = packet_rssi_dbm
        return meta, payload

    # Legacy demo
    def receive(self) -> None:
        pkt = self.recv_packet(timeout_s=0.5)
        if not pkt:
            return
        meta, payload = self.parse_packet(pkt)
        print(f"RX meta={meta} payload={payload!r}")

    def get_channel_rssi(self) -> None:
        GPIO.output(self.M1, GPIO.LOW)
        GPIO.output(self.M0, GPIO.LOW)
        time.sleep(0.1)
        self.ser.flushInput()
        self.ser.write(bytes([0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x02]))
        time.sleep(0.5)
        re_temp = b""
        if self.ser.inWaiting() > 0:
            time.sleep(0.1)
            re_temp = self.ser.read(self.ser.inWaiting())
        if len(re_temp) >= 4 and re_temp[0] == 0xC1 and re_temp[1] == 0x00 and re_temp[2] == 0x02:
            print("the current noise rssi value: -{0}dBm".format(256 - re_temp[3]))
        else:
            print("receive rssi value fail")
