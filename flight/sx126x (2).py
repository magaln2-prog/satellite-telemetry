"""sx126x.py

Driver helper for Waveshare / Ebyte-style SX126x UART LoRa modules.

This file began as a Waveshare demo and has been modified to be
*application-safe* for newline-delimited JSON telemetry.

Design principles:
  1) **Transport-only**: this driver should move bytes; it should not
     parse JSON or assume message boundaries.
  2) **No silent chunking on TX**: splitting a JSON payload into multiple
     UART writes is a common cause of "JSONDecodeError" on the receiver.
     If a frame is too large, we raise an error so the caller can shrink
     the packet intentionally.
  3) **RX helpers**: provide `recv_packet()` + `parse_packet()` so the
     ground receiver can safely reassemble newline-delimited messages.

NOTE:
  - These UART LoRa modules commonly use a 3-byte header in fixed-address
    mode: [DEST_H][DEST_L][FREQ_OFFSET_or_CHANNEL] + payload.
  - When RSSI is enabled, a final RSSI byte may be appended by the module.
"""

import RPi.GPIO as GPIO
import serial
import time
from typing import Any, Dict, Optional, Tuple

class sx126x:

    M0 = 22
    M1 = 27
    # if the header is 0xC0, then the LoRa register settings dont lost when it poweroff, and 0xC2 will be lost.
    # cfg_reg = [0xC0,0x00,0x09,0x00,0x00,0x00,0x62,0x00,0x17,0x43,0x00,0x00]
    cfg_reg = [0xC0,0x00,0x09,0x00,0x00,0x00,0x62,0x00,0x12,0x43,0x00,0x00]
    get_reg = bytes(12)
    rssi = False
    addr = 65535
    serial_n = ""
    addr_temp = 0

    #
    # start frequence of two lora module
    #
    # E22-400T22S           E22-900T22S
    # 410~493MHz      or    850~930MHz
    start_freq = 850

    #
    # offset between start and end frequence of two lora module
    #
    # E22-400T22S           E22-900T22S
    # 410~493MHz      or    850~930MHz
    offset_freq = 18

    # power = 22
    # air_speed =2400

    SX126X_UART_BAUDRATE_1200 = 0x00
    SX126X_UART_BAUDRATE_2400 = 0x20
    SX126X_UART_BAUDRATE_4800 = 0x40
    SX126X_UART_BAUDRATE_9600 = 0x60
    SX126X_UART_BAUDRATE_19200 = 0x80
    SX126X_UART_BAUDRATE_38400 = 0xA0
    SX126X_UART_BAUDRATE_57600 = 0xC0
    SX126X_UART_BAUDRATE_115200 = 0xE0

    SX126X_PACKAGE_SIZE_240_BYTE = 0x00
    SX126X_PACKAGE_SIZE_128_BYTE = 0x40
    SX126X_PACKAGE_SIZE_64_BYTE = 0x80
    SX126X_PACKAGE_SIZE_32_BYTE = 0xC0

    SX126X_Power_22dBm = 0x00
    SX126X_Power_17dBm = 0x01
    SX126X_Power_13dBm = 0x02
    SX126X_Power_10dBm = 0x03

    lora_air_speed_dic = {
        1200:0x01,
        2400:0x02,
        4800:0x03,
        9600:0x04,
        19200:0x05,
        38400:0x06,
        62500:0x07
    }

    lora_power_dic = {
        22:0x00,
        17:0x01,
        13:0x02,
        10:0x03
    }

    lora_buffer_size_dic = {
        240:SX126X_PACKAGE_SIZE_240_BYTE,
        128:SX126X_PACKAGE_SIZE_128_BYTE,
        64:SX126X_PACKAGE_SIZE_64_BYTE,
        32:SX126X_PACKAGE_SIZE_32_BYTE
    }

    def __init__(self,serial_num,freq,addr,power,rssi,air_speed=2400,\
                 net_id=0,buffer_size = 240,crypt=0,\
                 relay=False,lbt=False,wor=False):
        # -----------------------------
        # User-facing configuration
        # -----------------------------
        self.rssi = rssi
        self.addr = addr
        self.freq = freq
        self.serial_n = serial_num
        self.power = power
        # Keep a copy so callers can compute safe payload sizes.
        # This is the module "package size" configured in REG[7].
        self.buffer_size = buffer_size

        # Initial the GPIO for M0 and M1 Pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.M0,GPIO.OUT)
        GPIO.setup(self.M1,GPIO.OUT)
        GPIO.output(self.M0,GPIO.LOW)
        GPIO.output(self.M1,GPIO.HIGH)

        # The hardware UART of Pi3B+,Pi4B is /dev/ttyS0
        self.ser = serial.Serial(serial_num,9600)
        self.ser.flushInput()
        # Apply module settings (freq/address/air speed/etc.)
        self.set(freq,addr,power,rssi,air_speed,net_id,buffer_size,crypt,relay,lbt,wor)

    def set(self,freq,addr,power,rssi,air_speed=2400,\
            net_id=0,buffer_size = 240,crypt=0,\
            relay=False,lbt=False,wor=False):
        self.send_to = addr
        self.addr = addr
        # We should pull up the M1 pin when sets the module
        GPIO.output(self.M0,GPIO.LOW)
        GPIO.output(self.M1,GPIO.HIGH)
        time.sleep(0.1)

        low_addr = addr & 0xff
        high_addr = addr >> 8 & 0xff
        net_id_temp = net_id & 0xff
        if freq > 850:
            freq_temp = freq - 850
            self.start_freq = 850
            self.offset_freq = freq_temp
        elif freq >410:
            freq_temp = freq - 410
            self.start_freq  = 410
            self.offset_freq = freq_temp

        # Look up "coded" register values from human inputs.
        # If any are missing, fail fast so we don't write nonsense config.
        air_speed_temp = self.lora_air_speed_dic.get(air_speed, None)
        buffer_size_temp = self.lora_buffer_size_dic.get(buffer_size, None)
        power_temp = self.lora_power_dic.get(power, None)

        if air_speed_temp is None:
            raise ValueError(f"Unsupported air_speed={air_speed}. Choose one of {sorted(self.lora_air_speed_dic.keys())}")
        if buffer_size_temp is None:
            raise ValueError(f"Unsupported buffer_size={buffer_size}. Choose one of {sorted(self.lora_buffer_size_dic.keys())}")
        if power_temp is None:
            raise ValueError(f"Unsupported power={power}. Choose one of {sorted(self.lora_power_dic.keys())}")

        # Save validated values on the instance.
        self.freq = freq
        self.addr = addr
        self.power = power
        self.rssi = rssi
        self.buffer_size = buffer_size

        if rssi:
            # enable print rssi value
            rssi_temp = 0x80
        else:
            # disable print rssi value
            rssi_temp = 0x00

        # get crypt
        l_crypt = crypt & 0xff
        h_crypt = crypt >> 8 & 0xff

        if relay==False:
            self.cfg_reg[3] = high_addr
            self.cfg_reg[4] = low_addr
            self.cfg_reg[5] = net_id_temp
            self.cfg_reg[6] = self.SX126X_UART_BAUDRATE_9600 + air_speed_temp
            #
            # it will enable to read noise rssi value when add 0x20 as follow
            #
            self.cfg_reg[7] = buffer_size_temp + power_temp + 0x20
            self.cfg_reg[8] = freq_temp
            #
            # it will output a packet rssi value following received message
            # when enable eighth bit with 06H register(rssi_temp = 0x80)
            #
            self.cfg_reg[9] = 0x43 + rssi_temp
            self.cfg_reg[10] = h_crypt
            self.cfg_reg[11] = l_crypt
        else:
            self.cfg_reg[3] = 0x01
            self.cfg_reg[4] = 0x02
            self.cfg_reg[5] = 0x03
            self.cfg_reg[6] = self.SX126X_UART_BAUDRATE_9600 + air_speed_temp
            #
            # it will enable to read noise rssi value when add 0x20 as follow
            #
            self.cfg_reg[7] = buffer_size_temp + power_temp + 0x20
            self.cfg_reg[8] = freq_temp
            #
            # it will output a packet rssi value following received message
            # when enable eighth bit with 06H register(rssi_temp = 0x80)
            #
            self.cfg_reg[9] = 0x03 + rssi_temp
            self.cfg_reg[10] = h_crypt
            self.cfg_reg[11] = l_crypt
        self.ser.flushInput()

        for i in range(2):
            self.ser.write(bytes(self.cfg_reg))
            r_buff = 0
            time.sleep(0.2)
            if self.ser.inWaiting() > 0:
                time.sleep(0.1)
                r_buff = self.ser.read(self.ser.inWaiting())
                if r_buff[0] == 0xC1:
                    pass
                    # print("parameters setting is :",end='')
                    # for i in self.cfg_reg:
                        # print(hex(i),end=' ')

                    # print('\r\n')
                    # print("parameters return is  :",end='')
                    # for i in r_buff:
                        # print(hex(i),end=' ')
                    # print('\r\n')
                else:
                    pass
                    #print("parameters setting fail :",r_buff)
                break
            else:
                print("setting fail,setting again")
                self.ser.flushInput()
                time.sleep(0.2)
                print('\x1b[1A',end='\r')
                if i == 1:
                    print("setting fail,Press Esc to Exit and run again")
                    # time.sleep(2)
                    # print('\x1b[1A',end='\r')

        GPIO.output(self.M0,GPIO.LOW)
        GPIO.output(self.M1,GPIO.LOW)
        time.sleep(0.1)


    def get_settings(self):
        # Enter setting mode
        GPIO.output(self.M1, GPIO.HIGH)
        time.sleep(0.1)

        # Send command to get current configuration
        self.ser.write(bytes([0xC1, 0x00, 0x09]))
        time.sleep(0.1)

        if self.ser.inWaiting() > 0:
            self.get_reg = self.ser.read(self.ser.inWaiting())

        # Check response
        if len(self.get_reg) >= 10 and self.get_reg[0] == 0xC1 and self.get_reg[2] == 0x09:
            freq_temp = self.get_reg[8]
            addr_temp = (self.get_reg[3] << 8) | self.get_reg[4]
            air_speed_temp = self.get_reg[6] & 0x07
            power_temp = self.get_reg[7] & 0x03

            air_speed_bps = None
            for k, v in self.lora_air_speed_dic.items():
                if v == air_speed_temp:
                    air_speed_bps = k
                    break

            power_dbm = None
            for k, v in self.lora_power_dic.items():
                if v == power_temp:
                    power_dbm = k
                    break

            print(f"Frequency: {self.start_freq + freq_temp}.125 MHz")
            print(f"Node address: {addr_temp}")
            print(f"Air speed: {air_speed_bps} bps")
            print(f"Power: {power_dbm} dBm")
        else:
            print("Failed to read LoRa settings")

        # Exit setting mode
        GPIO.output(self.M1, GPIO.LOW)
        time.sleep(0.1)


#
# the data format like as following
# "node address,frequence,payload"
# "20,868,Hello World"
    def send(self, data):
        """Send *one* UART frame to the radio.

        Telemetry safety rule:
          - Do NOT silently split frames.
            Splitting is exactly what causes partial JSON to arrive at the receiver.
            If the caller wants chunking, they should implement an explicit
            chunk format (message_id/part/total) at the application layer.

        Args:
            data: bytes or str. If str, encoded as UTF-8.

        Raises:
            ValueError: if data exceeds configured buffer_size.
        """
        if isinstance(data, str):
            data = data.encode("utf-8")

        # Ensure we're in normal mode (M0=0, M1=0) for TX.
        GPIO.output(self.M1, GPIO.LOW)
        GPIO.output(self.M0, GPIO.LOW)
        time.sleep(0.02)

        # `buffer_size` is the module packet size. The *entire* UART frame
        # (header + payload) must fit.
        max_len = int(self.max_uart_packet_bytes())
        if len(data) > max_len:
            raise ValueError(
                f"UART frame too large for buffer_size={self.buffer_size}: "
                f"len={len(data)} > max={max_len}. "
                f"Shrink payload or lower telemetry fields."
            )

        self.ser.write(data)

        # Give the module time to transmit. This is a conservative delay.
        # (UART write is fast; RF transmission is slower.)
        time.sleep(0.05 + 0.002 * len(data))


    # ---------------------------------------------------------------------
    # RX helpers (used by the ground receiver)
    # ---------------------------------------------------------------------
    def recv_packet(self, timeout_s: float = 0.5) -> Optional[bytes]:
        """Read one UART "burst" from the module.

        This does NOT guarantee a full application message.
        It just returns whatever bytes arrived within timeout_s.

        Returns:
            bytes if any were received, else None.
        """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            waiting = self.ser.inWaiting()
            if waiting > 0:
                # Small delay lets the UART buffer fill so we read fewer fragments.
                time.sleep(0.02)
                waiting = self.ser.inWaiting()
                return self.ser.read(waiting)
            time.sleep(0.01)
        return None

    def parse_packet(self, pkt: bytes) -> Tuple[Dict[str, Any], bytes]:
        """Parse a raw UART RX packet into metadata + payload.

        Typical RX format from these modules (fixed mode):
            [SRC_H][SRC_L][FREQ_OFFSET] + PAYLOAD + [RSSI?]

        Where:
          - SRC_H/SRC_L are the sender address
          - FREQ_OFFSET is (freq_mhz - start_freq), and start_freq is 850 or 410
          - RSSI byte is appended if RSSI output is enabled

        Returns:
            (meta, payload_bytes)
        """
        meta: Dict[str, Any] = {}
        if not pkt or len(pkt) < 3:
            return meta, b""

        src_addr = (pkt[0] << 8) | pkt[1]
        freq_off = pkt[2]
        freq_mhz = int(self.start_freq + freq_off)

        meta.update({
            "src_addr": src_addr,
            "freq_mhz": freq_mhz,
            "raw_len": len(pkt),
        })

        # If RSSI is enabled, the last byte is RSSI and should not be treated as payload.
        if self.rssi and len(pkt) >= 4:
            # Module reports RSSI as unsigned byte; demo prints -(256 - x)
            rssi_raw = pkt[-1]
            meta["packet_rssi_dbm"] = -(256 - int(rssi_raw))
            payload = pkt[3:-1]
        else:
            payload = pkt[3:]

        return meta, payload

    # ---------------------------------------------------------------------
    # Convenience helpers for the *application* layer
    # ---------------------------------------------------------------------
    def max_uart_packet_bytes(self) -> int:
        """Maximum UART frame bytes allowed by the current buffer_size."""
        return int(self.buffer_size)

    def max_app_payload_bytes(self, header_len: int = 3) -> int:
        """Maximum application payload size, given a header length.

        For fixed-addressing mode, header_len is commonly 3 bytes:
          [DEST_H][DEST_L][FREQ_OFFSET_or_CHANNEL]
        """
        return max(0, int(self.buffer_size) - int(header_len))

    def build_fixed_tx_frame(self, dest_addr: int, freq_off_or_chan: int, payload: bytes) -> bytes:
        """Build a fixed-mode TX frame (3-byte header + payload)."""
        return bytes([
            (dest_addr >> 8) & 0xFF,
            dest_addr & 0xFF,
            freq_off_or_chan & 0xFF,
        ]) + payload

    def receive(self):
        """Legacy demo receive (prints to console).

        For telemetry receivers, prefer:
            pkt = recv_packet(); meta, payload = parse_packet(pkt)
        """
        pkt = self.recv_packet(timeout_s=0.5)
        if not pkt:
            return

        meta, payload = self.parse_packet(pkt)
        if meta:
            print(
                "receive message from node address with frequence\033[1;32m %d,%d.125MHz\033[0m"
                % (meta.get("src_addr", 0), meta.get("freq_mhz", 0)),
                end='\r\n',
                flush=True,
            )
        print("message is " + str(payload), end='\r\n')

        if self.rssi and meta and meta.get("packet_rssi_dbm") is not None:
            print(f"the packet rssi value: {meta['packet_rssi_dbm']}dBm")
            self.get_channel_rssi()

    def get_channel_rssi(self):
        GPIO.output(self.M1,GPIO.LOW)
        GPIO.output(self.M0,GPIO.LOW)
        time.sleep(0.1)
        self.ser.flushInput()
        self.ser.write(bytes([0xC0,0xC1,0xC2,0xC3,0x00,0x02]))
        time.sleep(0.5)
        re_temp = bytes(5)
        if self.ser.inWaiting() > 0:
            time.sleep(0.1)
            re_temp = self.ser.read(self.ser.inWaiting())
        if re_temp[0] == 0xC1 and re_temp[1] == 0x00 and re_temp[2] == 0x02:
            print("the current noise rssi value: -{0}dBm".format(256-re_temp[3]))
            # print("the last receive packet rssi value: -{0}dBm".format(256-re_temp[4]))
        else:
            # pass
            print("receive rssi value fail")
            # print("receive rssi value fail: ",re_temp)