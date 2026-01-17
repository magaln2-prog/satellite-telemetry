# This file is used for LoRa and Raspberry pi4B related issues

import RPi.GPIO as GPIO
import serial
import time

# NOTE (patched):
# This driver is a thin UART bridge to the SX126x-based E22 modules.
# The radio/UART stack does NOT guarantee application-message boundaries.
# If you send JSON, you MUST avoid splitting a single JSON object across
# multiple writes unless you also implement reassembly on the receiver.
#
# This patch adds:
#   - safer TX defaults (no implicit chunk-splitting)
#   - RX helpers that can buffer and yield newline-delimited payload lines
#     (useful for JSONL-style telemetry: one JSON object per line)

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
        self.rssi = rssi
        self.addr = addr
        self.freq = freq
        self.serial_n = serial_num
        self.power = power
        self.buffer_size = buffer_size
        # RX reassembly buffer (bytes). Used by recv_* helpers.
        self._rx_buf = b""
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
        self.set(freq,addr,power,rssi,air_speed,net_id,buffer_size,crypt,relay,lbt,wor)

    def set(self,freq,addr,power,rssi,air_speed=2400,\
            net_id=0,buffer_size = 240,crypt=0,\
            relay=False,lbt=False,wor=False):
        # Remember current buffer size for TX safety checks.
        self.buffer_size = buffer_size
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

        air_speed_temp = self.lora_air_speed_dic.get(air_speed,None)
        # if air_speed_temp != None

        buffer_size_temp = self.lora_buffer_size_dic.get(buffer_size,None)
        # if air_speed_temp != None:

        power_temp = self.lora_power_dic.get(power,None)
        #if power_temp != None:

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
        """
        Sends data via LoRa.

        IMPORTANT (patched behavior):
        - By default, this function sends the data as a SINGLE UART write.
          This avoids splitting a JSON object (or any application message)
          across multiple chunks, which is a common cause of JSON decode
          errors on the receiver.
        - If you truly need chunking, call send_chunked(...).

        Args:
            data: str or bytes
        """
        if isinstance(data, str):
            data = data.encode("utf-8")

        # The module payload capacity depends on the configured buffer_size.
        # Some applications also add their own 6-byte addressing header.
        # We don't try to be clever here; we just prevent accidental splits.
        max_packet = int(self.buffer_size) if getattr(self, "buffer_size", None) else 240
        if len(data) > max_packet:
            raise ValueError(
                f"payload too large for configured buffer_size={max_packet}. "
                "Either reduce payload (recommended), increase buffer_size, "
                "or use send_chunked() with an application-level reassembler."
            )

        GPIO.output(self.M1, GPIO.LOW)
        GPIO.output(self.M0, GPIO.LOW)
        time.sleep(0.05)

        self.ser.write(data)
        # wait for radio to finish transmitting (~2ms per byte + small base)
        time.sleep(0.05 + 0.002 * len(data))

        # optional: check RSSI or flush input
        # if self.rssi:
            # self.get_channel_rssi()


    def send_chunked(self, data, max_packet: int = 230, inter_chunk_s: float = 0.1):
        """Send data in multiple UART writes.

        WARNING: This can split application messages (e.g., JSON) across chunks.
        Only use this if the receiver implements reassembly.
        """
        if isinstance(data, str):
            data = data.encode("utf-8")

        offset = 0
        GPIO.output(self.M1, GPIO.LOW)
        GPIO.output(self.M0, GPIO.LOW)
        time.sleep(0.05)

        while offset < len(data):
            chunk = data[offset : offset + max_packet]
            self.ser.write(chunk)
            time.sleep(inter_chunk_s + 0.002 * len(chunk))
            offset += max_packet


    def receive(self):
        """Legacy demo receive (prints to console)."""
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


    def recv_packet(self, timeout_s: float = 0.2) -> bytes:
        """Read one *radio packet* worth of bytes from UART.

        The module emits whole packets, but UART reads can still be partial.
        This helper waits briefly for bytes to arrive, then drains the UART.
        It returns raw bytes (including header and optional RSSI).
        """
        deadline = time.time() + max(0.0, float(timeout_s))
        # wait for first byte
        while self.ser.inWaiting() == 0 and time.time() < deadline:
            time.sleep(0.01)
        if self.ser.inWaiting() == 0:
            return b""
        # small additional wait to let the packet finish arriving
        time.sleep(0.05)
        return self.ser.read(self.ser.inWaiting())


    def parse_packet(self, pkt: bytes):
        """Parse a module packet into (meta, payload_bytes).

        Packet layout (typical Waveshare demo):
          [0..1]=src_addr (big endian), [2]=freq_offset, [3..]=payload (+ optional RSSI byte)
        If RSSI reporting is enabled, the last byte is RSSI.
        """
        if not pkt or len(pkt) < 4:
            return {}, pkt

        src_addr = (pkt[0] << 8) + pkt[1]
        freq_mhz = (pkt[2] + self.start_freq)

        if self.rssi and len(pkt) >= 5:
            payload = pkt[3:-1]
            packet_rssi_dbm = -(256 - pkt[-1])
        else:
            payload = pkt[3:]
            packet_rssi_dbm = None

        meta = {
            "src_addr": src_addr,
            "freq_mhz": freq_mhz,
            "packet_rssi_dbm": packet_rssi_dbm,
        }
        return meta, payload


    def recv_payload_lines(self, timeout_s: float = 0.2, delim: bytes = b"\n"):
        """Yield complete payload lines (newline-delimited) across packets.

        This is the safest way to carry JSON over this driver:
          - sender appends '\n' after each JSON object
          - receiver buffers until '\n' then json.loads(line)
        """
        pkt = self.recv_packet(timeout_s=timeout_s)
        if not pkt:
            return []

        _meta, payload = self.parse_packet(pkt)
        if not payload:
            return []

        self._rx_buf += payload
        out = []
        while True:
            idx = self._rx_buf.find(delim)
            if idx < 0:
                break
            line = self._rx_buf[:idx]
            self._rx_buf = self._rx_buf[idx + len(delim):]
            if line:
                out.append(line)
        return out

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
