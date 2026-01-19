# protocol_tm.py
import struct

MAGIC = b"TM"
VER = 1

# MAGIC(2), VER(1), MSG_ID(u16), FRAG_IDX(u8), FRAG_TOT(u8), PAYLEN(u8), CRC16(u16)
HDR_FMT = ">2sB H B B B H"
HDR_LEN = struct.calcsize(HDR_FMT)

def crc16_ccitt(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ poly) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

def try_parse_one(buf: bytes):
    """
    Stream parser:
      - Finds MAGIC anywhere (resync)
      - Parses one complete TM frame if available
      - Verifies CRC16
    Returns: (frame_dict, remaining_buf) or (None, buf) if incomplete/invalid.
    frame_dict: {"msg_id":..., "frag_idx":..., "frag_tot":..., "payload":...}
    """
    if len(buf) < HDR_LEN:
        return None, buf

    # Resync to MAGIC
    i = buf.find(MAGIC)
    if i == -1:
        # No magic found; drop buffer
        return None, b""
    if i > 0:
        buf = buf[i:]
        if len(buf) < HDR_LEN:
            return None, buf

    magic, ver, msg_id, frag_idx, frag_tot, paylen, crc = struct.unpack(HDR_FMT, buf[:HDR_LEN])
    if magic != MAGIC or ver != VER:
        # Move forward and resync again next call
        return None, buf[2:]

    need = HDR_LEN + paylen
    if len(buf) < need:
        return None, buf  # incomplete frame

    payload = buf[HDR_LEN:need]

    header_wo_crc = struct.pack(">B H B B B", ver, msg_id, frag_idx, frag_tot, paylen)
    crc_calc = crc16_ccitt(header_wo_crc + payload)
    if crc_calc != crc:
        return None, buf[2:]  # CRC mismatch, resync

    frame = {
        "msg_id": msg_id,
        "frag_idx": frag_idx,
        "frag_tot": frag_tot,
        "payload": payload,
    }
    return frame, buf[need:]
