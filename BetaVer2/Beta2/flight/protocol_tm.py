# protocol_tm.py
import struct

# -------------------------
# Framing constants
# -------------------------
MAGIC = b"TM"
VER = 1

# Header layout (big-endian):
#   MAGIC(2s) VER(u8) MSG_ID(u16) FRAG_IDX(u8) FRAG_TOT(u8) PAYLEN(u8) CRC16(u16)
HDR_FMT = ">2sB H B B B H"
HDR_LEN = struct.calcsize(HDR_FMT)


def crc16_ccitt(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE."""
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ poly) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_frame(msg_id: int, frag_idx: int, frag_tot: int, payload: bytes) -> bytes:
    """Build one TM frame: [TM][VER][msg_id][frag_idx][frag_tot][paylen][crc][payload]."""
    if not (1 <= frag_tot <= 255):
        raise ValueError("frag_tot out of range")
    if not (0 <= frag_idx < frag_tot):
        raise ValueError("frag_idx out of range")
    if len(payload) > 255:
        raise ValueError("payload too long (max 255)")

    header_wo_crc = struct.pack(">B H B B B", VER, msg_id & 0xFFFF, frag_idx, frag_tot, len(payload))
    crc = crc16_ccitt(header_wo_crc + payload)

    return struct.pack(HDR_FMT, MAGIC, VER, msg_id & 0xFFFF, frag_idx, frag_tot, len(payload), crc) + payload


def try_parse_one(buf: bytes):
    """
    Stream parser:
      - scans for MAGIC
      - waits for full frame
      - verifies CRC
    Returns (frame_dict, remaining_buf) OR (None, new_buf)
    """
    # Need at least header
    if len(buf) < HDR_LEN:
        return None, buf

    # Find MAGIC anywhere (resync)
    idx = buf.find(MAGIC)
    if idx < 0:
        return None, b""
    if idx > 0:
        buf = buf[idx:]

    if len(buf) < HDR_LEN:
        return None, buf

    magic, ver, msg_id, frag_idx, frag_tot, paylen, crc = struct.unpack(HDR_FMT, buf[:HDR_LEN])

    if magic != MAGIC or ver != VER:
        return None, buf[1:]

    need = HDR_LEN + paylen
    if len(buf) < need:
        return None, buf

    payload = buf[HDR_LEN:need]

    header_wo_crc = struct.pack(">B H B B B", ver, msg_id, frag_idx, frag_tot, paylen)
    crc_calc = crc16_ccitt(header_wo_crc + payload)
    if crc_calc != crc:
        return None, buf[1:]

    frame = {"msg_id": msg_id, "frag_idx": frag_idx, "frag_tot": frag_tot, "payload": payload}
    return frame, buf[need:]


# -------------------------
# Telemetry payload codec (binary, shared flight + ground)
# -------------------------
PKT_FAST = 0x01  # frequent updates
PKT_FULL = 0x02  # full state

# Scaling:
# temp_c: int16 (°C * 100)
# rh:     uint16 (% * 10)
# p_hpa:  uint16 (hPa * 10)
# lux:    uint32
# uvi:    uint16 (UVI * 10)  (we map your available UV value into this slot)
# cpu_c:  int16 (°C * 10)
# bv:     uint16 (V * 1000)
# bp:     uint8  (%)
# flags:  uint8  (bitfield)
# ts:     uint32 (unix seconds)

_FAST_FMT = ">B H I h H H I H"
_FULL_FMT = ">B H I h H H I H h H B B"


def _clamp_int(v: int, lo: int, hi: int) -> int:
    return lo if v < lo else hi if v > hi else v


def pack_fast(seq: int, ts_unix: int, t_c, rh, p_hpa, lux, uvi) -> bytes:
    t = int(round((t_c or 0.0) * 100))
    h = int(round((rh or 0.0) * 10))
    p = int(round((p_hpa or 0.0) * 10))
    lx = int(round(lux or 0.0))
    uv = int(round((uvi or 0.0) * 10))

    t = _clamp_int(t, -32768, 32767)
    h = _clamp_int(h, 0, 65535)
    p = _clamp_int(p, 0, 65535)
    lx = _clamp_int(lx, 0, 0xFFFFFFFF)
    uv = _clamp_int(uv, 0, 65535)

    return struct.pack(_FAST_FMT, PKT_FAST, seq & 0xFFFF, ts_unix & 0xFFFFFFFF, t, h, p, lx, uv)


def pack_full(seq: int, ts_unix: int, t_c, rh, p_hpa, lux, uvi, cpu_c, bv, bp, flags: int) -> bytes:
    t = int(round((t_c or 0.0) * 100))
    h = int(round((rh or 0.0) * 10))
    p = int(round((p_hpa or 0.0) * 10))
    lx = int(round(lux or 0.0))
    uv = int(round((uvi or 0.0) * 10))
    cpu = int(round((cpu_c or 0.0) * 10))
    bmv = int(round((bv or 0.0) * 1000))
    bpc = int(round(bp or 0.0))
    flg = int(flags or 0)

    t = _clamp_int(t, -32768, 32767)
    h = _clamp_int(h, 0, 65535)
    p = _clamp_int(p, 0, 65535)
    lx = _clamp_int(lx, 0, 0xFFFFFFFF)
    uv = _clamp_int(uv, 0, 65535)
    cpu = _clamp_int(cpu, -32768, 32767)
    bmv = _clamp_int(bmv, 0, 65535)
    bpc = _clamp_int(bpc, 0, 255)
    flg = _clamp_int(flg, 0, 255)

    return struct.pack(_FULL_FMT, PKT_FULL, seq & 0xFFFF, ts_unix & 0xFFFFFFFF, t, h, p, lx, uv, cpu, bmv, bpc, flg)


def unpack_payload(blob: bytes):
    """Decode a complete reassembled payload. Returns (pkt_type, fields_dict) or (None, None)."""
    if not blob:
        return None, None
    pkt_type = blob[0]

    try:
        if pkt_type == PKT_FAST:
            _, seq, ts, t, h, p, lx, uv = struct.unpack(_FAST_FMT, blob)
            return PKT_FAST, {
                "seq": int(seq),
                "ts_unix": int(ts),
                "t_c": t / 100.0,
                "rh": h / 10.0,
                "p_hpa": p / 10.0,
                "lux": float(lx),
                "uvi": uv / 10.0,
            }

        if pkt_type == PKT_FULL:
            _, seq, ts, t, h, p, lx, uv, cpu, bmv, bpc, flg = struct.unpack(_FULL_FMT, blob)
            return PKT_FULL, {
                "seq": int(seq),
                "ts_unix": int(ts),
                "t_c": t / 100.0,
                "rh": h / 10.0,
                "p_hpa": p / 10.0,
                "lux": float(lx),
                "uvi": uv / 10.0,
                "cpu_c": cpu / 10.0,
                "bv": bmv / 1000.0,
                "bp": float(bpc),
                "pwr_flags": int(flg),
            }
    except Exception:
        return None, None

    return None, None
