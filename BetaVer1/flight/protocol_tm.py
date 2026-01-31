# protocol_tm.py
import struct

# -------------------------
# Framing constants
# -------------------------
# 2-byte "magic" prefix used to find frame boundaries in a raw byte stream.
# If the receiver gets out of sync, it can scan for this marker and realign.
MAGIC = b"TM"

# Protocol version for forward compatibility.
# If you change the header layout or meaning, bump this.
VER = 1

# Header layout (big-endian, network byte order):
#   MAGIC(2s)   : fixed bytes b"TM"
#   VER (u8)    : protocol version
#   MSG_ID(u16) : message sequence / ID (wraps at 65535)
#   FRAG_IDX(u8): fragment index within the message [0 .. FRAG_TOT-1]
#   FRAG_TOT(u8): total fragments in this message [1 .. 255]
#   PAYLEN(u8)  : payload length in bytes [0 .. 255]
#   CRC16(u16)  : CRC-16/CCITT over (VER..PAYLEN + payload) (NOT including MAGIC)
#
# Note: CRC covering the header fields + payload lets us detect corruption and also
# reject false-positive MAGIC matches in random data.
HDR_FMT = ">2sB H B B B H"
HDR_LEN = struct.calcsize(HDR_FMT)

def crc16_ccitt(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    """
    Compute CRC-16/CCITT-FALSE style checksum.

    - poly: 0x1021 is the standard CCITT polynomial
    - init: 0xFFFF is a common initial value (CCITT-FALSE)

    We process one byte at a time, XOR it into the high byte of the CRC,
    then shift through 8 bits, applying the polynomial when the top bit is set.

    Returns:
        16-bit integer CRC (0..65535)
    """
    crc = init
    for b in data:
        # Bring the next input byte into the top 8 bits of the CRC.
        crc ^= (b << 8)
        for _ in range(8):
            # If the MSB is set, shift and XOR polynomial; else just shift.
            crc = ((crc << 1) ^ poly) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF  # keep CRC 16-bit
    return crc

def build_frame(msg_id: int, frag_idx: int, frag_tot: int, payload: bytes) -> bytes:
    """
    Build a single TM frame:
      [MAGIC 'TM'][VER][msg_id][frag_idx][frag_tot][paylen][crc16][payload]

    This is meant for transport over an unreliable / chunked link (e.g., LoRa UART),
    where you might receive partial bytes and need resynchronization.

    Args:
        msg_id:     0..65535 message identifier (often a sequence number)
        frag_idx:   fragment index for this message (0-based)
        frag_tot:   total fragments for this message (1..255)
        payload:    fragment payload (<=255 bytes)

    Returns:
        Complete encoded frame as bytes.
    """
    # Validate fragmentation bounds:
    # - frag_tot must be at least 1
    # - frag_idx must be a valid index into that count
    if not (1 <= frag_tot <= 255):
        raise ValueError("frag_tot out of range")
    if not (0 <= frag_idx < frag_tot):
        raise ValueError("frag_idx out of range")

    # PAYLEN is stored as a u8, so payload length must fit in 0..255.
    if len(payload) > 255:
        raise ValueError("payload too long (max 255)")

    # Build the header fields (excluding MAGIC and excluding CRC) for CRC coverage.
    # We include VER..PAYLEN so that a corrupted header is detected too.
    header_wo_crc = struct.pack(
        ">B H B B B",
        VER,
        msg_id & 0xFFFF,   # force into u16 range
        frag_idx,
        frag_tot,
        len(payload),
    )

    # Compute CRC over header (without MAGIC, without CRC field) + payload.
    crc = crc16_ccitt(header_wo_crc + payload)

    # Pack full header including MAGIC and CRC, then append payload.
    return (
        struct.pack(
            HDR_FMT,
            MAGIC,
            VER,
            msg_id & 0xFFFF,
            frag_idx,
            frag_tot,
            len(payload),
            crc,
        )
        + payload
    )

def try_parse_one(buf: bytes):
    """
    Stream parser for a continuous byte buffer.

    Behavior:
      - Searches for MAGIC anywhere in the buffer (resync in noisy streams).
      - If a full header is present, unpacks it.
      - If the full payload isn't present yet, returns (None, buf) to wait for more bytes.
      - Verifies CRC16; if CRC fails, slides forward by 1 byte and keeps scanning next call.

    Returns:
        (frame_dict, remaining_buf)  if a valid complete frame is parsed
        (None, buf)                 if incomplete or not yet parseable
        (None, b"")                 if no MAGIC found at all (drop all as junk)

    frame_dict:
        {
          "msg_id": int,
          "frag_idx": int,
          "frag_tot": int,
          "payload": bytes
        }
    """
    # Not enough bytes to even read a header yet.
    if len(buf) < HDR_LEN:
        return None, buf

    # -------------------------
    # Resync to MAGIC
    # -------------------------
    # Find the start of the next possible frame.
    i = buf.find(MAGIC)
    if i == -1:
        # No frame start marker at all -> discard buffer (all junk).
        return None, b""
    if i > 0:
        # Drop junk bytes before MAGIC, keep from MAGIC onward.
        buf = buf[i:]
        if len(buf) < HDR_LEN:
            # Now we have MAGIC but still not enough for the full header.
            return None, buf

    # -------------------------
    # Unpack the header
    # -------------------------
    magic, ver, msg_id, frag_idx, frag_tot, paylen, crc = struct.unpack(
        HDR_FMT, buf[:HDR_LEN]
    )

    # Quick reject:
    # - magic mismatch shouldn't happen here often, but keep it defensive
    # - version mismatch allows future protocol versions to be ignored cleanly
    if magic != MAGIC or ver != VER:
        # Slide one byte forward and try again next call.
        return None, buf[1:]

    # Total bytes needed to have a complete frame in the buffer.
    need = HDR_LEN + paylen
    if len(buf) < need:
        # Header is valid but payload hasn't fully arrived yet.
        return None, buf

    # Extract payload bytes for this frame.
    payload = buf[HDR_LEN:need]

    # -------------------------
    # CRC verification
    # -------------------------
    # Rebuild the CRC-covered header portion (same fields, same order).
    header_wo_crc = struct.pack(">B H B B B", ver, msg_id, frag_idx, frag_tot, paylen)

    # CRC is computed over VER..PAYLEN + payload (not including MAGIC).
    crc_calc = crc16_ccitt(header_wo_crc + payload)
    if crc_calc != crc:
        # Corruption or false MAGIC hit. Slide forward by 1 and keep searching.
        return None, buf[1:]

    # If we reach here, we have a valid frame.
    frame = {
        "msg_id": msg_id,
        "frag_idx": frag_idx,
        "frag_tot": frag_tot,
        "payload": payload,
    }

    # Return frame + remaining bytes after this frame (could contain next frames).
    return frame, buf[need:]
