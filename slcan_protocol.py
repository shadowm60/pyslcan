"""Encoding and parsing for the custom SLCAN protocol."""

from dataclasses import dataclass
from typing import Optional


BITRATES = {
    "S0": 10_000,
    "S1": 20_000,
    "S2": 50_000,
    "S3": 100_000,
    "S4": 125_000,
    "S5": 250_000,
    "S6": 500_000,
    "S7": 800_000,
    "S8": 1_000_000,
}


@dataclass(frozen=True)
class CanFrame:
    arbitration_id: int
    extended: bool
    remote: bool
    dlc: int
    data: bytes = b""
    timestamp: Optional[int] = None


def encode_frame(frame: CanFrame) -> str:
    if not 0 <= frame.dlc <= 8:
        raise ValueError("DLC must be between 0 and 8.")
    max_id = 0x1FFFFFFF if frame.extended else 0x7FF
    if not 0 <= frame.arbitration_id <= max_id:
        raise ValueError(
            f"Identifier must be between 0 and 0x{max_id:X} for this frame type."
        )
    if frame.remote:
        if frame.data:
            raise ValueError("Remote frames cannot contain data bytes.")
    elif len(frame.data) != frame.dlc:
        raise ValueError("The number of data bytes must match the DLC.")

    frame_type = (
        "R" if frame.extended else "r"
    ) if frame.remote else ("T" if frame.extended else "t")
    id_width = 8 if frame.extended else 3
    payload = "" if frame.remote else frame.data.hex().upper()
    return f"{frame_type}{frame.arbitration_id:0{id_width}X}{frame.dlc:X}{payload}"


def parse_frame(line: str) -> CanFrame:
    """Parse one received frame, with or without a four-digit timestamp."""
    if not line or line[0] not in "tTrR":
        raise ValueError("Not a CAN frame.")

    extended = line[0] in "TR"
    remote = line[0] in "rR"
    id_width = 8 if extended else 3
    header_end = 1 + id_width + 1
    if len(line) < header_end:
        raise ValueError("Frame is shorter than its identifier and DLC.")

    id_text = line[1 : 1 + id_width]
    dlc_text = line[1 + id_width : header_end]
    if any(char not in "0123456789abcdefABCDEF" for char in id_text + dlc_text):
        raise ValueError("Identifier and DLC must be hexadecimal.")

    arbitration_id = int(id_text, 16)
    dlc = int(dlc_text, 16)
    if dlc > 8:
        raise ValueError("DLC must be between 0 and 8.")
    max_id = 0x1FFFFFFF if extended else 0x7FF
    if arbitration_id > max_id:
        raise ValueError("Identifier is out of range for this frame type.")

    body = line[header_end:]
    expected_data_length = 0 if remote else dlc * 2
    timestamp = None
    if len(body) == expected_data_length + 4:
        timestamp_text = body[-4:]
        if any(char not in "0123456789abcdefABCDEF" for char in timestamp_text):
            raise ValueError("Timestamp must be four hexadecimal digits.")
        timestamp = int(timestamp_text, 16)
        body = body[:-4]
    if len(body) != expected_data_length:
        raise ValueError("Frame payload length does not match the DLC.")
    if any(char not in "0123456789abcdefABCDEF" for char in body):
        raise ValueError("Data bytes must be hexadecimal.")

    return CanFrame(
        arbitration_id=arbitration_id,
        extended=extended,
        remote=remote,
        dlc=dlc,
        data=bytes.fromhex(body),
        timestamp=timestamp,
    )


def is_frame_line(line: str) -> bool:
    return bool(line) and line[0] in "tTrR"


def estimate_can_frame_bits(frame: CanFrame) -> int:
    """Estimate Classical CAN wire bits, including average bit stuffing."""
    arbitration_bits = 32 if frame.extended else 12
    fixed_bits = 67 if frame.extended else 47
    stuffable_bits = 1 + arbitration_bits + 6 + frame.dlc * 8 + 15
    estimated_stuff_bits = (stuffable_bits + 4) // 5
    return fixed_bits + frame.dlc * 8 + estimated_stuff_bits
