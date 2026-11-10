"""Export the application's CAN frame records as a CANoe-style ASC trace."""

from datetime import datetime
from typing import Sequence


def format_asc_trace(records: Sequence[tuple]) -> str:
    if not records:
        raise ValueError("There are no CAN frames to export.")

    start = datetime.fromisoformat(records[0][0])
    lines = [
        "date " + start.strftime("%a %b %d %H:%M:%S.%f %p %Y"),
        "base hex timestamps absolute",
        "no internal events logged",
        "// version 8.0.0",
        "Begin Triggerblock",
        "   0.000000 Start of measurement",
    ]
    for record in records:
        (
            host_time,
            direction,
            frame_format,
            arbitration_id,
            dlc,
            data,
            device_time,
        ) = record
        timestamp = datetime.fromisoformat(host_time)
        elapsed = (timestamp - start).total_seconds()
        can_id = arbitration_id.upper()
        if frame_format.startswith("Extended"):
            can_id += "x"
        frame_kind = "r" if frame_format.endswith("remote") else "d"
        direction = {"RX": "Rx", "TX": "Tx"}.get(direction.upper())
        if direction is None:
            raise ValueError(f"Unsupported trace direction: {record[1]!r}.")
        fields = [
            f"{elapsed:10.6f}",
            "1",
            can_id,
            direction,
            frame_kind,
            str(int(dlc)),
        ]
        if frame_kind == "d":
            fields.extend(byte.upper() for byte in data.split())
        lines.append(" ".join(fields))
        if device_time:
            lines.append(f"// SLCAN device timestamp: {device_time.upper()}")
    end_time = datetime.fromisoformat(records[-1][0])
    lines.append(f"{(end_time - start).total_seconds():.6f} End TriggerBlock")
    return "\n".join(lines) + "\n"
