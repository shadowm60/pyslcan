"""OBD-II Mode 01 ECU simulator and JSON configuration."""

import json
import math
import re
import time
from pathlib import Path
from typing import List, Optional, Tuple

from slcan_protocol import CanFrame


PID_DEFINITIONS = {
    0x01: ("Monitor status since DTC clear", "", "status"),
    0x04: ("Calculated engine load", "%", "load"),
    0x05: ("Engine coolant temperature", "C", "coolant"),
    0x0C: ("Engine RPM", "rpm", "rpm"),
    0x0D: ("Vehicle speed", "km/h", "speed"),
    0x0F: ("Intake air temperature", "C", "intake_temp"),
    0x11: ("Throttle position", "%", "throttle"),
    0x1F: ("Run time since engine start", "s", "runtime"),
    0x2F: ("Fuel tank level", "%", "fuel"),
    0x42: ("Control module voltage", "V", "voltage"),
    0x46: ("Ambient air temperature", "C", "ambient_temp"),
    0x5C: ("Engine oil temperature", "C", "oil_temp"),
}

# Common standardized Mode 01 PIDs not currently modeled by the simulator's
# built-in signal encoders. These can be served as raw bytes.
PID_CATALOG = {
    0x02: {
        "name": "Freeze frame DTC",
        "description": "Reports the DTC stored with the freeze-frame data.",
        "measurement": "Stored diagnostic trouble code",
        "unit": "",
        "range_description": "Two-byte DTC encoding",
        "default_raw_bytes": "00 00",
    },
    0x03: {
        "name": "Fuel system status",
        "description": "Reports the operating status of fuel systems 1 and 2.",
        "measurement": "Fuel system status flags",
        "unit": "",
        "range_description": "Two status bytes",
        "default_raw_bytes": "01 01",
    },
    0x06: {
        "name": "Short-term fuel trim, bank 1",
        "description": "Short-term fuel trim for bank 1.",
        "measurement": "Fuel trim",
        "unit": "%",
        "minimum": -100,
        "maximum": 99.2,
        "default_raw_bytes": "80",
    },
    0x07: {
        "name": "Long-term fuel trim, bank 1",
        "description": "Long-term fuel trim for bank 1.",
        "measurement": "Fuel trim",
        "unit": "%",
        "minimum": -100,
        "maximum": 99.2,
        "default_raw_bytes": "80",
    },
    0x08: {
        "name": "Short-term fuel trim, bank 2",
        "description": "Short-term fuel trim for bank 2.",
        "measurement": "Fuel trim",
        "unit": "%",
        "minimum": -100,
        "maximum": 99.2,
        "default_raw_bytes": "80",
    },
    0x09: {
        "name": "Long-term fuel trim, bank 2",
        "description": "Long-term fuel trim for bank 2.",
        "measurement": "Fuel trim",
        "unit": "%",
        "minimum": -100,
        "maximum": 99.2,
        "default_raw_bytes": "80",
    },
    0x0A: {
        "name": "Fuel pressure",
        "description": "Fuel pressure (gauge).",
        "measurement": "Fuel pressure",
        "unit": "kPa",
        "minimum": 0,
        "maximum": 765,
        "default_raw_bytes": "00",
    },
    0x0B: {
        "name": "Intake manifold pressure",
        "description": "Absolute pressure in the intake manifold.",
        "measurement": "Manifold pressure",
        "unit": "kPa",
        "minimum": 0,
        "maximum": 255,
        "default_raw_bytes": "00",
    },
    0x0E: {
        "name": "Timing advance",
        "description": "Ignition timing advance relative to top dead center.",
        "measurement": "Timing advance",
        "unit": "degrees",
        "minimum": -64,
        "maximum": 63.5,
        "default_raw_bytes": "80",
    },
    0x10: {
        "name": "MAF air flow rate",
        "description": "Mass air flow rate.",
        "measurement": "Air flow",
        "unit": "g/s",
        "minimum": 0,
        "maximum": 655.35,
        "default_raw_bytes": "00 00",
    },
    0x12: {
        "name": "Commanded secondary air status",
        "description": "Reports the commanded secondary-air system status.",
        "measurement": "Secondary-air system status",
        "unit": "",
        "range_description": "Status code",
        "default_raw_bytes": "04",
    },
    0x13: {
        "name": "Oxygen sensors present",
        "description": "Bit mask indicating which oxygen sensors are present.",
        "measurement": "Oxygen sensor availability",
        "unit": "",
        "range_description": "Eight-bit sensor mask",
        "default_raw_bytes": "00",
    },
    0x1C: {
        "name": "OBD standards supported",
        "description": "Identifies the OBD standard supported by the vehicle.",
        "measurement": "OBD standard identifier",
        "unit": "",
        "range_description": "Standard code",
        "default_raw_bytes": "06",
    },
    0x21: {
        "name": "Distance traveled with MIL on",
        "description": "Distance traveled while the malfunction indicator lamp is on.",
        "measurement": "Distance",
        "unit": "km",
        "minimum": 0,
        "maximum": 65535,
        "default_raw_bytes": "00 00",
    },
    0x31: {
        "name": "Distance since DTCs cleared",
        "description": "Distance traveled since diagnostic trouble codes were cleared.",
        "measurement": "Distance",
        "unit": "km",
        "minimum": 0,
        "maximum": 65535,
        "default_raw_bytes": "00 00",
    },
    0x33: {
        "name": "Barometric pressure",
        "description": "Absolute barometric pressure.",
        "measurement": "Atmospheric pressure",
        "unit": "kPa",
        "minimum": 0,
        "maximum": 255,
        "default_raw_bytes": "00",
    },
}

DEFAULT_REQUEST_IDS = (0x7DF, 0x7E0)
DEFAULT_RESPONSE_ID = 0x7E8
PID_BITMAP_PAGES = tuple(range(0, 0x100, 0x20))
DEFAULT_DTC_CONFIG = (
    {"code": "P0133", "description": "O2 sensor circuit slow response", "active": False},
    {"code": "P0300", "description": "Random/multiple cylinder misfire detected", "active": False},
    {"code": "P0420", "description": "Catalyst system efficiency below threshold", "active": False},
)

DEFAULT_SWEEPS = {
    0x04: {"minimum": 15, "maximum": 75, "step": 3, "interval_s": 0.5},
    0x05: {"minimum": 70, "maximum": 100, "step": 2, "interval_s": 1.0},
    0x0C: {"minimum": 800, "maximum": 4500, "step": 200, "interval_s": 0.25},
    0x0D: {"minimum": 0, "maximum": 120, "step": 5, "interval_s": 0.4},
    0x0F: {"minimum": 20, "maximum": 45, "step": 1, "interval_s": 2.0},
    0x11: {"minimum": 0, "maximum": 80, "step": 4, "interval_s": 0.35},
    0x2F: {"minimum": 25, "maximum": 90, "step": 2, "interval_s": 3.0},
    0x42: {"minimum": 12000, "maximum": 14400, "step": 100, "interval_s": 1.0},
    0x46: {"minimum": 10, "maximum": 30, "step": 1, "interval_s": 5.0},
    0x5C: {"minimum": 75, "maximum": 115, "step": 2, "interval_s": 2.0},
}


class SimulatorConfigError(ValueError):
    pass


def default_config() -> dict:
    return {
        "version": 1,
        "request_ids": [f"0x{value:03X}" for value in DEFAULT_REQUEST_IDS],
        "response_id": f"0x{DEFAULT_RESPONSE_ID:03X}",
        "pids": {
            f"{pid:02X}": {
                "supported": True,
                **(
                    {"sweep": dict(DEFAULT_SWEEPS[pid])}
                    if pid in DEFAULT_SWEEPS
                    else {}
                ),
            }
            for pid in PID_DEFINITIONS
        },
        "dtcs": [dict(dtc) for dtc in DEFAULT_DTC_CONFIG],
    }


def _parse_can_id(value, field_name: str) -> int:
    if isinstance(value, bool):
        raise SimulatorConfigError(f"{field_name} must be a hexadecimal CAN ID.")
    try:
        result = int(value, 16) if isinstance(value, str) else int(value)
    except (TypeError, ValueError) as exc:
        raise SimulatorConfigError(
            f"{field_name} must be a hexadecimal CAN ID."
        ) from exc
    if not 0 <= result <= 0x7FF:
        raise SimulatorConfigError(f"{field_name} must be an 11-bit CAN ID.")
    return result


def _parse_raw_bytes(value, field_name: str) -> bytes:
    if not isinstance(value, str):
        raise SimulatorConfigError(f"{field_name} must be hexadecimal bytes.")
    parts = value.replace(",", " ").split()
    if not 1 <= len(parts) <= 5 or any(
        not re.fullmatch(r"[0-9A-Fa-f]{2}", part) for part in parts
    ):
        raise SimulatorConfigError(
            f"{field_name} must contain one to five two-digit hexadecimal bytes."
        )
    return bytes(int(part, 16) for part in parts)


def validate_config(config: dict) -> dict:
    if not isinstance(config, dict):
        raise SimulatorConfigError("The ECU simulator JSON root must be an object.")
    request_ids = config.get("request_ids")
    if not isinstance(request_ids, list) or not request_ids:
        raise SimulatorConfigError("'request_ids' must be a non-empty array.")
    parsed_request_ids = [
        _parse_can_id(value, f"request_ids[{index}]")
        for index, value in enumerate(request_ids)
    ]
    if len(set(parsed_request_ids)) != len(parsed_request_ids):
        raise SimulatorConfigError("'request_ids' cannot contain duplicates.")
    response_id = _parse_can_id(config.get("response_id"), "response_id")
    if response_id in parsed_request_ids:
        raise SimulatorConfigError("response_id must differ from every request ID.")

    pids = config.get("pids")
    if not isinstance(pids, dict):
        raise SimulatorConfigError("'pids' must be an object keyed by PID hex.")
    canonical_pids = {}
    for key, entry in pids.items():
        if not isinstance(key, str) or not re.fullmatch(r"[0-9A-Fa-f]{2}", key):
            raise SimulatorConfigError("PID keys must be two-digit hexadecimal.")
        canonical_key = key.upper()
        if canonical_key in canonical_pids:
            raise SimulatorConfigError(f"Duplicate PID key: {canonical_key}.")
        canonical_pids[canonical_key] = entry
    pids = canonical_pids
    normalized_pids = {}
    built_in_pid_keys = {f"{pid:02X}" for pid in PID_DEFINITIONS}
    for pid, (_name, _unit, _kind) in PID_DEFINITIONS.items():
        key = f"{pid:02X}"
        entry = pids.get(key, {"supported": False})
        if not isinstance(entry, dict) or not isinstance(entry.get("supported"), bool):
            raise SimulatorConfigError(
                f"pids.{key}.supported must be true or false."
            )
        normalized = {"supported": entry["supported"]}
        default_algorithm = "sweep" if pid in DEFAULT_SWEEPS else "standard"
        algorithm = entry.get("algorithm", default_algorithm)
        allowed_algorithms = ("sweep", "raw") if pid in DEFAULT_SWEEPS else ("standard", "raw")
        if algorithm not in allowed_algorithms:
            raise SimulatorConfigError(
                f"pids.{key}.algorithm must be one of {', '.join(allowed_algorithms)}."
            )
        normalized["algorithm"] = algorithm
        if algorithm == "raw":
            raw_bytes = _parse_raw_bytes(
                entry.get("raw_bytes"), f"pids.{key}.raw_bytes"
            )
            normalized["raw_bytes"] = " ".join(f"{byte:02X}" for byte in raw_bytes)
        if pid in DEFAULT_SWEEPS:
            sweep = entry.get("sweep", DEFAULT_SWEEPS[pid])
            if not isinstance(sweep, dict):
                raise SimulatorConfigError(f"pids.{key}.sweep must be an object.")
            values = {}
            for field in ("minimum", "maximum", "step", "interval_s"):
                try:
                    values[field] = float(sweep[field])
                except (KeyError, TypeError, ValueError) as exc:
                    raise SimulatorConfigError(
                        f"pids.{key}.sweep.{field} must be numeric."
                    ) from exc
            if (
                (values["minimum"] < 0 and pid not in (0x05, 0x0F, 0x46, 0x5C))
                or values["maximum"] <= values["minimum"]
                or values["step"] <= 0
                or values["interval_s"] <= 0
                or not all(math.isfinite(value) for value in values.values())
            ):
                raise SimulatorConfigError(
                    f"pids.{key}.sweep requires maximum > minimum and positive step/interval."
                )
            limits = {
                0x04: (0, 100),
                0x05: (-40, 215),
                0x0C: (0, 16383),
                0x0D: (0, 255),
                0x0F: (-40, 215),
                0x11: (0, 100),
                0x2F: (0, 100),
                0x42: (0, 65535),
                0x46: (-40, 215),
                0x5C: (-40, 215),
            }
            lower, upper = limits[pid]
            if values["minimum"] < lower or values["maximum"] > upper:
                raise SimulatorConfigError(
                    f"pids.{key}.sweep values must be between {lower} and {upper}."
                )
            normalized["sweep"] = values
        normalized_pids[key] = normalized

    for key, entry in pids.items():
        pid = int(key, 16)
        canonical_key = f"{pid:02X}"
        if canonical_key in built_in_pid_keys:
            continue
        if pid == 0 or pid in PID_BITMAP_PAGES:
            raise SimulatorConfigError(
                f"PID 0x{pid:02X} is reserved for a supported-PID bitmap."
            )
        if not isinstance(entry, dict) or not isinstance(entry.get("supported"), bool):
            raise SimulatorConfigError(
                f"pids.{canonical_key}.supported must be true or false."
            )
        name = entry.get("name")
        unit = entry.get("unit", "")
        if not isinstance(name, str) or not name.strip():
            raise SimulatorConfigError(f"pids.{canonical_key}.name must be text.")
        if not isinstance(unit, str):
            raise SimulatorConfigError(f"pids.{canonical_key}.unit must be text.")
        if entry.get("algorithm") != "raw":
            raise SimulatorConfigError(
                f"pids.{canonical_key}.algorithm must be 'raw' for custom PIDs."
            )
        raw_bytes = _parse_raw_bytes(
            entry.get("raw_bytes"), f"pids.{canonical_key}.raw_bytes"
        )
        normalized = {
            "supported": entry["supported"],
            "name": name.strip(),
            "unit": unit.strip(),
            "algorithm": "raw",
            "raw_bytes": " ".join(f"{byte:02X}" for byte in raw_bytes),
        }
        service = entry.get("service")
        if service is not None:
            if service != "01":
                raise SimulatorConfigError(
                    f"pids.{canonical_key}.service must be '01'."
                )
            normalized["service"] = service
        for field in ("description", "measurement", "range_description"):
            value = entry.get(field)
            if value is not None:
                if not isinstance(value, str):
                    raise SimulatorConfigError(
                        f"pids.{canonical_key}.{field} must be text."
                    )
                normalized[field] = value
        if "default_raw_bytes" in entry:
            default_raw = _parse_raw_bytes(
                entry["default_raw_bytes"],
                f"pids.{canonical_key}.default_raw_bytes",
            )
            normalized["default_raw_bytes"] = " ".join(
                f"{byte:02X}" for byte in default_raw
            )
        if "minimum" in entry or "maximum" in entry:
            if isinstance(entry.get("minimum"), bool) or isinstance(
                entry.get("maximum"), bool
            ):
                raise SimulatorConfigError(
                    f"pids.{canonical_key}.minimum and maximum must be numeric."
                )
            try:
                minimum = float(entry["minimum"])
                maximum = float(entry["maximum"])
            except (KeyError, TypeError, ValueError) as exc:
                raise SimulatorConfigError(
                    f"pids.{canonical_key}.minimum and maximum must be numeric."
                ) from exc
            if not math.isfinite(minimum) or not math.isfinite(maximum) or minimum > maximum:
                raise SimulatorConfigError(
                    f"pids.{canonical_key}.minimum must be finite and no greater than maximum."
                )
            normalized["minimum"] = minimum
            normalized["maximum"] = maximum
        normalized_pids[canonical_key] = normalized

    dtcs = config.get("dtcs", [])
    if not isinstance(dtcs, list):
        raise SimulatorConfigError("'dtcs' must be an array.")
    normalized_dtcs = []
    seen_codes = set()
    for index, dtc in enumerate(dtcs):
        if not isinstance(dtc, dict):
            raise SimulatorConfigError(f"dtcs[{index}] must be an object.")
        code = dtc.get("code")
        if not isinstance(code, str) or not re.fullmatch(r"[PCBU][0-3][0-9A-Fa-f]{3}", code):
            raise SimulatorConfigError(
                f"dtcs[{index}].code must be a five-character OBD-II DTC."
            )
        code = code.upper()
        if code in seen_codes:
            raise SimulatorConfigError(f"dtcs[{index}].code duplicates {code}.")
        seen_codes.add(code)
        if not isinstance(dtc.get("active", False), bool):
            raise SimulatorConfigError(f"dtcs[{index}].active must be true or false.")
        description = dtc.get("description", "")
        if not isinstance(description, str):
            raise SimulatorConfigError(f"dtcs[{index}].description must be text.")
        normalized_dtcs.append(
            {"code": code, "description": description, "active": dtc.get("active", False)}
        )
    if sum(dtc["active"] for dtc in normalized_dtcs) > 3:
        raise SimulatorConfigError(
            "At most three DTCs can be active because Mode 03 single-frame responses "
            "hold three codes."
        )

    mil_on = config.get("mil_on", False)
    if not isinstance(mil_on, bool):
        raise SimulatorConfigError("'mil_on' must be true or false.")

    return {
        "version": 1,
        "request_ids": parsed_request_ids,
        "response_id": response_id,
        "pids": normalized_pids,
        "dtcs": normalized_dtcs,
        "mil_on": mil_on,
    }


def load_config(path: Path) -> dict:
    try:
        with path.open("r", encoding="utf-8") as source:
            config = json.load(source)
    except (OSError, json.JSONDecodeError) as exc:
        raise SimulatorConfigError(f"Could not read ECU config '{path}': {exc}") from exc
    return validate_config(config)


def save_config(path: Path, config: dict) -> dict:
    normalized = validate_config(config)
    serializable = {
        "version": normalized["version"],
        "request_ids": [f"0x{value:03X}" for value in normalized["request_ids"]],
        "response_id": f"0x{normalized['response_id']:03X}",
        "pids": normalized["pids"],
        "dtcs": normalized["dtcs"],
        "mil_on": normalized["mil_on"],
    }
    try:
        with path.open("w", encoding="utf-8", newline="\n") as output:
            json.dump(serializable, output, indent=2)
            output.write("\n")
    except OSError as exc:
        raise SimulatorConfigError(f"Could not write ECU config '{path}': {exc}") from exc
    return normalized


class EcuSimulator:
    def __init__(self, config: dict, clock=time.monotonic) -> None:
        self.config = validate_config(config)
        self._clock = clock
        self._started_at = clock()

    @property
    def request_ids(self) -> Tuple[int, ...]:
        return tuple(self.config["request_ids"])

    @property
    def response_id(self) -> int:
        return self.config["response_id"]

    def is_supported(self, pid: int) -> bool:
        if pid == 0x00:
            return True
        if pid in PID_BITMAP_PAGES:
            return any(
                entry["supported"] and int(key, 16) > pid
                for key, entry in self.config["pids"].items()
            )
        entry = self.config["pids"].get(f"{pid:02X}")
        return bool(entry and entry["supported"])

    def update_config(self, config: dict, reset_clock: bool = False) -> None:
        self.config = validate_config(config)
        if reset_clock:
            self._started_at = self._clock()

    def reset_clock(self) -> None:
        self._started_at = self._clock()

    def current_value(self, pid: int) -> Tuple[str, bytes]:
        entry = self.config["pids"].get(f"{pid:02X}")
        if entry is None:
            raise ValueError(f"PID 0x{pid:02X} is not implemented.")
        if not self.is_supported(pid):
            return "Unsupported", b""

        algorithm = entry.get(
            "algorithm", "sweep" if pid in DEFAULT_SWEEPS else "standard"
        )
        if algorithm == "raw":
            raw = bytes.fromhex(entry["raw_bytes"])
            return f"Raw bytes {raw.hex(' ').upper()}", raw

        name, unit, kind = PID_DEFINITIONS[pid]
        if kind == "status":
            active_dtcs = sum(dtc["active"] for dtc in self.config["dtcs"])
            status = (0x80 if self.config["mil_on"] else 0) | active_dtcs
            raw = bytes((status, 0, 0, 0))
            return f"MIL {'on' if self.config['mil_on'] else 'off'}; {active_dtcs} DTCs", raw

        if kind == "runtime":
            seconds = min(int(self._clock() - self._started_at), 0xFFFF)
            raw = seconds.to_bytes(2, "big")
            return f"{seconds} {unit}", raw

        value = self._value(pid)
        if kind == "load":
            raw = bytes((round(value * 255 / 100),))
            display = f"{value:.0f} {unit}"
        elif kind in ("coolant", "intake_temp", "ambient_temp", "oil_temp"):
            raw = bytes((int(value) + 40,))
            display = f"{int(value)} {unit}"
        elif kind == "rpm":
            encoded = int(value) * 4
            raw = encoded.to_bytes(2, "big")
            display = f"{int(value)} {unit}"
        elif kind == "speed":
            raw = bytes((int(value),))
            display = f"{int(value)} {unit}"
        elif kind == "throttle":
            raw = bytes((round(value * 255 / 100),))
            display = f"{value:.0f} {unit}"
        elif kind == "fuel":
            raw = bytes((round(value * 255 / 100),))
            display = f"{value:.0f} {unit}"
        elif kind == "voltage":
            raw = int(value).to_bytes(2, "big")
            display = f"{value / 1000:.2f} V"
        else:
            raise ValueError(f"PID 0x{pid:02X} has no encoder.")
        return display, raw

    def _value(self, pid: int) -> float:
        spec = self.config["pids"][f"{pid:02X}"]["sweep"]
        minimum = spec["minimum"]
        maximum = spec["maximum"]
        step = spec["step"]
        interval = spec["interval_s"]
        steps = max(1, math.ceil((maximum - minimum) / step))
        phase = int((self._clock() - self._started_at) / interval) % (steps * 2)
        position = phase if phase <= steps else steps * 2 - phase
        return min(minimum + position * step, maximum)

    def supported_bitmap(self, page: int) -> bytes:
        if page not in PID_BITMAP_PAGES:
            raise ValueError("PID bitmap page must be a multiple of 0x20 from 0x00 to 0xE0.")
        bitmap = 0
        for offset in range(1, 0x20 + 1):
            pid = page + offset
            if pid == 0x20:
                supported = self.is_supported(0x20)
            elif pid == 0x40:
                supported = self.is_supported(0x40)
            else:
                supported = self.is_supported(pid)
            if supported:
                bitmap |= 1 << (32 - offset)
        return bitmap.to_bytes(4, "big")

    def handle_request(self, frame: CanFrame) -> Optional[CanFrame]:
        if (
            frame.extended
            or frame.remote
            or frame.arbitration_id not in self.request_ids
            or not frame.data
        ):
            return None

        first = frame.data[0]
        if first >> 4 != 0:
            return None
        length = first & 0x0F
        if length < 1 or length + 1 > len(frame.data):
            return None
        payload = frame.data[1 : length + 1]
        if not payload:
            return None
        if payload[0] == 0x03 and len(payload) == 1:
            active_dtcs = [dtc["code"] for dtc in self.config["dtcs"] if dtc["active"]]
            encoded_codes = b"".join(self._encode_dtc(code) for code in active_dtcs)
            response_data = bytes((0x43,)) + encoded_codes
        elif payload[0] == 0x01 and len(payload) >= 2:
            pid = payload[1]
            if pid in PID_BITMAP_PAGES and self.is_supported(pid):
                data = self.supported_bitmap(pid)
            elif self.is_supported(pid):
                _display, data = self.current_value(pid)
            else:
                return None
            response_data = bytes((0x41, pid)) + data
        else:
            return None
        if len(response_data) > 7:
            return None
        frame_data = bytes((len(response_data),)) + response_data
        frame_data += bytes(8 - len(frame_data))
        return CanFrame(
            arbitration_id=self.response_id,
            extended=False,
            remote=False,
            dlc=8,
            data=frame_data,
        )

    @staticmethod
    def _encode_dtc(code: str) -> bytes:
        system = {"P": 0, "C": 1, "B": 2, "U": 3}[code[0]]
        first = (system << 6) | (int(code[1]) << 4) | int(code[2], 16)
        return bytes((first, int(code[3:], 16)))

    def display_rows(self) -> List[dict]:
        definitions = {
            pid: {"name": name, "unit": unit}
            for pid, (name, unit, _kind) in PID_DEFINITIONS.items()
        }
        for key, entry in self.config["pids"].items():
            pid = int(key, 16)
            if pid not in definitions:
                definitions[pid] = {
                    "name": entry["name"],
                    "unit": entry["unit"],
                }

        rows = []
        for page in PID_BITMAP_PAGES:
            if page != 0 and not self.is_supported(page):
                continue
            start = page + 1
            end = min(page + 0x20, 0xFF)
            rows.append(
                {
                    "pid": f"{page:02X}",
                    "name": f"Supported PID bitmap ({start:02X}-{end:02X})",
                    "unit": "",
                    "supported": "Yes",
                    "value": self.supported_bitmap(page).hex(" ").upper(),
                }
            )

        for pid, definition in sorted(definitions.items()):
            supported = self.is_supported(pid)
            display = "—"
            if supported:
                value, raw = self.current_value(pid)
                display = f"{value}  [{raw.hex(' ').upper()}]"
            rows.append(
                {
                    "pid": f"{pid:02X}",
                    "name": definition["name"],
                    "unit": definition["unit"],
                    "supported": "Yes" if supported else "No",
                    "value": display,
                }
            )
        return rows
