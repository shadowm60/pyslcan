"""OBD-II Mode 01 ECU simulator and JSON configuration."""

import json
import math
import random
import re
import time
from pathlib import Path
from typing import List, Optional, Tuple

from slcan_protocol import CAN_PADDING_BYTE, CanFrame


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

STANDARD_PID_CATALOG = {
    0x08: ("Short-term fuel trim, bank 2", 1),
    0x09: ("Long-term fuel trim, bank 2", 1),
    0x0A: ("Fuel pressure", 1),
    0x0B: ("Intake manifold absolute pressure", 1),
    0x0E: ("Timing advance", 1),
    0x10: ("Mass air flow rate", 2),
    0x12: ("Commanded secondary air status", 1),
    0x13: ("Oxygen sensors present", 1),
    0x14: ("Oxygen sensor 1 voltage and short-term trim", 2),
    0x15: ("Oxygen sensor 2 voltage and short-term trim", 2),
    0x16: ("Oxygen sensor 3 voltage and short-term trim", 2),
    0x17: ("Oxygen sensor 4 voltage and short-term trim", 2),
    0x18: ("Oxygen sensor 5 voltage and short-term trim", 2),
    0x19: ("Oxygen sensor 6 voltage and short-term trim", 2),
    0x1A: ("Oxygen sensor 7 voltage and short-term trim", 2),
    0x1B: ("Oxygen sensor 8 voltage and short-term trim", 2),
    0x1C: ("OBD standards supported", 1),
    0x1D: ("Oxygen sensors present across four banks", 1),
    0x1E: ("Auxiliary input status", 1),
    0x21: ("Distance with malfunction indicator lamp on", 2),
    0x22: ("Fuel rail pressure relative to manifold vacuum", 2),
    0x23: ("Fuel rail gauge pressure", 2),
    0x24: ("Wide-range oxygen sensor 1 equivalence ratio and voltage", 4),
    0x25: ("Wide-range oxygen sensor 2 equivalence ratio and voltage", 4),
    0x26: ("Wide-range oxygen sensor 3 equivalence ratio and voltage", 4),
    0x27: ("Wide-range oxygen sensor 4 equivalence ratio and voltage", 4),
    0x28: ("Wide-range oxygen sensor 5 equivalence ratio and voltage", 4),
    0x29: ("Wide-range oxygen sensor 6 equivalence ratio and voltage", 4),
    0x2A: ("Wide-range oxygen sensor 7 equivalence ratio and voltage", 4),
    0x2B: ("Wide-range oxygen sensor 8 equivalence ratio and voltage", 4),
    0x2C: ("Commanded exhaust gas recirculation", 1),
    0x2D: ("Exhaust gas recirculation error", 1),
    0x2E: ("Commanded evaporative purge", 1),
    0x30: ("Warm-ups since DTCs cleared", 1),
    0x32: ("Evaporative system vapor pressure", 2),
    0x34: ("Wide-range oxygen sensor 1 equivalence ratio and current", 4),
    0x35: ("Wide-range oxygen sensor 2 equivalence ratio and current", 4),
    0x36: ("Wide-range oxygen sensor 3 equivalence ratio and current", 4),
    0x37: ("Wide-range oxygen sensor 4 equivalence ratio and current", 4),
    0x38: ("Wide-range oxygen sensor 5 equivalence ratio and current", 4),
    0x39: ("Wide-range oxygen sensor 6 equivalence ratio and current", 4),
    0x3A: ("Wide-range oxygen sensor 7 equivalence ratio and current", 4),
    0x3B: ("Wide-range oxygen sensor 8 equivalence ratio and current", 4),
    0x3C: ("Catalyst temperature, bank 1 sensor 1", 2),
    0x3D: ("Catalyst temperature, bank 2 sensor 1", 2),
    0x3E: ("Catalyst temperature, bank 1 sensor 2", 2),
    0x3F: ("Catalyst temperature, bank 2 sensor 2", 2),
    0x41: ("Monitor status this drive cycle", 4),
    0x43: ("Absolute load value", 2),
    0x44: ("Commanded air-fuel equivalence ratio", 2),
    0x45: ("Relative throttle position", 1),
    0x47: ("Absolute throttle position B", 1),
    0x48: ("Absolute throttle position C", 1),
    0x49: ("Accelerator pedal position D", 1),
    0x4A: ("Accelerator pedal position E", 1),
    0x4B: ("Accelerator pedal position F", 1),
    0x4C: ("Commanded throttle actuator", 1),
    0x4D: ("Time run with malfunction indicator lamp on", 2),
    0x4E: ("Time since DTCs cleared", 2),
    0x4F: ("Maximum sensor values", 4),
    0x50: ("Maximum mass air flow sensor value", 4),
    0x51: ("Fuel type", 1),
    0x52: ("Ethanol fuel percentage", 1),
    0x53: ("Absolute evaporative system vapor pressure", 2),
    0x54: ("Evaporative system vapor pressure", 2),
    0x55: ("Short-term secondary oxygen sensor trim, banks 1 and 3", 2),
    0x56: ("Long-term secondary oxygen sensor trim, banks 1 and 3", 2),
    0x57: ("Short-term secondary oxygen sensor trim, banks 2 and 4", 2),
    0x58: ("Long-term secondary oxygen sensor trim, banks 2 and 4", 2),
    0x59: ("Fuel rail absolute pressure", 2),
    0x5A: ("Relative accelerator pedal position", 1),
    0x5B: ("Hybrid battery pack remaining life", 1),
    0x5D: ("Fuel injection timing", 2),
    0x5E: ("Engine fuel rate", 2),
    0x5F: ("Vehicle emission requirements", 1),
    0x61: ("Driver-demanded engine torque", 1),
    0x62: ("Actual engine torque", 1),
    0x63: ("Engine reference torque", 2),
    0x64: ("Engine percent torque data", 5),
    0x65: ("Auxiliary input and output status", 2),
    0x66: ("Mass air flow sensors A and B", 5),
    0x67: ("Engine coolant temperature sensors", 3),
    0x68: ("Intake air temperature sensors", 3),
    0x69: ("Exhaust gas recirculation data", 7),
    0x6A: ("Diesel intake air flow control data", 5),
    0x6B: ("Exhaust gas recirculation temperature", 5),
    0x6C: ("Throttle actuator control data", 5),
    0x6D: ("Fuel pressure control system", 11),
    0x6E: ("Injection pressure control system", 9),
    0x6F: ("Turbocharger compressor inlet pressure", 3),
    0x70: ("Boost pressure control", 10),
    0x71: ("Variable-geometry turbocharger control", 6),
    0x72: ("Wastegate control", 5),
    0x73: ("Exhaust pressure", 5),
    0x74: ("Turbocharger speed", 5),
    0x75: ("Turbocharger temperature, sensor 1", 7),
    0x76: ("Turbocharger temperature, sensor 2", 7),
    0x77: ("Charge air cooler temperature", 5),
    0x78: ("Exhaust gas temperature, bank 1", 9),
    0x79: ("Exhaust gas temperature, bank 2", 9),
    0x7A: ("Diesel particulate filter differential pressure", 7),
    0x7B: ("Diesel particulate filter data", 7),
    0x7C: ("Diesel particulate filter temperature", 9),
    0x7D: ("NOx NTE control area status", 1),
    0x7E: ("Particulate matter NTE control area status", 1),
    0x7F: ("Engine run time data", 13),
    0x81: ("Auxiliary emissions control device run time", 41),
    0x82: ("Auxiliary emissions control device run time", 41),
    0x83: ("NOx sensor data", 9),
    0x84: ("Manifold surface temperature", 1),
    0x85: ("NOx reagent system data", 10),
    0x86: ("Particulate matter sensor data", 5),
    0x87: ("Intake manifold absolute pressure sensors", 5),
    0x88: ("SCR inducement system status", 13),
    0x89: ("Auxiliary emissions control device run time, devices 11-15", 41),
    0x8A: ("Auxiliary emissions control device run time, devices 16-20", 41),
    0x8B: ("Diesel aftertreatment data", 7),
    0x8C: ("Wide-range oxygen sensor data", 17),
    0x8E: ("Engine friction torque", 1),
    0x8F: ("Particulate matter sensor banks 1 and 2", 7),
    0x90: ("WWH-OBD vehicle system information", 3),
    0x91: ("WWH-OBD vehicle system information", 5),
    0x92: ("Fuel system control", 2),
    0x93: ("WWH-OBD vehicle counters supported", 3),
    0x94: ("NOx warning and inducement system", 12),
    0x98: ("Exhaust gas temperature sensors, bank 1", 9),
    0x99: ("Exhaust gas temperature sensors, bank 2", 9),
    0x9A: ("Hybrid and electric vehicle battery data", 6),
    0x9B: ("Diesel exhaust fluid sensor data", 4),
    0x9C: ("Oxygen sensor data", 17),
    0x9D: ("Engine fuel rate", 4),
    0x9E: ("Engine exhaust flow rate", 2),
    0x9F: ("Fuel system percentage use", 9),
    0xA1: ("Corrected NOx sensor data", 9),
    0xA2: ("Cylinder fuel rate", 2),
    0xA3: ("Evaporative system vapor pressure", 9),
    0xA4: ("Transmission actual gear", 4),
    0xA5: ("Commanded diesel exhaust fluid dosing", 4),
    0xA6: ("Odometer", 4),
    0xA7: ("NOx concentration sensors 3 and 4", 4),
    0xA8: ("Corrected NOx concentration sensors 3 and 4", 4),
    0xA9: ("ABS disable switch state", 4),
    0xC3: ("Fuel level inputs A and B", 2),
    0xC4: ("Exhaust particulate control diagnostics", 8),
    0xC5: ("Fuel pressures A and B", 4),
    0xC7: ("Distance since reflash or module replacement", 2),
    0xC8: ("NOx and particulate control warning lamp status", 1),
}

for _pid, (_pid_name, _response_length) in STANDARD_PID_CATALOG.items():
    if _pid in PID_CATALOG:
        continue
    _raw_length = min(_response_length, 5)
    _multiframe = _response_length > 5
    PID_CATALOG[_pid] = {
        "name": _pid_name,
        "description": f"Standard OBD-II Mode 01 data for {_pid_name.lower()}.",
        "measurement": _pid_name,
        "unit": "",
        "range_description": (
            f"{_response_length} response bytes; multi-frame responses are not "
            "supported by this simulator."
            if _multiframe
            else f"{_response_length} response byte(s); configure the raw value."
        ),
        "default_raw_bytes": " ".join(["00"] * _raw_length),
        "default_supported": not _multiframe,
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

NUMERIC_PID_LIMITS = {
    0x04: (0, 100),
    0x05: (-40, 215),
    0x06: (-100, 99.2),
    0x07: (-100, 99.2),
    0x08: (-100, 99.2),
    0x09: (-100, 99.2),
    0x0A: (0, 765),
    0x0B: (0, 255),
    0x0C: (0, 16383),
    0x0D: (0, 255),
    0x0E: (-64, 63.5),
    0x0F: (-40, 215),
    0x10: (0, 655.35),
    0x11: (0, 100),
    0x22: (0, 0.079 * 0xFFFF),
    0x23: (0, 655350),
    0x2C: (0, 100),
    0x2D: (-100, 99.2),
    0x2E: (0, 100),
    0x21: (0, 65535),
    0x30: (0, 255),
    0x2F: (0, 100),
    0x31: (0, 65535),
    0x32: (-8192, 8191.75),
    0x33: (0, 255),
    0x3C: (-40, 6513.5),
    0x3D: (-40, 6513.5),
    0x3E: (-40, 6513.5),
    0x3F: (-40, 6513.5),
    0x42: (0, 65535),
    0x43: (0, 100 * 0xFFFF / 255),
    0x44: (0, 2),
    0x45: (0, 100),
    0x46: (-40, 215),
    0x47: (0, 100),
    0x48: (0, 100),
    0x49: (0, 100),
    0x4A: (0, 100),
    0x4B: (0, 100),
    0x4C: (0, 100),
    0x4D: (0, 65535),
    0x4E: (0, 65535),
    0x52: (0, 100),
    0x53: (0, 65.535),
    0x54: (-32768, 32767),
    0x59: (0, 655350),
    0x5A: (0, 100),
    0x5B: (0, 100),
    0x5C: (-40, 215),
    0x5D: (-210, 301.9921875),
    0x5E: (0, 3212.75),
    0x61: (-125, 130),
    0x62: (-125, 130),
    0x63: (0, 65535),
    0x84: (-40, 215),
    0xA6: (0, 0xFFFFFFFF / 10),
    0xC7: (0, 65535),
}

MULTI_VALUE_PID_LIMITS = {
    0x24: {
        "lambda": (0, 2),
        "voltage": (0, 8),
    }
}

STANDARD_NUMERIC_ENCODINGS = {
    0x06: (-100, 1.28, 1, "%", False),
    0x07: (-100, 1.28, 1, "%", False),
    0x08: (-100, 1.28, 1, "%", False),
    0x09: (-100, 1.28, 1, "%", False),
    0x0A: (0, 1 / 3, 1, "kPa", False),
    0x0B: (0, 1, 1, "kPa", False),
    0x0E: (-64, 2, 1, "degrees", False),
    0x10: (0, 100, 2, "g/s", False),
    0x21: (0, 1, 2, "km", False),
    0x22: (0, 1 / 0.079, 2, "kPa", False),
    0x23: (0, 0.1, 2, "kPa", False),
    0x2C: (0, 2.55, 1, "%", False),
    0x2D: (-100, 1.28, 1, "%", False),
    0x2E: (0, 2.55, 1, "%", False),
    0x30: (0, 1, 1, "warm-ups", False),
    0x31: (0, 1, 2, "km", False),
    0x32: (0, 4, 2, "Pa", True),
    0x33: (0, 1, 1, "kPa", False),
    0x3C: (-40, 10, 2, "C", False),
    0x3D: (-40, 10, 2, "C", False),
    0x3E: (-40, 10, 2, "C", False),
    0x3F: (-40, 10, 2, "C", False),
    0x43: (0, 255 / 100, 2, "%", False),
    0x44: (0, 0xFFFF / 2, 2, "ratio", False),
    0x45: (0, 2.55, 1, "%", False),
    0x47: (0, 2.55, 1, "%", False),
    0x48: (0, 2.55, 1, "%", False),
    0x49: (0, 2.55, 1, "%", False),
    0x4A: (0, 2.55, 1, "%", False),
    0x4B: (0, 2.55, 1, "%", False),
    0x4C: (0, 2.55, 1, "%", False),
    0x4D: (0, 1, 2, "min", False),
    0x4E: (0, 1, 2, "min", False),
    0x52: (0, 2.55, 1, "%", False),
    0x53: (0, 1000, 2, "kPa", False),
    0x54: (0, 1, 2, "Pa", True),
    0x59: (0, 0.1, 2, "kPa", False),
    0x5A: (0, 2.55, 1, "%", False),
    0x5B: (0, 2.55, 1, "%", False),
    0x5D: (-210, 128, 2, "degrees", False),
    0x5E: (0, 20, 2, "L/h", False),
    0x61: (-125, 1, 1, "%", False),
    0x62: (-125, 1, 1, "%", False),
    0x63: (0, 1, 2, "Nm", False),
    0x84: (-40, 1, 1, "C", False),
    0xA6: (0, 10, 4, "km", False),
    0xC7: (0, 1, 2, "km", False),
}

for _pid, (_pid_minimum, _pid_maximum) in NUMERIC_PID_LIMITS.items():
    if _pid in PID_CATALOG and _pid in STANDARD_NUMERIC_ENCODINGS:
        PID_CATALOG[_pid]["minimum"] = _pid_minimum
        PID_CATALOG[_pid]["maximum"] = _pid_maximum
        PID_CATALOG[_pid]["unit"] = STANDARD_NUMERIC_ENCODINGS[_pid][3]

PID_CATALOG[0x24]["range_description"] = (
    "Equivalence ratio: 0-2; sensor voltage: 0-8 V."
)


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
        "multi_pid_response_active": False,
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
        allowed_algorithms = (
            ("sweep", "random", "raw")
            if pid in NUMERIC_PID_LIMITS
            else ("standard", "raw")
        )
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
        if algorithm in ("sweep", "random"):
            default_sweep = DEFAULT_SWEEPS.get(
                pid,
                {
                    "minimum": NUMERIC_PID_LIMITS[pid][0],
                    "maximum": NUMERIC_PID_LIMITS[pid][1],
                    "step": 1,
                    "interval_s": 1,
                },
            )
            sweep = entry.get("sweep", default_sweep)
            if not isinstance(sweep, dict):
                raise SimulatorConfigError(f"pids.{key}.sweep must be an object.")
            values = {}
            fields = (
                ("minimum", "maximum", "step", "interval_s")
                if algorithm == "sweep"
                else ("minimum", "maximum")
            )
            for field in fields:
                try:
                    values[field] = float(sweep[field])
                except (KeyError, TypeError, ValueError) as exc:
                    raise SimulatorConfigError(
                        f"pids.{key}.sweep.{field} must be numeric."
                    ) from exc
            if (
                values["minimum"] >= values["maximum"]
                or not all(math.isfinite(value) for value in values.values())
                or (
                    algorithm == "sweep"
                    and (values["step"] <= 0 or values["interval_s"] <= 0)
                )
            ):
                raise SimulatorConfigError(
                    f"pids.{key}.sweep requires a finite maximum greater than minimum"
                    + (" and positive step/interval." if algorithm == "sweep" else ".")
                )
            lower, upper = NUMERIC_PID_LIMITS[pid]
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
        algorithm = entry.get("algorithm", "raw")
        if algorithm not in ("raw", "sweep", "random") or (
            algorithm != "raw"
            and pid not in NUMERIC_PID_LIMITS
            and pid not in MULTI_VALUE_PID_LIMITS
        ):
            raise SimulatorConfigError(
                f"pids.{canonical_key}.algorithm must be 'raw' "
                "or a numeric algorithm supported for this PID."
            )
        normalized = {
            "supported": entry["supported"],
            "name": name.strip(),
            "unit": unit.strip(),
            "algorithm": algorithm,
        }
        if algorithm == "raw":
            raw_bytes = _parse_raw_bytes(
                entry.get("raw_bytes"), f"pids.{canonical_key}.raw_bytes"
            )
            normalized["raw_bytes"] = " ".join(
                f"{byte:02X}" for byte in raw_bytes
            )
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
        if algorithm in ("sweep", "random"):
            if pid in MULTI_VALUE_PID_LIMITS:
                field_ranges = MULTI_VALUE_PID_LIMITS[pid]
                default_sweep = {
                    f"{field}_{bound}": limit
                    for field, bounds in field_ranges.items()
                    for bound, limit in zip(("minimum", "maximum"), bounds)
                }
                default_sweep.update(
                    {
                        f"{field}_step": (bounds[1] - bounds[0]) / 10
                        for field, bounds in field_ranges.items()
                    }
                )
                default_sweep["interval_s"] = 1
                sweep = entry.get("sweep", default_sweep)
                if not isinstance(sweep, dict):
                    raise SimulatorConfigError(
                        f"pids.{canonical_key}.sweep must be an object."
                    )
                values = {}
                fields = []
                for field, (lower, upper) in field_ranges.items():
                    fields.extend((f"{field}_minimum", f"{field}_maximum"))
                    if algorithm == "sweep":
                        fields.append(f"{field}_step")
                if algorithm == "sweep":
                    fields.append("interval_s")
                for field in fields:
                    try:
                        values[field] = float(sweep[field])
                    except (KeyError, TypeError, ValueError) as exc:
                        raise SimulatorConfigError(
                            f"pids.{canonical_key}.sweep.{field} must be numeric."
                        ) from exc
                for field, (lower, upper) in field_ranges.items():
                    minimum = values[f"{field}_minimum"]
                    maximum = values[f"{field}_maximum"]
                    if (
                        not math.isfinite(minimum)
                        or not math.isfinite(maximum)
                        or minimum >= maximum
                        or minimum < lower
                        or maximum > upper
                    ):
                        raise SimulatorConfigError(
                            f"pids.{canonical_key}.sweep {field} boundaries must "
                            f"satisfy {lower} <= minimum < maximum <= {upper}."
                        )
                    step_key = f"{field}_step"
                    if algorithm == "sweep" and (
                        not math.isfinite(values[step_key]) or values[step_key] <= 0
                    ):
                        raise SimulatorConfigError(
                            f"pids.{canonical_key}.sweep.{step_key} must be positive."
                        )
                if algorithm == "sweep" and (
                    not math.isfinite(values["interval_s"])
                    or values["interval_s"] <= 0
                ):
                    raise SimulatorConfigError(
                        f"pids.{canonical_key}.sweep.interval_s must be positive."
                    )
                normalized["sweep"] = values
                normalized_pids[canonical_key] = normalized
                continue
            default_sweep = {
                "minimum": normalized.get("minimum", NUMERIC_PID_LIMITS[pid][0]),
                "maximum": normalized.get("maximum", NUMERIC_PID_LIMITS[pid][1]),
                "step": 1,
                "interval_s": 1,
            }
            sweep = entry.get("sweep", default_sweep)
            if not isinstance(sweep, dict):
                raise SimulatorConfigError(
                    f"pids.{canonical_key}.sweep must be an object."
                )
            fields = (
                ("minimum", "maximum", "step", "interval_s")
                if algorithm == "sweep"
                else ("minimum", "maximum")
            )
            values = {}
            for field in fields:
                try:
                    values[field] = float(sweep[field])
                except (KeyError, TypeError, ValueError) as exc:
                    raise SimulatorConfigError(
                        f"pids.{canonical_key}.sweep.{field} must be numeric."
                    ) from exc
            if (
                values["minimum"] >= values["maximum"]
                or not all(math.isfinite(value) for value in values.values())
                or (
                    algorithm == "sweep"
                    and (values["step"] <= 0 or values["interval_s"] <= 0)
                )
            ):
                raise SimulatorConfigError(
                    f"pids.{canonical_key}.sweep requires a finite maximum greater "
                    "than minimum"
                    + (" and positive step/interval." if algorithm == "sweep" else ".")
                )
            lower, upper = NUMERIC_PID_LIMITS[pid]
            if values["minimum"] < lower or values["maximum"] > upper:
                raise SimulatorConfigError(
                    f"pids.{canonical_key}.sweep values must be between {lower} and {upper}."
                )
            normalized["sweep"] = values
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
    multi_pid_response_active = config.get("multi_pid_response_active", False)
    if not isinstance(multi_pid_response_active, bool):
        raise SimulatorConfigError(
            "'multi_pid_response_active' must be true or false."
        )

    return {
        "version": 1,
        "request_ids": parsed_request_ids,
        "response_id": response_id,
        "pids": normalized_pids,
        "dtcs": normalized_dtcs,
        "mil_on": mil_on,
        "multi_pid_response_active": multi_pid_response_active,
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
        "multi_pid_response_active": normalized["multi_pid_response_active"],
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
        self._dtcs_cleared = False
        self._random_values = {}

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

    def start_session(self) -> None:
        self.reset_clock()
        self._dtcs_cleared = False

    def current_value(
        self, pid: int, refresh_random: bool = False
    ) -> Tuple[str, bytes]:
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

        if pid in STANDARD_NUMERIC_ENCODINGS:
            value = self._value(pid, refresh_random)
            offset, scale, byte_count, unit, signed = STANDARD_NUMERIC_ENCODINGS[pid]
            bits = byte_count * 8
            encoded = round((value - offset) * scale)
            lower = -(1 << (bits - 1)) if signed else 0
            upper = (1 << (bits - 1)) - 1 if signed else (1 << bits) - 1
            encoded = min(max(encoded, lower), upper)
            raw = (encoded & ((1 << bits) - 1)).to_bytes(byte_count, "big")
            value = encoded / scale + offset
            display = f"{value:g} {unit}".strip()
            return display, raw

        if pid in MULTI_VALUE_PID_LIMITS:
            lambda_value, voltage_value = self._multi_value(pid, refresh_random)
            lambda_raw = min(round(lambda_value / 2 * 65536), 0xFFFF)
            voltage_raw = min(round(voltage_value / 8 * 65536), 0xFFFF)
            raw = lambda_raw.to_bytes(2, "big") + voltage_raw.to_bytes(2, "big")
            return f"{lambda_value:g} lambda, {voltage_value:g} V", raw

        name, unit, kind = PID_DEFINITIONS[pid]
        if kind == "status":
            active_dtcs = (
                0
                if self._dtcs_cleared
                else sum(dtc["active"] for dtc in self.config["dtcs"])
            )
            mil_on = self.config["mil_on"] and not self._dtcs_cleared
            status = (0x80 if mil_on else 0) | active_dtcs
            raw = bytes((status, 0, 0, 0))
            return f"MIL {'on' if mil_on else 'off'}; {active_dtcs} DTCs", raw

        if kind == "runtime":
            seconds = min(int(self._clock() - self._started_at), 0xFFFF)
            raw = seconds.to_bytes(2, "big")
            return f"{seconds} {unit}", raw

        value = self._value(pid, refresh_random)
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

    def _value(self, pid: int, refresh_random: bool = False) -> float:
        entry = self.config["pids"][f"{pid:02X}"]
        spec = entry.get("sweep", DEFAULT_SWEEPS.get(pid))
        if spec is None:
            raise ValueError(f"PID 0x{pid:02X} has no numeric value range.")
        minimum = spec["minimum"]
        maximum = spec["maximum"]
        if entry.get("algorithm") == "random":
            if refresh_random:
                self._random_values[pid] = random.uniform(minimum, maximum)
            return self._random_values.get(pid, minimum)
        step = spec["step"]
        interval = spec["interval_s"]
        steps = max(1, math.ceil((maximum - minimum) / step))
        phase = int((self._clock() - self._started_at) / interval) % (steps * 2)
        position = phase if phase <= steps else steps * 2 - phase
        return min(minimum + position * step, maximum)

    def _multi_value(
        self, pid: int, refresh_random: bool
    ) -> Tuple[float, float]:
        entry = self.config["pids"][f"{pid:02X}"]
        spec = entry["sweep"]
        if entry["algorithm"] == "random":
            if refresh_random:
                self._random_values[pid] = (
                    random.uniform(spec["lambda_minimum"], spec["lambda_maximum"]),
                    random.uniform(spec["voltage_minimum"], spec["voltage_maximum"]),
                )
            return self._random_values.get(
                pid, (spec["lambda_minimum"], spec["voltage_minimum"])
            )

        elapsed = self._clock() - self._started_at
        values = []
        for field in ("lambda", "voltage"):
            minimum = spec[f"{field}_minimum"]
            maximum = spec[f"{field}_maximum"]
            step = spec[f"{field}_step"]
            steps = max(1, math.ceil((maximum - minimum) / step))
            phase = int(elapsed / spec["interval_s"]) % (steps * 2)
            position = phase if phase <= steps else steps * 2 - phase
            values.append(min(minimum + position * step, maximum))
        return values[0], values[1]

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
        response_data = self.handle_request_payload(frame)
        if response_data is None or len(response_data) > 7:
            return None
        frame_data = bytes((len(response_data),)) + response_data
        frame_data += bytes((CAN_PADDING_BYTE,)) * (8 - len(frame_data))
        return CanFrame(
            arbitration_id=self.response_id,
            extended=False,
            remote=False,
            dlc=8,
            data=frame_data,
        )

    def handle_request_payload(self, frame: CanFrame) -> Optional[bytes]:
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
            active_dtcs = (
                []
                if self._dtcs_cleared
                else [dtc["code"] for dtc in self.config["dtcs"] if dtc["active"]]
            )
            encoded_codes = b"".join(self._encode_dtc(code) for code in active_dtcs)
            response_data = bytes((0x43,)) + encoded_codes
        elif payload[0] == 0x04 and len(payload) == 1:
            self._dtcs_cleared = True
            response_data = bytes((0x44,))
        elif payload[0] == 0x01 and len(payload) >= 2:
            requested_pids = payload[1:]
            if not self.config["multi_pid_response_active"]:
                requested_pids = requested_pids[:1]
            pid_data = []
            seen_pids = set()
            for pid in requested_pids:
                if pid in seen_pids or not self.is_supported(pid):
                    continue
                seen_pids.add(pid)
                if pid in PID_BITMAP_PAGES:
                    data = self.supported_bitmap(pid)
                else:
                    _display, data = self.current_value(pid, refresh_random=True)
                pid_data.append(bytes((pid,)) + data)
            if not pid_data:
                return None
            response_data = bytes((0x41,)) + b"".join(pid_data)
        else:
            return None
        return response_data

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
