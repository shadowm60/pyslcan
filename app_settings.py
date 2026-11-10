"""Persistence for application startup preferences."""

import json
import os
import sys
from pathlib import Path
from typing import Optional, Tuple


class AppSettingsError(ValueError):
    pass


def settings_path() -> Path:
    if os.name == "nt":
        root = Path(os.environ.get("APPDATA", Path.home() / "AppData" / "Roaming"))
    elif sys.platform == "darwin":
        root = Path.home() / "Library" / "Application Support"
    else:
        root = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
    return root / "pyslcan" / "settings.json"


def load_connection_preferences(path: Path) -> Tuple[Optional[str], str]:
    if not path.exists():
        return None, "115200"
    try:
        with path.open("r", encoding="utf-8") as source:
            settings = json.load(source)
    except (OSError, json.JSONDecodeError) as exc:
        raise AppSettingsError(
            f"Could not read startup settings '{path}': {exc}"
        ) from exc
    if not isinstance(settings, dict):
        raise AppSettingsError("Startup settings must contain a JSON object.")

    device = settings.get("last_device")
    baudrate = settings.get("last_baudrate", "115200")
    if device is not None and (not isinstance(device, str) or not device.strip()):
        raise AppSettingsError("'last_device' must be non-empty text or null.")
    if (
        isinstance(baudrate, bool)
        or not str(baudrate).isdigit()
        or int(baudrate) <= 0
    ):
        raise AppSettingsError("'last_baudrate' must be a positive integer.")
    return device, str(baudrate)


def save_connection_preferences(path: Path, device: str, baudrate: str) -> None:
    try:
        parsed_baudrate = int(baudrate)
    except (TypeError, ValueError) as exc:
        raise AppSettingsError("'last_baudrate' must be a positive integer.") from exc
    if not device.strip() or parsed_baudrate <= 0:
        raise AppSettingsError("Device must be non-empty and baud rate must be positive.")
    settings = {
        "last_device": device,
        "last_baudrate": str(parsed_baudrate),
    }
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with temporary_path.open("w", encoding="utf-8", newline="\n") as output:
            json.dump(settings, output, indent=2)
            output.write("\n")
        os.replace(temporary_path, path)
    except OSError as exc:
        raise AppSettingsError(
            f"Could not save startup settings '{path}': {exc}"
        ) from exc
