"""Persistence for named DBC transmitter configurations."""

import json
import os
from pathlib import Path
from typing import Any, Dict


class DbcProfilesError(ValueError):
    pass


def load_profiles(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {"last_profile": None, "profiles": {}}
    try:
        with path.open("r", encoding="utf-8") as source:
            data = json.load(source)
    except (OSError, json.JSONDecodeError) as exc:
        raise DbcProfilesError(f"Could not read DBC configurations '{path}': {exc}") from exc

    if not isinstance(data, dict) or data.get("version") != 1:
        raise DbcProfilesError("DBC configuration file has an unsupported format.")
    profiles = data.get("profiles")
    if not isinstance(profiles, dict) or any(
        not isinstance(name, str) or not name.strip() or not isinstance(profile, dict)
        for name, profile in profiles.items()
    ):
        raise DbcProfilesError("DBC configuration profiles must be named objects.")
    last_profile = data.get("last_profile")
    if last_profile is not None and (
        not isinstance(last_profile, str) or last_profile not in profiles
    ):
        raise DbcProfilesError("The last DBC configuration does not exist in the file.")
    return {"last_profile": last_profile, "profiles": profiles}


def save_profiles(path: Path, data: Dict[str, Any]) -> None:
    profiles = data.get("profiles")
    if not isinstance(profiles, dict):
        raise DbcProfilesError("DBC configuration profiles must be an object.")
    last_profile = data.get("last_profile")
    if last_profile is not None and (
        not isinstance(last_profile, str) or last_profile not in profiles
    ):
        raise DbcProfilesError("The last DBC configuration must exist in the profiles.")
    document = {
        "version": 1,
        "last_profile": last_profile,
        "profiles": profiles,
    }
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with temporary_path.open("w", encoding="utf-8", newline="\n") as output:
            json.dump(document, output, indent=2, allow_nan=False)
            output.write("\n")
        os.replace(temporary_path, path)
    except (OSError, TypeError, ValueError) as exc:
        raise DbcProfilesError(f"Could not save DBC configurations '{path}': {exc}") from exc
