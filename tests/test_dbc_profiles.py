import json
import tempfile
import unittest
from pathlib import Path

from dbc_profiles import DbcProfilesError, load_profiles, save_profiles


class DbcProfilesTests(unittest.TestCase):
    def test_round_trip_multiple_profiles_and_last_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pyslcan" / "dbc_profiles.json"
            profiles = {
                "last_profile": "Track",
                "profiles": {
                    "Track": {"dbc_path": "/tmp/track.dbc", "messages": []},
                    "Bench": {"dbc_path": "/tmp/bench.dbc", "messages": []},
                },
            }

            save_profiles(path, profiles)

            self.assertEqual(load_profiles(path), profiles)

    def test_missing_file_returns_empty_profile_store(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(
                load_profiles(Path(directory) / "missing.json"),
                {"last_profile": None, "profiles": {}},
            )

    def test_invalid_store_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profiles.json"
            path.write_text(json.dumps({"version": 1, "profiles": []}))

            with self.assertRaisesRegex(DbcProfilesError, "named objects"):
                load_profiles(path)

    def test_last_profile_must_exist(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profiles.json"
            with self.assertRaisesRegex(DbcProfilesError, "must exist"):
                save_profiles(
                    path, {"last_profile": "Missing", "profiles": {"Track": {}}}
                )


if __name__ == "__main__":
    unittest.main()
