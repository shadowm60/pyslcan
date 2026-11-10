import json
import tempfile
import unittest
from pathlib import Path

from app_settings import (
    AppSettingsError,
    load_connection_preferences,
    save_connection_preferences,
)


class AppSettingsTests(unittest.TestCase):
    def test_connection_preferences_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pyslcan" / "settings.json"

            self.assertEqual(load_connection_preferences(path), (None, "115200"))
            save_connection_preferences(path, "/dev/ttyUSB0", "230400")

            self.assertEqual(
                load_connection_preferences(path), ("/dev/ttyUSB0", "230400")
            )

    def test_invalid_settings_are_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.json"
            path.write_text(json.dumps({"last_device": "", "last_baudrate": 0}))

            with self.assertRaisesRegex(AppSettingsError, "last_device"):
                load_connection_preferences(path)

            path.write_text(json.dumps({"last_device": "/dev/ttyUSB0", "last_baudrate": 0}))
            with self.assertRaisesRegex(AppSettingsError, "last_baudrate"):
                load_connection_preferences(path)


if __name__ == "__main__":
    unittest.main()
