from collections import deque
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from app import SlcanApp


class FakeCombo:
    def __init__(self, values=()):
        self.values = values

    def __getitem__(self, key):
        if key == "values":
            return self.values
        raise KeyError(key)

    def __setitem__(self, key, value):
        if key == "values":
            self.values = value
            return
        raise KeyError(key)


class FakeVariable:
    def __init__(self, value=""):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class FakeStatus(FakeVariable):
    pass


class AppPortTests(unittest.TestCase):
    def test_bus_load_is_a_rolling_ten_second_estimate(self):
        app = SlcanApp.__new__(SlcanApp)
        app.bus_load_var = FakeVariable()
        app._bus_load_bitrate = 500_000
        app._bus_load_samples = deque(((5.0, 50_000),))

        with patch("app.time.monotonic", return_value=10.0):
            app._update_bus_load()
        self.assertIn("1.00% over the last 10 s", app.bus_load_var.get())

        with patch("app.time.monotonic", return_value=16.0):
            app._update_bus_load()
        self.assertIn("0.00% over the last 10 s", app.bus_load_var.get())

    def test_selects_new_bluetooth_port_by_description(self):
        app = SlcanApp.__new__(SlcanApp)
        app.port_combo = FakeCombo(("COM1 - Existing serial port",))
        app.port_var = FakeVariable("COM1 - Existing serial port")
        app.status_var = FakeStatus()
        ports = [
            type("Port", (), {"device": "COM1", "description": "Existing serial port"})(),
            type(
                "Port",
                (),
                {
                    "device": "COM7",
                    "description": "Standard Serial over Bluetooth link",
                },
            )(),
            type("Port", (), {"device": "COM8", "description": "USB Serial Device"})(),
        ]

        with patch("app.serial.tools.list_ports.comports", return_value=ports):
            found = app.refresh_ports(automatic=True)

        self.assertEqual(found, {"COM1", "COM7", "COM8"})
        self.assertEqual(
            app.port_var.get(), "COM7 - Standard Serial over Bluetooth link"
        )
        self.assertIn("COM7", app.status_var.get())

    def test_open_simulator_config_uses_linux_file_opener(self):
        app = SlcanApp.__new__(SlcanApp)
        app.simulator_config_path = Path("/tmp/ecu.json")

        with patch("app.sys.platform", "linux"), patch("app.subprocess.Popen") as popen:
            app._open_simulator_config()

        popen.assert_called_once_with(["xdg-open", "/tmp/ecu.json"])

    def test_simulator_config_fields_include_mil_setting(self):
        app = SlcanApp.__new__(SlcanApp)
        app.simulator = SimpleNamespace(
            config={
                "request_ids": [0x7DF, 0x7E0],
                "response_id": 0x7E8,
                "pids": {},
                "dtcs": [],
                "mil_on": False,
            }
        )
        app.simulator_request_ids_var = FakeVariable("7DF, 7E0")
        app.simulator_response_id_var = FakeVariable("7E8")
        app.simulator_mil_var = FakeVariable(True)

        config = app._simulator_config_from_fields()

        self.assertTrue(config["mil_on"])
        self.assertEqual(config["request_ids"], ["0x7DF", "0x7E0"])

    def test_connect_bluetooth_explains_linux_setup_without_windows_settings(self):
        app = SlcanApp.__new__(SlcanApp)

        with patch("app.sys.platform", "linux"), patch("app.messagebox.showinfo") as showinfo:
            app.connect_bluetooth()

        showinfo.assert_called_once()
        self.assertIn("system Bluetooth tools", showinfo.call_args.args[1])


if __name__ == "__main__":
    unittest.main()
