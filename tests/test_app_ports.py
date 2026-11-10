from collections import deque
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from app import SlcanApp
from ecu_simulator import EcuSimulator, default_config
from slcan_protocol import CanFrame, encode_frame, parse_frame


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


class FakeRoot:
    def __init__(self):
        self.callbacks = {}
        self.next_id = 0

    def after(self, delay, callback):
        self.next_id += 1
        self.callbacks[self.next_id] = (delay, callback)
        return self.next_id

    def after_cancel(self, callback_id):
        self.callbacks.pop(callback_id, None)


class FakeWorker:
    def __init__(self):
        self.commands = []

    def send(self, command, tag):
        self.commands.append((command, tag))


class AppPortTests(unittest.TestCase):
    def make_simulator_app(self, config=None):
        app = SlcanApp.__new__(SlcanApp)
        app.simulator = EcuSimulator(config or default_config(), clock=lambda: 100.0)
        app.simulator_running = True
        app.can_open = True
        app.mode_var = FakeVariable("Normal")
        app.connected = True
        app.worker = FakeWorker()
        app.root = FakeRoot()
        app.simulator_status_var = FakeVariable()
        app.simulator_response_count = 0
        app._simulator_isotp_sender = None
        app._simulator_response_command_pending = False
        app._simulator_flow_control_after = None
        app._simulator_segment_after = None
        app._add_frame = lambda *_args: None
        return app

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
        app.preferred_device = None
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

    def test_refresh_ports_selects_the_remembered_device_when_available(self):
        app = SlcanApp.__new__(SlcanApp)
        app.port_combo = FakeCombo()
        app.port_var = FakeVariable("")
        app.preferred_device = "/dev/ttyUSB1"
        app.status_var = FakeStatus()
        ports = [
            type("Port", (), {"device": "/dev/ttyUSB0", "description": "First"})(),
            type("Port", (), {"device": "/dev/ttyUSB1", "description": "Saved"})(),
        ]

        with patch("app.serial.tools.list_ports.comports", return_value=ports):
            app.refresh_ports()

        self.assertEqual(app.port_var.get(), "/dev/ttyUSB1 - Saved")

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
                "multi_pid_response_active": True,
            }
        )
        app.simulator_request_ids_var = FakeVariable("7DF, 7E0")
        app.simulator_response_id_var = FakeVariable("7E8")
        app.simulator_mil_var = FakeVariable(True)
        app.simulator_multi_pid_var = FakeVariable(False)

        config = app._simulator_config_from_fields()

        self.assertTrue(config["mil_on"])
        self.assertFalse(config["multi_pid_response_active"])
        self.assertEqual(config["request_ids"], ["0x7DF", "0x7E0"])

    def test_multi_pid_response_waits_for_flow_control_and_sends_segments(self):
        config = default_config()
        config["multi_pid_response_active"] = True
        app = self.make_simulator_app(config)
        request = CanFrame(
            0x7DF, False, False, 8, bytes((3, 1, 0x01, 0x0C, 0, 0, 0, 0))
        )

        app._handle_simulator_request(encode_frame(request))
        self.assertEqual(len(app.worker.commands), 1)
        self.assertTrue(app._simulator_isotp_sender.waiting_for_flow_control)
        app._command_completed(
            {
                "tag": "ecu_response",
                "ok": True,
                "response": "",
                "command": app.worker.commands[0][0],
            }
        )
        self.assertEqual(
            next(iter(app.root.callbacks.values()))[0],
            1000,
        )

        flow_control = CanFrame(
            0x7E0, False, False, 8, bytes((0x30, 0, 0, 0, 0, 0, 0, 0))
        )
        app._handle_simulator_request(encode_frame(flow_control))
        segment_callback = next(
            callback
            for delay, callback in app.root.callbacks.values()
            if delay == 0
        )
        segment_callback()

        self.assertEqual(len(app.worker.commands), 2)
        self.assertEqual(app.worker.commands[1][1], "ecu_response")
        app._command_completed(
            {
                "tag": "ecu_response",
                "ok": True,
                "response": "",
                "command": app.worker.commands[1][0],
            }
        )
        self.assertIsNone(app._simulator_isotp_sender)
        self.assertEqual(app.simulator_response_count, 1)

    def test_small_multi_pid_response_is_returned_as_single_frame(self):
        config = default_config()
        config["multi_pid_response_active"] = True
        app = self.make_simulator_app(config)
        request = CanFrame(
            0x7DF, False, False, 8, bytes((3, 1, 0x0C, 0x0D, 0, 0, 0, 0))
        )

        app._handle_simulator_request(encode_frame(request))

        self.assertEqual(len(app.worker.commands), 1)
        response = app.worker.commands[0][0]
        self.assertTrue(response.startswith("t7E8806410C0C800D0055"))

    def test_mode04_response_updates_status_without_indexing_missing_pid(self):
        app = self.make_simulator_app()
        request = CanFrame(0x7DF, False, False, 8, bytes((1, 4, 0, 0, 0, 0, 0, 0)))

        app._handle_simulator_request(encode_frame(request))

        self.assertEqual(len(app.worker.commands), 1)
        response = parse_frame(app.worker.commands[0][0])
        self.assertEqual(
            response.data,
            bytes((1, 0x44, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55)),
        )
        self.assertIn("Mode 04", app.simulator_status_var.get())

    def test_mode03_response_updates_status_with_positive_service(self):
        app = self.make_simulator_app()
        request = CanFrame(0x7DF, False, False, 8, bytes((1, 3, 0, 0, 0, 0, 0, 0)))

        app._handle_simulator_request(encode_frame(request))

        self.assertEqual(len(app.worker.commands), 1)
        self.assertIn("Mode 03", app.simulator_status_var.get())

    def test_connect_bluetooth_explains_linux_setup_without_windows_settings(self):
        app = SlcanApp.__new__(SlcanApp)

        with patch("app.sys.platform", "linux"), patch("app.messagebox.showinfo") as showinfo:
            app.connect_bluetooth()

        showinfo.assert_called_once()
        self.assertIn("system Bluetooth tools", showinfo.call_args.args[1])


if __name__ == "__main__":
    unittest.main()
