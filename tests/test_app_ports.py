from collections import deque
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cantools

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


class FakeTree:
    def __init__(self):
        self.rows = {}
        self.selected = ()

    def selection(self):
        return self.selected

    def selection_set(self, keys):
        self.selected = tuple(keys)

    def exists(self, key):
        return key in self.rows

    def item(self, key, option=None, **kwargs):
        if "values" in kwargs:
            self.rows[key] = kwargs["values"]
        if option == "values":
            return self.rows[key]
        return {"values": self.rows[key]}


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

    def test_dbc_profile_serializes_selection_and_running_state_independently(self):
        database = cantools.database.load_string(
            """
            VERSION ""
            NS_ :
            BS_:
            BU_: ECU
            BO_ 1 First: 1 ECU
            BO_ 2 Second: 1 ECU
            """
        )
        app = SlcanApp.__new__(SlcanApp)
        app.dbc_database = database
        app.dbc_messages = {
            "0": database.get_message_by_name("First"),
            "1": database.get_message_by_name("Second"),
        }
        app.dbc_message_table = FakeTree()
        app.dbc_message_table.selected = ("1",)
        app.dbc_cycle_overrides = {}
        app.dbc_signal_settings = {}
        app.dbc_path_var = FakeVariable("/tmp/example.dbc")
        app._dbc_next_tx = {"0": 1.0}
        app._dbc_profile_start_keys = {"1"}

        profile = app._serialize_dbc_profile()

        self.assertEqual(
            profile["messages"],
            [
                {"frame_id": 1, "message": "First", "selected": False, "running": True},
                {"frame_id": 2, "message": "Second", "selected": True, "running": True},
            ],
        )

    def test_dbc_profile_restores_selection_and_armed_state_without_transmitting(self):
        database = cantools.database.load_string(
            """
            VERSION ""
            NS_ :
            BS_:
            BU_: ECU
            BO_ 1 First: 1 ECU
            BO_ 2 Second: 1 ECU
            """
        )
        app = SlcanApp.__new__(SlcanApp)
        app._populate_dbc_database = lambda _path, loaded: self._populate_fake_dbc(
            app, loaded
        )
        app._show_dbc_signals = lambda: None
        app._update_dbc_availability = lambda: None

        app._apply_dbc_profile(
            {
                "messages": [
                    {"frame_id": 1, "message": "First", "selected": False, "running": True},
                    {"frame_id": 2, "message": "Second", "selected": True, "running": False},
                ],
                "cycle_overrides": [],
                "signals": [],
            },
            "/tmp/example.dbc",
            database,
        )

        self.assertEqual(app.dbc_message_table.selection(), ("1",))
        self.assertEqual(app._dbc_profile_start_keys, {"0"})
        self.assertEqual(app._dbc_next_tx, {})
        self.assertEqual(app.dbc_message_table.rows["0"][-1], "Ready")

    @staticmethod
    def _populate_fake_dbc(app, database):
        app.dbc_database = database
        messages = sorted(
            database.messages, key=lambda message: (message.frame_id, message.name)
        )
        app.dbc_messages = {
            str(index): message for index, message in enumerate(messages)
        }
        app.dbc_cycle_overrides = {}
        app.dbc_signal_settings = {}
        app._dbc_profile_start_keys = set()
        app._dbc_next_tx = {}
        app._dbc_pending = set()
        app.dbc_message_table = FakeTree()
        for key, message in app.dbc_messages.items():
            app.dbc_message_table.rows[key] = (
                f"{message.frame_id:03X}",
                message.name,
                message.length,
                "--",
                "",
                "100 (fallback)",
                "Stopped",
            )

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
