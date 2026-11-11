"""Tkinter UI for testing a custom SLCAN CAN interface."""

import csv
import copy
import ctypes
import math
import os
import queue
import subprocess
import sys
import threading
import tkinter as tk
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, simpledialog, ttk
from typing import Any, Dict, Optional, Sequence, Set, Tuple

import cantools
import serial.tools.list_ports
from cantools.database.can import Message

from bluetooth_windows import BluetoothDevice, BluetoothUnavailableError, discover_devices
from app_settings import (
    AppSettingsError,
    load_connection_preferences,
    save_connection_preferences,
    settings_path,
)
from can_trace import format_asc_trace
from dbc_profiles import DbcProfilesError, load_profiles, save_profiles
from dbc_transmitter import (
    SignalTxSettings,
    default_signal_settings,
    encode_message,
    load_dbc,
    signal_value_bounds,
)
from ecu_simulator import (
    DEFAULT_SWEEPS,
    EcuSimulator,
    MULTI_VALUE_PID_LIMITS,
    NUMERIC_PID_LIMITS,
    PID_BITMAP_PAGES,
    PID_CATALOG,
    PID_DEFINITIONS,
    SimulatorConfigError,
    default_config,
    load_config,
    save_config,
)
from isotp import IsoTpSender
from serial_worker import SerialWorker
from slcan_protocol import (
    BITRATES,
    CanFrame,
    encode_frame,
    estimate_can_frame_bits,
    parse_frame,
)


BAUD_RATES = ("9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600")


class SlcanApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("SLCAN CAN Interface Tester")
        self.root.geometry("1020x680")
        self.events: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.bluetooth_events: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.bluetooth_devices: Dict[str, BluetoothDevice] = {}
        self.worker: Optional[SerialWorker] = None
        self.connected = False
        self.can_open = False
        self._closing = False
        self._records = []
        self._bus_load_samples = deque()
        self._bus_load_bitrate = None
        self.bus_load_var = tk.StringVar(
            value="CAN closed. Estimate includes observed RX and TX frames only."
        )
        self.debug_enabled_var = tk.BooleanVar(value=False)
        self.debug_messages = []
        self.debug_window = None
        self.debug_text = None
        self.last_serial_error = None
        self.app_settings_path = settings_path()
        self.preferred_device = None
        self._pending_connection_preferences = None
        self._settings_warning = None
        try:
            self.preferred_device, preferred_baudrate = load_connection_preferences(
                self.app_settings_path
            )
        except AppSettingsError as exc:
            preferred_baudrate = "115200"
            self._settings_warning = str(exc)
        self.simulator_config_path = Path(__file__).with_name("ecu_simulator.json")
        self.simulator_running = False
        self.simulator_response_count = 0
        self._simulator_refresh_after = None
        self.simulator_status_var = tk.StringVar(value="CAN must be open in Normal mode.")
        self.simulator_request_ids_var = tk.StringVar()
        self.simulator_response_id_var = tk.StringVar()
        self.simulator_config_path_var = tk.StringVar()
        self.simulator_mil_var = tk.BooleanVar(value=False)
        self.simulator_multi_pid_var = tk.BooleanVar(value=False)
        self._simulator_isotp_sender: Optional[IsoTpSender] = None
        self._simulator_response_command_pending = False
        self._simulator_flow_control_after = None
        self._simulator_segment_after = None
        self.dbc_database: Optional[cantools.database.Database] = None
        self.dbc_messages: Dict[str, Message] = {}
        self.dbc_signal_settings: Dict[Tuple[str, str], SignalTxSettings] = {}
        self.dbc_cycle_overrides: Dict[str, float] = {}
        self.dbc_path_var = tk.StringVar(value="No DBC loaded")
        self.dbc_status_var = tk.StringVar(value="Load a DBC file to configure cyclic messages.")
        self.dbc_running = False
        self._dbc_after = None
        self._dbc_next_tx: Dict[str, float] = {}
        self._dbc_periods: Dict[str, float] = {}
        self._dbc_pending: Set[str] = set()
        self._dbc_sweep_values: Dict[str, Dict[str, float]] = {}
        self._dbc_sweep_directions: Dict[str, Dict[str, int]] = {}
        self._dbc_profile_start_keys: Set[str] = set()
        self.dbc_load_button: Optional[ttk.Button] = None
        self.dbc_start_saved_button: Optional[ttk.Button] = None
        self.dbc_profiles_path = self.app_settings_path.parent / "dbc_profiles.json"
        self.dbc_profiles: Dict[str, Any] = {"last_profile": None, "profiles": {}}
        self.dbc_profile_var = tk.StringVar()
        try:
            if self.simulator_config_path.exists():
                simulator_config = load_config(self.simulator_config_path)
            else:
                simulator_config = save_config(
                    self.simulator_config_path, default_config()
                )
            self.simulator_config_error = None
        except SimulatorConfigError as exc:
            simulator_config = default_config()
            self.simulator_config_error = str(exc)
        self.simulator = EcuSimulator(simulator_config)

        self.port_var = tk.StringVar(value=self.preferred_device or "")
        self.baud_var = tk.StringVar(value=preferred_baudrate)
        self.bitrate_var = tk.StringVar(value="S6 - 500 kbit/s")
        self.mode_var = tk.StringVar(value="Normal")
        self.status_var = tk.StringVar(value="Disconnected")
        self.frame_type_var = tk.StringVar(value="Standard data")
        self.frame_id_var = tk.StringVar(value="123")
        self.dlc_var = tk.StringVar(value="2")
        self.data_var = tk.StringVar(value="DE AD")
        self.bluetooth_var = tk.StringVar()

        self._build_ui()
        self._restore_last_dbc_profile()
        self.refresh_ports()
        if self._settings_warning:
            self.status_var.set(self._settings_warning)
        self.root.after(50, self._poll_events)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.notebook = ttk.Notebook(self.root)
        self.main_tab = ttk.Frame(self.notebook)
        self.simulator_tab = ttk.Frame(self.notebook)
        self.dbc_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.main_tab, text="SLCAN monitor")
        self.notebook.add(self.simulator_tab, text="ECU simulator")
        self.notebook.add(self.dbc_tab, text="DBC transmitter")
        self.notebook.pack(fill="both", expand=True)

        if sys.platform == "win32":
            bluetooth = ttk.LabelFrame(self.main_tab, text="Bluetooth SPP device")
            bluetooth.pack(fill="x", padx=8, pady=(6, 2))
            ttk.Label(bluetooth, text="Device").grid(row=0, column=0, padx=4, pady=5)
            self.bluetooth_combo = ttk.Combobox(
                bluetooth, textvariable=self.bluetooth_var, state="readonly", width=56
            )
            self.bluetooth_combo.grid(row=0, column=1, padx=4, pady=5)
            self.scan_bluetooth_button = ttk.Button(
                bluetooth, text="Scan Bluetooth", command=self.scan_bluetooth
            )
            self.scan_bluetooth_button.grid(row=0, column=2, padx=4)
            self.bluetooth_connect_button = ttk.Button(
                bluetooth, text="Pair / connect in Windows", command=self.connect_bluetooth
            )
            self.bluetooth_connect_button.grid(row=0, column=3, padx=4)
            ttk.Label(
                bluetooth,
                text="Windows manages pairing and creates the virtual serial port; the port list will refresh automatically.",
                foreground="#555555",
            ).grid(row=1, column=0, columnspan=4, sticky="w", padx=4, pady=(0, 4))
        else:
            bluetooth = ttk.LabelFrame(self.main_tab, text="Bluetooth SPP")
            bluetooth.pack(fill="x", padx=8, pady=(6, 2))
            bluetooth_setup = (
                "Pair/connect using Linux Bluetooth tools and configure the adapter's "
                "SPP service as a serial device (for example, /dev/rfcomm0), then "
                "select it below or use Refresh."
                if sys.platform.startswith("linux")
                else "Pair/connect using your system Bluetooth tools and configure "
                "the adapter's SPP service as a serial device, then select it below "
                "or use Refresh."
            )
            ttk.Label(
                bluetooth,
                text=bluetooth_setup,
                foreground="#555555",
                wraplength=950,
            ).pack(anchor="w", padx=4, pady=5)

        connection = ttk.LabelFrame(self.main_tab, text="Serial connection and CAN")
        connection.pack(fill="x", padx=8, pady=4)

        ttk.Label(connection, text="Port").grid(row=0, column=0, padx=4, pady=5)
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=32)
        self.port_combo.grid(row=0, column=1, padx=4, pady=5)
        ttk.Button(connection, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=2, padx=4
        )
        ttk.Label(connection, text="Serial baud").grid(row=0, column=3, padx=4)
        ttk.Combobox(
            connection, textvariable=self.baud_var, values=BAUD_RATES, width=10
        ).grid(row=0, column=4, padx=4)
        self.connect_button = ttk.Button(
            connection, text="Connect serial", command=self.connect
        )
        self.connect_button.grid(row=0, column=5, padx=4)
        self.disconnect_button = ttk.Button(
            connection, text="Disconnect", command=self.disconnect, state="disabled"
        )
        self.disconnect_button.grid(row=0, column=6, padx=4)
        ttk.Checkbutton(
            connection,
            text="Serial debug",
            variable=self.debug_enabled_var,
            command=self._toggle_debug,
        ).grid(row=2, column=0, sticky="w", padx=4, pady=3)
        ttk.Button(
            connection, text="View debug log", command=self.show_debug_log
        ).grid(row=2, column=1, sticky="w", padx=4, pady=3)

        ttk.Label(connection, text="CAN bitrate").grid(row=1, column=0, padx=4, pady=5)
        ttk.Combobox(
            connection,
            textvariable=self.bitrate_var,
            values=[f"{code} - {rate // 1000} kbit/s" for code, rate in BITRATES.items()],
            state="readonly",
            width=18,
        ).grid(row=1, column=1, padx=4)
        ttk.Label(connection, text="Mode").grid(row=1, column=2, padx=4)
        ttk.Combobox(
            connection,
            textvariable=self.mode_var,
            values=("Normal", "Listen-only"),
            state="readonly",
            width=12,
        ).grid(row=1, column=3, padx=4)
        self.open_button = ttk.Button(
            connection, text="Set bitrate and open CAN", command=self.open_can, state="disabled"
        )
        self.open_button.grid(row=1, column=4, columnspan=2, padx=4)
        self.close_can_button = ttk.Button(
            connection, text="Close CAN", command=self.close_can, state="disabled"
        )
        self.close_can_button.grid(row=1, column=6, padx=4)

        tools = ttk.LabelFrame(self.main_tab, text="Device commands")
        tools.pack(fill="x", padx=8, pady=4)
        self._command_buttons = []
        for label, command, tag in (
            ("Status flags (F)", "F", "query"),
            ("Firmware (V)", "V", "query"),
            ("Firmware (v)", "v", "query"),
            ("Serial number (N)", "N", "query"),
            ("Timestamps on (Z1)", "Z1", "timestamp_on"),
            ("Timestamps off (Z0)", "Z0", "timestamp_off"),
        ):
            button = ttk.Button(
                tools, text=label, command=lambda c=command, t=tag: self._send_command(c, t)
            )
            button.pack(side="left", padx=4, pady=5)
            self._command_buttons.append(button)

        bus_load = ttk.LabelFrame(self.main_tab, text="Estimated CAN bus load")
        bus_load.pack(fill="x", padx=8, pady=4)
        ttk.Label(bus_load, textvariable=self.bus_load_var, anchor="w").pack(
            fill="x", padx=6, pady=4
        )

        transmit = ttk.LabelFrame(self.main_tab, text="Transmit CAN frame")
        transmit.pack(fill="x", padx=8, pady=4)
        ttk.Label(transmit, text="Frame").grid(row=0, column=0, padx=4, pady=5)
        self.frame_type_combo = ttk.Combobox(
            transmit,
            textvariable=self.frame_type_var,
            values=("Standard data", "Extended data", "Standard remote", "Extended remote"),
            state="readonly",
            width=18,
        )
        self.frame_type_combo.grid(row=0, column=1, padx=4)
        self.frame_type_combo.bind("<<ComboboxSelected>>", self._frame_type_changed)
        ttk.Label(transmit, text="ID (hex)").grid(row=0, column=2, padx=4)
        ttk.Entry(transmit, textvariable=self.frame_id_var, width=12).grid(
            row=0, column=3, padx=4
        )
        ttk.Label(transmit, text="DLC").grid(row=0, column=4, padx=4)
        ttk.Spinbox(transmit, from_=0, to=8, textvariable=self.dlc_var, width=4).grid(
            row=0, column=5, padx=4
        )
        ttk.Label(transmit, text="Data bytes (hex)").grid(row=0, column=6, padx=4)
        self.data_entry = ttk.Entry(transmit, textvariable=self.data_var, width=24)
        self.data_entry.grid(row=0, column=7, padx=4)
        self.send_button = ttk.Button(
            transmit, text="Send", command=self.send_frame, state="disabled"
        )
        self.send_button.grid(row=0, column=8, padx=6)
        ttk.Label(
            transmit, text="Listen-only mode rejects transmissions.", foreground="#555555"
        ).grid(row=1, column=0, columnspan=9, sticky="w", padx=4, pady=(0, 4))

        frames = ttk.LabelFrame(self.main_tab, text="Received and transmitted frames")
        frames.pack(fill="both", expand=True, padx=8, pady=4)
        trace_actions = ttk.Frame(frames)
        trace_actions.pack(anchor="e", fill="x", padx=4, pady=(2, 0))
        ttk.Button(trace_actions, text="Export ASC...", command=self.export_asc).pack(
            side="right", padx=3
        )
        ttk.Button(trace_actions, text="Export CSV", command=self.export_csv).pack(
            side="right", padx=3
        )
        columns = ("time", "direction", "format", "id", "dlc", "data", "timestamp")
        self.frame_table = ttk.Treeview(frames, columns=columns, show="headings", height=12)
        headings = {
            "time": ("Host time", 145),
            "direction": ("Dir", 55),
            "format": ("Format", 115),
            "id": ("ID", 100),
            "dlc": ("DLC", 55),
            "data": ("Data", 250),
            "timestamp": ("Device timestamp", 135),
        }
        for column, (title, width) in headings.items():
            self.frame_table.heading(column, text=title)
            self.frame_table.column(column, width=width, anchor="w")
        scrollbar = ttk.Scrollbar(frames, orient="vertical", command=self.frame_table.yview)
        self.frame_table.configure(yscrollcommand=scrollbar.set)
        self.frame_table.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self._build_simulator_tab()
        self._build_dbc_tab()
        status = ttk.Label(self.root, textvariable=self.status_var, anchor="w", relief="sunken")
        status.pack(fill="x", side="bottom")

    def _build_dbc_tab(self) -> None:
        controls = ttk.LabelFrame(self.dbc_tab, text="DBC file and cyclic transmission")
        controls.pack(fill="x", padx=8, pady=6)
        self.dbc_load_button = ttk.Button(
            controls, text="Load DBC...", command=self._choose_dbc
        )
        self.dbc_load_button.grid(
            row=0, column=0, padx=4, pady=5
        )
        ttk.Label(
            controls, textvariable=self.dbc_path_var, foreground="#555555"
        ).grid(row=0, column=1, columnspan=6, sticky="w", padx=4)
        ttk.Label(controls, text="Configuration").grid(
            row=1, column=0, padx=4, pady=4, sticky="w"
        )
        self.dbc_profile_combo = ttk.Combobox(
            controls,
            textvariable=self.dbc_profile_var,
            state="readonly",
            width=24,
        )
        self.dbc_profile_combo.grid(row=1, column=1, sticky="w", padx=4, pady=4)
        ttk.Button(controls, text="Save as...", command=self._save_dbc_profile_as).grid(
            row=1, column=2, padx=3, pady=4
        )
        ttk.Button(controls, text="Save", command=self._save_dbc_profile).grid(
            row=1, column=3, padx=3, pady=4
        )
        ttk.Button(controls, text="Load", command=self._load_selected_dbc_profile).grid(
            row=1, column=4, padx=3, pady=4
        )
        self.dbc_start_button = ttk.Button(
            controls,
            text="Toggle selected",
            command=self._toggle_selected_dbc_messages,
            state="disabled",
        )
        self.dbc_start_button.grid(row=1, column=5, padx=4)
        self.dbc_stop_button = ttk.Button(
            controls,
            text="Stop all",
            command=self._stop_dbc_transmission,
            state="disabled",
        )
        self.dbc_stop_button.grid(row=1, column=6, padx=4)
        self.dbc_start_saved_button = ttk.Button(
            controls,
            text="Start saved",
            command=self._start_saved_dbc_messages,
            state="disabled",
        )
        self.dbc_start_saved_button.grid(row=1, column=7, padx=4)
        ttk.Label(
            controls,
            text="Click a message state to toggle it; double-click a cycle-time cell or signal to edit.",
            foreground="#555555",
        ).grid(row=2, column=0, columnspan=8, sticky="w", padx=4, pady=(0, 4))

        message_frame = ttk.LabelFrame(
            self.dbc_tab, text="Messages (Ctrl/Shift-click to select multiple)"
        )
        message_frame.pack(fill="both", expand=True, padx=8, pady=(0, 4))
        message_columns = (
            "id",
            "name",
            "dlc",
            "dbc_cycle",
            "override",
            "tx_cycle",
            "state",
        )
        self.dbc_message_table = ttk.Treeview(
            message_frame,
            columns=message_columns,
            show="headings",
            selectmode="extended",
            height=8,
        )
        for column, title, width in (
            ("id", "CAN ID", 90),
            ("name", "Message", 220),
            ("dlc", "DLC", 55),
            ("dbc_cycle", "DBC cycle (ms)", 110),
            ("override", "Override (ms)", 105),
            ("tx_cycle", "Effective (ms)", 115),
            ("state", "State", 100),
        ):
            self.dbc_message_table.heading(column, text=title)
            self.dbc_message_table.column(column, width=width, anchor="w")
        message_scroll = ttk.Scrollbar(
            message_frame, orient="vertical", command=self.dbc_message_table.yview
        )
        self.dbc_message_table.configure(yscrollcommand=message_scroll.set)
        self.dbc_message_table.pack(side="left", fill="both", expand=True)
        message_scroll.pack(side="right", fill="y")
        self.dbc_message_table.bind(
            "<<TreeviewSelect>>", self._show_dbc_signals
        )
        self.dbc_message_table.bind("<ButtonRelease-1>", self._click_dbc_message)
        self.dbc_message_table.bind("<Double-1>", self._edit_dbc_cycle_time)

        signal_frame = ttk.LabelFrame(
            self.dbc_tab, text="Signals in selected message (double-click to configure)"
        )
        signal_frame.pack(fill="both", expand=True, padx=8, pady=(0, 4))
        signal_columns = (
            "name",
            "layout",
            "scale",
            "range",
            "unit",
            "mode",
            "settings",
        )
        self.dbc_signal_table = ttk.Treeview(
            signal_frame,
            columns=signal_columns,
            show="headings",
            selectmode="browse",
            height=8,
        )
        for column, title, width in (
            ("name", "Signal", 180),
            ("layout", "Start / length", 95),
            ("scale", "Scale / offset", 115),
            ("range", "Value bounds", 145),
            ("unit", "Unit", 85),
            ("mode", "Update mode", 100),
            ("settings", "Value / generator range", 220),
        ):
            self.dbc_signal_table.heading(column, text=title)
            self.dbc_signal_table.column(column, width=width, anchor="w")
        signal_scroll = ttk.Scrollbar(
            signal_frame, orient="vertical", command=self.dbc_signal_table.yview
        )
        self.dbc_signal_table.configure(yscrollcommand=signal_scroll.set)
        self.dbc_signal_table.pack(side="left", fill="both", expand=True)
        signal_scroll.pack(side="right", fill="y")
        self.dbc_signal_table.bind("<Double-1>", self._edit_dbc_signal)
        ttk.Label(
            self.dbc_tab, textvariable=self.dbc_status_var, anchor="w"
        ).pack(fill="x", padx=10, pady=(0, 5))

    def _choose_dbc(self) -> None:
        if self._dbc_pending:
            messagebox.showinfo(
                "Transmission in progress",
                "Wait for the in-flight DBC frame to finish before loading another file.",
            )
            return
        path = filedialog.askopenfilename(
            title="Load DBC file",
            filetypes=(("DBC files", "*.dbc"), ("All files", "*.*")),
        )
        if not path:
            return
        try:
            database = load_dbc(path)
        except (OSError, ValueError, cantools.database.errors.Error) as exc:
            messagebox.showerror("Could not load DBC", str(exc))
            return

        self._populate_dbc_database(path, database)

    def _populate_dbc_database(
        self, path: str, database: cantools.database.Database
    ) -> None:
        self._stop_dbc_transmission(silent=True)
        self.dbc_database = database
        self.dbc_messages.clear()
        self.dbc_signal_settings.clear()
        self.dbc_cycle_overrides.clear()
        self._dbc_sweep_values.clear()
        self._dbc_sweep_directions.clear()
        self._dbc_profile_start_keys.clear()
        for item in self.dbc_message_table.get_children():
            self.dbc_message_table.delete(item)
        self.dbc_path_var.set(path)
        for index, message in enumerate(
            sorted(database.messages, key=lambda item: (item.frame_id, item.name))
        ):
            key = str(index)
            self.dbc_messages[key] = message
            for signal in message.signals:
                self.dbc_signal_settings[(key, signal.name)] = (
                    default_signal_settings(signal)
                )
            frame_width = 8 if message.is_extended_frame else 3
            dbc_cycle = (
                "--"
                if message.cycle_time is None
                else f"{message.cycle_time:g}"
            )
            self.dbc_message_table.insert(
                "",
                "end",
                iid=key,
                values=(
                    f"{message.frame_id:0{frame_width}X}",
                    message.name,
                    message.length,
                    dbc_cycle,
                    "",
                    self._dbc_cycle_display(key),
                    "Stopped",
                ),
            )
        if not database.messages:
            self.dbc_status_var.set("The DBC contains no messages.")
        else:
            self.dbc_status_var.set(
                f"Loaded {len(database.messages)} message(s) from {Path(path).name}."
            )
        self._show_dbc_signals()
        self._update_dbc_availability()

    @staticmethod
    def _dbc_message_profile_key(message: Message) -> str:
        return f"{message.frame_id}:{message.name}"

    def _serialize_dbc_profile(self) -> Dict[str, Any]:
        if self.dbc_database is None:
            raise DbcProfilesError("Load a DBC file before saving a configuration.")
        selected = set(self.dbc_message_table.selection())
        messages = []
        cycle_overrides = []
        signals = []
        for key, message in self.dbc_messages.items():
            identity = {
                "frame_id": message.frame_id,
                "message": message.name,
                "selected": key in selected,
                "running": (
                    key in self._dbc_next_tx
                    or key in self._dbc_profile_start_keys
                ),
            }
            messages.append(identity)
        for key, cycle_time in self.dbc_cycle_overrides.items():
            message = self.dbc_messages[key]
            cycle_overrides.append(
                {**self._dbc_message_profile_identity(message), "cycle_ms": cycle_time}
            )
        for (key, signal_name), settings in self.dbc_signal_settings.items():
            message = self.dbc_messages[key]
            signals.append(
                {
                    **self._dbc_message_profile_identity(message),
                    "signal": signal_name,
                    "mode": settings.mode,
                    "value": settings.value,
                    "minimum": settings.minimum,
                    "maximum": settings.maximum,
                    "step": settings.step,
                }
            )
        return {
            "dbc_path": str(Path(self.dbc_path_var.get()).resolve()),
            "messages": messages,
            "cycle_overrides": cycle_overrides,
            "signals": signals,
        }

    @staticmethod
    def _dbc_message_profile_identity(message: Message) -> Dict[str, Any]:
        return {"frame_id": message.frame_id, "message": message.name}

    def _refresh_dbc_profile_names(self) -> None:
        names = tuple(sorted(self.dbc_profiles["profiles"], key=str.casefold))
        self.dbc_profile_combo.configure(values=names)
        if self.dbc_profile_var.get() not in names:
            self.dbc_profile_var.set("")

    def _save_dbc_profile_as(self) -> None:
        name = simpledialog.askstring(
            "Save DBC configuration",
            "Configuration name:",
            parent=self.root,
        )
        if name is None:
            return
        name = name.strip()
        if not name:
            messagebox.showerror("Invalid name", "Enter a configuration name.")
            return
        if name in self.dbc_profiles["profiles"] and not messagebox.askyesno(
            "Replace configuration",
            f"A configuration named '{name}' already exists. Replace it?",
            parent=self.root,
        ):
            return
        self._save_dbc_profile(name)

    def _save_dbc_profile(self, name: Optional[str] = None) -> None:
        profile_name = name or self.dbc_profile_var.get().strip()
        if not profile_name:
            self._save_dbc_profile_as()
            return
        try:
            profile = self._serialize_dbc_profile()
            profiles = dict(self.dbc_profiles["profiles"])
            profiles[profile_name] = profile
            updated = {"last_profile": profile_name, "profiles": profiles}
            save_profiles(self.dbc_profiles_path, updated)
        except (DbcProfilesError, OSError, TypeError, ValueError) as exc:
            messagebox.showerror("Could not save DBC configuration", str(exc))
            return
        self.dbc_profiles = updated
        self.dbc_profile_var.set(profile_name)
        self._refresh_dbc_profile_names()
        self.dbc_profile_var.set(profile_name)
        self.dbc_status_var.set(f"Saved DBC configuration '{profile_name}'.")

    def _load_selected_dbc_profile(self) -> None:
        profile_name = self.dbc_profile_var.get().strip()
        if not profile_name:
            return
        self._load_dbc_profile(profile_name)

    def _restore_last_dbc_profile(self) -> None:
        try:
            self.dbc_profiles = load_profiles(self.dbc_profiles_path)
        except DbcProfilesError as exc:
            self.dbc_status_var.set(str(exc))
            return
        self._refresh_dbc_profile_names()
        last_profile = self.dbc_profiles["last_profile"]
        if last_profile is not None:
            self.dbc_profile_var.set(last_profile)
            self._load_dbc_profile(last_profile, startup=True)

    def _load_dbc_profile(self, profile_name: str, startup: bool = False) -> None:
        profile = self.dbc_profiles["profiles"].get(profile_name)
        if profile is None:
            messagebox.showerror(
                "Configuration not found",
                f"DBC configuration '{profile_name}' does not exist.",
            )
            return
        if self._dbc_pending:
            messagebox.showinfo(
                "Transmission in progress",
                "Wait for the in-flight DBC frame to finish before loading a configuration.",
            )
            return
        dbc_path = profile.get("dbc_path")
        if not isinstance(dbc_path, str) or not dbc_path:
            self._show_dbc_profile_error(
                f"Configuration '{profile_name}' does not have a DBC file path.",
                startup,
            )
            return
        try:
            database = load_dbc(dbc_path)
            self._apply_dbc_profile(profile, dbc_path, database)
        except (OSError, ValueError, TypeError, KeyError, cantools.database.errors.Error) as exc:
            self._show_dbc_profile_error(
                f"Could not load DBC configuration '{profile_name}': {exc}",
                startup,
            )
            return

        updated = {
            "last_profile": profile_name,
            "profiles": self.dbc_profiles["profiles"],
        }
        try:
            save_profiles(self.dbc_profiles_path, updated)
        except DbcProfilesError as exc:
            messagebox.showerror("Could not save DBC configuration state", str(exc))
        else:
            self.dbc_profiles = updated
        self.dbc_profile_var.set(profile_name)
        self.dbc_status_var.set(
            f"Loaded '{profile_name}'. Open CAN in Normal mode, then use Start saved "
            "to resume previously running messages."
        )

    def _show_dbc_profile_error(self, message: str, startup: bool) -> None:
        self.dbc_status_var.set(message)
        if not startup:
            messagebox.showerror("Could not load DBC configuration", message)

    def _apply_dbc_profile(
        self,
        profile: Dict[str, Any],
        dbc_path: str,
        database: cantools.database.Database,
    ) -> None:
        if not isinstance(profile.get("messages"), list):
            raise ValueError("'messages' must be a list.")
        if not isinstance(profile.get("cycle_overrides"), list):
            raise ValueError("'cycle_overrides' must be a list.")
        if not isinstance(profile.get("signals"), list):
            raise ValueError("'signals' must be a list.")

        self._populate_dbc_database(dbc_path, database)
        message_keys = {
            self._dbc_message_profile_key(message): key
            for key, message in self.dbc_messages.items()
        }
        selected_keys = []
        saved_running_keys = set()
        for entry in profile["messages"]:
            profile_key = self._validate_dbc_profile_identity(entry)
            if not isinstance(entry.get("selected", True), bool):
                raise ValueError(f"{profile_key}: selected must be true or false.")
            if not isinstance(entry.get("running", False), bool):
                raise ValueError(f"{profile_key}: running must be true or false.")
            key = message_keys.get(profile_key)
            if key is not None and entry.get("selected", True) is True:
                selected_keys.append(key)
            if key is not None and entry.get("running", False) is True:
                saved_running_keys.add(key)

        for entry in profile["cycle_overrides"]:
            profile_key = self._validate_dbc_profile_identity(entry)
            key = message_keys.get(profile_key)
            if key is None:
                continue
            cycle_ms = float(entry["cycle_ms"])
            if not 1 <= cycle_ms <= 3_600_000:
                raise ValueError(f"{profile_key}: invalid cycle override.")
            self.dbc_cycle_overrides[key] = cycle_ms
            self._refresh_dbc_message(key)

        for entry in profile["signals"]:
            profile_key = self._validate_dbc_profile_identity(entry)
            key = message_keys.get(profile_key)
            if key is None:
                continue
            signal_name = entry.get("signal")
            setting_key = (key, signal_name)
            if setting_key not in self.dbc_signal_settings:
                continue
            settings = SignalTxSettings(
                mode=entry["mode"],
                value=float(entry["value"]),
                minimum=float(entry["minimum"]),
                maximum=float(entry["maximum"]),
                step=float(entry["step"]),
            )
            if settings.mode not in ("raw", "sweep", "random"):
                raise ValueError(f"{profile_key}.{signal_name}: invalid mode.")
            signal = self.dbc_messages[key].get_signal_by_name(signal_name)
            minimum, maximum = signal_value_bounds(signal)
            if not all(
                math.isfinite(value)
                for value in (
                    settings.value,
                    settings.minimum,
                    settings.maximum,
                    settings.step,
                )
            ):
                raise ValueError(f"{profile_key}.{signal_name}: values must be finite.")
            if settings.mode == "raw" and not minimum <= settings.value <= maximum:
                raise ValueError(f"{profile_key}.{signal_name}: static value out of range.")
            if settings.mode in ("sweep", "random") and not (
                minimum <= settings.minimum <= settings.maximum <= maximum
            ):
                raise ValueError(f"{profile_key}.{signal_name}: bounds are out of range.")
            if settings.mode == "sweep" and not (
                0 < settings.step <= settings.maximum - settings.minimum
            ):
                raise ValueError(f"{profile_key}.{signal_name}: invalid sweep step.")
            self.dbc_signal_settings[setting_key] = settings

        if selected_keys:
            self.dbc_message_table.selection_set(selected_keys)
        self._dbc_profile_start_keys = saved_running_keys
        for key in self.dbc_messages:
            self._refresh_dbc_message(key)
        self._show_dbc_signals()
        self._update_dbc_availability()

    @staticmethod
    def _validate_dbc_profile_identity(entry: Any) -> str:
        if not isinstance(entry, dict):
            raise ValueError("A message configuration must be an object.")
        frame_id = entry.get("frame_id")
        message_name = entry.get("message")
        if (
            isinstance(frame_id, bool)
            or not isinstance(frame_id, int)
            or not isinstance(message_name, str)
        ):
            raise ValueError("A message configuration needs a CAN ID and message name.")
        return f"{frame_id}:{message_name}"

    def _dbc_cycle_display(self, key: str) -> str:
        override = self.dbc_cycle_overrides.get(key)
        if override is not None:
            return f"{override:g} (override)"
        message = self.dbc_messages[key]
        if message.cycle_time is not None:
            return f"{message.cycle_time:g} (DBC)"
        return "100 (fallback)"

    def _refresh_dbc_message(self, key: str) -> None:
        if not self.dbc_message_table.exists(key):
            return
        message = self.dbc_messages[key]
        dbc_cycle = (
            "--"
            if message.cycle_time is None
            else f"{message.cycle_time:g}"
        )
        override = self.dbc_cycle_overrides.get(key)
        self.dbc_message_table.item(
            key,
            values=(
                self.dbc_message_table.item(key, "values")[0],
                message.name,
                message.length,
                dbc_cycle,
                "" if override is None else f"{override:g}",
                self._dbc_cycle_display(key),
                (
                    "Running"
                    if key in self._dbc_next_tx
                    else "Ready"
                    if key in self._dbc_profile_start_keys
                    else "Stopping..."
                    if key in self._dbc_pending
                    else "Stopped"
                ),
            ),
        )

    def _edit_dbc_cycle_time(self, event) -> None:
        if self.dbc_message_table.identify_column(event.x) not in ("#4", "#5", "#6"):
            return
        key = self.dbc_message_table.identify_row(event.y)
        if not key:
            return

        window = tk.Toplevel(self.root)
        window.title("Override cycle time")
        window.transient(self.root)
        fields = ttk.Frame(window, padding=10)
        fields.pack(fill="both", expand=True)
        ttk.Label(fields, text=f"Cycle time for {self.dbc_messages[key].name} (ms)").grid(
            row=0, column=0, sticky="w"
        )
        value_var = tk.StringVar(
            value=(
                ""
                if key not in self.dbc_cycle_overrides
                else f"{self.dbc_cycle_overrides[key]:g}"
            )
        )
        entry = ttk.Entry(fields, textvariable=value_var, width=18)
        entry.grid(row=1, column=0, sticky="ew", pady=5)
        ttk.Label(
            fields,
            text="Leave blank to use the DBC cycle time (100 ms if none is defined).",
            foreground="#555555",
        ).grid(row=2, column=0, sticky="w")

        def save() -> None:
            text = value_var.get().strip()
            if text:
                try:
                    cycle_ms = float(text)
                except ValueError:
                    messagebox.showerror(
                        "Invalid cycle time", "Enter a positive number of milliseconds.", parent=window
                    )
                    return
                if not 1 <= cycle_ms <= 3_600_000:
                    messagebox.showerror(
                        "Invalid cycle time",
                        "Cycle time must be at least 1 and no more than 3,600,000 ms.",
                        parent=window,
                    )
                    return
                self.dbc_cycle_overrides[key] = cycle_ms
            else:
                self.dbc_cycle_overrides.pop(key, None)
            if key in self._dbc_periods:
                self._dbc_periods[key] = self._dbc_effective_cycle(key) / 1000
                self._dbc_next_tx[key] = time.monotonic() + self._dbc_periods[key]
            self._refresh_dbc_message(key)
            window.destroy()

        buttons = ttk.Frame(fields)
        buttons.grid(row=3, column=0, sticky="e", pady=(8, 0))
        ttk.Button(buttons, text="Cancel", command=window.destroy).pack(
            side="right", padx=(4, 0)
        )
        ttk.Button(buttons, text="Save", command=save).pack(side="right")
        entry.focus_set()
        window.bind("<Return>", lambda _event: save())
        window.wait_visibility()
        window.grab_set()

    def _dbc_effective_cycle(self, key: str) -> float:
        override = self.dbc_cycle_overrides.get(key)
        if override is not None:
            return override
        cycle_time = self.dbc_messages[key].cycle_time
        return 100.0 if cycle_time is None else float(cycle_time)

    def _show_dbc_signals(self, _event=None) -> None:
        for item in self.dbc_signal_table.get_children():
            self.dbc_signal_table.delete(item)
        selection = self.dbc_message_table.selection()
        if not selection:
            return
        key = selection[0]
        message = self.dbc_messages.get(key)
        if message is None:
            return
        for signal in message.signals:
            settings = self.dbc_signal_settings[(key, signal.name)]
            minimum, maximum = signal_value_bounds(signal)
            self.dbc_signal_table.insert(
                "",
                "end",
                iid=signal.name,
                values=(
                    signal.name,
                    f"{signal.start} / {signal.length}",
                    f"{signal.scale:g} / {signal.offset:g}",
                    f"{minimum:g} .. {maximum:g}",
                    signal.unit or "",
                    settings.mode,
                    self._dbc_signal_settings_display(settings),
                ),
            )

    @staticmethod
    def _dbc_signal_settings_display(settings: SignalTxSettings) -> str:
        if settings.mode == "raw":
            return f"{settings.value:g}"
        return f"{settings.minimum:g} .. {settings.maximum:g}, step {settings.step:g}"

    def _edit_dbc_signal(self, event) -> None:
        key_selection = self.dbc_message_table.selection()
        signal_name = self.dbc_signal_table.identify_row(event.y)
        if not key_selection or not signal_name:
            return
        message_key = key_selection[0]
        message = self.dbc_messages[message_key]
        signal = message.get_signal_by_name(signal_name)
        settings = self.dbc_signal_settings[(message_key, signal_name)]

        window = tk.Toplevel(self.root)
        window.title(f"Configure {message.name}.{signal.name}")
        window.transient(self.root)
        fields = ttk.Frame(window, padding=10)
        fields.pack(fill="both", expand=True)
        ttk.Label(
            fields,
            text=f"{signal.name} ({signal.unit or 'unitless'})",
        ).grid(row=0, column=0, columnspan=2, sticky="w")
        mode_var = tk.StringVar(value=settings.mode)
        value_var = tk.StringVar(value=f"{settings.value:g}")
        minimum_var = tk.StringVar(value=f"{settings.minimum:g}")
        maximum_var = tk.StringVar(value=f"{settings.maximum:g}")
        step_var = tk.StringVar(value=f"{settings.step:g}")
        for row, label, variable in (
            (1, "Update mode", mode_var),
            (2, "Static value", value_var),
            (3, "Minimum", minimum_var),
            (4, "Maximum", maximum_var),
            (5, "Sweep step", step_var),
        ):
            ttk.Label(fields, text=label).grid(row=row, column=0, sticky="w", pady=2)
            if row == 1:
                ttk.Combobox(
                    fields,
                    textvariable=variable,
                    values=("raw", "sweep", "random"),
                    state="readonly",
                    width=18,
                ).grid(row=row, column=1, sticky="ew", pady=2)
            else:
                ttk.Entry(fields, textvariable=variable, width=20).grid(
                    row=row, column=1, sticky="ew", pady=2
                )
        ttk.Label(
            fields,
            text="Static values and generator ranges use physical (scaled) units.",
            foreground="#555555",
        ).grid(row=6, column=0, columnspan=2, sticky="w", pady=(4, 0))

        def save() -> None:
            try:
                updated = SignalTxSettings(
                    mode=mode_var.get(),
                    value=float(value_var.get()),
                    minimum=float(minimum_var.get()),
                    maximum=float(maximum_var.get()),
                    step=float(step_var.get()),
                )
            except ValueError:
                messagebox.showerror(
                    "Invalid signal value",
                    "Signal values, bounds, and step must be numeric.",
                    parent=window,
                )
                return
            if updated.mode == "sweep" and (
                updated.maximum <= updated.minimum
                or updated.step <= 0
                or updated.step > updated.maximum - updated.minimum
            ):
                messagebox.showerror(
                    "Invalid sweep range",
                    "A sweep requires maximum > minimum and a positive step no "
                    "larger than the range.",
                    parent=window,
                )
                return
            if updated.mode == "random" and updated.minimum > updated.maximum:
                messagebox.showerror(
                    "Invalid random range",
                    "Random minimum must not exceed maximum.",
                    parent=window,
                )
                return
            minimum, maximum = signal_value_bounds(signal)
            if updated.mode in ("sweep", "random") and (
                updated.minimum < minimum or updated.maximum > maximum
            ):
                messagebox.showerror(
                    "Invalid signal range",
                    f"Generator bounds must be between {minimum:g} and {maximum:g}.",
                    parent=window,
                )
                return
            if updated.mode == "raw" and not minimum <= updated.value <= maximum:
                messagebox.showerror(
                    "Invalid static value",
                    f"Value must be between {minimum:g} and {maximum:g}.",
                    parent=window,
                )
                return
            self.dbc_signal_settings[(message_key, signal_name)] = updated
            self._show_dbc_signals()
            window.destroy()

        buttons = ttk.Frame(fields)
        buttons.grid(row=7, column=0, columnspan=2, sticky="e", pady=(8, 0))
        ttk.Button(buttons, text="Cancel", command=window.destroy).pack(
            side="right", padx=(4, 0)
        )
        ttk.Button(buttons, text="Save", command=save).pack(side="right")
        window.wait_visibility()
        window.grab_set()

    def _update_dbc_availability(self) -> None:
        can_transmit = (
            self.dbc_database is not None
            and bool(self.dbc_messages)
            and self.connected
            and self.can_open
            and self.mode_var.get() == "Normal"
        )
        self.dbc_start_button.configure(state="normal" if can_transmit else "disabled")
        self.dbc_stop_button.configure(
            state="normal" if self.dbc_running else "disabled"
        )
        if self.dbc_start_saved_button is not None:
            self.dbc_start_saved_button.configure(
                state=(
                    "normal"
                    if can_transmit and self._dbc_profile_start_keys
                    else "disabled"
                )
            )
        if self.dbc_load_button is not None:
            self.dbc_load_button.configure(
                state="disabled" if self._dbc_pending else "normal"
            )

    def _click_dbc_message(self, event):
        if self.dbc_message_table.identify_column(event.x) != "#7":
            return
        key = self.dbc_message_table.identify_row(event.y)
        if not key:
            return
        self.dbc_message_table.selection_set(key)
        self.dbc_message_table.focus(key)
        self._show_dbc_signals()
        self._toggle_dbc_message(key)
        return "break"

    def _toggle_selected_dbc_messages(self) -> None:
        selected = self.dbc_message_table.selection()
        if not selected:
            messagebox.showinfo(
                "Select messages", "Select one or more DBC messages to toggle."
            )
            return
        to_start = [key for key in selected if key not in self._dbc_next_tx]
        if to_start and not self._validate_dbc_start(to_start):
            return
        for key in selected:
            if key in self._dbc_next_tx:
                self._stop_dbc_message(key)
            else:
                self._start_dbc_message(key)
        self._update_dbc_availability()
        self._schedule_dbc_transmitter()

    def _toggle_dbc_message(self, key: str) -> None:
        if key in self._dbc_next_tx:
            self._stop_dbc_message(key)
        else:
            if not self._validate_dbc_start([key]):
                return
            self._start_dbc_message(key)
        self._update_dbc_availability()
        self._schedule_dbc_transmitter()

    def _start_saved_dbc_messages(self) -> None:
        keys = sorted(self._dbc_profile_start_keys)
        if not keys:
            return
        if not self._validate_dbc_start(keys):
            return
        for key in keys:
            self._start_dbc_message(key)
        self._dbc_profile_start_keys.clear()
        self._update_dbc_availability()
        self._schedule_dbc_transmitter()

    def _validate_dbc_start(self, keys: Sequence[str]) -> bool:
        if not self.can_open or self.mode_var.get() != "Normal" or self.worker is None:
            messagebox.showerror(
                "CAN is not ready",
                "Open CAN in Normal mode before starting DBC transmission.",
            )
            return False
        unsupported = [
            self.dbc_messages[key]
            for key in keys
            if self.dbc_messages[key].length > 8
        ]
        if unsupported:
            names = ", ".join(message.name for message in unsupported)
            messagebox.showerror(
                "Unsupported CAN frame size",
                f"The current SLCAN interface supports up to 8 data bytes. "
                f"Remove these CAN FD or oversized messages from the selection: {names}",
            )
            return False
        invalid_cycles = [
            self.dbc_messages[key].name
            for key in keys
            if not 1 <= self._dbc_effective_cycle(key) <= 3_600_000
        ]
        if invalid_cycles:
            messagebox.showerror(
                "Invalid cycle time",
                "Set a cycle time from 1 to 3,600,000 ms for: "
                + ", ".join(invalid_cycles),
            )
            return False
        return True

    def _start_dbc_message(self, key: str) -> None:
        self._dbc_profile_start_keys.discard(key)
        self._dbc_next_tx[key] = time.monotonic()
        self._dbc_periods[key] = self._dbc_effective_cycle(key) / 1000
        self._dbc_sweep_values[key] = {}
        self._dbc_sweep_directions[key] = {}
        self.dbc_running = True
        self.dbc_status_var.set(
            f"Running {self.dbc_messages[key].name}; "
            f"{len(self._dbc_next_tx)} message(s) active."
        )
        self._refresh_dbc_message(key)

    def _stop_dbc_message(self, key: str) -> None:
        self._dbc_profile_start_keys.discard(key)
        self._dbc_next_tx.pop(key, None)
        self._dbc_periods.pop(key, None)
        self._dbc_sweep_values.pop(key, None)
        self._dbc_sweep_directions.pop(key, None)
        self.dbc_running = bool(self._dbc_next_tx)
        self.dbc_status_var.set(
            f"Stopped {self.dbc_messages[key].name}; "
            f"{len(self._dbc_next_tx)} message(s) active."
        )
        self._refresh_dbc_message(key)

    def _schedule_dbc_transmitter(self) -> None:
        if not self.dbc_running or self._dbc_after is not None:
            return
        self._dbc_after = self.root.after(1, self._run_dbc_transmitter)

    def _stop_dbc_transmission(self, silent: bool = False) -> None:
        was_running = self.dbc_running or bool(self._dbc_profile_start_keys)
        self.dbc_running = False
        self.dbc_message_table.configure(selectmode="extended")
        if self._dbc_after is not None:
            self.root.after_cancel(self._dbc_after)
            self._dbc_after = None
        self._dbc_next_tx.clear()
        self._dbc_periods.clear()
        self._dbc_sweep_values.clear()
        self._dbc_sweep_directions.clear()
        self._dbc_profile_start_keys.clear()
        self._update_dbc_availability()
        for key in self.dbc_messages:
            self._refresh_dbc_message(key)
        if was_running and not silent:
            self.dbc_status_var.set(
                "Stopped all DBC messages. An in-flight frame may still complete."
            )

    def _run_dbc_transmitter(self) -> None:
        self._dbc_after = None
        if (
            not self.dbc_running
            or not self.can_open
            or self.mode_var.get() != "Normal"
            or not self.connected
            or self.worker is None
        ):
            self._stop_dbc_transmission(silent=True)
            self.dbc_status_var.set(
                "DBC transmission stopped because CAN is no longer open in Normal mode."
            )
            return

        now = time.monotonic()
        due = sorted(
            (deadline, key)
            for key, deadline in self._dbc_next_tx.items()
            if deadline <= now
        )
        for deadline, key in due:
            period = self._dbc_periods[key]
            missed_periods = max(1, int((now - deadline) / period) + 1)
            self._dbc_next_tx[key] = deadline + missed_periods * period
            if key not in self._dbc_pending:
                self._queue_dbc_message(key)

        if self._dbc_next_tx:
            next_deadline = min(self._dbc_next_tx.values())
            delay_ms = max(1, int((next_deadline - time.monotonic()) * 1000))
            self._dbc_after = self.root.after(delay_ms, self._run_dbc_transmitter)

    def _queue_dbc_message(self, key: str) -> None:
        if self.worker is None:
            return
        message = self.dbc_messages[key]
        settings = {
            signal.name: self.dbc_signal_settings[(key, signal.name)]
            for signal in message.signals
        }
        try:
            payload = encode_message(
                message,
                settings,
                self._dbc_sweep_values[key],
                sweep_directions=self._dbc_sweep_directions[key],
            )
            command = encode_frame(
                CanFrame(
                    message.frame_id,
                    message.is_extended_frame,
                    False,
                    message.length,
                    payload,
                )
            )
        except (ValueError, cantools.database.errors.Error) as exc:
            self._stop_dbc_message(key)
            messagebox.showerror(
                "DBC transmission failed",
                f"{message.name} was stopped: {exc}",
            )
            return

        self._dbc_pending.add(key)
        self.worker.send(command, f"dbc_tx:{key}")
        self._refresh_dbc_message(key)
        self._update_dbc_availability()

    def _build_simulator_tab(self) -> None:
        controls = ttk.LabelFrame(self.simulator_tab, text="OBD-II ECU simulator")
        controls.pack(fill="x", padx=8, pady=6)
        self.simulator_start_button = ttk.Button(
            controls,
            text="Start simulator",
            command=self._toggle_simulator,
            state="disabled",
        )
        self.simulator_start_button.grid(row=0, column=0, padx=4, pady=5)
        ttk.Label(controls, text="Accepted request IDs (hex)").grid(
            row=0, column=1, padx=4
        )
        ttk.Entry(controls, textvariable=self.simulator_request_ids_var, width=18).grid(
            row=0, column=2, padx=4
        )
        ttk.Label(controls, text="Response ID (hex)").grid(row=0, column=3, padx=4)
        ttk.Entry(
            controls, textvariable=self.simulator_response_id_var, width=10
        ).grid(row=0, column=4, padx=4)
        ttk.Button(controls, text="Save config", command=self._save_simulator_config).grid(
            row=0, column=5, padx=4
        )
        ttk.Button(controls, text="Load JSON...", command=self._choose_simulator_config).grid(
            row=0, column=6, padx=4
        )
        ttk.Button(controls, text="Open JSON", command=self._open_simulator_config).grid(
            row=0, column=7, padx=4
        )
        ttk.Label(
            controls,
            textvariable=self.simulator_config_path_var,
            foreground="#555555",
        ).grid(row=1, column=0, columnspan=8, sticky="w", padx=4, pady=(0, 4))
        ttk.Checkbutton(
            controls,
            text="MIL on",
            variable=self.simulator_mil_var,
            command=self._save_simulator_config,
        ).grid(row=2, column=0, sticky="w", padx=4, pady=(0, 4))
        ttk.Checkbutton(
            controls,
            text="Multi-PID responses",
            variable=self.simulator_multi_pid_var,
            command=self._save_simulator_config,
        ).grid(row=2, column=1, sticky="w", padx=4, pady=(0, 4))

        ttk.Label(
            self.simulator_tab,
            textvariable=self.simulator_status_var,
            anchor="w",
        ).pack(fill="x", padx=10, pady=(0, 5))

        config_tabs = ttk.Notebook(self.simulator_tab)
        config_tabs.pack(fill="both", expand=True, padx=8, pady=4)
        pid_tab = ttk.Frame(config_tabs)
        dtc_tab = ttk.Frame(config_tabs)
        config_tabs.add(pid_tab, text="Mode 01 PIDs")
        config_tabs.add(dtc_tab, text="Active DTCs (Mode 03)")

        pid_frame = ttk.LabelFrame(
            pid_tab,
            text="Double-click Supported to toggle; double-click another cell to edit",
        )
        pid_frame.pack(fill="both", expand=True)
        columns = ("pid", "name", "unit", "supported", "value")
        self.simulator_table = ttk.Treeview(
            pid_frame, columns=columns, show="headings", selectmode="browse"
        )
        headings = {
            "pid": ("PID", 65),
            "name": ("Parameter", 270),
            "unit": ("Unit", 65),
            "supported": ("Supported", 85),
            "value": ("Current value / raw bytes", 420),
        }
        for column, (title, width) in headings.items():
            self.simulator_table.heading(column, text=title)
            self.simulator_table.column(column, width=width, anchor="w")
        y_scroll = ttk.Scrollbar(
            pid_frame, orient="vertical", command=self.simulator_table.yview
        )
        self.simulator_table.configure(yscrollcommand=y_scroll.set)
        self.simulator_table.pack(side="left", fill="both", expand=True)
        y_scroll.pack(side="right", fill="y")
        self.simulator_table.bind("<Double-1>", self._toggle_pid_support)
        self.simulator_table.bind(
            "<space>", lambda _event: self._toggle_pid_support()
        )
        pid_buttons = ttk.Frame(pid_tab)
        pid_buttons.pack(fill="x", pady=(4, 0))
        ttk.Button(pid_buttons, text="Add PID", command=self._add_pid).pack(
            side="left", padx=(0, 4)
        )
        ttk.Button(pid_buttons, text="Edit PID", command=self._edit_selected_pid).pack(
            side="left", padx=4
        )
        ttk.Button(
            pid_buttons, text="Remove custom PID", command=self._remove_selected_pid
        ).pack(side="left", padx=4)

        dtc_frame = ttk.LabelFrame(
            dtc_tab,
            text="Mode 03 reports active codes (up to three per single-frame response)",
        )
        dtc_frame.pack(fill="both", expand=True)
        dtc_columns = ("code", "description", "active")
        self.dtc_table = ttk.Treeview(
            dtc_frame, columns=dtc_columns, show="headings", selectmode="browse"
        )
        for column, title, width in (
            ("code", "DTC", 90),
            ("description", "Description", 520),
            ("active", "Active", 90),
        ):
            self.dtc_table.heading(column, text=title)
            self.dtc_table.column(column, width=width, anchor="w")
        dtc_scroll = ttk.Scrollbar(
            dtc_frame, orient="vertical", command=self.dtc_table.yview
        )
        self.dtc_table.configure(yscrollcommand=dtc_scroll.set)
        self.dtc_table.pack(side="left", fill="both", expand=True)
        dtc_scroll.pack(side="right", fill="y")
        self.dtc_table.bind("<Double-1>", self._toggle_dtc_active)
        dtc_buttons = ttk.Frame(dtc_tab)
        dtc_buttons.pack(fill="x", pady=(4, 0))
        ttk.Button(dtc_buttons, text="Add DTC", command=self._add_dtc).pack(
            side="left", padx=(0, 4)
        )
        ttk.Button(dtc_buttons, text="Edit DTC", command=self._edit_selected_dtc).pack(
            side="left", padx=4
        )
        ttk.Button(
            dtc_buttons, text="Remove DTC", command=self._remove_selected_dtc
        ).pack(side="left", padx=4)
        self.simulator_config_path_var.set(str(self.simulator_config_path))
        self.simulator_mil_var.set(self.simulator.config["mil_on"])
        self._refresh_simulator_fields()
        self._refresh_simulator_table()
        self._refresh_dtc_table()
        if self.simulator_config_error:
            self.simulator_status_var.set(
                f"Config load failed; using defaults: {self.simulator_config_error}"
            )
        self._schedule_simulator_refresh()

    def _refresh_simulator_fields(self) -> None:
        self.simulator_request_ids_var.set(
            ", ".join(f"{value:03X}" for value in self.simulator.request_ids)
        )
        self.simulator_response_id_var.set(f"{self.simulator.response_id:03X}")
        self.simulator_mil_var.set(self.simulator.config["mil_on"])
        self.simulator_multi_pid_var.set(
            self.simulator.config["multi_pid_response_active"]
        )

    def _refresh_simulator_table(self) -> None:
        rows = self.simulator.display_rows()
        expected = {row["pid"] for row in rows}
        for item in self.simulator_table.get_children():
            if item not in expected:
                self.simulator_table.delete(item)
        for index, row in enumerate(rows):
            values = (
                row["pid"],
                row["name"],
                row["unit"],
                row["supported"],
                row["value"],
            )
            if self.simulator_table.exists(row["pid"]):
                self.simulator_table.item(row["pid"], values=values)
            else:
                self.simulator_table.insert(
                    "", index, iid=row["pid"], values=values
                )
            self.simulator_table.move(row["pid"], "", index)

    def _refresh_dtc_table(self) -> None:
        expected = {dtc["code"] for dtc in self.simulator.config["dtcs"]}
        for item in self.dtc_table.get_children():
            if item not in expected:
                self.dtc_table.delete(item)
        for index, dtc in enumerate(self.simulator.config["dtcs"]):
            values = (
                dtc["code"],
                dtc["description"],
                "Yes" if dtc["active"] else "No",
            )
            if self.dtc_table.exists(dtc["code"]):
                self.dtc_table.item(dtc["code"], values=values)
            else:
                self.dtc_table.insert(
                    "", index, iid=dtc["code"], values=values
                )
            self.dtc_table.move(dtc["code"], "", index)

    def _schedule_simulator_refresh(self) -> None:
        if self._closing:
            return
        self._refresh_simulator_table()
        self._simulator_refresh_after = self.root.after(
            500, self._schedule_simulator_refresh
        )

    def _toggle_pid_support(self, event=None) -> None:
        if event and hasattr(event, "x") and self.simulator_table.identify_column(event.x) != "#4":
            item = self.simulator_table.identify_row(event.y)
            if item:
                self._show_pid_editor(int(item, 16))
            return
        item = self.simulator_table.identify_row(event.y) if event and hasattr(event, "y") else ""
        if not item and self.simulator_table.selection():
            item = self.simulator_table.selection()[0]
        try:
            pid = int(item, 16)
        except (TypeError, ValueError):
            return
        if f"{pid:02X}" not in self.simulator.config["pids"]:
            return
        config = copy.deepcopy(self.simulator.config)
        entry = config["pids"][f"{pid:02X}"]
        entry["supported"] = not entry["supported"]
        self._commit_simulator_config(config, "Could not save ECU config")

    def _commit_simulator_config(self, config: dict, error_title: str) -> bool:
        old_config = self.simulator.config
        try:
            self.simulator.config = save_config(self.simulator_config_path, config)
        except SimulatorConfigError as exc:
            self.simulator.config = old_config
            messagebox.showerror(error_title, str(exc))
            return False
        self.simulator_mil_var.set(self.simulator.config["mil_on"])
        self._refresh_simulator_table()
        self._refresh_dtc_table()
        return True

    def _add_pid(self) -> None:
        used = {int(key, 16) for key in self.simulator.config["pids"]}
        pid = next(
            (
                value
                for value in range(1, 0x100)
                if value not in used and value not in PID_BITMAP_PAGES
            ),
            None,
        )
        if pid is None:
            messagebox.showerror("No PID available", "All configurable PID values are in use.")
            return
        self._show_pid_editor(pid, is_new=True)

    def _edit_selected_pid(self) -> None:
        selection = self.simulator_table.selection()
        if not selection:
            messagebox.showinfo("Select a PID", "Select a PID row to edit.")
            return
        self._show_pid_editor(int(selection[0], 16))

    def _show_pid_editor(self, pid: int, is_new: bool = False) -> None:
        key = f"{pid:02X}"
        old_entry = self.simulator.config["pids"].get(key, {})
        built_in = PID_DEFINITIONS.get(pid)
        metadata = PID_CATALOG.get(pid, old_entry)
        if not is_new and built_in is None and not old_entry:
            return
        if is_new:
            name = metadata.get("name", "")
            unit = metadata.get("unit", "")
            algorithm = (
                "sweep"
                if pid in NUMERIC_PID_LIMITS or pid in MULTI_VALUE_PID_LIMITS
                else "raw"
            )
        elif built_in:
            name, unit, _kind = built_in
            algorithm = old_entry.get(
                "algorithm", "sweep" if pid in DEFAULT_SWEEPS else "standard"
            )
        else:
            name, unit = old_entry["name"], old_entry["unit"]
            algorithm = old_entry.get("algorithm", "raw")

        window = tk.Toplevel(self.root)
        window.title("Add Mode 01 PID" if is_new else f"Edit PID 0x{pid:02X}")
        window.transient(self.root)
        fields = ttk.Frame(window, padding=10)
        fields.pack(fill="both", expand=True)
        pid_var = tk.StringVar(value=f"{pid:02X}")
        pid_selection_var = tk.StringVar()
        name_var = tk.StringVar(value=name)
        unit_var = tk.StringVar(value=unit)
        description_var = tk.StringVar(value=metadata.get("description", ""))
        measurement_var = tk.StringVar(value=metadata.get("measurement", ""))
        if "minimum" in metadata and "maximum" in metadata:
            range_text = (
                f"{metadata['minimum']:g} to {metadata['maximum']:g} "
                f"{metadata.get('unit', '')}".strip()
            )
        else:
            range_text = metadata.get("range_description", "Not specified")
        range_var = tk.StringVar(value=range_text)
        algorithm_var = tk.StringVar(value=algorithm)
        supported_var = tk.BooleanVar(
            value=old_entry.get(
                "supported", metadata.get("default_supported", True)
            )
        )
        raw_var = tk.StringVar(
            value=old_entry.get(
                "raw_bytes", metadata.get("default_raw_bytes", "00")
            )
        )
        def default_sweep_for(selected_pid, selected_metadata):
            if selected_pid in MULTI_VALUE_PID_LIMITS:
                default = {"interval_s": 1}
                for field, (minimum, maximum) in MULTI_VALUE_PID_LIMITS[
                    selected_pid
                ].items():
                    default.update(
                        {
                            f"{field}_minimum": minimum,
                            f"{field}_maximum": maximum,
                            f"{field}_step": (maximum - minimum) / 10,
                        }
                    )
                return default
            return DEFAULT_SWEEPS.get(
                selected_pid,
                {
                    "minimum": selected_metadata.get(
                        "minimum", NUMERIC_PID_LIMITS.get(selected_pid, (0, 100))[0]
                    ),
                    "maximum": selected_metadata.get(
                        "maximum", NUMERIC_PID_LIMITS.get(selected_pid, (0, 100))[1]
                    ),
                    "step": 1,
                    "interval_s": 1,
                },
            )

        default_sweep = default_sweep_for(pid, metadata)
        sweep = old_entry.get("sweep", default_sweep)
        sweep_vars = {
            field: tk.StringVar(value=str(sweep.get(field, default)))
            for field, default in {
                "minimum": 0,
                "maximum": 100,
                "step": 1,
                "interval_s": 1,
                "lambda_minimum": 0,
                "lambda_maximum": 2,
                "lambda_step": 0.2,
                "voltage_minimum": 0,
                "voltage_maximum": 8,
                "voltage_step": 0.8,
            }.items()
        }

        row = 0
        ttk.Label(fields, text="PID (hex)").grid(row=row, column=0, sticky="w", pady=3)
        if is_new:
            used = {
                int(existing_key, 16)
                for existing_key in self.simulator.config["pids"]
            }
            pid_options = {
                f"{available_pid:02X} - "
                f"{PID_CATALOG.get(available_pid, {}).get('name', 'Custom / vendor PID')}":
                available_pid
                for available_pid in range(1, 0x100)
                if available_pid not in used
                and available_pid not in PID_BITMAP_PAGES
            }
            pid_selector = ttk.Combobox(
                fields,
                textvariable=pid_selection_var,
                values=tuple(pid_options),
                state="readonly",
                width=42,
            )
            pid_selector.grid(row=row, column=1, sticky="ew", pady=3)

            def update_pid_defaults(_event=None) -> None:
                selected_pid = pid_options.get(pid_selection_var.get())
                if selected_pid is None:
                    return
                pid_var.set(f"{selected_pid:02X}")
                selected_metadata = PID_CATALOG.get(selected_pid, {})
                name_var.set(selected_metadata.get("name", ""))
                unit_var.set(selected_metadata.get("unit", ""))
                description_var.set(selected_metadata.get("description", ""))
                measurement_var.set(selected_metadata.get("measurement", ""))
                supported_var.set(
                    selected_metadata.get("default_supported", True)
                )
                can_sweep = (
                    selected_pid in NUMERIC_PID_LIMITS
                    or selected_pid in MULTI_VALUE_PID_LIMITS
                )
                selected_algorithm = "sweep" if can_sweep else "raw"
                algorithm_var.set(selected_algorithm)
                algorithm_combo.configure(
                    values=(
                        ("sweep", "random", "raw")
                        if can_sweep
                        else ("raw",)
                    )
                )
                selected_sweep = default_sweep_for(
                    selected_pid, selected_metadata
                )
                for field, variable in sweep_vars.items():
                    if field in selected_sweep:
                        variable.set(str(selected_sweep[field]))
                refresh_sweep_fields()
                if "minimum" in selected_metadata and "maximum" in selected_metadata:
                    range_var.set(
                        f"{selected_metadata['minimum']:g} to "
                        f"{selected_metadata['maximum']:g} "
                        f"{selected_metadata.get('unit', '')}".strip()
                    )
                else:
                    range_var.set(
                        selected_metadata.get("range_description", "Not specified")
                    )
                raw_var.set(selected_metadata.get("default_raw_bytes", "00"))

            pid_selector.bind("<<ComboboxSelected>>", update_pid_defaults)
            selected_option = next(
                label
                for label, available_pid in pid_options.items()
                if available_pid == pid
            )
            pid_selection_var.set(selected_option)
        else:
            pid_entry = ttk.Entry(fields, textvariable=pid_var, width=14)
            pid_entry.grid(row=row, column=1, sticky="ew", pady=3)
            pid_entry.configure(state="readonly")
        row += 1
        ttk.Label(fields, text="Service").grid(
            row=row, column=0, sticky="w", pady=3
        )
        ttk.Label(fields, text="01 - Current data").grid(
            row=row, column=1, sticky="w", pady=3
        )
        row += 1
        for label, variable in (
            ("Function", description_var),
            ("Measurement", measurement_var),
            ("Valid range", range_var),
        ):
            ttk.Label(fields, text=label).grid(
                row=row, column=0, sticky="w", pady=3
            )
            ttk.Label(
                fields, textvariable=variable, wraplength=420, justify="left"
            ).grid(row=row, column=1, sticky="w", pady=3)
            row += 1
        for label, variable in (("Name", name_var), ("Unit", unit_var)):
            ttk.Label(fields, text=label).grid(row=row, column=0, sticky="w", pady=3)
            entry = ttk.Entry(fields, textvariable=variable, width=34)
            entry.grid(row=row, column=1, sticky="ew", pady=3)
            if built_in:
                entry.configure(state="readonly")
            row += 1
        ttk.Checkbutton(fields, text="Supported", variable=supported_var).grid(
            row=row, column=1, sticky="w", pady=3
        )
        row += 1

        if pid in NUMERIC_PID_LIMITS or pid in MULTI_VALUE_PID_LIMITS:
            algorithms = ("sweep", "random", "raw")
        elif built_in:
            algorithms = ("standard", "raw")
        else:
            algorithms = ("raw",)
        ttk.Label(fields, text="Algorithm").grid(row=row, column=0, sticky="w", pady=3)
        algorithm_combo = ttk.Combobox(
            fields,
            textvariable=algorithm_var,
            values=algorithms,
            state="readonly",
            width=14,
        )
        algorithm_combo.grid(row=row, column=1, sticky="w", pady=3)
        row += 1
        raw_label = "Raw bytes (hex)"
        if is_new:
            raw_label += " - catalog default prefilled"
        ttk.Label(fields, text=raw_label).grid(
            row=row, column=0, sticky="w", pady=3
        )
        raw_entry = ttk.Entry(fields, textvariable=raw_var, width=34)
        raw_entry.grid(row=row, column=1, sticky="ew", pady=3)
        row += 1

        sweep_fields_frame = ttk.Frame(fields)
        sweep_fields_frame.grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(2, 0)
        )
        row += 1

        def refresh_sweep_fields() -> None:
            for child in sweep_fields_frame.winfo_children():
                child.destroy()
            selected_pid = int(pid_var.get(), 16)
            algorithm = algorithm_var.get()
            if selected_pid in MULTI_VALUE_PID_LIMITS:
                available_fields = [
                    ("lambda_minimum", "Equivalence ratio minimum"),
                    ("lambda_maximum", "Equivalence ratio maximum"),
                    ("voltage_minimum", "Voltage minimum (V)"),
                    ("voltage_maximum", "Voltage maximum (V)"),
                ]
                if algorithm == "sweep":
                    available_fields.extend(
                        (
                            ("lambda_step", "Equivalence ratio step"),
                            ("voltage_step", "Voltage step (V)"),
                            ("interval_s", "Update interval (s)"),
                        )
                    )
            elif algorithm in ("sweep", "random"):
                available_fields = [
                    ("minimum", "Minimum"),
                    ("maximum", "Maximum"),
                ]
                if algorithm == "sweep":
                    available_fields.extend(
                        (("step", "Step"), ("interval_s", "Update interval (s)"))
                    )
            else:
                available_fields = []
            for field_row, (field, label) in enumerate(available_fields):
                ttk.Label(sweep_fields_frame, text=label).grid(
                    row=field_row, column=0, sticky="w", pady=3
                )
                ttk.Entry(
                    sweep_fields_frame, textvariable=sweep_vars[field], width=14
                ).grid(row=field_row, column=1, sticky="w", pady=3)

        def update_algorithm_fields(*_args) -> None:
            raw_state = "normal" if algorithm_var.get() == "raw" else "disabled"
            raw_entry.configure(state=raw_state)
            refresh_sweep_fields()

        algorithm_var.trace_add("write", update_algorithm_fields)
        update_algorithm_fields()
        buttons = ttk.Frame(fields)
        buttons.grid(row=row, column=0, columnspan=2, sticky="e", pady=(8, 0))

        def save() -> None:
            try:
                pid_value = int(pid_var.get().strip().removeprefix("0x").removeprefix("0X"), 16)
            except ValueError:
                messagebox.showerror("Invalid PID", "PID must be hexadecimal.", parent=window)
                return
            if not 1 <= pid_value <= 0xFF:
                messagebox.showerror(
                    "Invalid PID", "PID must be between 01 and FF.", parent=window
                )
                return
            new_key = f"{pid_value:02X}"
            if pid_value == 0 or pid_value in PID_BITMAP_PAGES:
                messagebox.showerror(
                    "Invalid PID", "PID 00 and bitmap page PIDs are reserved.", parent=window
                )
                return
            if is_new and new_key in self.simulator.config["pids"]:
                messagebox.showerror("PID already exists", f"PID 0x{new_key} is already configured.", parent=window)
                return
            if not name_var.get().strip():
                messagebox.showerror("Invalid PID", "Enter a PID name.", parent=window)
                return

            config = copy.deepcopy(self.simulator.config)
            if built_in:
                entry = config["pids"][key]
            else:
                entry = copy.deepcopy(old_entry)
                entry.update(
                    {
                        "name": name_var.get().strip(),
                        "unit": unit_var.get().strip(),
                        "supported": supported_var.get(),
                    }
                )
                config["pids"][new_key] = entry
                catalog_entry = PID_CATALOG.get(pid_value)
                if catalog_entry:
                    entry.update(
                        {
                            "service": "01",
                            "description": catalog_entry["description"],
                            "measurement": catalog_entry["measurement"],
                            "default_raw_bytes": catalog_entry["default_raw_bytes"],
                        }
                    )
                    for field in ("minimum", "maximum", "range_description"):
                        entry.pop(field, None)
                    for field in ("minimum", "maximum", "range_description"):
                        if field in catalog_entry:
                            entry[field] = catalog_entry[field]
            entry["supported"] = supported_var.get()
            entry["algorithm"] = algorithm_var.get()
            if algorithm_var.get() == "raw":
                entry["raw_bytes"] = raw_var.get()
            else:
                entry.pop("raw_bytes", None)
            if algorithm_var.get() in ("sweep", "random"):
                try:
                    if pid_value in MULTI_VALUE_PID_LIMITS:
                        fields_to_save = (
                            (
                                "lambda_minimum",
                                "lambda_maximum",
                                "lambda_step",
                                "voltage_minimum",
                                "voltage_maximum",
                                "voltage_step",
                                "interval_s",
                            )
                            if algorithm_var.get() == "sweep"
                            else (
                                "lambda_minimum",
                                "lambda_maximum",
                                "voltage_minimum",
                                "voltage_maximum",
                            )
                        )
                    else:
                        fields_to_save = (
                            ("minimum", "maximum", "step", "interval_s")
                            if algorithm_var.get() == "sweep"
                            else ("minimum", "maximum")
                        )
                    entry["sweep"] = {
                        field: float(sweep_vars[field].get())
                        for field in fields_to_save
                    }
                except ValueError:
                    messagebox.showerror(
                        "Invalid value range",
                        "PID value boundaries and sweep timing must be numeric.",
                        parent=window,
                    )
                    return
            if self._commit_simulator_config(config, "Invalid PID configuration"):
                window.destroy()

        ttk.Button(buttons, text="Cancel", command=window.destroy).pack(
            side="right", padx=(4, 0)
        )
        ttk.Button(buttons, text="Save", command=save).pack(side="right")
        window.columnconfigure(0, weight=1)
        fields.columnconfigure(1, weight=1)
        window.wait_visibility()
        window.grab_set()

    def _remove_selected_pid(self) -> None:
        selection = self.simulator_table.selection()
        if not selection:
            messagebox.showinfo("Select a PID", "Select a custom PID to remove.")
            return
        key = selection[0]
        if int(key, 16) in PID_DEFINITIONS:
            messagebox.showinfo(
                "Built-in PID", "Built-in PIDs cannot be removed; toggle Supported instead."
            )
            return
        config = copy.deepcopy(self.simulator.config)
        del config["pids"][key]
        self._commit_simulator_config(config, "Could not remove PID")

    def _toggle_dtc_active(self, event=None) -> None:
        if event and hasattr(event, "x") and self.dtc_table.identify_column(event.x) != "#3":
            code = self.dtc_table.identify_row(event.y)
            if code:
                self._show_dtc_editor(code)
            return
        code = self.dtc_table.identify_row(event.y) if event and hasattr(event, "y") else ""
        if not code and self.dtc_table.selection():
            code = self.dtc_table.selection()[0]
        config = copy.deepcopy(self.simulator.config)
        dtc = next((item for item in config["dtcs"] if item["code"] == code), None)
        if dtc is None:
            return
        dtc["active"] = not dtc["active"]
        self._commit_simulator_config(config, "Could not save DTC")

    def _add_dtc(self) -> None:
        self._show_dtc_editor()

    def _edit_selected_dtc(self) -> None:
        selection = self.dtc_table.selection()
        if not selection:
            messagebox.showinfo("Select a DTC", "Select a DTC row to edit.")
            return
        self._show_dtc_editor(selection[0])

    def _show_dtc_editor(self, code: Optional[str] = None) -> None:
        dtc = next(
            (item for item in self.simulator.config["dtcs"] if item["code"] == code),
            {},
        )
        window = tk.Toplevel(self.root)
        window.title("Add DTC" if code is None else f"Edit {code}")
        window.transient(self.root)
        fields = ttk.Frame(window, padding=10)
        fields.pack(fill="both", expand=True)
        code_var = tk.StringVar(value=dtc.get("code", "P0000"))
        description_var = tk.StringVar(value=dtc.get("description", ""))
        active_var = tk.BooleanVar(value=dtc.get("active", False))
        ttk.Label(fields, text="DTC code").grid(row=0, column=0, sticky="w", pady=3)
        code_entry = ttk.Entry(fields, textvariable=code_var, width=18)
        code_entry.grid(row=0, column=1, sticky="ew", pady=3)
        if code:
            code_entry.configure(state="readonly")
        ttk.Label(fields, text="Description").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Entry(fields, textvariable=description_var, width=42).grid(
            row=1, column=1, sticky="ew", pady=3
        )
        ttk.Checkbutton(fields, text="Active", variable=active_var).grid(
            row=2, column=1, sticky="w", pady=3
        )
        buttons = ttk.Frame(fields)
        buttons.grid(row=3, column=0, columnspan=2, sticky="e", pady=(8, 0))

        def save() -> None:
            config = copy.deepcopy(self.simulator.config)
            new_code = code_var.get().strip().upper()
            updated = {
                "code": new_code,
                "description": description_var.get().strip(),
                "active": active_var.get(),
            }
            if code:
                dtc_index = next(
                    index for index, item in enumerate(config["dtcs"]) if item["code"] == code
                )
                config["dtcs"][dtc_index] = updated
            else:
                config["dtcs"].append(updated)
            if self._commit_simulator_config(config, "Invalid DTC configuration"):
                window.destroy()

        ttk.Button(buttons, text="Cancel", command=window.destroy).pack(
            side="right", padx=(4, 0)
        )
        ttk.Button(buttons, text="Save", command=save).pack(side="right")
        fields.columnconfigure(1, weight=1)
        window.wait_visibility()
        window.grab_set()

    def _remove_selected_dtc(self) -> None:
        selection = self.dtc_table.selection()
        if not selection:
            messagebox.showinfo("Select a DTC", "Select a DTC row to remove.")
            return
        config = copy.deepcopy(self.simulator.config)
        config["dtcs"] = [
            dtc for dtc in config["dtcs"] if dtc["code"] != selection[0]
        ]
        self._commit_simulator_config(config, "Could not remove DTC")

    def _simulator_config_from_fields(self) -> dict:
        config = copy.deepcopy(self.simulator.config)
        request_ids_text = self.simulator_request_ids_var.get()
        try:
            request_ids = [
                f"0x{int(item.strip().removeprefix('0x').removeprefix('0X'), 16):03X}"
                for item in request_ids_text.split(",")
                if item.strip()
            ]
            response_id = int(
                self.simulator_response_id_var.get().strip().removeprefix("0x").removeprefix("0X"),
                16,
            )
        except ValueError as exc:
            raise SimulatorConfigError("CAN IDs must be hexadecimal.") from exc
        config["request_ids"] = request_ids
        config["response_id"] = f"0x{response_id:03X}"
        config["mil_on"] = self.simulator_mil_var.get()
        config["multi_pid_response_active"] = self.simulator_multi_pid_var.get()
        return config

    def _save_simulator_config(self) -> None:
        try:
            config = self._simulator_config_from_fields()
            self.simulator.config = save_config(self.simulator_config_path, config)
        except SimulatorConfigError as exc:
            self.simulator_mil_var.set(self.simulator.config["mil_on"])
            self.simulator_multi_pid_var.set(
                self.simulator.config["multi_pid_response_active"]
            )
            messagebox.showerror("Invalid ECU config", str(exc))
            return
        self._refresh_simulator_fields()
        self._refresh_simulator_table()
        self._refresh_dtc_table()
        self.simulator_status_var.set(f"Saved configuration to {self.simulator_config_path}")

    def _choose_simulator_config(self) -> None:
        path = filedialog.askopenfilename(
            title="Load ECU simulator configuration",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
        )
        if not path:
            return
        try:
            config = load_config(Path(path))
        except SimulatorConfigError as exc:
            messagebox.showerror("Could not load ECU config", str(exc))
            return
        self.simulator_config_path = Path(path)
        self.simulator.update_config(config, reset_clock=True)
        self.simulator_config_path_var.set(str(self.simulator_config_path))
        self._refresh_simulator_fields()
        self._refresh_simulator_table()
        self._refresh_dtc_table()
        self.simulator_status_var.set(f"Loaded configuration from {path}")

    def _open_simulator_config(self) -> None:
        try:
            if sys.platform == "win32":
                os.startfile(str(self.simulator_config_path))
            elif sys.platform == "darwin":
                subprocess.Popen(["open", str(self.simulator_config_path)])
            else:
                subprocess.Popen(["xdg-open", str(self.simulator_config_path)])
        except OSError as exc:
            messagebox.showerror("Could not open ECU config", str(exc))

    @property
    def simulator_can_run(self) -> bool:
        return self.can_open and self.mode_var.get() == "Normal" and self.connected

    def _set_simulator_availability(self) -> None:
        if not self.simulator_can_run and self.simulator_running:
            self._stop_simulator()
        self.simulator_start_button.configure(
            state="normal" if self.simulator_can_run else "disabled"
        )

    def _toggle_simulator(self) -> None:
        if self.simulator_running:
            self._stop_simulator()
            return
        if not self.simulator_can_run:
            self.simulator_status_var.set(
                "Open CAN in Normal mode before starting the simulator."
            )
            return
        try:
            self.simulator.config = save_config(
                self.simulator_config_path,
                self._simulator_config_from_fields(),
            )
        except SimulatorConfigError as exc:
            messagebox.showerror("Invalid ECU config", str(exc))
            return
        self.simulator.start_session()
        self.simulator_running = True
        self.simulator_start_button.configure(text="Stop simulator")
        self.simulator_status_var.set(
            f"Running; request IDs "
            f"{', '.join(f'0x{value:03X}' for value in self.simulator.request_ids)}, "
            f"response ID 0x{self.simulator.response_id:03X}."
        )

    def _stop_simulator(self) -> None:
        self.simulator_running = False
        self._clear_simulator_isotp()
        if hasattr(self, "simulator_start_button"):
            self.simulator_start_button.configure(text="Start simulator")
        if hasattr(self, "simulator_status_var"):
            self.simulator_status_var.set("Stopped.")

    def refresh_ports(self, automatic: bool = False) -> set:
        port_infos = list(serial.tools.list_ports.comports())
        ports = {port.device for port in port_infos}
        labels = {
            port.device: (
                f"{port.device} - {port.description}"
                if port.description and port.description != port.device
                else port.device
            )
            for port in port_infos
        }
        previous = {
            value.split(" - ", 1)[0] for value in self.port_combo["values"]
        }
        self.port_combo["values"] = [labels[port.device] for port in port_infos]
        if self.preferred_device in ports:
            self.port_var.set(labels[self.preferred_device])
        added = [port.device for port in port_infos if port.device not in previous]
        bluetooth_added = [
            port for port in added if "bluetooth" in labels[port].casefold()
        ]
        if automatic and len(bluetooth_added) == 1:
            selected = bluetooth_added[0]
            self.port_var.set(labels[selected])
            self.status_var.set(f"Bluetooth serial port {selected} is ready.")
        elif automatic and len(added) == 1:
            selected = added[0]
            self.port_var.set(labels[selected])
            self.status_var.set(f"New serial port {selected} is ready.")
        elif automatic and len(added) > 1:
            self.status_var.set(
                "Multiple new serial ports found; select the Bluetooth serial port from the list."
            )
        elif self.port_var.get().split(" - ", 1)[0] not in ports:
            self.port_var.set(labels[port_infos[0].device] if port_infos else "")
        return set(ports)

    def scan_bluetooth(self) -> None:
        self.scan_bluetooth_button.configure(state="disabled")
        self.status_var.set("Scanning for Bluetooth devices...")

        def scan() -> None:
            try:
                devices = discover_devices()
                self.bluetooth_events.put({"type": "devices", "devices": devices})
            except BluetoothUnavailableError as exc:
                self.bluetooth_events.put({"type": "error", "message": str(exc)})
            except (OSError, AttributeError, ctypes.ArgumentError) as exc:
                self.bluetooth_events.put({"type": "error", "message": str(exc)})

        threading.Thread(target=scan, daemon=True).start()

    def connect_bluetooth(self) -> None:
        if sys.platform != "win32":
            messagebox.showinfo(
                "Bluetooth setup",
                "Pair and configure the Bluetooth SPP serial device using your "
                "system Bluetooth tools, then select its serial port.",
            )
            return
        device = self.bluetooth_devices.get(self.bluetooth_var.get())
        if device is None:
            messagebox.showinfo("Select a device", "Scan for and select a Bluetooth device first.")
            return
        try:
            os.startfile("ms-settings:bluetooth")
        except OSError as exc:
            messagebox.showerror("Could not open Bluetooth settings", str(exc))
            return

        baseline = self.refresh_ports()
        bluetooth_ports = [
            label
            for label in self.port_combo["values"]
            if "bluetooth" in label.casefold()
        ]
        if len(bluetooth_ports) == 1:
            self.port_var.set(bluetooth_ports[0])
            self.status_var.set(
                f"Bluetooth serial port {bluetooth_ports[0].split(' - ', 1)[0]} is available."
            )
            return
        self.status_var.set(
            f"Use Windows Bluetooth settings to pair/connect {device.name or device.address}; "
            "checking for its serial port..."
        )
        self.root.after(2000, lambda: self._watch_bluetooth_port(baseline, 59))

    def _watch_bluetooth_port(self, baseline, remaining: int) -> None:
        if self._closing:
            return
        current = self.refresh_ports(automatic=True)
        if current - baseline or remaining <= 0:
            if not current - baseline:
                self.status_var.set(
                    "No new serial port detected yet. Pair/connect in Windows, then use Refresh."
                )
            return
        self.root.after(
            2000, lambda: self._watch_bluetooth_port(baseline, remaining - 1)
        )

    def connect(self) -> None:
        port = self.port_var.get().split(" - ", 1)[0].strip()
        if not port:
            messagebox.showerror("No serial port", "Select a serial port first.")
            return
        try:
            baud_rate = int(self.baud_var.get())
            if baud_rate <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid baud rate", "Enter a positive integer baud rate.")
            return

        self.status_var.set(f"Connecting to {port}...")
        self._pending_connection_preferences = (port, str(baud_rate))
        self.last_serial_error = None
        self.connect_button.configure(state="disabled")
        self.worker = SerialWorker(
            port, baud_rate, self.events, debug=self.debug_enabled_var.get()
        )
        self.worker.start()

    def _toggle_debug(self) -> None:
        enabled = self.debug_enabled_var.get()
        if self.worker is not None:
            self.worker.set_debug(enabled)
        self.status_var.set(
            "Serial debug logging enabled." if enabled else "Serial debug logging disabled."
        )
        if enabled:
            self.show_debug_log()

    def show_debug_log(self) -> None:
        if self.debug_window is not None and self.debug_window.winfo_exists():
            self.debug_window.deiconify()
            self.debug_window.lift()
            return

        window = tk.Toplevel(self.root)
        window.title("SLCAN serial debug log")
        window.geometry("850x360")
        self.debug_window = window
        toolbar = ttk.Frame(window)
        toolbar.pack(fill="x", padx=6, pady=4)
        ttk.Button(toolbar, text="Save log", command=self.save_debug_log).pack(
            side="right", padx=(4, 0)
        )
        ttk.Button(toolbar, text="Clear", command=self._clear_debug_log).pack(
            side="right"
        )
        log_frame = ttk.Frame(window)
        log_frame.pack(fill="both", expand=True, padx=6, pady=(0, 6))
        text = tk.Text(log_frame, wrap="none", state="disabled", height=16)
        y_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=text.yview)
        x_scroll = ttk.Scrollbar(log_frame, orient="horizontal", command=text.xview)
        text.configure(yscrollcommand=y_scroll.set, xscrollcommand=x_scroll.set)
        text.grid(row=0, column=0, sticky="nsew")
        y_scroll.grid(row=0, column=1, sticky="ns")
        x_scroll.grid(row=1, column=0, sticky="ew")
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        self.debug_text = text
        window.protocol("WM_DELETE_WINDOW", window.withdraw)
        for line in self.debug_messages:
            self._append_debug_line(line)

    def _append_debug_line(self, line: str) -> None:
        if self.debug_text is None or not self.debug_text.winfo_exists():
            return
        self.debug_text.configure(state="normal")
        self.debug_text.insert("end", line + "\n")
        self.debug_text.see("end")
        self.debug_text.configure(state="disabled")

    def _clear_debug_log(self) -> None:
        self.debug_messages.clear()
        if self.debug_text is not None and self.debug_text.winfo_exists():
            self.debug_text.configure(state="normal")
            self.debug_text.delete("1.0", "end")
            self.debug_text.configure(state="disabled")

    def save_debug_log(self) -> None:
        if not self.debug_messages:
            messagebox.showinfo("No debug log", "There are no debug messages to save.")
            return
        path = filedialog.asksaveasfilename(
            title="Save serial debug log",
            defaultextension=".txt",
            filetypes=(("Text files", "*.txt"), ("All files", "*.*")),
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8", newline="\n") as output:
                output.write("\n".join(self.debug_messages))
                output.write("\n")
        except OSError as exc:
            messagebox.showerror("Could not save debug log", str(exc))

    def disconnect(self) -> None:
        if self.worker is None:
            return
        self._stop_dbc_transmission(silent=True)
        self.disconnect_button.configure(state="disabled")
        self.open_button.configure(state="disabled")
        self.close_can_button.configure(state="disabled")
        self.send_button.configure(state="disabled")
        self._update_dbc_availability()
        self.status_var.set("Disconnecting...")
        self._send_command("C", "disconnect_close")

    def open_can(self) -> None:
        if not self.connected or self.can_open:
            return
        code = self.bitrate_var.get().split(" - ", 1)[0]
        if code not in BITRATES:
            messagebox.showerror("Invalid bitrate", "Select a supported CAN bitrate.")
            return
        self.open_button.configure(state="disabled")
        self.status_var.set(f"Selecting {BITRATES[code]} bit/s...")
        self._send_command(code, "set_bitrate")

    def close_can(self) -> None:
        if self.can_open:
            self._stop_dbc_transmission(silent=True)
            self.close_can_button.configure(state="disabled")
            self._send_command("C", "close_can")

    def send_frame(self) -> None:
        try:
            frame_type = self.frame_type_var.get()
            extended = frame_type.startswith("Extended")
            remote = frame_type.endswith("remote")
            identifier_text = self.frame_id_var.get().strip()
            if not identifier_text:
                raise ValueError("Enter a hexadecimal CAN identifier.")
            arbitration_id = int(identifier_text, 16)
            dlc = int(self.dlc_var.get())
            data_text = self.data_var.get().strip()
            if remote:
                data = b""
            else:
                compact = data_text.replace(" ", "").replace(",", "")
                if len(compact) % 2:
                    raise ValueError("Data must contain pairs of hexadecimal digits.")
                data = bytes.fromhex(compact)
            command = encode_frame(
                CanFrame(arbitration_id, extended, remote, dlc, data)
            )
        except (ValueError, OverflowError) as exc:
            messagebox.showerror("Invalid CAN frame", str(exc))
            return
        self._send_command(command, "transmit")

    def _send_command(self, command: str, tag: str) -> None:
        if self.worker is not None and self.connected:
            self.worker.send(command, tag)
            self.status_var.set(f"Sent {command}; waiting for device response...")

    def _poll_events(self) -> None:
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            self._handle_event(event)
        while True:
            try:
                event = self.bluetooth_events.get_nowait()
            except queue.Empty:
                break
            if event["type"] == "devices":
                self.bluetooth_devices = {
                    device.label: device for device in event["devices"]
                }
                labels = list(self.bluetooth_devices)
                self.bluetooth_combo["values"] = labels
                self.bluetooth_var.set(labels[0] if labels else "")
                self.scan_bluetooth_button.configure(state="normal")
                self.status_var.set(
                    f"Found {len(labels)} Bluetooth device(s)."
                    if labels
                    else "No Bluetooth devices found. Check that the adapter is on and the device is discoverable."
                )
            elif event["type"] == "error":
                self.scan_bluetooth_button.configure(state="normal")
                self.status_var.set(f"Bluetooth scan failed: {event['message']}")
                messagebox.showerror("Bluetooth scan failed", event["message"])
        self._update_bus_load()
        if not self._closing:
            self.root.after(50, self._poll_events)

    def _start_bus_load_measurement(self) -> None:
        code = self.bitrate_var.get().split(" - ", 1)[0]
        self._bus_load_bitrate = BITRATES.get(code)
        self._bus_load_samples.clear()
        self._update_bus_load()

    def _stop_bus_load_measurement(self) -> None:
        self._bus_load_bitrate = None
        self._bus_load_samples.clear()
        self._update_bus_load()

    def _update_bus_load(self) -> None:
        if self._bus_load_bitrate is None:
            message = "CAN closed. Estimate includes observed RX and TX frames only."
        else:
            now = time.monotonic()
            cutoff = now - 10.0
            while self._bus_load_samples and self._bus_load_samples[0][0] < cutoff:
                self._bus_load_samples.popleft()
            total_bits = sum(bits for _timestamp, bits in self._bus_load_samples)
            utilization = total_bits / (self._bus_load_bitrate * 10.0) * 100
            message = (
                f"{utilization:.2f}% over the last 10 s "
                f"({total_bits} estimated bits at "
                f"{self._bus_load_bitrate // 1000} kbit/s). "
                "Observed RX/TX only; actual load may be higher."
            )
        if self.bus_load_var.get() != message:
            self.bus_load_var.set(message)

    def _handle_event(self, event: Dict[str, Any]) -> None:
        event_type = event["type"]
        if event_type == "connected":
            self.connected = True
            self.connect_button.configure(state="disabled")
            self.disconnect_button.configure(state="normal")
            self.open_button.configure(state="normal")
            for button in self._command_buttons:
                button.configure(state="normal")
            self.status_var.set("Serial connected; CAN is closed.")
            self._save_connection_preferences_after_connect()
            self._update_dbc_availability()
        elif event_type == "disconnected":
            self._stop_dbc_transmission(silent=True)
            self.connected = False
            self._pending_connection_preferences = None
            self.can_open = False
            self._stop_bus_load_measurement()
            self._stop_simulator()
            self._set_simulator_availability()
            self.worker = None
            self.connect_button.configure(state="normal")
            self.disconnect_button.configure(state="disabled")
            self.open_button.configure(state="disabled")
            self.close_can_button.configure(state="disabled")
            self.send_button.configure(state="disabled")
            for button in self._command_buttons:
                button.configure(state="disabled")
            if self.last_serial_error:
                self.status_var.set(f"Disconnected after serial error: {self.last_serial_error}")
            else:
                self.status_var.set("Disconnected")
            if self._closing:
                self.root.destroy()
        elif event_type == "frame":
            self._add_frame(event["line"], "RX")
            self._handle_simulator_request(event["line"])
        elif event_type == "debug":
            self.debug_messages.append(event["message"])
            if len(self.debug_messages) > 3000:
                del self.debug_messages[: len(self.debug_messages) - 3000]
            self._append_debug_line(event["message"])
        elif event_type == "text":
            self.status_var.set(f"Device: {event['line']}")
        elif event_type == "error":
            self.last_serial_error = event["message"]
            self.status_var.set(f"Serial error: {event['message']} (see debug log)")
            if self.worker is None or not self.connected:
                messagebox.showerror("Serial error", event["message"])
        elif event_type == "command":
            self._command_completed(event)

    def _handle_simulator_request(self, line: str) -> None:
        if not self.simulator_running or not self.simulator_can_run:
            return
        try:
            request = parse_frame(line)
        except ValueError as exc:
            self.simulator_status_var.set(f"Could not parse OBD request: {exc}")
            return

        sender = self._simulator_isotp_sender
        if sender is not None:
            if sender.accept_flow_control(request):
                self._cancel_simulator_flow_control_timeout()
                if sender.error:
                    self._clear_simulator_isotp()
                    self.simulator_status_var.set(
                        f"ISO-TP response stopped: {sender.error}"
                    )
                else:
                    self._continue_simulator_isotp()
                return
            if not sender.complete or self._simulator_response_command_pending:
                return
            self._clear_simulator_isotp()

        response_data = self.simulator.handle_request_payload(request)
        if response_data is None:
            return
        self.simulator_response_count += 1
        if self.worker is None:
            self._stop_simulator()
            return

        flow_control_id = self._simulator_flow_control_id(request.arbitration_id)
        sender = IsoTpSender(
            self.simulator.response_id, flow_control_id, response_data
        )
        response_frame = sender.initial_frame()
        if not sender.complete:
            self._simulator_isotp_sender = sender
        self._send_simulator_response(response_frame)

        service = response_data[0]
        if service == 0x43:
            message = "OBD Mode 03 DTC request received; replying..."
        elif service == 0x44:
            message = "OBD Mode 04 clear-DTC request received; replying..."
        elif service == 0x41 and (request.data[0] & 0x0F) > 2:
            pid_count = (request.data[0] & 0x0F) - 1
            requested = ", ".join(
                f"0x{pid:02X}" for pid in request.data[2 : 2 + pid_count]
            )
            message = f"OBD multi-PID request ({requested}) received; replying..."
        elif service == 0x41 and len(response_data) > 1:
            message = (
                f"OBD Mode 01 PID 0x{response_data[1]:02X} request received; replying..."
            )
        else:
            message = f"OBD response 0x{service:02X} sent."
        self.simulator_status_var.set(message)

    def _simulator_flow_control_id(self, request_id: int) -> int:
        if request_id != 0x7DF:
            return request_id
        expected_id = self.simulator.response_id - 8
        physical_ids = [
            can_id for can_id in self.simulator.request_ids if can_id != 0x7DF
        ]
        if expected_id in physical_ids:
            return expected_id
        return physical_ids[0] if physical_ids else request_id

    def _send_simulator_response(self, frame: CanFrame) -> None:
        if self.worker is None:
            self._stop_simulator()
            return
        if self._simulator_isotp_sender is not None:
            self._simulator_response_command_pending = True
        self.worker.send(encode_frame(frame), "ecu_response")

    def _continue_simulator_isotp(self) -> None:
        sender = self._simulator_isotp_sender
        if sender is None or self._simulator_response_command_pending:
            return
        if sender.error:
            self._clear_simulator_isotp()
            self.simulator_status_var.set(f"ISO-TP response stopped: {sender.error}")
        elif sender.complete:
            self._clear_simulator_isotp()
            self.simulator_status_var.set(
                f"Running; sent {self.simulator_response_count} OBD response(s)."
            )
        elif sender.waiting_for_flow_control:
            self._start_simulator_flow_control_timeout()
        elif self._simulator_segment_after is None:
            self._simulator_segment_after = self.root.after(
                sender.st_min_ms, self._send_next_simulator_segment
            )

    def _send_next_simulator_segment(self) -> None:
        self._simulator_segment_after = None
        sender = self._simulator_isotp_sender
        if sender is None or self._simulator_response_command_pending:
            return
        frame = sender.next_frame()
        if frame is None:
            self._continue_simulator_isotp()
            return
        self._send_simulator_response(frame)

    def _start_simulator_flow_control_timeout(self) -> None:
        self._cancel_simulator_flow_control_timeout()
        self._simulator_flow_control_after = self.root.after(
            1000, self._simulator_flow_control_timed_out
        )

    def _cancel_simulator_flow_control_timeout(self) -> None:
        if self._simulator_flow_control_after is not None:
            self.root.after_cancel(self._simulator_flow_control_after)
            self._simulator_flow_control_after = None

    def _simulator_flow_control_timed_out(self) -> None:
        self._simulator_flow_control_after = None
        sender = self._simulator_isotp_sender
        if sender is not None and sender.waiting_for_flow_control:
            self._clear_simulator_isotp()
            self.simulator_status_var.set(
                "ISO-TP response stopped: timed out waiting for tester flow control."
            )

    def _clear_simulator_isotp(self) -> None:
        self._cancel_simulator_flow_control_timeout()
        if self._simulator_segment_after is not None:
            self.root.after_cancel(self._simulator_segment_after)
            self._simulator_segment_after = None
        self._simulator_isotp_sender = None
        self._simulator_response_command_pending = False

    def _command_completed(self, event: Dict[str, Any]) -> None:
        tag = event["tag"]
        ok = event["ok"]
        response = event["response"]
        if not ok:
            if tag.startswith("dbc_tx:"):
                key = tag.removeprefix("dbc_tx:")
                self._dbc_pending.discard(key)
                self._stop_dbc_message(key)
                self.dbc_status_var.set(
                    f"DBC transmission failed for {self.dbc_messages[key].name}: {response}"
                )
                self._update_dbc_availability()
                messagebox.showerror("DBC transmission failed", response)
                return
            if tag == "disconnect_close":
                if self.worker is not None:
                    self.worker.disconnect()
                return
            self.status_var.set(f"Command failed: {response}")
            if tag == "ecu_response":
                self._clear_simulator_isotp()
                self.simulator_status_var.set(
                    f"OBD response failed: {response}"
                )
            if tag in ("set_bitrate", "open_can"):
                self.open_button.configure(state="normal")
            if tag == "close_can":
                self.close_can_button.configure(state="normal")
            return

        if tag == "set_bitrate":
            mode_command = "O" if self.mode_var.get() == "Normal" else "L"
            self.status_var.set(f"Opening CAN in {self.mode_var.get().lower()} mode...")
            self._send_command(mode_command, "open_can")
        elif tag == "open_can":
            self.can_open = True
            self._start_bus_load_measurement()
            self.open_button.configure(state="disabled")
            self.close_can_button.configure(state="normal")
            self.send_button.configure(
                state="normal" if self.mode_var.get() == "Normal" else "disabled"
            )
            self.status_var.set(f"CAN open ({self.mode_var.get().lower()} mode). {response}".strip())
            self._set_simulator_availability()
            self._update_dbc_availability()
        elif tag == "close_can":
            self.can_open = False
            self._stop_dbc_transmission(silent=True)
            self._stop_bus_load_measurement()
            self.open_button.configure(state="normal")
            self.close_can_button.configure(state="disabled")
            self.send_button.configure(state="disabled")
            self.status_var.set("CAN closed.")
            self._set_simulator_availability()
            self._update_dbc_availability()
        elif tag == "disconnect_close":
            self.can_open = False
            self._stop_dbc_transmission(silent=True)
            if self.worker is not None:
                self.worker.disconnect()
        elif tag == "transmit":
            self.status_var.set(f"Transmit accepted. {response}".strip())
            self._add_frame(event["command"], "TX")
        elif tag.startswith("dbc_tx:"):
            key = tag.removeprefix("dbc_tx:")
            self._dbc_pending.discard(key)
            self._add_frame(event["command"], "TX")
            if self.dbc_running:
                self._schedule_dbc_transmitter()
            self._update_dbc_availability()
            self._refresh_dbc_message(key)
        elif tag == "ecu_response":
            self._add_frame(event["command"], "TX")
            if self._simulator_isotp_sender is not None:
                self._simulator_response_command_pending = False
                self._continue_simulator_isotp()
            else:
                self.simulator_status_var.set(
                    f"Running; sent {self.simulator_response_count} OBD response(s)."
                )
        elif tag == "timestamp_on":
            self.status_var.set("Receive timestamps enabled.")
        elif tag == "timestamp_off":
            self.status_var.set("Receive timestamps disabled.")
        else:
            self.status_var.set(f"Response: {response or '(OK)'}")

    def _add_frame(self, line: str, direction: str) -> None:
        try:
            frame = parse_frame(line)
        except ValueError as exc:
            self.status_var.set(f"Invalid received frame: {exc}")
            return

        now = datetime.now().astimezone().isoformat(timespec="milliseconds")
        frame_format = (
            "Extended" if frame.extended else "Standard"
        ) + (" remote" if frame.remote else " data")
        data = " ".join(f"{byte:02X}" for byte in frame.data)
        timestamp = "" if frame.timestamp is None else f"{frame.timestamp:04X}"
        values = (
            now,
            direction,
            frame_format,
            f"{frame.arbitration_id:0{8 if frame.extended else 3}X}",
            frame.dlc,
            data,
            timestamp,
        )
        self.frame_table.insert("", "end", values=values)
        self.frame_table.yview_moveto(1)
        self._records.append(values)
        if self.can_open and self._bus_load_bitrate is not None:
            self._bus_load_samples.append(
                (time.monotonic(), estimate_can_frame_bits(frame))
            )
            self._update_bus_load()

    def export_csv(self) -> None:
        if not self._records:
            messagebox.showinfo("No frames", "There are no frames to export yet.")
            return
        path = filedialog.asksaveasfilename(
            title="Export CAN frames",
            defaultextension=".csv",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as output:
                writer = csv.writer(output)
                writer.writerow(
                    ("host_time", "direction", "format", "id", "dlc", "data", "device_timestamp")
                )
                writer.writerows(self._records)
        except OSError as exc:
            messagebox.showerror("Export failed", str(exc))
            return
        self.status_var.set(f"Exported {len(self._records)} frames to {path}")

    def export_asc(self) -> None:
        if not self._records:
            messagebox.showinfo("No frames", "There are no frames to export yet.")
            return
        path = filedialog.asksaveasfilename(
            title="Export CANoe ASC trace",
            defaultextension=".asc",
            filetypes=(("CANoe ASC files", "*.asc"), ("All files", "*.*")),
        )
        if not path:
            return
        try:
            trace = format_asc_trace(self._records)
            with open(path, "w", encoding="utf-8", newline="\n") as output:
                output.write(trace)
        except (OSError, ValueError) as exc:
            messagebox.showerror("ASC export failed", str(exc))
            return
        self.status_var.set(f"Exported {len(self._records)} frames to {path}")

    def _save_connection_preferences_after_connect(self) -> None:
        preferences = self._pending_connection_preferences
        self._pending_connection_preferences = None
        if preferences is None:
            return
        device, baudrate = preferences
        try:
            save_connection_preferences(self.app_settings_path, device, baudrate)
        except AppSettingsError as exc:
            self.status_var.set(f"Connected, but could not save startup settings: {exc}")
            return
        self.preferred_device = device

    def _frame_type_changed(self, _event: Any = None) -> None:
        remote = self.frame_type_var.get().endswith("remote")
        self.data_entry.configure(state="disabled" if remote else "normal")

    def _on_close(self) -> None:
        self._closing = True
        self._stop_dbc_transmission(silent=True)
        if self.worker is None:
            self.root.destroy()
            return
        if self.connected:
            self.worker.send("C", "disconnect_close")
        self.worker.disconnect()
        self.root.after(100, self._close_when_disconnected)

    def _close_when_disconnected(self) -> None:
        if self.worker is None:
            self.root.destroy()
        else:
            self.root.after(100, self._close_when_disconnected)


def main() -> None:
    root = tk.Tk()
    SlcanApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
