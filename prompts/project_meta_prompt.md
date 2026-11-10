# Project Meta-Prompt: SLCAN CAN Interface Tester

Use the following prompt when asking an AI assistant to continue work on this
project:

---

You are helping maintain and extend a cross-platform desktop test application
for a custom SLCAN CAN interface. The application is written in Python with
Tkinter and supports Windows and Linux. It is a hardware-development and
diagnostic tool, so protocol correctness, clear error reporting, and safe,
observable serial behavior are more important than adding features quickly.

## Before making changes

1. Inspect the current source, tests, and README before assuming how a feature
   works. Treat the checked-in implementation as authoritative if it differs
   from this prompt.
2. Keep changes focused, preserve existing behavior, and use the project's
   current patterns and dependencies unless there is a clear reason not to.
3. If device behavior or a requirement is ambiguous, ask for clarification
   instead of inventing protocol semantics.
4. Do not claim physical-device validation unless the hardware was actually
   tested. Distinguish unit-test results from hardware results.

## Current application

- `app.py` provides the Tkinter user interface for serial configuration,
  CAN bitrate and mode selection, transmitting frames, displaying received
  and transmitted frames, querying device information, exporting frames as
  CSV, and hosting the ECU simulator tab.
- `serial_worker.py` owns serial I/O in a background thread. It communicates
  with the UI through a thread-safe event queue so serial reads do not block
  the Tkinter event loop.
- `slcan_protocol.py` defines the `CanFrame` model, bitrate table, frame
  encoder, frame parser, and approximate Classical CAN wire-bit estimator.
  Keep protocol logic independent of Tk widgets so future DBC or OBD-II
  decoding can use the same frame model.
- `bluetooth_windows.py` discovers Windows Bluetooth Classic devices for SPP.
  Windows handles pairing/connection; the resulting virtual COM port is then
  opened through pySerial. On Linux, pair/connect and configure the SPP service
  as a serial device with system Bluetooth tools, then choose its port in the
  app. The app does not set up pairing or RFCOMM on Linux.
- `ecu_simulator.py` implements ECU simulator JSON configuration and OBD
  responses. It handles standard 11-bit single-frame Mode 01 PID requests and
  Mode 03/04 DTC requests. Numeric PIDs support configured sweeps, on-read
  random values, and fixed raw bytes; compound/status PIDs may remain raw.
- `ecu_simulator.json` is the default user-editable ECU profile: it accepts
  functional `0x7DF` and physical `0x7E0` requests and replies on `0x7E8`.
  Its built-in Mode 01 PIDs are enabled by default and it includes sample
  DTCs. `ecu_simulator_dtc.json` is an additional profile with active DTCs.
- `tests/` contains protocol, serial-worker, Bluetooth-discovery, port
  selection, and ECU simulator tests. Extend or add targeted tests with
  behavior changes.
- `README.md` documents Windows/Linux setup, connection flow, device commands,
  debug behavior, estimated bus load, and the ECU simulator. Update it when
  user-visible behavior changes.

## Custom device protocol

Serial commands are ASCII and terminated with carriage return (`CR`).

- `O`: open CAN in normal mode.
- `L`: open CAN in listen-only mode; transmit is not allowed.
- `C`: close CAN.
- `S0` through `S8`: select bitrate while CAN is closed:
  10, 20, 50, 100, 125, 250, 500, 800 kbit/s, and 1 Mbit/s respectively.
- `t` / `T`: standard / extended data frame.
- `r` / `R`: standard / extended remote frame.
- `Z0` / `Z1`: disable / enable receive timestamps.
- `F`: return status flags as two hex digits followed by CR.
- `V` / `v`: return firmware version followed by CR.
- `N`: return six hex digits of the serial number.
- Successful action commands return an empty CR; a failed command returns
  BEL. Data queries return a non-empty response line followed by CR.

CAN frames use hexadecimal fields:

- Standard IDs have 3 hex digits; extended IDs have 8.
- DLC is one hex digit from 0 to 8.
- Data frames contain two hex digits per payload byte.
- Remote frames contain no payload, regardless of DLC.
- Received lines may have an optional four-hex-digit timestamp appended before
  CR.

The SLCAN monitor reports an estimated rolling 10-second CAN utilization using
the selected bitrate and received/acknowledged TX frames visible to the app.
The estimator includes frame overhead and approximate bit stuffing. It does
not know about adapter-filtered or otherwise unseen traffic, so it is an
estimate of observed frames, not a measurement of total physical bus load.

## ECU simulator behavior

The ECU configuration tab remains available while disconnected. Configuration
can be edited and saved at any time, but the simulator only runs when CAN is
open in Normal mode and the user explicitly starts it. It is stopped when CAN
closes or the serial link disconnects. The engine is in `ecu_simulator.py`;
the Tkinter tab, frame dispatch, and run/stop lifecycle are in `app.py`.

The default `ecu_simulator.json` profile uses these 11-bit CAN IDs:

- Request IDs: functional `0x7DF` and physical `0x7E0`.
- Response ID: `0x7E8`.

The request/response IDs are editable in the tab and persisted to the JSON
profile. The profile defines supported flags, PID algorithms, numeric sweep
or random bounds, raw bytes, MIL state, and DTC objects. Loading an alternate
JSON profile changes which file is used for subsequent saves. JSON comments
are not supported; document configuration with field names and README
examples, not comment syntax.

The built-in Mode 01 PID set is `01`, `04`, `05`, `0C`, `0D`, `0F`, `11`,
`1F`, `2F`, `42`, `46`, and `5C`: monitor status, calculated load, coolant
temperature, RPM, vehicle speed, intake temperature, throttle, runtime, fuel
level, control module voltage, ambient temperature, and oil temperature.
Numeric built-ins use per-PID minimum, maximum, step, and interval fields for
triangular sweeps. Supported numeric PIDs can use `sweep`, `random`, or `raw`
algorithms. Sweep values advance on a time interval; random values are
generated only when that PID is requested, not by a timer or table refresh.
Standard PID encoders convert engineering values into response bytes.
Compound/status PIDs without a single numeric range remain raw unless a
specific composite encoding is implemented. PID `0x24` is implemented as a
composite numeric PID with independent equivalence-ratio (`0`-`2`) and sensor
voltage (`0`-`8 V`) bounds, separate sweep steps, and the corresponding
four-byte encoding.

In the **Mode 01 PIDs** GUI tab, users can toggle support, configure an
algorithm and its boundaries, and add or remove custom PIDs. The standard
Mode 01 catalog pre-fills names and raw response defaults. A custom/vendor PID
needs a name, unit, supported flag, and one to five raw bytes; custom PIDs use
the raw algorithm unless they have an implemented numeric encoder. The table
shows current engineering values and response bytes. Double-clicking the
**Supported** cell toggles support; double-clicking other cells opens the
editor. PIDs requiring more than five response bytes are disabled by default
because ISO-TP multi-frame replies are not implemented.

Supported-PID bitmaps are derived from supported configured PIDs, including
continuation pages from `0x00` through `0xE0`. PID `0x00` and multiples of
`0x20` are reserved for bitmap responses. Adding, removing, or toggling a
custom PID must update the appropriate bitmap page and continuation flags.

The **Active DTCs (Mode 03)** GUI tab allows adding, editing, removing, and
activating configured DTCs. Mode 03 responds with the active DTC list, limited
to three active codes so it fits in the current single-frame response. The
`mil_on` configuration flag is independent of DTC activation. PID `0x01`
reports the MIL bit and active-DTC count. A valid Mode 04 request receives a
positive response and suppresses configured DTCs and MIL status until the next
explicit simulator session start; it does not modify the saved DTC profile.
Multi-frame ISO-TP and OBD services other than Mode 01, 03, and 04 are not
implemented. The add-PID catalog covers standardized Mode 01 PID identifiers,
but sweep/random behavior is available only for PIDs with an implemented
numeric encoder and boundaries. Do not infer further protocol behavior than
the implementation and tests establish.

When extending the simulator, preserve separation between frame parsing,
simulation/configuration logic, and Tkinter widgets. Add tests for the exact
OBD request bytes, response CAN ID, ISO-TP length byte, PID encoding and
bitmap page continuation, DTC encoding, MIL/count status, unsupported-PID
behavior, and configuration validation/persistence. Preserve raw CAN logging
so tester-visible responses can be compared with the table values.

Do not silently treat malformed input, a timeout, BEL, or a serial transport
exception as success. Keep command responses distinguishable from CAN frames
that may arrive while a command is in progress.

## Debugging and reliability

The UI has an opt-in **Serial debug** mode and a **View debug log** window.
Debug output includes timestamps, serial open settings, transmitted bytes,
raw received bytes, parsed lines, timeouts, and exception details. It is
bounded in memory and can be saved by the user; do not write serial traffic to
disk automatically.

When diagnosing a disconnect:

1. Preserve and inspect the serial exception type/message and debug sequence.
2. Determine whether the device returned BEL, the response timed out, or the
   operating system/serial driver raised an exception.
3. Avoid swallowing exceptions or reporting a successful connection after a
   failed serial operation.
4. Ensure an error does not accidentally cause duplicate close/disconnect
   commands or mask the original failure.
5. Add regression tests for the observed failure path using mocks where
   practical.

## Development and validation

- Keep serial I/O and Bluetooth discovery off the Tkinter UI thread.
- Tkinter modal editor windows must be mapped before setting a grab: call
  `wait_visibility()` before `grab_set()` for newly created `Toplevel`s. Apply
  this to PID and DTC dialogs to avoid `TclError: grab failed: window not
  viewable` when the dialog is opened from a table callback.
- Use only the dependencies already declared in `requirements.txt` unless a
  new dependency is necessary.
- Run the narrowest relevant tests, then the full test suite when practical:
  `python -m unittest discover -s tests -v` (use `python3` if needed).
- Check syntax/imports with:
  `python -m py_compile app.py bluetooth_windows.py ecu_simulator.py serial_worker.py slcan_protocol.py`
  (use `python3` if needed).
- Summarize changed files, validation results, and any remaining hardware
  verification needed.

## Likely future extensions

Potential extensions include multi-frame ISO-TP responses, additional OBD-II
services, DBC-based signal decoding, improved filtering/search, and more
detailed status flag presentation. The bus-load display is an estimate based
only on traffic observed by this app; exact physical utilization would require
adapter/hardware-level support. Implement extensions as separable layers over
`CanFrame` and keep raw frame logging available so decoding never hides the
original traffic. Validate protocol semantics against tests and document
current single-frame limitations.

---
