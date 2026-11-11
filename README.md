# SLCAN CAN Interface Tester

A small Tkinter application for testing a custom serial SLCAN interface. It
supports standard and extended CAN data/remote frames, normal and listen-only
CAN modes, bitrate selection, live receive, device status queries, optional
receive timestamps, and CSV export.

## Requirements and launch

- Python 3.9 or newer
- Tk support for Python
- A serial connection to the interface

On Linux, use a virtual environment so `pip` can install the app's dependencies
without modifying the system-managed Python installation. If you already have
a `.venv` in the project directory, activate it and skip the creation step:

```sh
source .venv/bin/activate
python -m pip install -r requirements.txt
python app.py
```

To create the environment once, run these commands from the project directory:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python app.py
```

If `python3 -m venv` reports that `venv` is unavailable, install your
distribution's venv package (for example, `sudo apt install python3-venv` on
Debian/Ubuntu), then retry. Tkinter is also required; on Debian/Ubuntu install
it with `sudo apt install python3-tk` for the Python version used to create the
environment. When the environment is active, `python` and `pip` refer to the
`.venv` interpreter. Later, run `source .venv/bin/activate` before launching
with `python app.py`; run `deactivate` when finished. Linux users may need to
add their account to the system's serial-device group (commonly `dialout`) and
re-login before opening `/dev/ttyUSB*` or `/dev/ttyACM*` ports.

Select the serial port and its UART baud rate, then connect. Select a CAN
bitrate and mode and choose **Set bitrate and open CAN**. The bitrate command
is sent before the CAN open command, as required by the device. Close CAN
before changing bitrate. Serial baud rate is independent of CAN bitrate.
The app remembers the last successfully connected serial device and baud rate
in a per-user settings file and restores them on the next launch.

The main frame table records received and transmitted CAN frames. **Export
ASC...** saves the current trace in a CANoe-style ASC file with relative host
timestamps, channel 1, frame direction/type, CAN ID, DLC, and data bytes.
Device timestamps are retained as comments when present. The export includes
only frames observed and recorded by this app.

## Serial debug mode

Enable **Serial debug** before connecting (or while connected) to open the
timestamped debug log. It records serial open settings, transmitted command
bytes, raw received bytes, parsed responses/frames, timeouts, and serial
exceptions. The log is held in memory (up to 3,000 entries) and can be cleared
from its window or saved as a text file. It is not automatically written to
disk. A transport-level serial exception closes the port and is reported in
the status bar with the exception type and message. A device `BEL` or response
timeout is logged as a command failure without by itself closing the serial
connection.

## Bluetooth SPP

### Windows
Choose **Scan Bluetooth** to discover nearby and previously paired Bluetooth
Classic devices. Select the adapter and choose **Pair / connect in Windows**.
Windows Bluetooth settings handles pairing and connecting; complete that step
there. The app checks for newly created serial ports for up to two minutes,
refreshes the port list, and selects a single new port automatically. If
the Bluetooth serial port already exists, it is selected immediately. If
Windows creates multiple ports, or the port appears later, use **Refresh** and
select the port whose description identifies it as a Bluetooth serial link.
Bluetooth SPP connections appear to the CAN app as regular serial COM ports.

### Linux
Pair and connect the adapter using the desktop's Bluetooth tools. Configure its
SPP service as a serial device using the Bluetooth tools available on the
system (commonly `/dev/rfcomm0`), then select that port in the app and use
**Refresh** if it is not listed yet. The app does not configure pairing or
RFCOMM itself on Linux.

The frame editor accepts a hexadecimal identifier and hexadecimal data bytes
separated by spaces or commas. DLC must match the number of bytes for data
frames. Remote frames carry a DLC but no data. In listen-only mode the app
disables transmission.

The device protocol is implemented in `slcan_protocol.py`; serial I/O runs on
a background worker so the UI remains responsive.

## DBC cyclic transmitter

The **DBC transmitter** tab loads a `.dbc` file and lists its messages. Select
one or more messages and use **Toggle selected**, or click a row's **State**
cell, to start or stop individual messages while CAN is open in Normal mode.
Several messages can run together; **Stop all** stops them at once. Each
message uses its DBC cycle time by default; double-click a **DBC cycle (ms)**,
**Override (ms)**, or **Effective (ms)** cell to set a custom period. If the
DBC has no cycle time, the transmitter uses a visible 100 ms fallback that can
also be overridden.

Select a message to view its signals, then double-click a signal to configure
it as a fixed physical value (`raw`), a bouncing sweep, or a random value
within a physical range. Signal values are encoded using the DBC scaling and
layout. Sweeps bounce between their minimum and maximum. Default signal bounds
come from the DBC; if bounds are missing, the UI falls back to the full range
representable by that signal's bit length and scale. The default sweep step is
the signal scale (its physical resolution). For practical ranges such as an
engine's RPM limit, set the signal's minimum and maximum in the DBC; DBC does
not define a separate signal step. Configured values and ranges are checked
against the DBC bounds. The transmitter supports up to 8 data bytes per
message, matching the current SLCAN protocol. Frames are serialized through
the serial worker; periods shorter than the interface can send are not
guaranteed.

Use **Save as...** to create multiple named DBC configurations; **Save**
updates the selected configuration. A profile stores its DBC path, selected
messages, which messages were running, cycle overrides, and signal
values/generator settings. The last loaded or saved profile is restored on the
next launch, but transmission does not start automatically. After opening CAN
in Normal mode, use **Start saved** to resume messages that were running when
the profile was saved. Profiles are stored in the per-user `dbc_profiles.json`
settings file.

## Estimated CAN bus load

The monitor shows a rolling 10-second utilization estimate at the selected CAN
bitrate. It counts frames observed by the adapter in both directions and
estimates Classical CAN frame bits, including protocol overhead and approximate
bit stuffing. Frames hidden by adapter filters or traffic not received by the
app are not counted, so this is an estimate of observed traffic rather than a
measurement of total physical bus utilization.

## OBD-II ECU simulator

The ECU configuration can be edited in the GUI while disconnected; the
simulator itself still only runs with CAN open in Normal mode. Start it
explicitly to answer standard 11-bit requests. By default it
accepts functional requests on `0x7DF` and physical requests on `0x7E0`, and
replies on `0x7E8`. Unsupported PIDs receive no response.

The **Multi-PID responses** checkbox controls whether a Mode 01 request
containing multiple PIDs returns all supported requested values (enabled) or
only the first PID (disabled). It is saved in `ecu_simulator.json` and defaults
to disabled. A combined response is sent as one ISO-TP payload, segmented into
single- or multi-frame CAN messages as needed. Multi-frame replies wait for
tester flow control and honor its block size and separation time; random values
refresh only for requested PIDs.
Unused bytes in outgoing OBD and ISO-TP frames are padded with `55` for easier
trace inspection.

The **Mode 01 PIDs** tab lets you toggle support, edit built-in signal sweeps
(minimum, maximum, step, and interval), select on-read random values for
supported numeric PIDs, switch a PID to fixed raw bytes, and add or remove
custom PIDs. Random values are generated only when that PID is requested, not
on a timer. Numeric standard PIDs with known single-value encodings offer
sweep, random, and raw algorithms; compound/status PIDs remain raw. Custom
vendor PIDs have a name, unit, support flag, and one-to-five raw response bytes.
Supported-PID bitmaps are regenerated
from the configured support flags, including continuation pages through
`0xE0`. PID `0x00` and multiples of `0x20` are reserved for these bitmaps.
The table displays both the current engineering value and raw encoded bytes.
When adding a PID, the editor offers standardized Mode 01 PIDs by number and
name, including suggested raw response bytes. Catalog entries with supported
numeric encoders also provide physical ranges and standard byte conversion.
PID `0x24` offers independent sweep/random bounds for equivalence ratio
(`0`-`2`) and sensor voltage (`0`-`8 V`).
Entries needing more than five response bytes are identified as requiring
multi-frame CAN and are disabled by default. Multi-PID replies may use ISO-TP
multi-frame transport, but configured raw PID values remain limited to five
bytes.
The list is based on the [OBD-II PID reference](https://en.wikipedia.org/wiki/OBD-II_PIDs);
manufacturer-specific or otherwise unlisted PIDs can still be entered manually.
The simulator responds to Mode 01 data, Mode 03 DTC queries, and Mode 04
clear-DTC requests.

The **Active DTCs (Mode 03)** tab lists configured codes and lets you add,
edit, remove, and activate them. The **MIL on** checkbox controls the MIL bit
independently; PID `0x01` also reports the number of active DTCs. Mode 03 uses
a single CAN frame, so at most three DTCs can be active at once. A Mode 04
clear-DTC request receives a positive response and suppresses configured DTCs
and the MIL status for the remainder of that simulator session. Starting a new
simulator session makes the configured DTCs reportable again.

`ecu_simulator.json` stores request/response CAN IDs, the
`multi_pid_response_active` setting, PID definitions, algorithms and sweep
boundaries, the `mil_on` flag, and DTC descriptions and activation flags.
**Load JSON...**, **Save config**, and **Open JSON** let you use or edit a
configuration. JSON does not support comments; use the
self-describing field names or the GUI rather than adding comment syntax. For
example, a custom raw PID and active DTC use these fields:

```json
{
  "mil_on": true,
  "pids": {
    "61": {
      "name": "Custom pressure",
      "unit": "kPa",
      "supported": true,
      "algorithm": "raw",
      "raw_bytes": "12 34"
    }
  },
  "dtcs": [
    {
      "code": "P0133",
      "description": "O2 sensor circuit slow response",
      "active": true
    }
  ]
}
```

The example shows the relevant fields only; retain `version`, `request_ids`,
`response_id`, `multi_pid_response_active`, and the rest of the `pids` object
in a complete configuration. ISO-TP multi-frame support is currently used for
aggregated Mode 01 replies; other OBD services and the complete standardized
PID catalog are not implemented.

For reusable context and development guidance when continuing this project
with an AI assistant, see [the project meta-prompt](./prompts/project_meta_prompt.md).

## Device commands

The app uses the device's `O`, `L`, `C`, `S0`-`S8`, `t`/`T`, `r`/`R`,
`Z0`/`Z1`, `F`, `V`/`v`, and `N` commands. Commands are ASCII and terminated
with carriage return. A device `BEL` or missing response is reported in the
status bar. Data-returning queries (`F`, `V`, `v`, and `N`) complete when their
response line is received; other commands complete on the empty-CR
acknowledgement.
