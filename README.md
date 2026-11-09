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
a background worker so the UI remains responsive. `CanFrame` is the frame
representation intended to be reused by future DBC or OBD-II decoders.

## OBD-II ECU simulator

The ECU configuration can be edited in the GUI while disconnected; the
simulator itself still only runs with CAN open in Normal mode. Start it
explicitly to answer standard 11-bit single-frame requests. By default it
accepts functional requests on `0x7DF` and physical requests on `0x7E0`, and
replies on `0x7E8`. Unsupported PIDs receive no response.

The **Mode 01 PIDs** tab lets you toggle support, edit built-in signal sweeps
(minimum, maximum, step, and interval), switch a built-in PID to fixed raw
bytes, and add or remove custom PIDs. A custom PID has a name, unit, support
flag, and one-to-five raw response bytes. Supported-PID bitmaps are regenerated
from the configured support flags, including continuation pages through
`0xE0`. PID `0x00` and multiples of `0x20` are reserved for these bitmaps.
The table displays both the current engineering value and raw encoded bytes.
When adding a PID, the editor offers common standardized Mode 01 PIDs with
their function, measurement, unit, valid range, and suggested raw bytes.
Selecting one stores that reference information with the custom PID. The
catalog is a guide, not a complete OBD-II PID list; unlisted PIDs can still be
entered manually, and the simulator currently responds only to Mode 01.

The **Active DTCs (Mode 03)** tab lists configured codes and lets you add,
edit, remove, and activate them. The **MIL on** checkbox controls the MIL bit
independently; PID `0x01` also reports the number of active DTCs. Mode 03 uses
a single CAN frame, so at most three DTCs can be active at once.

`ecu_simulator.json` stores request/response CAN IDs, PID definitions,
algorithms and sweep boundaries, the `mil_on` flag, and DTC descriptions and
activation flags. **Load JSON...**, **Save config**, and **Open JSON** let you
use or edit a configuration. JSON does not support comments; use the
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
`response_id`, and the rest of the `pids` object in a complete configuration.
Multi-frame ISO-TP, other OBD services, and the complete standardized PID
catalog are not implemented.

For reusable context and development guidance when continuing this project
with an AI assistant, see [the project meta-prompt](./prompts/project_meta_prompt.md).

## Device commands

The app uses the device's `O`, `L`, `C`, `S0`-`S8`, `t`/`T`, `r`/`R`,
`Z0`/`Z1`, `F`, `V`/`v`, and `N` commands. Commands are ASCII and terminated
with carriage return. A device `BEL` or missing response is reported in the
status bar. Data-returning queries (`F`, `V`, `v`, and `N`) complete when their
response line is received; other commands complete on the empty-CR
acknowledgement.
