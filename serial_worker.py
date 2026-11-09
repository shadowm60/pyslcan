"""Non-blocking serial transport for the SLCAN Tkinter application."""

import queue
import threading
import time
from datetime import datetime
from typing import Any, Dict, Optional, Tuple

import serial

from slcan_protocol import is_frame_line


class SerialWorker(threading.Thread):
    def __init__(
        self,
        port: str,
        baud_rate: int,
        events: "queue.Queue[Dict[str, Any]]",
        response_timeout: float = 1.5,
        debug: bool = False,
    ) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud_rate = baud_rate
        self.events = events
        self.response_timeout = response_timeout
        self._debug_enabled = debug
        self._debug_lock = threading.Lock()
        self.commands: "queue.Queue[Optional[Tuple[str, str]]]" = queue.Queue()
        self._serial: Optional[serial.Serial] = None
        self._buffer = bytearray()

    def send(self, command: str, tag: str) -> None:
        self.commands.put((command, tag))

    def disconnect(self) -> None:
        self.commands.put(None)

    def set_debug(self, enabled: bool) -> None:
        with self._debug_lock:
            self._debug_enabled = enabled

    def _debug(self, message: str) -> None:
        with self._debug_lock:
            enabled = self._debug_enabled
        if enabled:
            timestamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
            self.events.put({"type": "debug", "message": f"{timestamp} {message}"})

    def run(self) -> None:
        try:
            self._debug(
                f"Opening serial port {self.port} at {self.baud_rate} baud "
                f"(timeout=0.05s, write_timeout=1.0s)"
            )
            self._serial = serial.Serial(
                self.port, self.baud_rate, timeout=0.05, write_timeout=1.0
            )
            self._debug("Serial port opened successfully.")
            self.events.put({"type": "connected"})
            while True:
                try:
                    item = self.commands.get_nowait()
                except queue.Empty:
                    item = ...
                if item is None:
                    self._debug("Disconnect requested.")
                    break
                if item is not ...:
                    command, tag = item
                    self._execute(command, tag)
                self._read_available()
        except (serial.SerialException, OSError) as exc:
            self._debug(f"Serial worker failed: {type(exc).__name__}: {exc!r}")
            self.events.put({"type": "error", "message": str(exc)})
        finally:
            if self._serial is not None and self._serial.is_open:
                try:
                    self._serial.close()
                    self._debug("Serial port closed.")
                except (serial.SerialException, OSError) as exc:
                    self._debug(f"Error closing serial port: {type(exc).__name__}: {exc!r}")
                    self.events.put({"type": "error", "message": f"Error closing serial port: {exc}"})
            self.events.put({"type": "disconnected"})

    def _execute(self, command: str, tag: str) -> None:
        assert self._serial is not None
        try:
            payload = command.encode("ascii") + b"\r"
            self._debug(f"TX {payload!r} (hex={payload.hex(' ').upper()}) tag={tag}")
            written = self._serial.write(payload)
            self._debug(f"Serial write returned {written} byte(s).")
            self._serial.flush()
            deadline = time.monotonic() + self.response_timeout
            response = []
            while time.monotonic() < deadline:
                line = self._read_line()
                if line is None:
                    continue
                if line == "\x07":
                    self._debug(f"RX BEL while waiting for command {command!r}.")
                    self.events.put(
                        {
                            "type": "command",
                            "tag": tag,
                            "command": command,
                            "ok": False,
                            "response": "Device returned BEL (command failed).",
                        }
                    )
                    return
                if command in ("F", "V", "v", "N") and line and not is_frame_line(line):
                    self._debug(f"Command {command!r} response line: {line!r}")
                    self.events.put(
                        {
                            "type": "command",
                            "tag": tag,
                            "command": command,
                            "ok": True,
                            "response": line,
                        }
                    )
                    return
                if not line:
                    self._debug(
                        f"Command {command!r} acknowledged with CR; response={response!r}"
                    )
                    self.events.put(
                        {
                            "type": "command",
                            "tag": tag,
                            "command": command,
                            "ok": True,
                            "response": " ".join(response),
                        }
                    )
                    return
                if is_frame_line(line):
                    self._debug(f"RX CAN frame while awaiting {command!r}: {line!r}")
                    self.events.put({"type": "frame", "line": line})
                else:
                    self._debug(f"RX response line while awaiting {command!r}: {line!r}")
                    response.append(line)
            self._debug(f"Timeout waiting for command {command!r} response.")
            self.events.put(
                {
                    "type": "command",
                    "tag": tag,
                    "command": command,
                    "ok": False,
                    "response": "Timed out waiting for the command response.",
                }
            )
        except (serial.SerialException, OSError) as exc:
            self._debug(f"Command {command!r} caused {type(exc).__name__}: {exc!r}")
            self.events.put(
                {
                    "type": "command",
                    "tag": tag,
                    "command": command,
                    "ok": False,
                    "response": f"{type(exc).__name__}: {exc}",
                }
            )
            raise

    def _read_available(self) -> None:
        while True:
            line = self._read_line()
            if line is None:
                return
            if not line:
                continue
            if line == "\x07":
                self._debug("RX BEL while idle.")
                self.events.put(
                    {"type": "error", "message": "Device reported a command error (BEL)."}
                )
            elif is_frame_line(line):
                self._debug(f"RX CAN frame: {line!r}")
                self.events.put({"type": "frame", "line": line})
            else:
                self._debug(f"RX unsolicited line: {line!r}")
                self.events.put({"type": "text", "line": line})

    def _read_line(self) -> Optional[str]:
        assert self._serial is not None
        if 0x0D not in self._buffer and 0x07 not in self._buffer:
            data = self._serial.read(256)
            if data:
                self._buffer.extend(data)
                self._debug(f"RX raw {data!r} (hex={data.hex(' ').upper()})")

        bell = self._buffer.find(0x07)
        try:
            terminator = self._buffer.index(0x0D)
        except ValueError:
            terminator = -1
        if bell >= 0 and (terminator < 0 or bell < terminator):
            del self._buffer[bell]
            if bell < len(self._buffer) and self._buffer[bell] == 0x0D:
                del self._buffer[bell]
            return "\x07"
        try:
            terminator = self._buffer.index(0x0D)
        except ValueError:
            return None

        raw_line = bytes(self._buffer[:terminator])
        del self._buffer[: terminator + 1]
        if raw_line.endswith(b"\n"):
            raw_line = raw_line[:-1]
        return raw_line.decode("ascii", errors="replace")
