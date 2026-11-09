import queue
import unittest
from unittest.mock import Mock, patch

from serial_worker import SerialWorker


class SerialWorkerTests(unittest.TestCase):
    def make_worker(self, incoming):
        events = queue.Queue()
        worker = SerialWorker("mock-port", 115200, events, response_timeout=0.1)
        serial_port = Mock()
        serial_port.read.side_effect = [incoming, b""]
        serial_port.write.return_value = len(incoming)
        worker._serial = serial_port
        return worker, events

    def test_bel_without_carriage_return_is_command_failure(self):
        worker, events = self.make_worker(b"\x07")
        worker._execute("O", "open")
        event = events.get_nowait()
        self.assertEqual(event["type"], "command")
        self.assertFalse(event["ok"])
        self.assertIn("BEL", event["response"])

    def test_empty_carriage_return_acknowledges_command(self):
        worker, events = self.make_worker(b"\r")
        worker._execute("O", "open")
        event = events.get_nowait()
        self.assertEqual(event["type"], "command")
        self.assertTrue(event["ok"])
        self.assertEqual(event["response"], "")

    def test_receive_frame_does_not_consume_command_response(self):
        worker, events = self.make_worker(b"t1231AA\rF12\r")
        worker._execute("F", "status")
        frame_event = events.get_nowait()
        command_event = events.get_nowait()
        self.assertEqual(frame_event, {"type": "frame", "line": "t1231AA"})
        self.assertEqual(command_event["type"], "command")
        self.assertTrue(command_event["ok"])
        self.assertEqual(command_event["response"], "F12")

    def test_debug_log_contains_command_bytes_and_response_bytes(self):
        worker, events = self.make_worker(b"\r")
        worker.set_debug(True)
        worker._execute("O", "open")

        debug_events = []
        while not events.empty():
            event = events.get_nowait()
            if event["type"] == "debug":
                debug_events.append(event["message"])

        self.assertTrue(any("TX b'O\\r'" in line for line in debug_events))
        self.assertTrue(any("RX raw b'\\r'" in line for line in debug_events))
        self.assertTrue(any("acknowledged with CR" in line for line in debug_events))

    def test_serial_exception_is_reported_with_exception_type(self):
        worker, events = self.make_worker(b"")
        worker._serial.write.side_effect = OSError("device removed")

        with self.assertRaisesRegex(OSError, "device removed"):
            worker._execute("S6", "set_bitrate")

        command_event = events.get_nowait()
        self.assertEqual(command_event["type"], "command")
        self.assertFalse(command_event["ok"])
        self.assertIn("OSError", command_event["response"])

    def test_worker_closes_port_and_reports_transport_failure(self):
        events = queue.Queue()
        worker = SerialWorker("mock-port", 115200, events, debug=True)
        worker.send("S6", "set_bitrate")
        serial_port = Mock()
        serial_port.is_open = True
        serial_port.write.side_effect = OSError("device removed")
        with patch("serial_worker.serial.Serial", return_value=serial_port):
            worker.run()

        received = []
        while not events.empty():
            received.append(events.get_nowait())
        self.assertIn("connected", [event["type"] for event in received])
        self.assertIn("error", [event["type"] for event in received])
        self.assertEqual(received[-1]["type"], "disconnected")
        self.assertTrue(any("OSError" in event["message"] for event in received if event["type"] == "debug"))
        serial_port.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
