import unittest

from isotp import IsoTpSender
from slcan_protocol import CanFrame


def flow_control(status=0, block_size=0, separation=0, arbitration_id=0x7E0):
    return CanFrame(
        arbitration_id,
        False,
        False,
        8,
        bytes((0x30 | status, block_size, separation, 0, 0, 0, 0, 0)),
    )


class IsoTpSenderTests(unittest.TestCase):
    def test_short_payload_is_a_single_frame(self):
        sender = IsoTpSender(0x7E8, 0x7E0, b"\x41\x0C\x0C\x80")

        frame = sender.initial_frame()

        self.assertEqual(
            frame.data, bytes((4, 0x41, 0x0C, 0x0C, 0x80, 0x55, 0x55, 0x55))
        )
        self.assertTrue(sender.complete)

    def test_multiframe_waits_for_flow_control_then_segments_payload(self):
        payload = bytes(range(1, 16))
        sender = IsoTpSender(0x7E8, 0x7E0, payload)

        first_frame = sender.initial_frame()

        self.assertEqual(first_frame.data, bytes((0x10, 15, 1, 2, 3, 4, 5, 6)))
        self.assertTrue(sender.waiting_for_flow_control)
        self.assertIsNone(sender.next_frame())
        self.assertTrue(sender.accept_flow_control(flow_control()))

        consecutive_1 = sender.next_frame()
        consecutive_2 = sender.next_frame()

        self.assertEqual(consecutive_1.data, bytes((0x21, 7, 8, 9, 10, 11, 12, 13)))
        self.assertEqual(
            consecutive_2.data, bytes((0x22, 14, 15, 0x55, 0x55, 0x55, 0x55, 0x55))
        )
        self.assertTrue(sender.complete)

    def test_block_size_requires_a_new_flow_control_frame(self):
        sender = IsoTpSender(0x7E8, 0x7E0, bytes(range(20)))
        sender.initial_frame()
        sender.accept_flow_control(flow_control(block_size=1))

        self.assertIsNotNone(sender.next_frame())
        self.assertTrue(sender.waiting_for_flow_control)
        self.assertIsNone(sender.next_frame())
        self.assertTrue(sender.accept_flow_control(flow_control(block_size=1)))
        self.assertIsNotNone(sender.next_frame())
        self.assertTrue(sender.complete)

    def test_separation_time_and_flow_control_errors_are_reported(self):
        sender = IsoTpSender(0x7E8, 0x7E0, bytes(range(10)))
        sender.initial_frame()
        self.assertFalse(sender.accept_flow_control(flow_control(arbitration_id=0x7E1)))
        self.assertTrue(sender.accept_flow_control(flow_control(separation=5)))
        self.assertEqual(sender.st_min_ms, 5)

        overflow = IsoTpSender(0x7E8, 0x7E0, bytes(range(10)))
        overflow.initial_frame()
        self.assertTrue(overflow.accept_flow_control(flow_control(status=2)))
        self.assertTrue(overflow.complete)
        self.assertIn("overflow", overflow.error)


if __name__ == "__main__":
    unittest.main()
