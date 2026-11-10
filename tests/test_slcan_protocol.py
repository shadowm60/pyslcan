import unittest

from slcan_protocol import (
    BITRATES,
    CanFrame,
    encode_frame,
    estimate_can_frame_bits,
    parse_frame,
)


class SlcanProtocolTests(unittest.TestCase):
    def test_bitrate_table(self):
        self.assertEqual(BITRATES["S0"], 10_000)
        self.assertEqual(BITRATES["S8"], 1_000_000)

    def test_estimates_classic_can_frame_wire_bits(self):
        standard = CanFrame(0x123, False, False, 8, bytes(8))
        extended = CanFrame(0x1234567, True, False, 8, bytes(8))

        self.assertEqual(estimate_can_frame_bits(standard), 131)
        self.assertEqual(estimate_can_frame_bits(extended), 155)

    def test_standard_data_frame_round_trip(self):
        frame = CanFrame(0x123, False, False, 2, bytes.fromhex("DE AD"))
        command = encode_frame(frame)
        self.assertEqual(command, "t1232DEAD")
        self.assertEqual(parse_frame(command), frame)

    def test_extended_frame_with_timestamp(self):
        frame = parse_frame("T01ABCDEE2AABB03F2")
        self.assertEqual(frame.arbitration_id, 0x01ABCDEE)
        self.assertEqual(frame.data, bytes.fromhex("AA BB"))
        self.assertEqual(frame.timestamp, 0x03F2)
        self.assertTrue(frame.extended)

    def test_remote_frame(self):
        frame = CanFrame(0x7FF, False, True, 8)
        command = encode_frame(frame)
        self.assertEqual(command, "r7FF8")
        self.assertEqual(parse_frame(command), frame)

    def test_extended_data_frame_round_trip(self):
        frame = CanFrame(0x1ABCDEFF, True, False, 1, bytes.fromhex("5A"))
        command = encode_frame(frame)
        self.assertEqual(command, "T1ABCDEFF15A")
        self.assertEqual(parse_frame(command), frame)

    def test_standard_receive_frame_without_timestamp(self):
        frame = parse_frame("t4561FE")
        self.assertEqual(frame, CanFrame(0x456, False, False, 1, b"\xFE"))

    def test_rejects_mismatched_data_length(self):
        with self.assertRaisesRegex(ValueError, "match the DLC"):
            encode_frame(CanFrame(0x123, False, False, 2, b"\x01"))
        with self.assertRaisesRegex(ValueError, "payload length"):
            parse_frame("t1232AA")

    def test_rejects_invalid_identifier(self):
        with self.assertRaisesRegex(ValueError, "Identifier"):
            encode_frame(CanFrame(0x800, False, False, 0))

    def test_rejects_invalid_dlc(self):
        with self.assertRaisesRegex(ValueError, "DLC"):
            parse_frame("t1239")


if __name__ == "__main__":
    unittest.main()
