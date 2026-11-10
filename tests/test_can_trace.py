import unittest

from can_trace import format_asc_trace


class CanTraceTests(unittest.TestCase):
    def test_formats_rx_tx_standard_extended_and_remote_frames(self):
        records = [
            (
                "2026-11-10T12:00:00.000000+00:00",
                "RX",
                "Standard data",
                "7E0",
                3,
                "02 01 0C",
                "1234",
            ),
            (
                "2026-11-10T12:00:00.125000+00:00",
                "TX",
                "Standard data",
                "7E8",
                4,
                "04 41 0C 0C",
                "",
            ),
            (
                "2026-11-10T12:00:00.250000+00:00",
                "RX",
                "Extended remote",
                "18DAF110",
                8,
                "",
                "",
            ),
        ]

        trace = format_asc_trace(records)

        self.assertIn("date Tue Nov 10 12:00:00.000000 PM 2026", trace)
        self.assertIn("0.000000 Start of measurement", trace)
        self.assertIn("0.000000 1 7E0 Rx d 3 02 01 0C", trace)
        self.assertIn("// SLCAN device timestamp: 1234", trace)
        self.assertIn("0.125000 1 7E8 Tx d 4 04 41 0C 0C", trace)
        self.assertIn("0.250000 1 18DAF110x Rx r 8", trace)
        self.assertTrue(trace.endswith("0.250000 End TriggerBlock\n"))

    def test_rejects_empty_or_invalid_trace_records(self):
        with self.assertRaisesRegex(ValueError, "no CAN frames"):
            format_asc_trace([])
        with self.assertRaisesRegex(ValueError, "Unsupported trace direction"):
            format_asc_trace(
                [("2026-11-10T12:00:00+00:00", "IN", "Standard data", "123", 0, "", "")]
            )


if __name__ == "__main__":
    unittest.main()
