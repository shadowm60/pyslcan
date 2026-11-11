import math
import unittest
from unittest.mock import Mock

import cantools

from dbc_transmitter import (
    SignalTxSettings,
    default_signal_settings,
    encode_message,
    signal_value_bounds,
)


DBC = """
VERSION ""
NS_ :
BS_:
BU_: ECU
BO_ 291 Example: 2 ECU
 SG_ Count : 0|8@1+ (1,0) [0|10] "" ECU
 SG_ Temperature : 8|8@1- (0.5,-40) [-40|23.5] "degC" ECU
"""


class DbcTransmitterTests(unittest.TestCase):
    def setUp(self):
        self.message = cantools.database.load_string(DBC).get_message_by_name("Example")

    def test_bounds_use_dbc_physical_minimum_and_maximum(self):
        count = self.message.get_signal_by_name("Count")
        temperature = self.message.get_signal_by_name("Temperature")

        self.assertEqual(signal_value_bounds(count), (0.0, 10.0))
        self.assertEqual(signal_value_bounds(temperature), (-40.0, 23.5))

    def test_missing_dbc_bounds_use_representable_physical_range(self):
        signal = cantools.database.load_string(
            """
            VERSION ""
            NS_ :
            BS_:
            BU_: ECU
            BO_ 1 Unbounded: 1 ECU
             SG_ Value : 0|4@1- (2,-1) [0|0] "" ECU
            """
        ).get_message_by_name("Unbounded").signals[0]

        self.assertEqual(signal_value_bounds(signal), (-17.0, 13.0))

    def test_raw_sweep_and_random_values_are_encoded_in_dbc_layout(self):
        settings = {
            "Count": SignalTxSettings(mode="raw", value=5),
            "Temperature": SignalTxSettings(
                mode="sweep", minimum=-40, maximum=23.5, step=31.5
            ),
        }
        sweep_values = {}
        sweep_directions = {}

        first = encode_message(
            self.message, settings, sweep_values, sweep_directions=sweep_directions
        )
        second = encode_message(
            self.message, settings, sweep_values, sweep_directions=sweep_directions
        )
        self.assertEqual(first, bytes((5, 0)))
        self.assertEqual(second, bytes((5, 0x3F)))

        settings["Temperature"] = SignalTxSettings(
            mode="random", minimum=-40, maximum=23.5
        )
        choose_value = Mock(return_value=0)
        payload = encode_message(
            self.message,
            settings,
            sweep_values,
            random_value=choose_value,
            sweep_directions=sweep_directions,
        )

        self.assertEqual(payload, bytes((5, 0x50)))
        choose_value.assert_called_once_with(-40, 23.5)

    def test_sweep_bounces_between_minimum_and_maximum(self):
        settings = {
            "Count": SignalTxSettings(value=0),
            "Temperature": SignalTxSettings(
                mode="sweep", minimum=-40, maximum=23.5, step=31.5
            ),
        }
        sweep_values = {}
        sweep_directions = {}
        observed_values = []

        for _ in range(5):
            payload = encode_message(
                self.message,
                settings,
                sweep_values,
                sweep_directions=sweep_directions,
            )
            observed_values.append(
                self.message.decode(payload)["Temperature"]
            )

        self.assertEqual(observed_values, [-40, -8.5, 23, 23.5, -8])
        self.assertEqual(sweep_directions["Temperature"], -1)

    def test_sweep_requires_a_valid_step(self):
        settings = {
            "Count": SignalTxSettings(value=0),
            "Temperature": SignalTxSettings(
                mode="sweep", minimum=0, maximum=2, step=3
            ),
        }

        with self.assertRaisesRegex(ValueError, "step"):
            encode_message(self.message, settings, {})

    def test_non_finite_generator_bounds_are_rejected(self):
        settings = {
            "Count": SignalTxSettings(value=0),
            "Temperature": SignalTxSettings(
                mode="random", minimum=0, maximum=math.inf
            ),
        }

        with self.assertRaisesRegex(ValueError, "finite"):
            encode_message(self.message, settings, {})

    def test_defaults_are_static_and_within_bounds(self):
        signal = self.message.get_signal_by_name("Count")

        settings = default_signal_settings(signal)

        self.assertEqual(settings.mode, "raw")
        self.assertEqual(settings.value, 0)
        self.assertEqual((settings.minimum, settings.maximum), (0, 10))
        self.assertEqual(settings.step, 1)
        temperature = self.message.get_signal_by_name("Temperature")
        self.assertEqual(default_signal_settings(temperature).step, 0.5)

    def test_generator_bounds_cannot_exceed_dbc_signal_range(self):
        settings = {
            "Count": SignalTxSettings(value=0),
            "Temperature": SignalTxSettings(
                mode="random", minimum=-41, maximum=23.5
            ),
        }

        with self.assertRaisesRegex(ValueError, "random bounds must be between"):
            encode_message(self.message, settings, {})


if __name__ == "__main__":
    unittest.main()
