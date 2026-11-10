import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from ecu_simulator import (
    EcuSimulator,
    PID_CATALOG,
    SimulatorConfigError,
    default_config,
    load_config,
    save_config,
)
from slcan_protocol import CanFrame


class EcuSimulatorTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.simulator = EcuSimulator(default_config(), clock=lambda: self.now)

    def test_default_config_has_engine_ids_supported_common_pids_and_passive_dtcs(self):
        config = default_config()
        self.assertEqual(config["request_ids"], ["0x7DF", "0x7E0"])
        self.assertEqual(config["response_id"], "0x7E8")
        self.assertTrue(all(entry["supported"] for entry in config["pids"].values()))
        self.assertTrue(all(not dtc["active"] for dtc in config["dtcs"]))

    def test_mode01_rpm_request_returns_iso_tp_single_frame_response(self):
        request = CanFrame(0x7DF, False, False, 8, bytes((2, 1, 0x0C, 0, 0, 0, 0, 0)))

        response = self.simulator.handle_request(request)

        self.assertEqual(response.arbitration_id, 0x7E8)
        self.assertEqual(response.dlc, 8)
        self.assertEqual(response.data, bytes((4, 0x41, 0x0C, 0x0C, 0x80, 0, 0, 0)))

    def test_physical_request_is_accepted_and_unsupported_pid_is_ignored(self):
        request = CanFrame(0x7E0, False, False, 8, bytes((2, 1, 0x0C, 0, 0, 0, 0, 0)))
        self.assertIsNotNone(self.simulator.handle_request(request))

        config = default_config()
        config["pids"]["0C"]["supported"] = False
        simulator = EcuSimulator(config, clock=lambda: self.now)
        self.assertIsNone(simulator.handle_request(request))

    def test_rejects_non_obd_and_unconfigured_frames(self):
        obd_payload = bytes((2, 1, 0x0C, 0, 0, 0, 0, 0))
        wrong_id = CanFrame(0x123, False, False, 8, obd_payload)
        extended = CanFrame(0x7DF, True, False, 8, obd_payload)
        wrong_service = CanFrame(0x7DF, False, False, 8, bytes((2, 3, 0x0C, 0, 0, 0, 0, 0)))
        self.assertIsNone(self.simulator.handle_request(wrong_id))
        self.assertIsNone(self.simulator.handle_request(extended))
        self.assertIsNone(self.simulator.handle_request(wrong_service))

    def test_supported_bitmap_marks_configured_pids_and_continuation_pages(self):
        page_00 = int.from_bytes(self.simulator.supported_bitmap(0x00), "big")
        page_20 = int.from_bytes(self.simulator.supported_bitmap(0x20), "big")
        page_40 = int.from_bytes(self.simulator.supported_bitmap(0x40), "big")
        self.assertTrue(page_00 & (1 << 28))  # PID 0x04
        self.assertTrue(page_00 & 1)  # PID 0x20 continuation
        self.assertTrue(page_20 & 1)  # PID 0x40 continuation
        self.assertTrue(page_40 & (1 << (32 - (0x42 - 0x40))))  # PID 0x42

    def test_pid_zero_request_advertises_the_supported_pid_bitmap(self):
        request = CanFrame(0x7DF, False, False, 8, bytes((2, 1, 0, 0, 0, 0, 0, 0)))

        response = self.simulator.handle_request(request)

        self.assertEqual(
            response.data,
            bytes((6, 0x41, 0x00, 0x98, 0x1A, 0x80, 0x03, 0)),
        )

    def test_display_rows_include_live_values_for_supported_pids(self):
        rows = {row["pid"]: row for row in self.simulator.display_rows()}

        self.assertIn("Supported PID bitmap", rows["00"]["name"])
        self.assertIn("800 rpm", rows["0C"]["value"])
        self.assertEqual(rows["0C"]["supported"], "Yes")
        self.assertEqual(rows["0D"]["unit"], "km/h")

    def test_sweep_values_rise_then_fall(self):
        config = default_config()
        config["pids"]["0C"]["sweep"] = {
            "minimum": 800,
            "maximum": 1000,
            "step": 100,
            "interval_s": 1,
        }
        simulator = EcuSimulator(config, clock=lambda: self.now)
        at_minimum = simulator.current_value(0x0C)[0]
        self.now += 1
        rising = simulator.current_value(0x0C)[0]
        self.now += 1
        at_maximum = simulator.current_value(0x0C)[0]
        self.now += 1
        falling = simulator.current_value(0x0C)[0]

        self.assertEqual(
            (at_minimum, rising, at_maximum, falling),
            ("800 rpm", "900 rpm", "1000 rpm", "900 rpm"),
        )

    def test_random_algorithm_changes_only_when_that_pid_is_requested(self):
        config = default_config()
        config["pids"]["0C"]["algorithm"] = "random"
        config["pids"]["0C"]["sweep"] = {
            "minimum": 800,
            "maximum": 1200,
        }
        simulator = EcuSimulator(config, clock=lambda: self.now)
        rpm_request = CanFrame(
            0x7DF, False, False, 8, bytes((2, 1, 0x0C, 0, 0, 0, 0, 0))
        )
        speed_request = CanFrame(
            0x7DF, False, False, 8, bytes((2, 1, 0x0D, 0, 0, 0, 0, 0))
        )

        with patch("ecu_simulator.random.uniform", side_effect=(1000, 1200)) as choose:
            self.assertEqual(simulator.current_value(0x0C)[0], "800 rpm")
            simulator.display_rows()
            self.assertEqual(choose.call_count, 0)

            first_response = simulator.handle_request(rpm_request)
            simulator.handle_request(speed_request)
            self.assertEqual(simulator.current_value(0x0C)[0], "1000 rpm")
            self.assertEqual(choose.call_count, 1)

            second_response = simulator.handle_request(rpm_request)
            self.assertEqual(choose.call_count, 2)

        self.assertEqual(first_response.data, bytes((4, 0x41, 0x0C, 0x0F, 0xA0, 0, 0, 0)))
        self.assertEqual(second_response.data, bytes((4, 0x41, 0x0C, 0x12, 0xC0, 0, 0, 0)))

    def test_config_round_trip_and_invalid_ids(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ecu.json"
            save_config(path, default_config())
            loaded = load_config(path)
            self.assertEqual(loaded["request_ids"], [0x7DF, 0x7E0])
            self.assertEqual(loaded["response_id"], 0x7E8)
            self.assertEqual(len(loaded["dtcs"]), 3)

            raw = json.loads(path.read_text(encoding="utf-8"))
            raw["response_id"] = "0x800"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(SimulatorConfigError, "11-bit"):
                load_config(path)

    def test_mode01_pid_catalog_has_descriptions_ranges_and_defaults(self):
        trim = PID_CATALOG[0x06]
        self.assertEqual(trim["description"], "Short-term fuel trim for bank 1.")
        self.assertEqual((trim["minimum"], trim["maximum"]), (-100, 99.2))
        self.assertEqual(trim["default_raw_bytes"], "80")
        self.assertEqual(PID_CATALOG[0x02]["range_description"], "Two-byte DTC encoding")

    def test_standard_mode01_pid_24_is_catalogued_and_uses_four_raw_bytes(self):
        catalog_entry = PID_CATALOG[0x24]
        self.assertIn("oxygen sensor 1", catalog_entry["name"].lower())
        self.assertEqual(catalog_entry["default_raw_bytes"], "00 00 00 00")
        self.assertIn("mass air flow", PID_CATALOG[0x50]["name"].lower())

        config = default_config()
        config["pids"]["24"] = {
            "name": catalog_entry["name"],
            "unit": catalog_entry["unit"],
            "supported": True,
            "algorithm": "raw",
            "raw_bytes": catalog_entry["default_raw_bytes"],
        }
        request = CanFrame(0x7DF, False, False, 8, bytes((2, 1, 0x24, 0, 0, 0, 0, 0)))

        response = EcuSimulator(config).handle_request(request)

        self.assertEqual(response.data, bytes((6, 0x41, 0x24, 0, 0, 0, 0, 0)))

    def test_pid24_sweep_encodes_lambda_and_voltage_with_separate_ranges(self):
        config = default_config()
        config["pids"]["24"] = {
            "name": PID_CATALOG[0x24]["name"],
            "unit": "",
            "supported": True,
            "algorithm": "sweep",
            "sweep": {
                "lambda_minimum": 0,
                "lambda_maximum": 2,
                "lambda_step": 1,
                "voltage_minimum": 0,
                "voltage_maximum": 8,
                "voltage_step": 4,
                "interval_s": 1,
            },
        }
        simulator = EcuSimulator(config, clock=lambda: self.now)
        request = CanFrame(
            0x7DF, False, False, 8, bytes((2, 1, 0x24, 0, 0, 0, 0, 0))
        )

        initial = simulator.handle_request(request)
        self.now += 1
        middle = simulator.handle_request(request)
        self.now += 1
        maximum = simulator.handle_request(request)

        self.assertEqual(initial.data, bytes((6, 0x41, 0x24, 0, 0, 0, 0, 0)))
        self.assertEqual(middle.data, bytes((6, 0x41, 0x24, 0x80, 0, 0x80, 0, 0)))
        self.assertEqual(maximum.data, bytes((6, 0x41, 0x24, 0xFF, 0xFF, 0xFF, 0xFF, 0)))

    def test_pid24_randomizes_lambda_and_voltage_only_when_requested(self):
        config = default_config()
        config["pids"]["24"] = {
            "name": PID_CATALOG[0x24]["name"],
            "unit": "",
            "supported": True,
            "algorithm": "random",
            "sweep": {
                "lambda_minimum": 0.5,
                "lambda_maximum": 1.5,
                "voltage_minimum": 1,
                "voltage_maximum": 5,
            },
        }
        simulator = EcuSimulator(config, clock=lambda: self.now)
        request = CanFrame(
            0x7DF, False, False, 8, bytes((2, 1, 0x24, 0, 0, 0, 0, 0))
        )

        with patch(
            "ecu_simulator.random.uniform", side_effect=(1.0, 4.0, 1.5, 5.0)
        ) as choose:
            simulator.current_value(0x24)
            simulator.display_rows()
            self.assertEqual(choose.call_count, 0)
            first = simulator.handle_request(request)
            second = simulator.handle_request(request)

        self.assertEqual(first.data, bytes((6, 0x41, 0x24, 0x80, 0, 0x80, 0, 0)))
        self.assertEqual(second.data, bytes((6, 0x41, 0x24, 0xC0, 0, 0xA0, 0, 0)))
        self.assertEqual(choose.call_count, 4)

    def test_pid24_validates_independent_boundaries_and_round_trips_config(self):
        config = default_config()
        config["pids"]["24"] = {
            "name": PID_CATALOG[0x24]["name"],
            "unit": "",
            "supported": True,
            "algorithm": "sweep",
            "sweep": {
                "lambda_minimum": 0.2,
                "lambda_maximum": 1.8,
                "lambda_step": 0.2,
                "voltage_minimum": 0.5,
                "voltage_maximum": 7.5,
                "voltage_step": 0.5,
                "interval_s": 0.25,
            },
        }

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ecu.json"
            save_config(path, config)
            loaded = load_config(path)

        self.assertEqual(loaded["pids"]["24"]["algorithm"], "sweep")
        self.assertEqual(
            loaded["pids"]["24"]["sweep"]["voltage_maximum"], 7.5
        )

        config["pids"]["24"]["sweep"]["lambda_maximum"] = 2.1
        with self.assertRaisesRegex(SimulatorConfigError, "lambda boundaries"):
            EcuSimulator(config)

    def test_multi_frame_standard_pid_is_catalogued_but_not_enabled_by_default(self):
        self.assertEqual(PID_CATALOG[0x6E]["default_supported"], False)
        self.assertIn("multi-frame", PID_CATALOG[0x6E]["range_description"])

    def test_custom_pid_catalog_metadata_survives_config_round_trip(self):
        config = default_config()
        catalog_entry = PID_CATALOG[0x06]
        config["pids"]["06"] = {
            "name": catalog_entry["name"],
            "unit": catalog_entry["unit"],
            "supported": True,
            "algorithm": "raw",
            "raw_bytes": catalog_entry["default_raw_bytes"],
            "service": "01",
            "description": catalog_entry["description"],
            "measurement": catalog_entry["measurement"],
            "minimum": catalog_entry["minimum"],
            "maximum": catalog_entry["maximum"],
            "default_raw_bytes": catalog_entry["default_raw_bytes"],
        }

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ecu.json"
            save_config(path, config)
            loaded = load_config(path)

        stored = loaded["pids"]["06"]
        self.assertEqual(stored["service"], "01")
        self.assertEqual(stored["description"], catalog_entry["description"])
        self.assertEqual(stored["minimum"], -100)
        self.assertEqual(stored["maximum"], 99.2)
        self.assertEqual(stored["default_raw_bytes"], "80")

    def test_catalog_numeric_pid_accepts_sweep_and_random_algorithms(self):
        for algorithm in ("sweep", "random"):
            config = default_config()
            catalog_entry = PID_CATALOG[0x06]
            config["pids"]["06"] = {
                "name": catalog_entry["name"],
                "unit": catalog_entry["unit"],
                "supported": True,
                "algorithm": algorithm,
                "minimum": catalog_entry["minimum"],
                "maximum": catalog_entry["maximum"],
                "sweep": {
                    "minimum": -50,
                    "maximum": 50,
                    **({"step": 10, "interval_s": 1} if algorithm == "sweep" else {}),
                },
            }

            simulator = EcuSimulator(config)

            self.assertEqual(simulator.current_value(0x06)[1], b"\x40")

    def test_numeric_catalog_random_values_use_each_pid_conversion(self):
        test_cases = (
            (0x06, -100, 99.2, 0, b"\x80"),
            (0x22, 0, 0.079 * 0xFFFF, 79, b"\x03\xE8"),
            (0x54, -32768, 32767, -1, b"\xFF\xFF"),
            (0xA6, 0, 0xFFFFFFFF / 10, 1.2, b"\x00\x00\x00\x0C"),
        )

        for pid, minimum, maximum, random_value, expected_bytes in test_cases:
            with self.subTest(pid=pid):
                catalog_entry = PID_CATALOG[pid]
                config = default_config()
                config["pids"][f"{pid:02X}"] = {
                    "name": catalog_entry["name"],
                    "unit": catalog_entry["unit"],
                    "supported": True,
                    "algorithm": "random",
                    "minimum": minimum,
                    "maximum": maximum,
                    "sweep": {"minimum": minimum, "maximum": maximum},
                }
                request = CanFrame(
                    0x7DF, False, False, 8, bytes((2, 1, pid, 0, 0, 0, 0, 0))
                )
                simulator = EcuSimulator(config)

                with patch(
                    "ecu_simulator.random.uniform", return_value=random_value
                ):
                    response = simulator.handle_request(request)

                self.assertEqual(response.data[3 : 3 + len(expected_bytes)], expected_bytes)

    def test_custom_pid_catalog_metadata_rejects_invalid_range(self):
        config = default_config()
        config["pids"]["06"] = {
            "name": "Fuel trim",
            "unit": "%",
            "supported": True,
            "algorithm": "raw",
            "raw_bytes": "80",
            "minimum": 100,
            "maximum": -100,
        }

        with self.assertRaisesRegex(SimulatorConfigError, "minimum"):
            EcuSimulator(config)

    def test_pid_support_changes_in_bitmap(self):
        config = copy.deepcopy(default_config())
        config["pids"]["0C"]["supported"] = False
        simulator = EcuSimulator(config, clock=lambda: self.now)
        bitmap = int.from_bytes(simulator.supported_bitmap(0), "big")
        self.assertFalse(bitmap & (1 << (32 - 0x0C)))

    def test_custom_raw_pid_is_served_and_added_to_supported_bitmap(self):
        config = default_config()
        config["pids"]["02"] = {
            "name": "Custom pressure",
            "unit": "kPa",
            "supported": True,
            "algorithm": "raw",
            "raw_bytes": "12 34",
        }
        simulator = EcuSimulator(config, clock=lambda: self.now)
        request = CanFrame(0x7DF, False, False, 8, bytes((2, 1, 2, 0, 0, 0, 0, 0)))

        response = simulator.handle_request(request)

        self.assertEqual(response.data, bytes((4, 0x41, 2, 0x12, 0x34, 0, 0, 0)))
        bitmap = int.from_bytes(simulator.supported_bitmap(0), "big")
        self.assertTrue(bitmap & (1 << (32 - 2)))
        self.assertEqual(
            next(row for row in simulator.display_rows() if row["pid"] == "02")["name"],
            "Custom pressure",
        )

    def test_custom_pid_on_later_bitmap_page_is_advertised(self):
        config = default_config()
        config["pids"]["61"] = {
            "name": "Extended value",
            "unit": "",
            "supported": True,
            "algorithm": "raw",
            "raw_bytes": "A5",
        }
        simulator = EcuSimulator(config, clock=lambda: self.now)
        page_40 = int.from_bytes(simulator.supported_bitmap(0x40), "big")
        page_60 = int.from_bytes(simulator.supported_bitmap(0x60), "big")

        self.assertTrue(page_40 & 1)
        self.assertTrue(page_60 & (1 << (32 - 1)))

    def test_mil_and_active_dtcs_are_reported_by_mode_01_and_mode_03(self):
        config = default_config()
        config["mil_on"] = True
        config["dtcs"][0]["active"] = True
        config["dtcs"][2]["active"] = True
        simulator = EcuSimulator(config, clock=lambda: self.now)
        status_request = CanFrame(
            0x7DF, False, False, 8, bytes((2, 1, 1, 0, 0, 0, 0, 0))
        )
        dtc_request = CanFrame(
            0x7DF, False, False, 8, bytes((1, 3, 0, 0, 0, 0, 0, 0))
        )

        status = simulator.handle_request(status_request)
        dtcs = simulator.handle_request(dtc_request)

        self.assertEqual(status.data, bytes((6, 0x41, 1, 0x82, 0, 0, 0, 0)))
        self.assertEqual(
            dtcs.data, bytes((5, 0x43, 0x01, 0x33, 0x04, 0x20, 0, 0))
        )

    def test_mode04_clears_reported_dtcs_until_the_next_simulation_session(self):
        config = default_config()
        config["mil_on"] = True
        config["dtcs"][0]["active"] = True
        config["dtcs"][2]["active"] = True
        simulator = EcuSimulator(config, clock=lambda: self.now)
        clear_request = CanFrame(
            0x7DF, False, False, 8, bytes((1, 4, 0, 0, 0, 0, 0, 0))
        )
        status_request = CanFrame(
            0x7DF, False, False, 8, bytes((2, 1, 1, 0, 0, 0, 0, 0))
        )
        dtc_request = CanFrame(
            0x7DF, False, False, 8, bytes((1, 3, 0, 0, 0, 0, 0, 0))
        )

        clear_response = simulator.handle_request(clear_request)
        status = simulator.handle_request(status_request)
        dtcs = simulator.handle_request(dtc_request)

        self.assertEqual(clear_response.data, bytes((1, 0x44, 0, 0, 0, 0, 0, 0)))
        self.assertEqual(status.data, bytes((6, 0x41, 1, 0, 0, 0, 0, 0)))
        self.assertEqual(dtcs.data, bytes((1, 0x43, 0, 0, 0, 0, 0, 0)))

        simulator.start_session()

        self.assertEqual(
            simulator.handle_request(status_request).data,
            bytes((6, 0x41, 1, 0x82, 0, 0, 0, 0)),
        )
        self.assertEqual(
            simulator.handle_request(dtc_request).data,
            bytes((5, 0x43, 0x01, 0x33, 0x04, 0x20, 0, 0)),
        )

    def test_raw_algorithm_overrides_builtin_pid_bytes(self):
        config = default_config()
        config["pids"]["0D"]["algorithm"] = "raw"
        config["pids"]["0D"]["raw_bytes"] = "FE"
        simulator = EcuSimulator(config, clock=lambda: self.now)

        self.assertEqual(simulator.current_value(0x0D)[1], b"\xFE")

    def test_lowercase_pid_keys_are_normalized(self):
        config = default_config()
        config["pids"]["0c"] = config["pids"].pop("0C")

        simulator = EcuSimulator(config, clock=lambda: self.now)

        self.assertTrue(simulator.is_supported(0x0C))
        self.assertIn("0C", simulator.config["pids"])

    def test_rejects_too_many_active_dtcs_for_single_frame_mode_03(self):
        config = default_config()
        for dtc in config["dtcs"]:
            dtc["active"] = True
        config["dtcs"].append(
            {"code": "P0001", "description": "extra", "active": True}
        )

        with self.assertRaisesRegex(SimulatorConfigError, "At most three DTCs"):
            EcuSimulator(config)


if __name__ == "__main__":
    unittest.main()
