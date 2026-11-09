import unittest
import ctypes
import os
from unittest.mock import patch

import bluetooth_windows
from bluetooth_windows import BluetoothDevice, _BluetoothDeviceInfo, _device_from_info


class FakeFunction:
    def __init__(self, callback):
        self.callback = callback

    def __call__(self, *args):
        return self.callback(*args)


class FakeBluetoothLibrary:
    def __init__(self):
        self.find_next_calls = 0
        self.BluetoothFindFirstDevice = FakeFunction(self.find_first)
        self.BluetoothFindNextDevice = FakeFunction(self.find_next)
        self.BluetoothFindDeviceClose = FakeFunction(lambda _handle: True)

    def _fill_info(self, argument, name, address):
        target = ctypes.cast(
            argument, ctypes.POINTER(_BluetoothDeviceInfo)
        ).contents
        target.dwSize = ctypes.sizeof(_BluetoothDeviceInfo)
        target.Address.ullLong = address
        target.szName = name
        target.fRemembered = True

    def find_first(self, search, info):
        params = ctypes.cast(
            search, ctypes.POINTER(bluetooth_windows._BluetoothDeviceSearchParams)
        ).contents
        self.assert_inquiry = bool(params.fIssueInquiry)
        self._fill_info(info, "CAN adapter", 0xA1B2C3D4E5F6)
        return 1

    def find_next(self, _handle, info):
        self.find_next_calls += 1
        if self.find_next_calls == 1:
            self._fill_info(info, "Other adapter", 0x001122334455)
            return True
        return False


class BluetoothWindowsTests(unittest.TestCase):
    def test_device_info_is_converted_to_selectable_device(self):
        info = _BluetoothDeviceInfo()
        info.Address.ullLong = 0xA1B2C3D4E5F6
        info.szName = "CAN adapter"
        info.fConnected = 1
        info.fAuthenticated = 1

        device = _device_from_info(info)

        self.assertEqual(
            device,
            BluetoothDevice("CAN adapter", "A1B2C3D4E5F6", True, True),
        )
        self.assertIn("connected", device.label)

    def test_unpaired_device_label_shows_available(self):
        device = BluetoothDevice("CAN adapter", "001122334455", False, False)
        self.assertEqual(device.label, "CAN adapter [001122334455] - available")

    @unittest.skipUnless(os.name == "nt", "Windows Bluetooth API test")
    def test_discovery_uses_windows_inquiry_and_returns_devices(self):
        library = FakeBluetoothLibrary()
        with patch.object(bluetooth_windows.ctypes, "WinDLL", return_value=library):
            devices = bluetooth_windows.discover_devices()

        self.assertTrue(library.assert_inquiry)
        self.assertEqual([device.name for device in devices], ["CAN adapter", "Other adapter"])
        self.assertTrue(all(device.paired for device in devices))

    @unittest.skipIf(os.name == "nt", "Non-Windows Bluetooth behavior test")
    def test_discovery_reports_unsupported_platform(self):
        with self.assertRaisesRegex(bluetooth_windows.BluetoothUnavailableError, "Windows only"):
            bluetooth_windows.discover_devices()


if __name__ == "__main__":
    unittest.main()
