"""Windows Bluetooth Classic device discovery for SPP serial adapters."""

import ctypes
import os
from ctypes import wintypes
from dataclasses import dataclass
from typing import List


@dataclass(frozen=True)
class BluetoothDevice:
    name: str
    address: str
    connected: bool
    paired: bool

    @property
    def label(self) -> str:
        state = "connected" if self.connected else ("paired" if self.paired else "available")
        return f"{self.name or 'Unknown device'} [{self.address}] - {state}"


class BluetoothUnavailableError(RuntimeError):
    pass


def _device_from_info(info: "_BluetoothDeviceInfo") -> BluetoothDevice:
    address = f"{info.Address.ullLong:012X}"[-12:]
    return BluetoothDevice(
        name=info.szName,
        address=address,
        connected=bool(info.fConnected),
        paired=bool(info.fAuthenticated or info.fRemembered),
    )


class _BluetoothAddress(ctypes.Union):
    _fields_ = (("ullLong", ctypes.c_ulonglong), ("rgBytes", ctypes.c_ubyte * 6))


class _SystemTime(ctypes.Structure):
    _fields_ = (
        ("wYear", wintypes.WORD),
        ("wMonth", wintypes.WORD),
        ("wDayOfWeek", wintypes.WORD),
        ("wDay", wintypes.WORD),
        ("wHour", wintypes.WORD),
        ("wMinute", wintypes.WORD),
        ("wSecond", wintypes.WORD),
        ("wMilliseconds", wintypes.WORD),
    )


class _BluetoothDeviceInfo(ctypes.Structure):
    _fields_ = (
        ("dwSize", wintypes.DWORD),
        ("Address", _BluetoothAddress),
        ("ulClassofDevice", wintypes.ULONG),
        ("fConnected", wintypes.BOOL),
        ("fRemembered", wintypes.BOOL),
        ("fAuthenticated", wintypes.BOOL),
        ("stLastSeen", _SystemTime),
        ("stLastUsed", _SystemTime),
        ("szName", wintypes.WCHAR * 248),
    )


class _BluetoothDeviceSearchParams(ctypes.Structure):
    _fields_ = (
        ("dwSize", wintypes.DWORD),
        ("fReturnAuthenticated", wintypes.BOOL),
        ("fReturnRemembered", wintypes.BOOL),
        ("fReturnUnknown", wintypes.BOOL),
        ("fReturnConnected", wintypes.BOOL),
        ("fIssueInquiry", wintypes.BOOL),
        ("cTimeoutMultiplier", wintypes.BYTE),
        ("hRadio", wintypes.HANDLE),
    )


def discover_devices() -> List[BluetoothDevice]:
    """Find nearby and previously known Bluetooth Classic devices."""
    if os.name != "nt":
        raise BluetoothUnavailableError(
            "Bluetooth SPP discovery is currently supported on Windows only."
        )

    try:
        bluetooth = ctypes.WinDLL("bthprops.cpl", use_last_error=True)
    except (AttributeError, OSError) as exc:
        raise BluetoothUnavailableError(
            f"Could not load the Windows Bluetooth API: {exc}"
        ) from exc

    try:
        bluetooth.BluetoothFindFirstDevice.argtypes = (
            ctypes.POINTER(_BluetoothDeviceSearchParams),
            ctypes.POINTER(_BluetoothDeviceInfo),
        )
        bluetooth.BluetoothFindFirstDevice.restype = wintypes.HANDLE
        bluetooth.BluetoothFindNextDevice.argtypes = (
            wintypes.HANDLE,
            ctypes.POINTER(_BluetoothDeviceInfo),
        )
        bluetooth.BluetoothFindNextDevice.restype = wintypes.BOOL
        bluetooth.BluetoothFindDeviceClose.argtypes = (wintypes.HANDLE,)
        bluetooth.BluetoothFindDeviceClose.restype = wintypes.BOOL
    except (AttributeError, OSError) as exc:
        raise BluetoothUnavailableError(
            f"The Windows Bluetooth discovery API is unavailable: {exc}"
        ) from exc

    search = _BluetoothDeviceSearchParams()
    search.dwSize = ctypes.sizeof(search)
    search.fReturnAuthenticated = True
    search.fReturnRemembered = True
    search.fReturnUnknown = True
    search.fReturnConnected = True
    search.fIssueInquiry = True
    search.cTimeoutMultiplier = 8
    info = _BluetoothDeviceInfo()
    info.dwSize = ctypes.sizeof(info)

    find_handle = bluetooth.BluetoothFindFirstDevice(
        ctypes.byref(search), ctypes.byref(info)
    )
    if not find_handle:
        error = ctypes.get_last_error()
        if error in (0, 259, 1168):
            return []
        raise BluetoothUnavailableError(
            f"Bluetooth discovery failed with Windows error {error}."
        )

    devices = []
    try:
        while True:
            devices.append(_device_from_info(info))
            info = _BluetoothDeviceInfo()
            info.dwSize = ctypes.sizeof(info)
            if not bluetooth.BluetoothFindNextDevice(find_handle, ctypes.byref(info)):
                break
    finally:
        bluetooth.BluetoothFindDeviceClose(find_handle)

    return sorted(devices, key=lambda device: (device.name.casefold(), device.address))
