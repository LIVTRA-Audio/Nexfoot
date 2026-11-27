# SPDX-License-Identifier: Apache-2.0
"""JL transport selection with strict device identity and one-device selection."""
from __future__ import annotations

import os
import struct
import errno
import sys
import subprocess
import time
from contextlib import contextmanager
from pathlib import Path

from nexfoot_jl import Jl
from nexfoot_package import digest
from nexfoot_paths import SDK

UBOOT_VID, UBOOT_PID = 0x4C4A, 0x3442
RUNTIME_VID, RUNTIME_PID = 0x4C4A, 0x4156
_expected_topology = None


class UsbBotDevice:
    def __init__(self, device):
        import usb.util
        self.device = device
        self.tag = 0
        cfg = device.get_active_configuration()
        interfaces = [i for i in cfg if i.bInterfaceClass == 8 and i.bInterfaceProtocol == 0x50]
        if len(interfaces) != 1:
            raise RuntimeError("expected one USB Bulk-Only Transport interface")
        interface = interfaces[0]
        self.interface = interface.bInterfaceNumber
        endpoints = [e for e in interface if usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK]
        ins = [e for e in endpoints if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN]
        outs = [e for e in endpoints if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT]
        if len(ins) != 1 or len(outs) != 1:
            raise RuntimeError("expected one bulk endpoint in each direction")
        self.ep_in, self.ep_out = ins[0].bEndpointAddress, outs[0].bEndpointAddress
        self.detached = False
        try:
            if device.is_kernel_driver_active(self.interface):
                device.detach_kernel_driver(self.interface)
                self.detached = True
        except NotImplementedError:
            pass
        except usb.core.USBError as error:
            # A macOS UBOOT interface released by IOUSBHost has no driver.
            if sys.platform != "darwin" or error.errno != errno.ENOENT:
                raise
        try:
            usb.util.claim_interface(device, self.interface)
        except Exception:
            self.close()
            raise

    def close(self):
        import usb.util
        try:
            usb.util.release_interface(self.device, self.interface)
        finally:
            if self.detached:
                try:
                    self.device.attach_kernel_driver(self.interface)
                except Exception:
                    pass
            usb.util.dispose_resources(self.device)

    def execute(self, cdb: bytes, data_out=None, data_in_len=0, timeout=8):
        if not 1 <= len(cdb) <= 16 or (data_out is not None and data_in_len):
            raise ValueError("invalid BOT transaction")
        self.tag = (self.tag + 1) & 0xFFFFFFFF
        length = data_in_len if data_out is None else len(data_out)
        cbw = struct.pack("<IIIBBB16s", 0x43425355, self.tag, length,
                          0x80 if data_in_len else 0, 0, len(cdb), cdb.ljust(16, b"\0"))
        ms = int(timeout * 1000)
        if self.device.write(self.ep_out, cbw, timeout=ms) != len(cbw):
            raise OSError("short BOT command write")
        result = b""
        if data_out is not None and length:
            if self.device.write(self.ep_out, data_out, timeout=ms) != length:
                raise OSError("short BOT data write")
        elif data_in_len:
            result = bytes(self.device.read(self.ep_in, data_in_len, timeout=ms))
            if len(result) != data_in_len:
                raise OSError("short BOT data read")
        csw = bytes(self.device.read(self.ep_in, 13, timeout=ms))
        if len(csw) != 13:
            raise OSError("short BOT status")
        signature, tag, residue, status = struct.unpack("<IIIB", csw)
        if signature != 0x53425355 or tag != self.tag or residue or status:
            raise OSError("BOT transaction status failed")
        return result


def usb_devices(vid, pid):
    import usb.core
    return list(usb.core.find(find_all=True, idVendor=vid, idProduct=pid))


def open_uboot():
    global _expected_topology
    if os.name == "nt":
        from nexfoot_scsi_windows import WinScsiDevice
        found = []
        # Scan only physical disks to avoid counting a volume and disk twice.
        for index in range(16):
            device = None
            try:
                device = WinScsiDevice(r"\\.\PhysicalDrive" + str(index))
                vendor, product, _ = Jl(device).inquiry()
                if vendor == "BD19" and product == "UBOOT1.00":
                    found.append(device)
                    device = None
            except OSError:
                pass
            finally:
                if device is not None:
                    device.close()
        if len(found) != 1:
            for device in found:
                device.close()
            raise RuntimeError(f"expected exactly one BD19 UBOOT device, found {len(found)}")
        return found[0]
    devices = usb_devices(UBOOT_VID, UBOOT_PID)
    if len(devices) != 1:
        raise RuntimeError(f"expected exactly one Nexfoot UBOOT USB device, found {len(devices)}; enter physical recovery")
    device = devices[0]
    topology = (device.bus, tuple(device.port_numbers or ()))
    if not topology[1]:
        raise RuntimeError("USB port topology is unavailable")
    if _expected_topology is not None and _expected_topology != topology:
        raise RuntimeError("UBOOT moved to another USB port")
    _expected_topology = topology
    return UsbBotDevice(device)


@contextmanager
def open_loader(root: Path):
    import json
    lock = json.loads((root / "dependencies.lock.json").read_text())
    name = "cpu/bd19/tools/bd19loader.bin"
    path = SDK / name
    loader = path.read_bytes()
    if digest(loader) != lock["sdkFiles"][name]:
        raise RuntimeError("RAM loader hash does not match pinned SDK")
    device = open_uboot()
    jl = Jl(device)
    try:
        vendor, product, _ = jl.inquiry()
        if vendor != "BD19" or product != "UBOOT1.00":
            raise RuntimeError("unsupported UBOOT inquiry identity")
        online = False
        try:
            online = jl.loader_online_device() == (0x03, 0xCD7113)
        except (RuntimeError, OSError):
            pass
        if not online:
            for offset in range(0, len(loader), 512):
                jl.uboot_mem_write(0x2000 + offset, loader[offset:offset + 512])
            try:
                jl.uboot_mem_jump(0x2000, 0x0081)
            except OSError:
                pass
            deadline = time.monotonic() + 5
            while True:
                try:
                    if jl.loader_online_device() == (0x03, 0xCD7113):
                        break
                except (RuntimeError, OSError):
                    pass
                if time.monotonic() >= deadline:
                    raise RuntimeError("RAM loader did not report the approved device/flash identity")
                time.sleep(0.1)
        yield jl
    finally:
        device.close()


def wait_runtime(timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if os.name == "nt":
            code = "@(Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -like 'USB\\VID_4C4A&PID_4156\\*' }).Count"
            result = subprocess.run(["powershell", "-NoProfile", "-Command", code],
                                    capture_output=True, text=True, timeout=5, check=True)
            if result.stdout.strip() == "1":
                return
        else:
            devices = usb_devices(RUNTIME_VID, RUNTIME_PID)
            if len(devices) == 1:
                device = devices[0]
                if (device.bus, tuple(device.port_numbers or ())) == _expected_topology:
                    return
        time.sleep(0.25)
    raise RuntimeError("programming verified, but runtime USB did not return; enter recovery and inspect the build")
