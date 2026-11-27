# SPDX-License-Identifier: Apache-2.0
# Adapted from the Nexfoot MD400 host transport. No flash policy lives here.
import ctypes
from ctypes import wintypes

GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
FILE_SHARE_READ = 0x00000001
FILE_SHARE_WRITE = 0x00000002
OPEN_EXISTING = 3
INVALID_HANDLE_VALUE = wintypes.HANDLE(-1).value
IOCTL_SCSI_PASS_THROUGH_DIRECT = 0x0004D014

SCSI_DATA_OUT = 0
SCSI_DATA_IN = 1
SCSI_DATA_UNSPEC = 2

LOADER_ARG = 0x0081
LOADER_BLOCK = 512
FLASH_SECTOR = 0x1000
FLASH_BLOCK = 0x10000
FLASH_IO_FALLBACK = 512
FLASH_IO_CAP = 32 * 1024
DEFAULT_FLASH_LIMIT = 512 * 1024
RAW_PRODUCT_OFFSET = 0x10
RAW_PRODUCT = b"AC632N"
ISDU_FOOTER_LEN = 16
ISDU_FOOTER_MARKER = b"ISDU"


class ScsiPassThroughDirect(ctypes.Structure):
    _fields_ = [
        ("Length", wintypes.USHORT),
        ("ScsiStatus", wintypes.BYTE),
        ("PathId", wintypes.BYTE),
        ("TargetId", wintypes.BYTE),
        ("Lun", wintypes.BYTE),
        ("CdbLength", wintypes.BYTE),
        ("SenseInfoLength", wintypes.BYTE),
        ("DataIn", wintypes.BYTE),
        ("DataTransferLength", wintypes.ULONG),
        ("TimeOutValue", wintypes.ULONG),
        ("DataBuffer", ctypes.c_void_p),
        ("SenseInfoOffset", wintypes.ULONG),
        ("Cdb", wintypes.BYTE * 16),
    ]


kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
kernel32.CreateFileW.argtypes = [
    wintypes.LPCWSTR,
    wintypes.DWORD,
    wintypes.DWORD,
    wintypes.LPVOID,
    wintypes.DWORD,
    wintypes.DWORD,
    wintypes.HANDLE,
]
kernel32.CreateFileW.restype = wintypes.HANDLE
kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
kernel32.CloseHandle.restype = wintypes.BOOL
kernel32.DeviceIoControl.argtypes = [
    wintypes.HANDLE,
    wintypes.DWORD,
    wintypes.LPVOID,
    wintypes.DWORD,
    wintypes.LPVOID,
    wintypes.DWORD,
    ctypes.POINTER(wintypes.DWORD),
    wintypes.LPVOID,
]
kernel32.DeviceIoControl.restype = wintypes.BOOL
kernel32.GetLogicalDrives.argtypes = []
kernel32.GetLogicalDrives.restype = wintypes.DWORD


def win_error(prefix: str) -> OSError:
    err = ctypes.get_last_error()
    return OSError(err, f"{prefix}: {ctypes.FormatError(err)}")


class WinScsiDevice:
    def __init__(self, path: str):
        self.path = path
        handle = kernel32.CreateFileW(
            path,
            GENERIC_READ | GENERIC_WRITE,
            FILE_SHARE_READ | FILE_SHARE_WRITE,
            None,
            OPEN_EXISTING,
            0,
            None,
        )
        if handle == INVALID_HANDLE_VALUE:
            raise win_error(f"open {path}")
        self.handle = handle

    def close(self) -> None:
        handle = getattr(self, "handle", None)
        if handle and handle != INVALID_HANDLE_VALUE:
            kernel32.CloseHandle(handle)
            self.handle = None

    def __enter__(self) -> "WinScsiDevice":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # type: ignore[no-untyped-def]
        self.close()

    def execute(
        self,
        cdb: bytes,
        data_out: bytes | None = None,
        data_in_len: int = 0,
        timeout: int = 8,
    ) -> bytes:
        if len(cdb) > 16:
            raise ValueError("CDB too long")

        sptd = ScsiPassThroughDirect()
        sptd.Length = ctypes.sizeof(ScsiPassThroughDirect)
        sptd.TargetId = 1
        sptd.CdbLength = len(cdb)
        sptd.DataIn = SCSI_DATA_UNSPEC
        sptd.TimeOutValue = timeout
        for i, b in enumerate(cdb):
            sptd.Cdb[i] = b

        buf = None
        if data_out is not None:
            buf = ctypes.create_string_buffer(data_out, len(data_out))
            sptd.DataIn = SCSI_DATA_OUT
            sptd.DataTransferLength = len(data_out)
            sptd.DataBuffer = ctypes.cast(buf, ctypes.c_void_p).value
        elif data_in_len:
            buf = ctypes.create_string_buffer(data_in_len)
            sptd.DataIn = SCSI_DATA_IN
            sptd.DataTransferLength = data_in_len
            sptd.DataBuffer = ctypes.cast(buf, ctypes.c_void_p).value

        returned = wintypes.DWORD(0)
        size = ctypes.sizeof(ScsiPassThroughDirect)
        ok = kernel32.DeviceIoControl(
            self.handle,
            IOCTL_SCSI_PASS_THROUGH_DIRECT,
            ctypes.byref(sptd),
            size,
            ctypes.byref(sptd),
            size,
            ctypes.byref(returned),
            None,
        )
        if not ok:
            raise win_error(f"SCSI pass-through {cdb[:2].hex().upper()} on {self.path}")

        if sptd.ScsiStatus != 0:
            raise OSError(f"SCSI status 0x{sptd.ScsiStatus:02x}")
        if data_in_len and buf is not None:
            if sptd.DataTransferLength != data_in_len:
                raise OSError("Short SCSI transfer")
            return bytes(buf.raw[:data_in_len])
        return b""
