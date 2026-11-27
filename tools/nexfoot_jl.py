# SPDX-License-Identifier: Apache-2.0
# JL UBOOT command encoding from the MD400 host transport.
import struct

def jl_crc16(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_cdb(cmd: int, args: bytes = b"") -> bytes:
    if len(args) > 14:
        raise ValueError("JL CDB args too long")
    return struct.pack(">H", cmd) + args + bytes([0xFF]) * (14 - len(args))


def addr_len_args(addr: int, length: int) -> bytes:
    return struct.pack(">IH", addr, length)


def write_args(addr: int, data: bytes) -> bytes:
    return addr_len_args(addr, len(data)) + b"\x00" + struct.pack("<H", jl_crc16(data, 0))


class Jl:
    def __init__(self, dev):
        self.dev = dev

    def inquiry(self) -> tuple[str, str, str]:
        data = self.dev.execute(bytes([0x12, 0, 0, 0, 0x24, 0]), data_in_len=36)
        text = lambda b: b.decode("ascii", errors="replace").strip()
        return text(data[8:16]), text(data[16:32]), text(data[32:36])

    def exec_raw(self, cmd: int, args: bytes = b"") -> tuple[int, bytes]:
        resp = self.dev.execute(build_cdb(cmd, args), data_in_len=16)
        return struct.unpack(">H", resp[:2])[0], resp[2:]

    def exec(self, cmd: int, args: bytes = b"") -> bytes:
        echoed, resp = self.exec_raw(cmd, args)
        if echoed != cmd:
            raise RuntimeError(f"command echo mismatch: sent 0x{cmd:04X}, got 0x{echoed:04X}")
        return resp

    def exec_in(self, cmd: int, args: bytes, length: int) -> bytes:
        return self.dev.execute(build_cdb(cmd, args), data_in_len=length)

    def exec_out(self, cmd: int, args: bytes, data: bytes) -> None:
        self.dev.execute(build_cdb(cmd, args), data_out=data)

    def uboot_mem_write(self, addr: int, data: bytes) -> None:
        self.exec_out(0xFB06, write_args(addr, data), data)

    def uboot_mem_jump(self, addr: int, arg: int) -> None:
        self.exec_raw(0xFB08, struct.pack(">IH", addr, arg))

    def loader_online_device(self) -> tuple[int, int]:
        r = self.exec(0xFC0A)
        return r[0], struct.unpack("<I", r[2:6])[0]

    def loader_flash_erase_block(self, addr: int) -> int:
        return self.exec(0xFB00, struct.pack(">I", addr))[0]

    def loader_flash_erase_sector(self, addr: int) -> None:
        self.exec(0xFB01, struct.pack(">I", addr))

    def loader_flash_write(self, addr: int, data: bytes) -> None:
        self.exec_out(0xFB04, write_args(addr, data), data)

    def loader_flash_read(self, addr: int, length: int) -> bytes:
        return self.exec_in(0xFD05, addr_len_args(addr, length), length)

    def loader_usb_buffer_size(self) -> int:
        r = self.exec(0xFC14)
        return struct.unpack(">I", r[:4])[0]

    def loader_run_app(self, arg: int = 1) -> None:
        self.exec(0xFC0C, struct.pack(">I", arg))
