# SPDX-License-Identifier: Apache-2.0
"""One-time Windows bench migration, through the existing native BD19 path."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from nexfoot_package import BOOT_END, CODE_END, FLASH_SIZE, PROTECTED_SHA256, SECTOR, digest
from nexfoot_transport import open_loader, wait_runtime
from nexfoot_update import read_exact
from nexfoot_web_package import WEB_BOOT_END, WEB_SCHEMA, validate_web_image


def pnp(mode):
    query = ("@(Get-PnpDevice -PresentOnly | Where-Object { "
             "$_.InstanceId -like 'USB\\VID_4C4A&PID_" + mode + "\\*' }).InstanceId")
    result = subprocess.run(["powershell", "-NoProfile", "-Command", query],
                            capture_output=True, text=True, timeout=8, check=True)
    return result.stdout.strip().splitlines()


def load_foundation(path, expected_digest):
    package = path.read_bytes()
    if digest(package) != expected_digest or len(package) > 0x90000:
        raise ValueError("Unexpected bench package or SHA-256")
    with zipfile.ZipFile(path) as archive:
        members = archive.infolist()
        if len(members) != 2 or {m.filename for m in members} != {"manifest.json", "firmware.bin"}:
            raise ValueError("Unexpected package members")
        if archive.getinfo("firmware.bin").file_size > CODE_END or archive.getinfo("manifest.json").file_size > 16384:
            raise ValueError("Invalid uncompressed package size")
        manifest = json.loads(archive.read("manifest.json"))
        image = archive.read("firmware.bin")
    validate_web_image(image)
    if (manifest["schema"] != WEB_SCHEMA or manifest["board"] != "md400-bd19-v1"
            or manifest["protectedBootBytes"] != WEB_BOOT_END
            or manifest["protectedBootSha256"] != digest(image[:WEB_BOOT_END])
            or manifest["firmwareSha256"] != digest(image)):
        raise ValueError("Unexpected WebHID foundation identity")
    return image


def capture():
    for attempt in range(3):
        if len(pnp("3442")) != 1 or pnp("4156"):
            raise RuntimeError("Expected a single Nexfoot BD19 UBOOT at 4C4A:3442")
        try:
            with open_loader(ROOT) as device:
                return read_exact(device, 0, FLASH_SIZE)
        except OSError:
            if attempt == 2:
                raise
            time.sleep(1)


def write_sector(device, address, data):
    if address % SECTOR or len(data) != SECTOR or address + SECTOR > CODE_END:
        raise ValueError("Sector is outside the Nexfoot update region")
    device.loader_flash_erase_sector(address)
    for offset in range(0, SECTOR, 512):
        block = data[offset:offset + 512]
        if block != b"\xff" * len(block):
            device.loader_flash_write(address + offset, block)
    if read_exact(device, address, SECTOR) != data:
        raise RuntimeError(f"Readback mismatch at 0x{address:05x}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", type=Path)
    parser.add_argument("sha256", help="SHA-256 of the reviewed WebHID foundation archive")
    parser.add_argument("--backup", required=True, type=Path)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    image = load_foundation(args.package, args.sha256)
    backup = capture()
    if digest(backup[:BOOT_END]) != PROTECTED_SHA256:
        raise RuntimeError("Unsupported installed recovery layout")
    with args.backup.open("xb") as stream:
        stream.write(backup)
    if args.backup.read_bytes() != backup:
        raise RuntimeError("Could not verify the backup")
    print(f"Saved and verified full bench backup at {args.backup}", flush=True)
    target = bytearray(image)
    footers = [backup[end - 16:end] for end in range(BOOT_END + SECTOR, CODE_END + 1, SECTOR)
               if backup[end - 16:end - 12] == b"ISDU"]
    if len(footers) > 1:
        raise RuntimeError("Ambiguous installed ISDU footer")
    if footers:
        target[-16:] = footers[0]
    if not args.write:
        print("Preflight complete. No flash writes.")
        return
    if len(pnp("3442")) != 1:
        raise RuntimeError("Nexfoot UBOOT disappeared before the write")
    with open_loader(ROOT) as device:
        if read_exact(device, 0, FLASH_SIZE) != backup:
            raise RuntimeError("The connected device changed since the backup")
        # Program the application from the end, then the new boot body. Write
        # the first flash sector containing the boot header last.
        for address in range(len(target) - SECTOR, -1, -SECTOR):
            block = bytes(target[address:address + SECTOR])
            if block != backup[address:address + SECTOR]:
                write_sector(device, address, block)
        if read_exact(device, 0, len(target)) != target:
            raise RuntimeError("Foundation image did not verify")
        if read_exact(device, CODE_END, FLASH_SIZE - CODE_END) != backup[CODE_END:]:
            raise RuntimeError("Data outside the application changed")
        print("Foundation programmed and verified. User flash unchanged.", flush=True)
        try:
            device.loader_run_app()
        except OSError:
            pass
    wait_runtime(25)
    print("Nexfoot returned to runtime USB.", flush=True)


if __name__ == "__main__":
    main()
