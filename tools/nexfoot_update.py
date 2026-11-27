#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""A bounded updater. Recovery installation/migration is deliberately separate."""
from __future__ import annotations

import json
import time
from pathlib import Path

from nexfoot_package import BOOT_END, CODE_END, FLASH_SIZE, SECTOR, digest, validate_image
from nexfoot_paths import BACKUPS
from nexfoot_web_package import WEB_BOOT_END, WEB_SCHEMA, validate_web_image

LOADER_DEVICE_TYPE = 0x03
FLASH_ID = 0xCD7113
IO_SIZE = 512


def read_exact(jl, address: int, length: int) -> bytes:
    data = bytearray()
    for offset in range(0, length, IO_SIZE):
        size = min(IO_SIZE, length - offset)
        part = jl.loader_flash_read(address + offset, size)
        if len(part) != size:
            raise RuntimeError(f"short flash read at 0x{address + offset:x}")
        data.extend(part)
    return bytes(data)


def check_range(address: int, length: int, protected_end: int = BOOT_END) -> None:
    if length <= 0 or address < protected_end or address + length > CODE_END:
        raise RuntimeError("write/erase would touch recovery or reserved flash")


def prepare_plan(jl, image: bytes, protected_end: int = BOOT_END) -> tuple[bytes, bytes, list[int]]:
    if protected_end == WEB_BOOT_END:
        validate_web_image(image)
    elif protected_end == BOOT_END:
        validate_image(image)
    else:
        raise ValueError("Unknown Nexfoot recovery layout")
    device_type, flash_id = jl.loader_online_device()
    if device_type != LOADER_DEVICE_TYPE or flash_id != FLASH_ID:
        raise RuntimeError("unsupported loader/flash identity; no flash writes attempted")
    backup = read_exact(jl, 0, FLASH_SIZE)
    if backup[:protected_end] != image[:protected_end]:
        raise RuntimeError("installed recovery is incompatible; use the official recovery installation procedure")
    target = bytearray(image)
    # Retain ISDU at the destination slot, as in the product updater.
    footer = backup[len(image) - 16:len(image)]
    if footer.startswith(b"ISDU"):
        target[-16:] = footer
    else:
        # A changed image length can move the old footer. Preserve a single
        # sector-tail ISDU found in the code range; reject ambiguity.
        footers = [backup[end - 16:end] for end in range(protected_end + SECTOR, CODE_END + 1, SECTOR)
                   if backup[end - 16:end].startswith(b"ISDU")]
        if len(footers) > 1:
            raise RuntimeError("ambiguous ISDU footer; no flash writes attempted")
        if footers:
            target[-16:] = footers[0]
    changed = [at for at in range(len(image) - SECTOR, protected_end - 1, -SECTOR)
               if backup[at:at + SECTOR] != target[at:at + SECTOR]]
    return backup, bytes(target), changed


def program(jl, image: bytes, backup_path: Path, progress=print, *, protected_end: int = BOOT_END) -> dict:
    backup, target, changed = prepare_plan(jl, image, protected_end)
    backup_path.parent.mkdir(parents=True, exist_ok=True)
    with backup_path.open("xb") as stream:
        stream.write(backup)
        stream.flush()
        import os
        os.fsync(stream.fileno())
    # Verify the persisted backup before permitting the first erase.
    if backup_path.read_bytes() != backup:
        raise RuntimeError("backup readback failed")
    progress(f"Saved full-flash backup: {backup_path}")
    for index, at in enumerate(changed, 1):
        check_range(at, SECTOR, protected_end)
        jl.loader_flash_erase_sector(at)
        for offset in range(0, SECTOR, IO_SIZE):
            chunk = target[at + offset:at + offset + IO_SIZE]
            check_range(at + offset, len(chunk), protected_end)
            jl.loader_flash_write(at + offset, chunk)
        if read_exact(jl, at, SECTOR) != target[at:at + SECTOR]:
            raise RuntimeError(f"readback mismatch at 0x{at:x}; leave device in recovery and retry")
        progress(f"Verified sector {index}/{len(changed)} at 0x{at:x}")
    if read_exact(jl, 0, len(target)) != target:
        raise RuntimeError("final image readback mismatch; app was not started")
    if read_exact(jl, CODE_END, FLASH_SIZE - CODE_END) != backup[CODE_END:]:
        raise RuntimeError("reserved flash changed; app was not started")
    result = {"verified": True, "recoveryPreserved": True, "reservedFlashPreserved": True,
              "changedSectors": len(changed), "backupSha256": digest(backup),
              "programmedSha256": digest(target), "backup": str(backup_path)}
    return result


def update(package_path: Path, manifest: dict, image: bytes, root: Path, *, write: bool) -> dict:
    from nexfoot_transport import open_loader
    protected_end = WEB_BOOT_END if manifest.get("schema") == WEB_SCHEMA else BOOT_END
    with open_loader(root) as jl:
        if not write:
            _, _, changed = prepare_plan(jl, image, protected_end)
            return {"dryRun": True, "changedSectors": len(changed), "recoveryCompatible": True}
        backup_path = BACKUPS / f"nexfoot-{time.time_ns()}.bin"
        result = program(jl, image, backup_path, protected_end=protected_end)
        (backup_path.with_suffix(".json")).write_text(json.dumps(result, indent=2) + "\n")
        try:
            jl.loader_run_app(1)
        except OSError:
            # USB disconnection can complete the command. Runtime verification
            # below, rather than an ignored exception, decides success.
            pass
    from nexfoot_transport import wait_runtime
    wait_runtime(timeout=25)
    result["runtimeUsbReturned"] = True
    result["package"] = str(package_path)
    result["communityVersion"] = manifest["version"]
    print("Runtime USB returned. Use NanoBridge GET_INFO to inspect the running firmware.")
    return result
