#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Community packages. Hashes provide integrity, not publisher authentication."""
from __future__ import annotations

import hashlib
import json
import re
import zipfile
from pathlib import Path

from nexfoot_paths import ROOT
SECTOR = 0x1000
BOOT_END = 0x2000
CODE_END = 0x48000
FLASH_SIZE = 0x80000
SCHEMA = "nexfoot-community-v1"
PROTECTED_SHA256 = "57c1988eccbaa35ac8c4219d4e43459bd06864aff5ae8998856abd562905bf1e"
RECOVERY_SHA256 = "a4a40e500436d19076eddb2d5b19abec057b37eb65fc7879b210afd828081744"
SDK_COMMIT = "fdc018de81629e21e594d677cf05f87d288be0f0"


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def validate_image(image: bytes) -> None:
    if not BOOT_END + SECTOR <= len(image) <= CODE_END or len(image) % SECTOR:
        raise ValueError("image must be sector aligned, larger than recovery, and end by 0x48000")
    if image[0x10:0x16] != b"AC632N":
        raise ValueError("expected an AC632N jl_isd.bin, not app.bin/UFw/full-flash backup")
    if digest(image[:BOOT_END]) != PROTECTED_SHA256:
        raise ValueError("image does not contain the approved recovery region; rebuild with Nexfoot")
    if image[-16:] != b"\xff" * 16:
        raise ValueError("image must have an erased ISDU footer placeholder")


def validate_manifest(manifest: dict, image: bytes) -> None:
    validate_image(image)
    expected = {
        "schema": SCHEMA,
        "product": "Nexfoot",
        "mcu": "AC632N",
        "board": "md400-bd19-v1",
        "flashBytes": FLASH_SIZE,
        "protectedBootBytes": BOOT_END,
        "codeEnd": CODE_END,
        "protectedBootSha256": PROTECTED_SHA256,
        "recoveryArtifactSha256": RECOVERY_SHA256,
        "sdkCommit": SDK_COMMIT,
        "firmwareBytes": len(image),
        "firmwareSha256": digest(image),
    }
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise ValueError(f"invalid package contract: {key}")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._+-]{0,63}", str(manifest.get("version", ""))):
        raise ValueError("invalid community version")
    if manifest.get("origin") != "community":
        raise ValueError("community package cannot identify itself as an official release")


def write_package(image: bytes, output: Path, version: str, build: dict) -> dict:
    validate_image(image)
    if digest((ROOT / "recovery/uboot_recovery.boot").read_bytes()) != RECOVERY_SHA256:
        raise ValueError("approved recovery artifact has changed")
    manifest = {
        "schema": SCHEMA, "origin": "community", "product": "Nexfoot",
        "mcu": "AC632N", "board": "md400-bd19-v1", "version": version,
        "flashBytes": FLASH_SIZE, "protectedBootBytes": BOOT_END,
        "codeEnd": CODE_END, "protectedBootSha256": PROTECTED_SHA256,
        "recoveryArtifactSha256": RECOVERY_SHA256, "sdkCommit": SDK_COMMIT,
        "firmwareBytes": len(image), "firmwareSha256": digest(image), "build": build,
    }
    validate_manifest(manifest, image)
    output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation avoids replacing a previously reviewed package.
    with zipfile.ZipFile(output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in [("manifest.json", (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode()),
                           ("firmware.bin", image)]:
            info = zipfile.ZipInfo(name, date_time=(2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            archive.writestr(info, data)
    return manifest


def load_package(path: Path) -> tuple[dict, bytes]:
    if path.stat().st_size > FLASH_SIZE + 65536:
        raise ValueError("package is too large")
    with zipfile.ZipFile(path) as archive:
        entries = archive.infolist()
        if len(entries) != 2 or {entry.filename for entry in entries} != {"manifest.json", "firmware.bin"}:
            raise ValueError("package must contain exactly manifest.json and firmware.bin")
        for entry in entries:
            limit = 16384 if entry.filename == "manifest.json" else CODE_END
            if entry.file_size > limit or entry.flag_bits & 1:
                raise ValueError("oversized or encrypted package entry")
        manifest = json.loads(archive.read("manifest.json"))
        image = archive.read("firmware.bin")
    if not isinstance(manifest, dict):
        raise ValueError("manifest must be an object")
    from nexfoot_web_package import WEB_SCHEMA, validate_web_manifest
    if manifest.get("schema") == WEB_SCHEMA:
        validate_web_manifest(manifest, image)
    else:
        validate_manifest(manifest, image)
    return manifest, image
