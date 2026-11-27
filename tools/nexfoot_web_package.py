# SPDX-License-Identifier: Apache-2.0
"""Build and validate the Nexfoot WebHID prototype flash layout."""
import json
from pathlib import Path
import re
import zipfile

from nexfoot_package import BOOT_END, CODE_END, FLASH_SIZE, SECTOR, digest, validate_image

WEB_BOOT_END = BOOT_END + SECTOR
WEB_SCHEMA = "nexfoot-community-web-v1"


def validate_web_image(image):
    if not WEB_BOOT_END + SECTOR <= len(image) <= CODE_END or len(image) % SECTOR:
        raise ValueError("WebHID image exceeds the application region")
    if image[0x10:0x16] != b"AC632N" or image[-16:] != b"\xff" * 16:
        raise ValueError("WebHID image has an invalid AC632N header or ISDU placeholder")


def validate_web_manifest(manifest, image):
    validate_web_image(image)
    expected = dict(schema=WEB_SCHEMA, product="Nexfoot", origin="community", mcu="AC632N",
                    board="md400-bd19-v1", flashBytes=FLASH_SIZE, codeEnd=CODE_END,
                    protectedBootBytes=WEB_BOOT_END, protectedBootSha256=digest(image[:WEB_BOOT_END]),
                    firmwareBytes=len(image), firmwareSha256=digest(image))
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise ValueError(f"Invalid WebHID package contract: {key}")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._+-]{0,63}", str(manifest.get("version", ""))):
        raise ValueError("Invalid community version")
    if not re.fullmatch(r"[a-f0-9]{64}", str(manifest.get("recoveryArtifactSha256", ""))):
        raise ValueError("Invalid WebHID boot artifact digest")


def write_web_package(image, baseline, boot, output, version, build):
    validate_image(baseline)
    validate_web_image(image)
    if len(image) != len(baseline) + SECTOR or image[WEB_BOOT_END + 0x100: -16] != baseline[BOOT_END + 0x100: -16]:
        raise ValueError("The vendor packer did not relocate the application by one sector")
    manifest = dict(schema=WEB_SCHEMA, product="Nexfoot", origin="community", mcu="AC632N",
                    board="md400-bd19-v1", version=version, flashBytes=FLASH_SIZE,
                    protectedBootBytes=WEB_BOOT_END, codeEnd=CODE_END,
                    protectedBootSha256=digest(image[:WEB_BOOT_END]),
                    recoveryArtifactSha256=digest(Path(boot).read_bytes()),
                    firmwareBytes=len(image), firmwareSha256=digest(image), build=build)
    validate_web_manifest(manifest, image)
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("manifest.json", json.dumps(manifest, indent=2))
        archive.writestr("firmware.bin", image)
    return manifest
