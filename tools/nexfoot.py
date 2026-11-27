#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Supported Nexfoot community build, inspection, and recovery update entry point."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

from nexfoot_package import RECOVERY_SHA256, ROOT, SDK_COMMIT, digest, load_package, write_package
from nexfoot_paths import WORKSPACE, SDK, STAGE


def run(command, *, cwd=ROOT, env=None):
    print("+", " ".join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), cwd=cwd, env=env, check=True)


def bootstrap():
    lock = json.loads((ROOT / "dependencies.lock.json").read_text())
    if lock["sdk"]["commit"] != SDK_COMMIT:
        raise ValueError("SDK pin does not match community package policy")
    if not SDK.exists():
        SDK.parent.mkdir(parents=True, exist_ok=True)
        run(["git", "init", SDK])
        run(["git", "-C", SDK, "remote", "add", "origin", lock["sdk"]["url"]])
        run(["git", "-C", SDK, "fetch", "--depth", "1", "origin", SDK_COMMIT])
        run(["git", "-C", SDK, "checkout", "--detach", SDK_COMMIT])
    head = subprocess.check_output(["git", "-C", SDK, "rev-parse", "HEAD"], text=True).strip()
    if head != SDK_COMMIT:
        raise ValueError("cached SDK is at an unexpected commit")
    status = subprocess.check_output(["git", "-C", SDK, "status", "--porcelain", "--untracked-files=no"], text=True)
    if status.strip():
        raise ValueError("cached SDK has tracked modifications; use a pristine dependency checkout")
    for name, expected in lock["sdkFiles"].items():
        if digest((SDK / name).read_bytes()) != expected:
            raise ValueError(f"SDK dependency hash mismatch: {name}")
    if digest((ROOT / "recovery/uboot_recovery.boot").read_bytes()) != RECOVERY_SHA256:
        raise ValueError("recovery artifact hash mismatch")
    print("Pinned SDK and recovery dependencies verified.")


def prepare_stage():
    # Regenerate an isolated stage every time. An old sdk.elf or jl_isd.bin
    # can never satisfy a new build, and no private working tree is needed.
    if STAGE.exists():
        shutil.rmtree(STAGE)
    STAGE.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(SDK, STAGE, ignore=shutil.ignore_patterns(".git", "objs", "*.elf", "*.o", "*.d", "*.map", "*.lst",
                                                           "app.bin", "jl_isd.bin", "jl_isd.fw", "*.ufw"))
    for directory in ("apps", "cpu", "include_lib"):
        for source in (ROOT / directory).rglob("*"):
            if source.is_file():
                target = STAGE / source.relative_to(ROOT)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, target)
    shutil.copy2(ROOT / "recovery/uboot_recovery.boot", STAGE / "cpu/bd19/tools/uboot.boot")


def toolchain_paths(path: Path):
    if os.name == "nt":
        binary = path / "bin"
        compiler, objcopy = binary / "clang.exe", binary / "llvm-objcopy.exe"
        make = SDK / "tools/utils/make.exe"
        args = [f"TOOL_DIR={binary.as_posix()}", f"SYS_LIB_DIR={(path / 'q32s-lib').as_posix()}",
                f"SYS_INC_DIR={(path / 'q32s-include').as_posix()}"]
    else:
        binary = path / "bin"
        compiler, objcopy = binary / "clang", binary / "objcopy"
        make = Path(shutil.which("make") or "make")
        args = [f"TOOL_DIR={binary.as_posix()}", f"SYS_LIB_DIR={(path / 'lib').as_posix()}",
                f"SYS_INC_DIR={(path / 'include').as_posix()}"]
    if not compiler.is_file() or not objcopy.is_file():
        raise ValueError(f"JL q32s compiler/objcopy not found in {binary}; install the vendor toolchain, see docs/build.md")
    return compiler, objcopy, make, args


def exe_command(path: Path):
    if os.name == "nt":
        return [path]
    wine = shutil.which("wine")
    if not wine:
        raise ValueError("Wine is required for the vendor packer on Linux/macOS")
    return [wine, path]


def extract_and_pack(objcopy: Path, compiler_run=run):
    tools = STAGE / "cpu/bd19/tools"
    elf = tools / "sdk.elf"
    sections = ["text", "data", "data_code", "overlay_aec", "overlay_aac", "overlay_aptx", "common"]
    for section in sections:
        compiler_run([objcopy, "-O", "binary", "-j", "." + section, elf, tools / (section + ".bin")])
    banks = []
    for index in range(21):
        target = tools / f"bank{index}.bin"
        compiler_run([objcopy, "-O", "binary", "-j", f".overlay_bank{index}", elf, target])
        if not target.exists() or not target.stat().st_size:
            break
        banks += [target.name, "0x0"]
    run(exe_command(tools / "lz4_packet.exe") + ["-dict", "text.bin", "-input", "common.bin", "0"] + banks + ["-o", "bank.bin"], cwd=tools)
    app_sections = ["text", "data", "data_code", "overlay_aec", "overlay_aac", "bank", "overlay_aptx"]
    (tools / "app.bin").write_bytes(b"".join((tools / (name + ".bin")).read_bytes() for name in app_sections))
    # Crucially, never invoke the vendor download.bat. It can program a
    # connected device. -package-only keeps builds free of USB flash writes.
    env = dict(os.environ, WINEDEBUG="-all", MVK_CONFIG_LOG_LEVEL="0")
    run(exe_command(tools / "isd_download.exe") + ["isd_config.ini", "-package-only", "-tonorflash", "-dev", "bd19",
        "-boot", "0x2000", "-div8", "-uboot", "uboot.boot", "-app", "app.bin", "cfg_tool.bin",
        "-res", "p11_code.bin", "-uboot_compress", "-flash-params", "flash_params.bin"], cwd=tools, env=env)
    image = tools / "jl_isd.bin"
    if not image.is_file():
        raise ValueError("offline packer did not produce jl_isd.bin")
    return image.read_bytes()


def build(args):
    if args.container:
        # Use a local image resolved to an immutable ID; never grant the
        # compiler network access or map USB devices into the container.
        container_id = subprocess.check_output(["docker", "image", "inspect", args.container,
                                                "--format", "{{.Id}}"], text=True).strip()
        toolchain = Path(args.toolchain or "/opt/jieli/q32s")
        objcopy = toolchain / "bin/objcopy"
        make = "make"
        make_args = [f"TOOL_DIR={toolchain}/bin", f"SYS_LIB_DIR={toolchain}/lib", f"SYS_INC_DIR={toolchain}/include"]
        def compile_run(command):
            command = [str(arg).replace(str(STAGE), "/workspace") for arg in command]
            return run(["docker", "run", "--rm", "--platform", "linux/amd64", "--network", "none",
                        "--ulimit", "nofile=8192:8192", "--mount", f"type=bind,source={STAGE},target=/workspace",
                        "--workdir", "/workspace", container_id] + command)
        compiler_hash = subprocess.check_output(["docker", "run", "--rm", "--platform", "linux/amd64",
            "--network", "none", container_id, "sha256sum", str(toolchain / "bin/clang")], text=True).split()[0]
    else:
        default = "C:/JL/pi32" if os.name == "nt" else "/opt/jieli/q32s"
        compiler, objcopy, make, make_args = toolchain_paths(Path(args.toolchain or default).resolve())
        compile_run = run
        container_id = None
        compiler_hash = digest(compiler.read_bytes())
    bootstrap()
    prepare_stage()
    target = STAGE / "apps/spp_and_le/board/bd19"
    # Explicit targets skip 'all' and its inherited programmer post-script.
    # Keep pre-build sequential because the generated linker/config files
    # must exist before compiling and linking.
    if args.web_boot:
        make_args += ["USER_EXTRA_CFLAGS=-DNEXFOOT_WEB_BOOT"]
    compile_run([make, "-C", target, "pre_build"] + make_args)
    compile_run([make, "-C", target, "-j", str(args.jobs), "../../../../cpu/bd19/tools/sdk.elf"] + make_args)
    image = extract_and_pack(objcopy, compile_run)
    source_files = sorted(p for directory in ("apps", "cpu", "include_lib", "recovery")
                          for p in (ROOT / directory).rglob("*") if p.is_file())
    source_hash = hashlib.sha256()
    for path in source_files:
        source_hash.update(path.relative_to(ROOT).as_posix().encode() + b"\0" + path.read_bytes() + b"\0")
    firmware_source = (ROOT / "apps/spp_and_le/examples/trans_data/ble_trans.c").read_text(encoding="utf-8")
    firmware_version = ".".join(re.search(r"#define\s+MD400_FW_INFO_" + part + r"\s+(\d+)", firmware_source).group(1)
                                for part in ("MAJOR", "MINOR", "PATCH"))
    metadata = {"sourceSha256": source_hash.hexdigest(), "firmwareVersion": firmware_version,
                "compilerSha256": compiler_hash, "compilerContainerId": container_id,
                "elfSha256": digest((STAGE / "cpu/bd19/tools/sdk.elf").read_bytes()), "offlinePackaging": True}
    version = args.version or firmware_version + "-community"
    out = Path(args.output) if args.output else ROOT / "dist" / f"nexfoot-{version}.nexfoot"
    if args.web_boot:
        from nexfoot_web_package import write_web_package
        boot = Path(args.web_boot).resolve()
        shutil.copy2(boot, STAGE / "cpu/bd19/tools/uboot.boot")
        web_image = extract_and_pack(objcopy, compile_run)
        manifest = write_web_package(web_image, image, boot, out, version, metadata)
    else:
        manifest = write_package(image, out, version, metadata)
    print(json.dumps({"package": str(out), **manifest}, indent=2))


def check():
    # Basic local tool/policy validation has no private test-suite dependency.
    for path in (ROOT / "tools").glob("*.py"):
        compile(path.read_bytes(), str(path), "exec")
    lock = json.loads((ROOT / "dependencies.lock.json").read_text())
    if lock["sdk"]["commit"] != SDK_COMMIT:
        raise ValueError("SDK pin does not match community package policy")
    if lock["recovery"]["sha256"] != RECOVERY_SHA256:
        raise ValueError("recovery pin does not match community package policy")
    if digest((ROOT / "recovery/uboot_recovery.boot").read_bytes()) != RECOVERY_SHA256:
        raise ValueError("recovery artifact hash mismatch")
    print("Build/update tool syntax and recovery policy verified.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("bootstrap", help="fetch and verify the pinned public JL SDK")
    b = sub.add_parser("build", help="compile, package offline, and validate recovery")
    b.add_argument("--toolchain", help="JL pi32/q32s toolchain root")
    b.add_argument("--container", help="local linux/amd64 Docker image containing the JL q32s toolchain")
    b.add_argument("--version", help="package label; default is firmware-version-community")
    b.add_argument("--output")
    b.add_argument("--web-boot", help="bench WebHID boot artifact")
    b.add_argument("--jobs", type=int, default=4)
    i = sub.add_parser("inspect", help="validate a community package without opening hardware")
    i.add_argument("package", type=Path)
    u = sub.add_parser("update", help="update a device already in physical UBOOT recovery")
    u.add_argument("package", type=Path)
    u.add_argument("--yes", action="store_true", help="perform the planned writes; default is read-only preflight")
    sub.add_parser("check", help="verify tool syntax and the approved recovery policy without hardware")
    sub.add_parser("paths", help="show the local source, SDK cache, and build workspace paths")
    args = parser.parse_args()
    try:
        if args.command == "bootstrap":
            bootstrap()
        elif args.command == "build":
            if args.jobs < 1:
                raise ValueError("jobs must be positive")
            build(args)
        elif args.command == "inspect":
            print(json.dumps(load_package(args.package)[0], indent=2))
        elif args.command == "update":
            # Validate before even fetching the loader, much less opening USB.
            manifest, image = load_package(args.package)
            bootstrap()
            from nexfoot_update import update
            print(json.dumps(update(args.package, manifest, image, ROOT, write=args.yes), indent=2))
        elif args.command == "check":
            check()
        elif args.command == "paths":
            print(json.dumps({"source": str(ROOT), "workspace": str(WORKSPACE),
                              "sdk": str(SDK), "build": str(STAGE)}, indent=2))
    except (OSError, ValueError, RuntimeError, zipfile.BadZipFile, subprocess.CalledProcessError) as exc:
        print(f"Nexfoot: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
