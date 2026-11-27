# Approved recovery boot

Source: JieLi `fw-Bootloader` commit
`ff3b9299d1b94d834b995fd858b3f75e2faa9840`, Apache-2.0, plus
`recovery_main.patch` in this directory. The patch preserves the board power
latch and provides USB-connected power-key recovery independently of the JL app.

`uboot_recovery.boot` is the packed recovery boot. The build wrapper stages
it as `uboot.boot` for the JieLi packer. `protected-region.bin` is the
integrity reference for the recovery region. Both files cover the recovery
boot only.

The reference SHA-256 hashes are recorded in `dependencies.lock.json` and
`tools/nexfoot_package.py`. Community builds must match them. Changing recovery
source does not automatically replace the approved artifact or enable boot writes.

To rebuild the recovery boot on Windows, bootstrap the SDK and run:

```powershell
powershell -ExecutionPolicy Bypass -File recovery\build.ps1
```

The recipe fetches the pinned JieLi bootloader, applies the patch, and writes
the result to `build/recovery/uboot.boot` within the local workspace. Test
recovery on hardware before updating the reference hashes.
