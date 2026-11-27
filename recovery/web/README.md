# Experimental browser firmware updates

This WebHID prototype is for development on units with compatible WebHID
recovery firmware. It uses a `.nexfoot` package built with the matching
recovery boot and retains UBOOT as the recovery path.

## Build on Windows

Run `python tools/nexfoot.py bootstrap`. Check out the JieLi bootloader
revision in [the dependency lockfile](../../dependencies.lock.json)
in a separate working directory. From the Nexfoot source root, run the
following with paths that match your setup.

```powershell
python recovery/web/configure.py C:/work/fw-Bootloader
Push-Location C:/work/fw-Bootloader/user_boot
& C:/path/to/jl-sdk/tools/utils/make.exe bd19_uboot -j4
Pop-Location
python tools/nexfoot.py build --toolchain C:/JL/pi32 --web-boot C:/work/fw-Bootloader/user_boot/cpu/bd19/output/uboot.boot --version web-test
```

## Browser

Serve `recovery/web/browser` over HTTPS or localhost and open it in Chrome
or Edge. With a compatible device in WebHID update mode, load its
`.nexfoot` file and select the pedal in the browser's device picker.
The updater writes and verifies the application while preserving recovery.

If an update is interrupted, use [UBOOT recovery](../../docs/recovery.md)
and run the native updater with the same compatible package. Use a firmware
package from a source you trust.
