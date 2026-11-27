# Nexfoot firmware

Build community firmware for the Nexfoot MIDI foot controller. The firmware
handles USB and Bluetooth MIDI, presets, and the on-device display. This
repository also includes display widgets and a Python SDK for DAW integrations.

See [Source and licenses](docs/source-scope.md) for licensing and attribution.

## Quick start

Use Python 3.10 or later and Git. Install the
[JieLi q32s toolchain](docs/build.md#dependencies) for your build environment.

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
python tools/nexfoot.py check
python tools/nexfoot.py bootstrap
```

On Windows PowerShell, create the environment with `py -3 -m venv .venv`
and activate it with `.\.venv\Scripts\Activate.ps1`.

```sh
python tools/nexfoot.py build --toolchain C:/JL/pi32 --version 1.0.1-community
python tools/nexfoot.py inspect dist/nexfoot-1.0.1-community.nexfoot
```

`check` validates the build tools and recovery policy. `bootstrap` fetches the
pinned SDK. `build` creates the `.nexfoot` package offline in `dist/`.
Replace the example toolchain path with your installation path.
`Build_Nexfoot.bat` also accepts the build options on Windows.

## Build a display or DAW integration

Use the [widget API](docs/widgets.md) to build a firmware page and the
[DAW SDK](daw/README.md) to build a controller script. The Ableton Live and
FL Studio integrations use the SDK and are packaged separately.

## Update and recovery

Use the community updater on a Nexfoot with a compatible recovery boot.
Connect one Nexfoot by USB. Hold its power button during startup for two
seconds to enter UBOOT, then run preflight.

```sh
python tools/nexfoot.py update dist/nexfoot-1.0.1-community.nexfoot
```

Preflight checks compatibility without writing flash. After a successful
preflight, repeat the command with `--yes` to install the package. The updater
saves a local backup, verifies the application after writing it, and checks
that USB MIDI returns. Use a firmware build you trust. The community updater
still requires hardware qualification for general use. See
[Recovery and updates](docs/recovery.md) for host requirements, recovery steps
and what recovery covers. If an update fails and the pedal shows nothing, it may
already be in UBOOT.

## Documentation

- [Build environment and dependency pins](docs/build.md)
- [Recovery boot and reference files](recovery/README.md)
- [ENC Control Protocol](docs/enc-control-protocol.md)
- [USB/BLE NanoBridge SysEx API](NEXFOOT_SYSEX_API.md)
- [Contributing](CONTRIBUTING.md)
