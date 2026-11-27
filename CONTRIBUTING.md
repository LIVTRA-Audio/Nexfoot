# Contributing

Run `python tools/nexfoot.py check` before submitting changes. For firmware
changes, build with `tools/nexfoot.py` and inspect the generated package.
Run `python -m unittest discover -s daw/tests` for changes to the DAW SDK.
Describe what you tested in the pull request.

Keep the [ENC Control Protocol](docs/enc-control-protocol.md) compatible
with shipped devices. Document the compatibility plan and include examples
for any new protocol version.
Changes to the recovery boot, dependency pins, or flash layout require
maintainer review and testing on hardware.

Contribute under [Apache-2.0](LICENSE) and preserve upstream notices.
Share code and test data you have permission to redistribute. Keep
credentials and device backups out of contributions.
