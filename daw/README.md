# Nexfoot DAW SDK

The shared Python module in `common/nexfoot_daw.py` provides the MIDI
control map, SysEx framing, display widgets, and DAW session handling.
The firmware implements the device side of the protocol.

DAW apps live in the sibling `nexfoot-ableton` and `nexfoot-fl-studio`
projects. Each app builds its own installer ZIP with the SDK module.

## Tools

- Run `python3 -m unittest discover -s daw/tests` from the SDK root.
- Run `python3 daw/tools/nexfoot_screen.py replay capture.txt screen.png`
  to render a recorded draw stream.
- Install `mido` and `python-rtmidi`, then run
  `python3 daw/tools/nexfoot_daw_demo.py check` to test a connected pedal
  without a DAW.

See the [DAW mode protocol](../NEXFOOT_SYSEX_API.md#daw-mode-2026-11-08)
and the [widget API](../docs/widgets.md) to build another host app.
