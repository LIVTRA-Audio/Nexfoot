# ENC Control Protocol

Nexfoot firmware uses this protocol to receive encoder and pedal input from
the encoder controller and to set the pedal mode.

## Connection

The serial link uses 115200 baud, 8 data bits, no parity, and one stop bit.

## Frame

`A5 5A VER TYPE SEQ LEN PAYLOAD[LEN] XOR`

All fields are eight-bit bytes. `VER` is `01` and `LEN` is the payload byte
count, up to 16. Calculate `XOR` over `VER`, `TYPE`, `SEQ`, `LEN`, and
each payload byte. The frame uses no escaping and the sequence wraps after
255. The parser checks the version and checksum, rejects oversized payloads,
retains partial frames, and resynchronizes at the next header.

| Type | Direction | Payload |
| --- | --- | --- |
| `10` | Controller → firmware | Raw encoder ID 1..4, event, signed int8 delta (3 bytes) |
| `20` | Controller → firmware | ADC channel and raw 8-bit ADC value (2 bytes) |
| `30` | Firmware → controller | Pedal mode (`0` expression, `1` switch) (1 byte) |
| `31` | Controller → firmware | Acknowledged pedal mode (1 byte) |
| `32` | Controller → firmware | Diagnostics snapshot (13 bytes) |

Encoder events: `01` left, `02` right, `03` press, `04` release. Rotation delta
is signed two's complement. The firmware maps raw encoder IDs `1,2,3,4` to
logical IDs `2,1,4,3` and reverses left/right before applying UI acceleration.
Consumers needing the physical layout should use that mapping, not assume the
raw ID equals a display slot. ADC channel 4 carries the pedal; MIDI-scale values
use `raw >> 1`.

Diagnostics offsets: 0 mode; 1..2 received-byte count (little-endian uint16);
3 accepted-frame count; 4 mode-command count; 5 checksum errors; 6 version errors;
7 length errors; 8 overflows; 9 last type; 10 last sequence; 11 last length;
12 last received byte. Single-byte counters wrap.

The firmware retries pedal-mode synchronization every 250 ms for 20
attempts, then at a 1000 ms backoff. It accepts a fresh acknowledgement reporting
the requested mode; the current implementation does not require an exact echoed
sequence. Preserve this behavior for compatibility with shipped companion firmware.

## Golden frames

```text
A5 5A 01 10 01 03 01 02 01 11  # raw encoder 1, right, delta +1
A5 5A 01 20 02 02 04 80 A5     # ADC4 = 128
A5 5A 01 30 03 01 01 32        # request switch mode
A5 5A 01 31 FF 01 01 CF        # acknowledge switch, seq 255
A5 5A 01 31 00 01 00 31        # acknowledge expression, seq wraps to 0
```
