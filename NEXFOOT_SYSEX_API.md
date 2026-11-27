# NEXFOOT NanoBridge SysEx API

NanoBridge is the Nexfoot SysEx protocol for configuring presets, reading
device state, and driving the display in DAW mode. This reference describes
protocol version `0x01` for the community firmware. Call `GET_INFO` when
connecting and handle unsupported commands on older firmware.

## 1. Transport and MIDI ports

The runtime USB device exposes two logical USB-MIDI cables through one MIDI
streaming interface:

| USB-MIDI cable | Logical name | Purpose |
| --- | --- | --- |
| `0` | NanoBridge | Control SysEx requests and replies described here |
| `1` | NanoControl | Normal musical MIDI input/output |

Select the NanoBridge input and output for the same Nexfoot. Port names
vary by host, especially when multiple devices are connected.

NanoBridge also supports BLE MIDI. Follow BLE MIDI timestamp and framing
rules and check `GET_INFO` on the selected connection.

## 2. Packet format

All bytes between `F0` and `F7` must be MIDI-safe 7-bit values (`00..7F`).

### 2.1 Request

```text
F0 7D 4E 42 01 CMD SEQ [PAYLOAD ...] CHECKSUM F7
```

| Offset | Field | Size | Meaning |
| ---: | --- | ---: | --- |
| 0 | `F0` | 1 | MIDI SysEx start |
| 1 | Manufacturer | 1 | `7D`, development/non-commercial ID |
| 2..3 | Signature | 2 | ASCII `N`, `B` = `4E 42` |
| 4 | Version | 1 | Protocol version, currently `01` |
| 5 | `CMD` | 1 | Command ID |
| 6 | `SEQ` | 1 | Request sequence, normally `01..7F` |
| 7..N | Payload | variable | Command-specific 7-bit data |
| N+1 | Checksum | 1 | 7-bit complement checksum |
| N+2 | `F7` | 1 | MIDI SysEx end |

### 2.2 Reply

```text
F0 7D 4E 42 01 CMD SEQ STATUS [PAYLOAD ...] CHECKSUM F7
```

The reply repeats the request's `CMD` and `SEQ`. There is no separate reply bit,
so match replies using both fields.

| Field | Meaning |
| --- | --- |
| `CMD` | Same command ID as the request |
| `SEQ` | Same sequence as the request |
| `STATUS` | Result code; `00` means success |
| Payload | Command-specific response data |

The reply length depends on the command.

### 2.3 Checksum

The manufacturer byte and `NB` signature are not included in the checksum.

For a request, calculate over:

```text
VERSION + CMD + SEQ + PAYLOAD
```

For a reply, calculate over:

```text
VERSION + CMD + SEQ + STATUS + PAYLOAD
```

The algorithm is:

```python
def checksum(data):
    return (0x80 - (sum(byte & 0x7F for byte in data) & 0x7F)) & 0x7F
```

The checksum is correct when the checksum-domain bytes plus `CHECKSUM` add to
zero modulo 128.

### 2.4 PING example

Request, command `01`, sequence `01`:

```text
F0 7D 4E 42 01 01 01 7D F7
```

Successful reply with ASCII `PONG`:

```text
F0 7D 4E 42 01 01 01 00 50 4F 4E 47 49 F7
```

### 2.5 Multi-byte integers

Normal counters and offsets use a little-endian 14-bit representation:

```text
u14 = LO7 | (HI7 << 7)
```

For example, decimal 300 is `2C 02` because `0x12C = 0x2C + (0x02 << 7)`.

## 3. Status codes

| Status | Name | Meaning / application response |
| ---: | --- | --- |
| `00` | `OK` | Request completed successfully |
| `01` | `BAD_CHECKSUM` | Version/checksum validation failed; rebuild the frame |
| `02` | `BAD_LENGTH` | Required payload bytes are missing or length is invalid |
| `03` | `BAD_VALUE` | A field is outside its valid range or key does not exist |
| `04` | `UNSUPPORTED` | Command or operation is not supported in this runtime mode |
| `05` | `BUSY` | A flash job or shared template buffer is busy; retry later |
| `06` | `FLASH_ERROR` | Persistent storage operation failed |

Applications should wait for the matching reply before sending another flash
write. For `BUSY`, retry after a short delay. Do not retry `FLASH_ERROR` in a
tight loop; report it to the user and avoid creating additional flash writes.

## 4. Common data definitions

### 4.1 Assignment fields

Encoder, gesture, expression-pedal, and pedal-mode FT assignments share this
9-byte structure:

```text
TYPE CHANNEL DATA1 MIN MAX DEFAULT BEHAVIOR OUTPUTS TEMPLATE
```

| Index | Field | Range | Meaning |
| ---: | --- | --- | --- |
| 0 | `TYPE` | `0..8` | MIDI message type |
| 1 | `CHANNEL` | `1..16` | Human-readable MIDI channel; firmware converts to `0..15` internally |
| 2 | `DATA1` | `0..127` | CC number, note number, or release/off template slot |
| 3 | `MIN` | `0..127` | Minimum/off value |
| 4 | `MAX` | `0..127` | Maximum/on value |
| 5 | `DEFAULT` | `0..127` | Initial value or absolute gesture value |
| 6 | `BEHAVIOR` | `0..3` | Absolute, toggle/relative, momentary, or pulse |
| 7 | `OUTPUTS` | `0..127` | Compatibility field; current firmware replaces it with all available outputs |
| 8 | `TEMPLATE` | `0..127` | SysEx/custom template slot, normally `1..4`, or `0` for none |

Current firmware fans performance MIDI out to every compiled and available
product output. Per-assignment output selection is no longer supported. A value
must still be supplied in the `OUTPUTS` byte, but the firmware stores its own
available-output mask instead.

### 4.2 MIDI types

| Type | Name | Generated output |
| ---: | --- | --- |
| `0` | CC | `Bn DATA1 VALUE` |
| `1` | Program Change | `Cn VALUE`; `DATA1` is not used for the program number |
| `2` | Note On | `9n DATA1 VALUE`; an off gate produces `8n DATA1 MIN` |
| `3` | Pitch Bend | `En 00 VALUE`; the 7-bit value is placed in the high 7 bits |
| `4` | Channel Pressure | `Dn VALUE` |
| `5` | SysEx template | Sends template `TEMPLATE`, substituting `VALUE` at its value index |
| `6` | Off | Sends nothing |
| `7` | Note Off | `8n DATA1 VALUE` |
| `8` | Custom MIDI template | Sends a stored raw MIDI byte sequence |

`n` is `CHANNEL - 1`.

For SysEx or custom assignments used as gates, `TEMPLATE` selects the on/press
slot. If `DATA1` is `1..4`, it selects the off/release slot; otherwise release
uses the same slot as press.

If a type-5 assignment references an empty or invalid custom SysEx slot, the
firmware falls back to this built-in message:

```text
F0 7D 4D 44 34 30 01 TEMPLATE CONTROL DATA1 VALUE F7
```

### 4.3 Behaviors

| Value | Rotary meaning | Button/gesture meaning |
| ---: | --- | --- |
| `0` | Absolute scaled value | Send `DEFAULT` once |
| `1` | Relative CC for a rotary encoder | Toggle: one message per click, `MAX` then `MIN` |
| `2` | Absolute for rotation | Momentary: press=`MAX`, release=`MIN` |
| `3` | Absolute for rotation | Pulse: send `MAX`, then immediately `MIN` |

Product Press is stored as SHORT (`0x05`). SHORT itself is generated after
release, so Momentary on that slot uses the physical down/up pair: `MAX` when
the FT goes down, `MIN` when it comes up. Toggle and Absolute still fire on
the SHORT click. Pulse still sends `MAX` then `MIN` immediately on the click.

Firmware PRESS (`0x03`) Momentary still uses its own down/up tracker. If the
SHORT assignment is Momentary, that hold wins and PRESS is not also sent.

Relative rotary CC uses `01..3F` for clockwise/increment and `7F..41` for
counter-clockwise/decrement. Reversed ranges (`MIN > MAX`) reverse direction.

### 4.4 Outputs mask

The mask reported by `GET_INFO` and assignment reads uses:

| Bit | Mask | Output |
| ---: | ---: | --- |
| 0 | `01` | USB device / NanoControl |
| 1 | `02` | USB-A host MIDI |
| 2 | `04` | BLE MIDI |
| 3 | `08` | Physical 5-pin MIDI OUT |

### 4.5 Gesture event and control values

| Event | Value | Valid control |
| --- | ---: | --- |
| Encoder left | `01` | Encoder `1..4` |
| Encoder right | `02` | Encoder `1..4` |
| Encoder press/down | `03` | Encoder `1..4` |
| Encoder release/up | `04` | Encoder `1..4` |
| Encoder short | `05` | Encoder `1..4` |
| Encoder long | `06` | Encoder `1..4` |
| Encoder double | `07` | Encoder `1..4` |
| Combo press | `08` | Mask `03`, `06`, or `0C` |
| Combo short | `09` | Mask `03`, `06`, or `0C` |
| Combo long | `0A` | Mask `03`, `06`, or `0C` |
| Edit-key short | `10` | `0` |
| Edit-key long | `11` | `0` |
| Edit-key double | `12` | `0` |
| Pedal test value | `13` | ADC channel, normally `4` |

Combo masks are bit masks of the four encoders: `03` = 1+2, `06` = 2+3,
and `0C` = 3+4.

### 4.6 UI state values

| Field | Values |
| --- | --- |
| UI mode | `0` Perform, `1` Edit, `2` Play |
| Foot mode | `0` MIDI, `1` Pedal |
| Pedal edit view | `0` action/MIDI, `1` style/label |
| Edit page | `0` preset, `1` message, `2` value, `3` short-message, `4` short-value |
| Edit subject | `0` rotate, `1` short, `2` long, `3` double, `4` press, `5` combo-press, `6` combo-short, `7` combo-long, `8` expression pedal |

## 5. Command summary

| Command | ID | Purpose |
| --- | ---: | --- |
| PING | `01` | Link test |
| GET_INFO | `02` | Capabilities and firmware version |
| GET_STATE | `03` | Current UI and mode state |
| GET_PRESET | `10` | MIDI preset status or one preset item |
| SET_PRESET | `11` | Change one MIDI preset item in RAM |
| SAVE_PRESET | `12` | Commit MIDI preset data to flash |
| LOAD_PRESET | `13` | Activate a MIDI preset |
| GET_PRESET_LABEL | `14` | Read MIDI preset name |
| SET_PRESET_LABEL | `15` | Change MIDI preset name in RAM |
| FACTORY_RESET | `16` | Restore all persistent defaults |
| GET_ASSIGNMENT | `20` | Read active MIDI preset encoder assignment |
| SET_ASSIGNMENT | `21` | Change active MIDI preset encoder assignment |
| GET_GESTURE_ASSIGNMENT | `22` | Read active MIDI preset gesture assignment |
| SET_GESTURE_ASSIGNMENT | `23` | Change active MIDI preset gesture assignment |
| GET_PEDAL_ASSIGNMENT | `24` | Read expression-pedal assignment in active MIDI preset |
| SET_PEDAL_ASSIGNMENT | `25` | Change expression-pedal assignment in active MIDI preset |
| GET_PEDAL_STATE | `26` | Expression/switch input telemetry |
| GET_MIDI_OUT_STATE | `27` | Physical MIDI OUT telemetry |
| GET_USB_HOST_STATE | `28` | USB-A host MIDI telemetry |
| GET_LED_STATE | `29` | LED policy and hardware state |
| GET_EDIT_KEY_STATE | `2A` | Physical edit-key scanner telemetry |
| GET_BLE_MIDI_STATE | `2B` | BLE MIDI telemetry |
| GET_DISPLAY_STATE | `2C` | Display/UI snapshot |
| EVENT_INJECT | `30` | Inject a test/UI event |
| GET_EVENT_LOG | `31` | Read four latest runtime events |
| GET_ACTION_STATE | `32` | Read toggle/momentary action masks |
| GET_PROTOCOL_STATE | `33` | Read NanoBridge protocol counters |
| GET_PLAY_LABEL | `34` | Read Play-mode text |
| SET_PLAY_LABEL | `35` | Write Play-mode text |
| GET_SYSEX_TEMPLATE | `36` | Read a 7-bit SysEx template chunk |
| SET_SYSEX_TEMPLATE | `37` | Write a 7-bit SysEx template chunk |
| GET_CUSTOM_TEMPLATE | `38` | Read a raw MIDI template chunk |
| SET_CUSTOM_TEMPLATE | `39` | Write a raw MIDI template chunk |
| ENTER_EDIT | `40` | Enter edit mode |
| EXIT_EDIT | `41` | Exit edit mode |
| GET_FOOT_MODE | `42` | Read MIDI/Pedal mode and pedal state |
| SET_FOOT_MODE | `43` | Select MIDI or Pedal mode |
| GET_PEDAL_ACTION | `44` | Read one pedal-mode preset/page/FT action |
| SET_PEDAL_ACTION | `45` | Write one pedal-mode preset/page/FT action |
| GET_PEDAL_LABEL | `46` | Read one pedal-mode FT label |
| SET_PEDAL_LABEL | `47` | Write one pedal-mode FT label |
| DETACH_UPDATE | `7E` | Reboot into the wired JL UBOOT updater |
| RESET | `7F` | Reboot the runtime firmware |

## 6. Core commands

### 6.1 PING (`01`)

Request payload: empty.

Success payload:

```text
50 4F 4E 47
```

This is ASCII `PONG`.

### 6.2 GET_INFO (`02`)

Request payload: empty.

Success payload, 10 bytes:

| Index | Field | Meaning |
| ---: | --- | --- |
| 0 | Encoder count | `4` |
| 1 | MIDI preset count | `20` |
| 2 | Protocol version | `1` |
| 3 | Type mask | Bit `n` indicates basic type `n`; custom type 8 is accepted even though it cannot fit this 7-bit mask |
| 4 | Output mask | Available output bits |
| 5 | Preset-store schema | Current persistent MIDI-preset schema version |
| 6 | Firmware major | Decimal version component |
| 7 | Firmware minor | Decimal version component |
| 8 | Firmware patch | Decimal version component |
| 9 | Feature flags | Optional capability bits below |

Feature flags:

| Bit | Mask | Feature |
| ---: | ---: | --- |
| 0 | `01` | USB host MIDI telemetry/path |
| 1 | `02` | Physical MIDI OUT |
| 2 | `04` | LED state API |
| 3 | `08` | UART/update detach support |
| 4 | `10` | Grouped MIDI-preset persistence |
| 5 | `20` | Edit-key telemetry |
| 6 | `40` | BLE MIDI telemetry |

Always call `GET_INFO` when connecting and reject an unsupported protocol
version before writing configuration.

### 6.3 GET_STATE (`03`)

Request payload: empty.

Success payload, 12 bytes:

```text
UI_MODE MIDI_ACTIVE MIDI_SELECTED EDIT_PAGE EDIT_TARGET EDIT_SUBJECT
FOOT_MODE PEDAL_ACTIVE PEDAL_SELECTED PEDAL_PAGE PEDAL_FT PEDAL_VIEW
```

Preset and page values are 1-based. `PEDAL_FT` is `1..4`.

### 6.4 GET_FOOT_MODE (`42`)

Request payload: empty.

Success payload, 7 bytes:

```text
FOOT_MODE MIDI_ACTIVE PEDAL_ACTIVE PEDAL_PAGE PEDAL_FT PEDAL_VIEW DIRTY
```

`DIRTY=1` means the current pedal-mode action record has pending changes.

### 6.5 SET_FOOT_MODE (`43`)

Request payload:

```text
MODE
```

| Mode | Meaning |
| ---: | --- |
| `0` | MIDI mode |
| `1` | Pedal mode |

Changing mode returns the UI to Perform mode and persists the mode header.

Example selecting Pedal mode with sequence `02`:

```text
F0 7D 4E 42 01 43 02 01 39 F7
```

Success reply:

```text
F0 7D 4E 42 01 43 02 00 3A F7
```

## 7. MIDI preset API

There are 20 MIDI presets. MIDI presets and Pedal-mode presets are separate
stores and have separate active-preset values.

### 7.1 GET_PRESET (`10`): preset status

Request:

```text
PRESET
```

`PRESET` is `1..20`.

Success payload:

```text
PRESET ACTIVE CHECKSUM GROUP_SLOT GROUP_SEQ_LO GROUP_SEQ_HI
```

`GROUP_SLOT` is the active A/B persistence slot (`1` or `2`) for the preset's
four-preset group. `GROUP_SEQ` is a `u14` generation counter.

### 7.2 GET_PRESET (`10`): preset item

Use one of these payloads:

```text
PRESET 00 ENCODER
PRESET 01 EVENT CONTROL
PRESET 02 01
PRESET 03 01
```

Sections:

| Section | Value | Key | Reply after key |
| --- | ---: | --- | --- |
| Encoder | `00` | Encoder `1..4` | 9 assignment fields |
| Gesture | `01` | Event and control | 9 assignment fields |
| Expression pedal | `02` | Constant `01` | 9 assignment fields |
| LED | `03` | Constant `01` | `LED_MODE LED_MASK` |

Replies repeat the request key before the returned fields.

### 7.3 SET_PRESET (`11`)

Use the same keys as the item form of `GET_PRESET`, followed by the replacement
fields:

```text
PRESET 00 ENCODER [9 assignment fields]
PRESET 01 EVENT CONTROL [9 assignment fields]
PRESET 02 01 [9 assignment fields]
PRESET 03 01 LED_MODE LED_MASK
```

LED modes are `0` auto, `1` off, and `2` manual. `LED_MASK` is `00..0F`, with
bits 0..3 corresponding to the four front-panel LEDs.

These changes update RAM. Call `SAVE_PRESET` after a successful series of
updates if the changes must survive reboot.

Example: MIDI preset 1, encoder 1, CC 74 on channel 1, range 0..127, absolute,
all-output compatibility byte `0F`, no template; sequence `04`:

```text
F0 7D 4E 42 01 11 04 01 00 01 00 01 4A 00 7F 00 00 0F 00 0F F7
```

### 7.4 SAVE_PRESET (`12`)

Request payload:

```text
empty
```

or:

```text
PRESET
```

An empty payload saves the four-preset group containing the current active MIDI
preset, plus the active-preset header. Supplying `PRESET` (`1..20`) chooses the
group to commit and currently updates the active-preset index as a side effect.
From firmware `0.49.67`, `SAVE_PRESET` also flushes a dirty pedal-mode VM
record first (readback verify, up to three attempts). If that flush fails the
command returns `FLASH_ERROR` and does not write the MIDI group.
Use `LOAD_PRESET` when the application explicitly wants to activate a preset
and synchronize all runtime values.

The flash operation is asynchronous. The device sends the reply only after the
job finishes. Do not treat request transmission as completion.

Example, save preset 1 with sequence `05`:

```text
F0 7D 4E 42 01 12 05 01 67 F7
```

### 7.5 LOAD_PRESET (`13`)

Request:

```text
PRESET
```

Activates MIDI preset `1..20`, loads its runtime default values, and persists
the active-preset header. The reply is delayed until the flash job completes.

### 7.6 GET/SET_PRESET_LABEL (`14`/`15`)

GET request:

```text
PRESET
```

GET success payload:

```text
PRESET LENGTH ASCII...
```

SET request uses the same structure. The label is `0..8` printable ASCII bytes
(`20..7E`). An empty label is valid.

`SET_PRESET_LABEL` changes the MIDI preset in RAM. Call `SAVE_PRESET` to make it
persistent.

## 8. Active MIDI-preset assignments

These commands are convenient aliases for editing the active MIDI preset.

### 8.1 GET/SET_ASSIGNMENT (`20`/`21`)

GET request:

```text
ENCODER
```

GET success payload:

```text
ENCODER [9 assignment fields] CURRENT_VALUE
```

SET request:

```text
ENCODER [9 assignment fields]
```

Setting an encoder resets its current runtime value to `DEFAULT`. Call
`SAVE_PRESET` to persist it.

### 8.2 GET/SET_GESTURE_ASSIGNMENT (`22`/`23`)

GET request:

```text
EVENT CONTROL
```

GET success payload:

```text
EVENT CONTROL [9 assignment fields] CURRENT_DEFAULT
```

SET request:

```text
EVENT CONTROL [9 assignment fields]
```

Valid stored gesture keys are encoder press/short/long/double for controls
`1..4`, and combo press/short/long for masks `03`, `06`, and `0C`.

Call `SAVE_PRESET` after SET.

### 8.3 GET/SET_PEDAL_ASSIGNMENT (`24`/`25`)

These commands configure the **expression/switch pedal input belonging to the
active MIDI preset**. They do not configure Pedal-mode FT1..FT4 actions.

GET request: empty.

GET success payload:

```text
01 [9 assignment fields] CURRENT_VALUE
```

SET request:

```text
[9 assignment fields]
```

Call `SAVE_PRESET` after SET.

## 9. Pedal-mode preset/page/FT API

Pedal mode has:

- 20 independent Pedal presets;
- 32 pages per preset;
- four FT actions per page.

The key used by all four commands is:

```text
PEDAL_PRESET PAGE FT
```

with preset `1..20`, page `1..32`, and FT `1..4`.

Reading or writing an arbitrary key does not activate that Pedal preset or
page. Runtime Pedal preset/page navigation remains a front-panel operation in
the current public API; `EVENT_INJECT` may simulate controls for automated
tests but should not be used as an editor's normal configuration mechanism.

### 9.1 GET_PEDAL_ACTION (`44`)

Request, 3 or 4 bytes:

```text
PEDAL_PRESET PAGE FT [ROTATE]
```

`ROTATE` is optional. Omit it or send `0` for the press assignment; send `1`
for the rotate assignment. Values other than `0`/`1` are rejected.

Success payload:

```text
PEDAL_PRESET PAGE FT [9 assignment fields] COLOR ROTATE
```

### 9.2 SET_PEDAL_ACTION (`45`)

Request, 13 or 14 payload bytes:

```text
PEDAL_PRESET PAGE FT [9 assignment fields] RESERVED
PEDAL_PRESET PAGE FT ROTATE [9 assignment fields] RESERVED
```

The 13-byte form writes press. The 14-byte form writes press when `ROTATE` is
`0` and rotate when `ROTATE` is `1`.

Press fires on FT release. Firmware forces press behavior to absolute and
forces both assignments' outputs to every available output. Rotate keeps the
supplied behavior so absolute and relative CC both work.

From 0.97.0, the trailing byte is reserved and ignored (send zero). Host
commands cannot change the grid colors. A/B/C/D always use cyan/orange/magenta/
yellow. GET_PEDAL_ACTION and VIEW_SNAPSHOT retain the existing color-position
byte for compatibility and report the fixed slot index 0/1/2/3. Old stored color
bits are ignored without rewriting or resizing the preset records.

From firmware `0.49.67` the command updates RAM and marks the pedal record
dirty. Call `SAVE_PRESET` to commit flash. A failed save reports
`FLASH_ERROR` and leaves RAM dirty; GET/SET of the already-loaded record
stay usable. Older `0.49.66` and below wrote the whole 1021-byte record on
every SET; that is not the product persist path.

For Program Change, `VALUE`/the assignment `DEFAULT` is the program number.

### 9.3 GET/SET_PEDAL_LABEL (`46`/`47`)

GET request:

```text
PEDAL_PRESET PAGE FT
```

GET success payload:

```text
PEDAL_PRESET PAGE FT LENGTH ASCII...
```

SET request uses the same structure. The label is `0..10` printable ASCII
characters. From firmware `0.49.67` SET updates RAM only; call
`SAVE_PRESET` to commit.

### 9.4 Pedal-mode rotary MIDI

Rotating encoder 1..4 in Pedal mode sends the stored rotate assignment for that
preset/page/FT (`GET/SET_PEDAL_ACTION` with `ROTATE=1`). Defaults are CC on
channel 1 with a page-scoped number:

```text
CC = ((PAGE - 1) * 4) + (ENCODER - 1)
```

Therefore page 1 defaults to CC `0..3`, page 2 to CC `4..7`, and page 32 to CC
`124..127`. Runtime values stay independent per Pedal preset/page/encoder.

## 10. Play-mode text (`34`/`35`)

Play-mode text is separate from the 8-character preset label and from
Pedal-mode FT labels. Each MIDI preset has a 10-character label for each of
five actions on each encoder.

Actions:

| Action | Value |
| --- | ---: |
| Rotate/knob | `00` |
| Press | `03` |
| Short | `05` |
| Long | `06` |
| Double | `07` |

### 10.1 GET_PLAY_LABEL (`34`)

Request:

```text
MIDI_PRESET ACTION ENCODER
```

Success payload:

```text
MIDI_PRESET ACTION ENCODER LENGTH ASCII...
```

### 10.2 SET_PLAY_LABEL (`35`)

Request:

```text
MIDI_PRESET ACTION ENCODER LENGTH ASCII...
```

The label is `0..10` printable ASCII bytes. This command writes the Play-label
record to flash immediately; `SAVE_PRESET` is not required.

Example: preset 1, encoder 1 rotate label `GAIN`, sequence `03`:

```text
F0 7D 4E 42 01 35 03 01 00 01 04 47 41 49 4E 22 F7
```

## 11. Performance SysEx templates (`36`/`37`)

Do not confuse a performance SysEx template with a NanoBridge control frame.
A template is MIDI data emitted by an assigned encoder or foot action.

Each MIDI preset owns four SysEx template slots. Each template body can contain
up to 128 7-bit bytes. Store only the bytes between `F0` and `F7`; firmware adds
the framing when it sends the template.

`VALUE_INDEX` selects one body byte that will be replaced by the current control
value. Use `7F` for no substitution.

### 11.1 GET_SYSEX_TEMPLATE (`36`)

Request:

```text
MIDI_PRESET SLOT OFFSET_LO OFFSET_HI
```

Success payload:

```text
MIDI_PRESET SLOT VALUE_INDEX TOTAL_LO TOTAL_HI
OFFSET_LO OFFSET_HI CHUNK_LENGTH CHUNK...
```

Read from offset zero and continue until the sum of returned chunk lengths
reaches `TOTAL`. The device returns at most 16 body bytes per reply.

### 11.2 SET_SYSEX_TEMPLATE (`37`)

Request:

```text
MIDI_PRESET SLOT VALUE_INDEX TOTAL_LO TOTAL_HI
OFFSET_LO OFFSET_HI CHUNK_LENGTH CHUNK...
```

Rules:

- preset is `1..20` and slot is `1..4`;
- total length is `0..128`;
- normal write chunks are at most 16 bytes;
- all template bytes must be `00..7F`;
- the first chunk must use offset zero;
- later chunks must repeat the same total length and value index;
- `OFFSET + CHUNK_LENGTH` must not exceed total length.

Clear a slot with:

```text
MIDI_PRESET SLOT 7F 00 00 00 00 00
```

Each SET chunk updates persistent template storage. Avoid unnecessary rewrites.

Assign that template as an encoder's type-5 output by setting `TEMPLATE=1`.

## 12. Custom raw-MIDI templates (`38`/`39`)

Each MIDI preset also owns four custom raw-MIDI slots of up to 128 bytes. These
templates may contain status bytes above `7F`, multiple MIDI messages, running
status, or complete `F0...F7` messages.

Because NanoBridge payload bytes must remain 7-bit clean, every raw byte is
encoded as two nibbles:

```text
RAW_BYTE = (HIGH_NIBBLE << 4) | LOW_NIBBLE
```

For example, raw `B0 4A 7F` is encoded as:

```text
0B 00 04 0A 07 0F
```

### 12.1 GET_CUSTOM_TEMPLATE (`38`)

Request:

```text
MIDI_PRESET SLOT OFFSET_LO OFFSET_HI
```

Success payload:

```text
MIDI_PRESET SLOT VALUE_INDEX TOTAL_LO TOTAL_HI
OFFSET_LO OFFSET_HI RAW_CHUNK_LENGTH NIBBLES...
```

There are two nibble bytes for every raw byte. The device returns at most eight
raw bytes per reply.

### 12.2 SET_CUSTOM_TEMPLATE (`39`)

Request uses the same header followed by nibble pairs:

```text
MIDI_PRESET SLOT VALUE_INDEX TOTAL_LO TOTAL_HI
OFFSET_LO OFFSET_HI RAW_CHUNK_LENGTH NIBBLES...
```

`RAW_CHUNK_LENGTH` counts decoded raw bytes, not nibble bytes. Every nibble must
be `00..0F`. A write chunk is at most eight raw bytes.

Clear a slot using the same zero-length form as a SysEx template. Each chunk is
written to persistent storage.

## 13. UI and test commands

### 13.1 ENTER_EDIT / EXIT_EDIT (`40`/`41`)

Request payload: empty. Success payload: empty.

These commands perform the same high-level transition as an edit-key long
gesture. EXIT saves/finishes the current physical edit session according to the
active MIDI or Pedal mode.

### 13.2 EVENT_INJECT (`30`)

General request:

```text
EVENT CONTROL REPEAT
```

`REPEAT` defaults to 1 if omitted or zero, and is capped at 16. Use this
command to test input events during development.

For event `13` (pedal test), the third byte is a value `0..127`, not a repeat
count:

```text
13 ADC_CHANNEL VALUE
```

### 13.3 GET_EVENT_LOG (`31`)

Request payload: empty.

Success payload:

```text
COUNT [SOURCE EVENT CONTROL VALUE]...
```

Up to the four most recent entries are returned. Source IDs are diagnostic and
may expand; editors should display them as raw values rather than relying on
them as a stable public enum.

## 14. Diagnostic commands

All counters below wrap at 14 bits unless stated otherwise.

### 14.1 GET_PEDAL_STATE (`26`)

Success payload, 20 bytes:

| Index | Field |
| ---: | --- |
| 0 | Logical ADC channel, normally `4` |
| 1 | AC632 ADC value scaled to 7 bits |
| 2 | Current assigned output value |
| 3 | Pedal value has been seen flag |
| 4 | Input mode: `0` EXP, `1` SWT |
| 5 | Switch state, or `7F` if unseen |
| 6 | Latest pedal ADC channel |
| 7 | Latest pedal sample scaled to 7 bits |
| 8..9 | Latest raw 8-bit sample as `u14` |
| 10..11 | Pedal sample count as `u14` |
| 12..13 | Observed raw minimum as `u14` |
| 14..15 | Observed raw maximum as `u14` |
| 16 | Acknowledged pedal mode, or `7F` if none |
| 17 | Mode acknowledgement sequence, 7-bit (`seq & 0x7F`) |
| 18..19 | Mode acknowledgement count as `u14` |

Firmware through `0.49.67` wrote the full 8-bit acknowledgement sequence
into byte 17. Values above `0x7F` could cause a timeout over USB-MIDI.
Firmware `0.49.68` and later uses the low seven bits.

### 14.2 GET_MIDI_OUT_STATE (`27`)

Success payload, 10 bytes:

```text
ENABLED AVAILABLE QUEUED:u14 WRITTEN:u14 DROPPED:u14 OPEN_FAIL:u14
```

### 14.3 GET_USB_HOST_STATE (`28`)

Success payload, 18 bytes:

```text
ENABLED MOUNTED TX_QUEUED:u14 TX_WRITTEN:u14 TX_PACKET:u14
TX_DROPPED:u14 RX_PACKET:u14 RX_QUEUED:u14 RX_DROPPED:u14 SEND_FAIL:u14
```

### 14.4 GET_LED_STATE (`29`)

Success payload, 8 bytes:

```text
POLICY_MODE REQUEST_MASK GPIO_MASK HELD_MASK EDIT_TARGET PRESET_POSITION
PRESET_LED_MODE PRESET_LED_MASK
```

### 14.5 GET_EDIT_KEY_STATE (`2A`)

Success payload, 18 bytes:

```text
ENABLED RAW STABLE LAST_RAW DEBOUNCE CLICKS LONG_SENT PRESS_TICKS:u14
SHORT_TIMER SHORT_COUNT:u14 LONG_COUNT:u14 DOUBLE_COUNT:u14 EDGE_COUNT:u14
```

### 14.6 GET_BLE_MIDI_STATE (`2B`)

NanoBridge returns the 28-byte base snapshot:

```text
ENABLED CONNECTED NOTIFY HANDLE:u14 TX_USED:u14 RX_USED:u14
TX_QUEUED:u14 TX_WRITTEN:u14 TX_PACKET:u14 TX_DROPPED:u14
TX_FAIL:u14 TX_UNAVAILABLE_DROP:u14 RX_PACKET:u14 RX_QUEUED:u14
RX_DROPPED:u14 LINK_EVENTS
```

### 14.7 GET_DISPLAY_STATE (`2C`)

NanoBridge returns the 22-byte role snapshot:

```text
READY UI_MODE DIRTY_MASK PRESSED_MASK
ROW_VALUE[4] ENCODER_VALUE[4] RENDERED_UI_VALUE[4]
DRAWING LAYOUT_VERSION ROW_ROLE[4]
```

Current layout version is `5`. Row-role IDs include:

| Role | ID | Role | ID |
| --- | ---: | --- | ---: |
| None | `00` | Encoder 1..4 | `01..04` |
| Edit subject | `10` | Edit type | `11` |
| Edit data1 | `12` | Edit value | `13` |
| Edit channel | `14` | Edit behavior | `16` |
| Edit min/max | `17`/`18` | Edit template | `19` |
| Edit page/action | `1A`/`1B` | Edit message/detail | `1C..1E` |
| Preset selected/active | `30`/`31` | Preset slot/sequence | `32`/`33` |
| BT toggle | `34` | EXP/SWT mode | `35` |
| Pedal calibration | `36` | Pedal FT/color | `37`/`38` |
| Label position/character | `39`/`3A` | Play message/value/label | `40..42` |

### 14.8 GET_ACTION_STATE (`32`)

Success payload, 12 bytes:

```text
ACTIVE_PRESET NOTE_TOGGLE ENC_PRESS_TOGGLE ENC_SHORT_TOGGLE
ENC_LONG_TOGGLE ENC_DOUBLE_TOGGLE COMBO_PRESS_TOGGLE
COMBO_SHORT_TOGGLE COMBO_LONG_TOGGLE ENC_PRESS_MOMENTARY
ENC_LONG_MOMENTARY COMBO_LONG_MOMENTARY
```

The masks use bits 0..3 for encoders; combo masks use the three combo slots.

### 14.9 GET_PROTOCOL_STATE (`33`)

Success payload, 19 bytes:

```text
LAYOUT VALID_RX:u14 REPLIES:u14 OK:u14 BAD_CHECKSUM:u14
BAD_LENGTH:u14 BAD_VALUE:u14 UNSUPPORTED:u14 BUSY:u14 FLASH_ERROR:u14
```

Current layout is `1`.

## 15. Reset, factory reset, and firmware-update detach

### 15.1 RESET (`7F`)

Request payload must be printable ASCII `RESET`:

```text
52 45 53 45 54
```

The device replies first, then reboots after approximately 120 ms.

### 15.2 FACTORY_RESET (`16`)

Request payload must be printable ASCII `FACTORY`:

```text
46 41 43 54 4F 52 59
```

The operation rewrites MIDI presets, Play labels, SysEx templates, custom MIDI
templates, Pedal-mode banks, mode state, and pedal calibration defaults. The
reply is delayed until the flash operation completes.

Success payload is one byte containing the restored MIDI-preset count (`20`).

### 15.3 DETACH_UPDATE (`7E`)

Request payload: empty.

The device replies, closes runtime USB, and reboots into the JL mask/UBOOT wired
updater. The host must expect the runtime MIDI ports to disappear.

See [Recovery and updates](docs/recovery.md) for the community firmware
update workflow.

## 16. Persistence rules

The word `SET` does not always mean that flash was written. Use this table to
avoid settings that disappear after reboot and to avoid unnecessary flash wear.

| Operation | Takes effect | Persistence action |
| --- | --- | --- |
| `SET_PRESET`, active encoder/gesture/expression setters | Immediately in RAM | Call `SAVE_PRESET` |
| `SET_PRESET_LABEL` | Immediately in RAM | Call `SAVE_PRESET` |
| `LOAD_PRESET` | After delayed successful reply | Active MIDI preset header is saved automatically |
| `SET_PLAY_LABEL` | Immediately | Written automatically |
| `SET_SYSEX_TEMPLATE` / `SET_CUSTOM_TEMPLATE` | Per chunk | Written automatically per chunk |
| `SET_FOOT_MODE` | Immediately | Mode header saved automatically |
| `SET_PEDAL_ACTION` / `SET_PEDAL_LABEL` | Immediately in RAM | Call `SAVE_PRESET` |
| `FACTORY_RESET` | After delayed reply | Entire configuration rewritten |

Recommended editor transaction for a MIDI preset:

1. Call `GET_INFO` and verify protocol compatibility.
2. Read the existing preset items that will be edited.
3. Send the required `SET_PRESET` and label requests, waiting for every reply.
4. Read the items back and compare them with the requested values.
5. Send one `SAVE_PRESET` after the batch.
6. Wait for its delayed `OK` reply.
7. Optionally reset the device and read the preset again to verify persistence.

Recommended editor transaction for a pedal page:

1. `SET_PEDAL_ACTION` / `SET_PEDAL_LABEL` for the FTs being edited.
2. If the expression jack changed, `SET_PRESET` section `PEDAL`.
3. One `SAVE_PRESET`.
4. Wait for its delayed `OK` reply. Do not retry `SET`. Retry `SAVE_PRESET` only.

## 17. Minimal Python implementation

This example uses `mido` to send a read-only PING. Pass the names of the
Nexfoot NanoBridge input and output as arguments. Install `mido` and
`python-rtmidi` to run it.

```python
import argparse
import time
import mido

PREFIX = [0x7D, ord("N"), ord("B")]
VERSION = 0x01

def checksum(data):
    return (0x80 - (sum(x & 0x7F for x in data) & 0x7F)) & 0x7F

def build_request(command, sequence, payload=()):
    body = PREFIX + [VERSION, command & 0x7F, sequence & 0x7F]
    body += [x & 0x7F for x in payload]
    body.append(checksum(body[3:]))
    return mido.Message("sysex", data=body)

def parse_reply(message):
    if message.type != "sysex":
        return None
    body = list(message.data)  # mido removes F0 and F7
    if len(body) < 8 or body[:3] != PREFIX or body[3] != VERSION:
        return None
    if checksum(body[3:-1]) != body[-1]:
        return None
    return {
        "command": body[4],
        "sequence": body[5],
        "status": body[6],
        "payload": body[7:-1],
    }

def request(in_port, out_port, command, sequence, payload=(), timeout=2.0):
    out_port.send(build_request(command, sequence, payload))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for message in in_port.iter_pending():
            reply = parse_reply(message)
            if reply and reply["command"] == command and reply["sequence"] == sequence:
                if reply["status"] != 0:
                    raise RuntimeError(f"device status {reply['status']}")
                return reply["payload"]
        time.sleep(0.005)
    raise TimeoutError("NanoBridge reply timeout")

parser = argparse.ArgumentParser(description="Ping a Nexfoot NanoBridge port pair.")
parser.add_argument("input_port")
parser.add_argument("output_port")
ports = parser.parse_args()

with mido.open_input(ports.input_port) as inp, mido.open_output(ports.output_port) as out:
    sequence = 1
    pong = request(inp, out, 0x01, sequence)
    print(bytes(pong).decode("ascii"))
```

Production applications should additionally:

- enumerate by VID/PID/serial and logical port order instead of display name alone;
- use a new nonzero sequence for every outstanding request;
- allow only one flash-writing request at a time;
- use a longer timeout for save, load, and factory reset;
- stop waiting if USB disconnects after RESET or DETACH_UPDATE;
- read values back before reporting a configuration write as successful.

## Preset/group synchronization and independent pedal preset names (2026-09-10)

These additions ship in 0.97.0. Probe the new command and handle UNSUPPORTED
on earlier firmware; the earlier packaged 0.96.0 build does not implement them. All command bytes below are hexadecimal.

| Command | Request payload | Reply payload |
| --- | --- | --- |
| `4A SELECT_VIEW` | `bank preset group` | Existing 12-byte GET_STATE layout |
| `4B GET_PEDAL_PRESET_NAME` | `preset` | `preset length ASCII...` |
| `4C SET_PEDAL_PRESET_NAME` | `preset length ASCII...` | Same as GET |
| `4D USB_STATE_SUBSCRIBE` | `enabled` (0/1) | Empty ACK |

SELECT_VIEW uses bank 0=MIDI, 1=PEDAL, preset 1..20. MIDI requires group 0;
PEDAL requires group 1..32. It updates both active and selected indices, the
foot mode and the device display. PLAY stays PLAY; other modes return to
PERFORM. Selection is runtime navigation, not a Save or a request to transmit
performance MIDI. Save/exit an edit first: pending pedal actions, pedal preset
names, expression edits or an active dirty MIDI edit session return BUSY without
switching. A failed pedal-record load returns FLASH_ERROR. Wait for the reply
and use its actual state to update the host, rather than changing host widgets
optimistically. This command does not promise to persist selection across reboot.

Names are independent of MIDI preset labels: 20 pedal presets, each with 0..8
printable ASCII characters. Empty names are valid (host may display Pedal N).
SET updates RAM; SAVE_PRESET commits them. The physical explicit pedal Save
also commits names. Exiting pedal edit or changing foot mode discards unsaved
names. Factory reset clears them. New journal record 249 stores a versioned,
checksummed 166-byte name table; existing preset record sizes are unchanged.
GET_FOOT_MODE's dirty flag includes pending names and expression edits.

VIEW_SNAPSHOT (48) now returns the appropriate bank's name in its existing
8-byte preset-label field. PRESET_LABELS (49) accepts optional third byte bank:
`first count bank`. Legacy `first count` still lists MIDI labels. Replies keep
`first count` followed by count fixed-width 8-byte labels; count remains 1..12.

USB notifications are opt-in to preserve compatibility with older request/reply
clients. Subscribe with 4D/01 and keep the same control-port input open. Send a
PING or other valid request at least every 5 seconds; subscription expires after
15 seconds without USB host activity. Unsubscribe with 4D/00. BLE retains its
existing editor activity/notification behavior; 4D is USB-only.

Both transports send event command 60, sequence 0, status OK, with the existing
15-byte payload: change-source, revision low/high (14-bit), then GET_STATE[12].
Check for state changes at approximately 100 ms intervals. Each transport tracks
its own last successfully queued event, so a full USB queue does not suppress
BLE or lose the USB retry. Intermediate rapid selections may be coalesced; the
latest state is authoritative. Legacy change-source/revision are advisory:
physical navigation is also detected by comparing state fields, without requiring
a revision increment. Re-subscription sends an initial state.

Host integration: keep unsolicited events separate from command responses;
match responses by command plus nonzero sequence. Apply events without echoing
a SELECT_VIEW back. When a name/content revision changes, re-fetch the visible
snapshot and the relevant bank's name list. New hosts should use SELECT_VIEW for
both MIDI preset and PEDAL preset/group changes instead of simulated combos.

## DAW mode (2026-11-08)

DAW mode lets a DAW script take over the pedal: the encoders, edit key and
expression jack send a fixed control map, and the DAW draws its own screen
and LEDs. It is RAM only. Nothing in this section writes flash. Firmware
before this addition answers `4E`/`4F` with `UNSUPPORTED` (`04`). The
[DAW SDK](daw/README.md) is under `daw/`. The host scripts live in the
separate `nexfoot-ableton` and `nexfoot-fl-studio` projects.

| Command | Request payload | Reply payload |
| --- | --- | --- |
| `4E DAW_MODE` | `enable` (0/1) `[host]` | `active pending` |
| `4F DAW_UPDATE` | list of ops (below) | `active`; no reply when `SEQ` is `00` |

`host` is informational: `0` generic, `1` Ableton Live, `2` FL Studio.

### Controller app installation (community 1.0.1)

USB command `52` stores one independent controller package for host 1 (Live)
or 2 (FL Studio). Query with the single host byte. A successful reply carries
the current package, or a single zero for an empty slot. Install by sending
the 21 raw bytes of a v1 `.nfapp` as the payload. The reply carries the record
read from the flash journal. Query again to verify. Older firmware reports
UNSUPPORTED. Each host has its own record; reinstalling preserves presets and
the other host. A new session loads the installed mapping.

| Bytes | Meaning |
| --- | --- |
| 0–3 | Schema (1), host ID, positive revision, show waiting page on disconnect (0/1) |
| 4–7 | Base CC for encoder rotate, press/release, tap, hold; each group uses four adjacent CCs |
| 8 | Expression CC |
| 9–20 | Printable ASCII app title, padded to 12 bytes with spaces |

On 1.0.1 the Live and FL scripts must send DAW_MODE with their host ID before
the pedal accepts updates after a reboot. A generic SDK host can still open a
session with host zero. A waiting app displays its installed title and
**Waiting for DAW**.

### Session rules

- `DAW_MODE 01` resets the DAW view to defaults and enters DAW mode at once,
  or as soon as an edit session ends (`pending` = 1 in the meantime).
- `DAW_MODE 00` ends the host session. Starting in 0.99.5, an active USB
  Ableton session shows **Waiting for Live** on disconnect or after 10 s
  without host traffic. The reply reports `active=0` while waiting. The
  pedal suppresses DAW and normal performance input on the waiting page.
  A new `DAW_MODE 01` resets the screen and announces the connected state
  and capabilities again. Double-press Edit to leave the waiting page.
  Other host sessions return to the normal Perform/Play screen.
- Hosts should send periodic `DAW_UPDATE` messages while connected.
  The waiting page accepts a new `DAW_MODE 01` to reconnect.
- On firmware before 1.0.1, a `DAW_UPDATE` from a host that never sent `DAW_MODE 01` (for example after
  the pedal rebooted mid-session) also arms DAW mode.
- On the pedal, edit key double press cycles the foot mode MIDI → PEDAL →
  DAW → MIDI. Leaving DAW mode with the edit key is sticky until the host
  goes quiet or sends `DAW_MODE 01` again. Re-entering requires a live host.
- `ENTER_EDIT` (`40`) leaves DAW mode. The pedal re-enters it after the edit
  session ends if the host is still active.
- The control map is sent only to the transport (USB or BLE) that carried the
  most recent `DAW_MODE 01`, never to 5-pin MIDI OUT or USB-A host.

Send `DAW_UPDATE` with `SEQ 00` while the DAW runs: the firmware applies it
without replying, keeping the control port quiet. Keep each frame within the
64-byte NanoBridge limit (55 payload bytes) and do not send more than about
20 frames per second; the pedal buffers two pending requests and drops more.

### Native cutoff plot

While entering USB DAW mode, a device with hardware cutoff rendering advertises
CC 125 value 127 on the performance port. The host can then send a fire-and-forget
DAW_DRAW (`50`, sequence zero) operation `15 VALUE`, where `VALUE` is the
normalized cutoff position, 0 to 127. The device composites the curve and grid
in the fixed Ableton plot pane at x 108, y 36, width 204, height 130. Hosts use
the rectangle fallback when the capability is absent. The plot illustrates
knob position. Firmware with the smoothed dial renderer also advertises CC 124
value 127. DAW_DRAW operation `16 CX CY R VALUE ACCENT TRACK BACKGROUND FACE
POINTER` composites one dial directly on the LCD. `CX` and `CY` are 14-bit
coordinates, `R` is 18 to 48 pixels, and `VALUE` is 0 to 127. The five colors use
three seven-bit-safe bytes apiece (R5, G6, B5). Hosts use the ARC fallback when
CC 124 is absent.

CC 118 value 127 announces the compact hardware plot. DAW_DRAW `19 VALUE`
composites an antialiased cutoff curve at x 108, y 32, width 204, height 104.
The shorter plot leaves room for the full-size encoder strip. Use the compact
primitive fallback when CC 118 is absent. Operation `15` retains its legacy
geometry.

### Retained display widgets

In USB DAW mode, performance CC 119 value 127 announces four retained widget slots.
DAW_DRAW `17 SLOT KIND X Y W H VALUE ACCENT TRACK BACKGROUND FACE POINTER` defines or
replaces a slot. Slots are 0–3. Kind 1 draws a knob, 2 a horizontal slider and 3 a
vertical slider. Geometry uses four unsigned 14-bit fields (high byte first, seven
bits per byte). Colors use the same RGB565 encoding as DIAL. `18 SLOT VALUE`
updates the slot with a 0–127 value. The display tick paints the latest pending
position. CLEAR and leaving DAW mode discard all definitions. Rebuild the page on
CC 126. See the [widget examples](docs/widgets.md) for dimensions and ownership.

### DAW_UPDATE ops

Ops follow each other in one payload and are applied in order. An unknown op
returns `BAD_VALUE` and a truncated op returns `BAD_LENGTH`. Ops before the
bad one have already been applied.

| Op | Bytes | Meaning |
| --- | --- | --- |
| `01` text | `01 FIELD LEN ASCII...` | Set a text field (printable ASCII, longer text is cut) |
| `02` card | `02 CARD METER R5 G6 B5 FLAGS` | Card `1..4` meter `0..7F`, RGB565 colour, flags |
| `03` LEDs | `03 MASK` | Front LEDs, bits 0..3 = LED 1..4 |
| `04` accent | `04 R5 G6 B5` | Top-bar accent colour |

Text fields:

| Field | Where | Max chars |
| ---: | --- | ---: |
| `00` | Title, top bar left | 20 |
| `01` | Status, top bar right | 8 |
| `02..05` | Card 1..4 label | 10 |
| `06..09` | Card 1..4 value | 8 |

Card flags: `01` lit (card background takes the card colour, value turns
white), `02` show the meter, `04` bipolar meter (centre at `40`, for pan).
Colours are RGB565 split into 7-bit-safe parts: `R5` = red `0..1F`,
`G6` = green `0..3F`, `B5` = blue `0..1F`.

Text uses the display's 5x7 font: `A..Z`, `a..z`, `0..9`, space and
`- / . : + % # < > ! ( ) _ = * ' , ? | [ ] &`. Values fit best at 4
characters (largest size), then 6, then 8. Firmware 0.99.6 and newer draws
DAW text, here and in DAW_DRAW `12 TEXT`, with lowercase letters. Each glyph
then also paints an eighth row below the cell for the descenders of `g j p q y`,
so leave one row of `SCALE` pixels under a line of text. Older firmware shows
lowercase as capitals at the same width, so hosts can always send mixed case.

Example, title `LIVE`, card 1 meter half green and lit, LEDs 1 and 3, no reply:

```text
F0 7D 4E 42 01 4F 00 01 00 04 4C 49 56 45 02 01 40 00 3F 00 03 03 05 6E F7
```

### Control map in DAW mode

All messages are CC on MIDI channel 16 (`BF`) on the performance port
(cable 1 over USB).

| CC | Control | Value |
| ---: | --- | --- |
| `10..13` (16..19) | Encoder 1..4 rotate | Relative: `01..3F` = +1..+63, `7F..41` = -1..-63 |
| `14..17` (20..23) | Encoder 1..4 press state | `7F` down, `00` up |
| `18..1B` (24..27) | Encoder 1..4 short press | `7F` on release |
| `1C..1F` (28..31) | Encoder 1..4 long press | `7F` after 0.9 s, still held |
| `20..22` (32..34) | Combo 1+2, 2+3, 3+4 short | `7F` |
| `23..25` (35..37) | Combo 1+2, 2+3, 3+4 long | `7F` |
| `2C` (44) | Expression pedal / footswitch | `00..7F` / `00`, `7F` |
| `2E` (46) | Edit key short | `7F` |
| `2F` (47) | Edit key long | `7F` |
| `7F` (127) | DAW mode state | `7F` entered, `00` left |

A combo also produces press/release for both encoders but no short or long
press for them. Rotation acceleration is folded into larger relative steps.
When the pedal sends CC 127 = `7F` (entered), resend the whole view: the pedal
may have rebooted or reset it.
