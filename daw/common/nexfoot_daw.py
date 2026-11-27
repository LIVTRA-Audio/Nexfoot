# Copyright 2026 Nexfoot contributors. Licensed under the Apache License 2.0.
"""Nexfoot DAW-mode protocol: control map and display frames.

This module has no dependencies so it runs inside Ableton Live's and
FL Studio's embedded Python. The canonical copy lives in ``daw/common``.
The separate Ableton and FL Studio apps bundle it when building their
installer ZIPs.

Wire format: NanoBridge SysEx on the pedal's control port (USB-MIDI cable 0,
"Nexfoot MIDI Port 1"). See ``NEXFOOT_SYSEX_API.md``, section "DAW mode".
"""

import math
import time

PREFIX = (0x7D, 0x4E, 0x42)  # development ID, "NB"
VERSION = 0x01

CMD_DAW_MODE = 0x4E
CMD_DAW_UPDATE = 0x4F

HOST_GENERIC = 0
HOST_ABLETON = 1
HOST_FL_STUDIO = 2

OP_TEXT = 0x01
OP_CARD = 0x02
OP_LEDS = 0x03
OP_ACCENT = 0x04

FIELD_TITLE = 0
FIELD_STATUS = 1
FIELD_LABEL = 2  # + card index 0..3
FIELD_VALUE = 6  # + card index 0..3

TITLE_MAX = 20
STATUS_MAX = 8
LABEL_MAX = 10
VALUE_MAX = 8

FLAG_LIT = 0x01
FLAG_METER = 0x02
FLAG_BIPOLAR = 0x04

# Firmware accepts at most 64 bytes per NanoBridge frame including F0/F7.
FRAME_LIMIT = 64
FRAME_OVERHEAD = 9  # F0 7D 4E 42 VER CMD SEQ ... SUM F7
PAYLOAD_LIMIT = FRAME_LIMIT - FRAME_OVERHEAD
# One full-speed 64-byte USB-MIDI packet carries up to 48 SysEx bytes.
USB_SAFE_PAYLOAD = 48 - FRAME_OVERHEAD

# ---------------------------------------------------------------------------
# Control map sent by the pedal in DAW mode: CC messages on MIDI channel 16.
# ---------------------------------------------------------------------------
STATUS_CC = 0xBF  # CC, channel 16
CC_ROTATE = 16       # 16..19 encoder 1..4, relative
CC_PRESS = 20        # 20..23 encoder 1..4 down (127) / up (0)
CC_SHORT = 24        # 24..27 encoder 1..4 short press
CC_LONG = 28         # 28..31 encoder 1..4 long press (while held)
CC_COMBO_SHORT = 32  # 32..34 combo 1+2, 2+3, 3+4
CC_COMBO_LONG = 35   # 35..37 combo 1+2, 2+3, 3+4
CC_EXP = 44          # expression pedal 0..127 / switch 0 or 127
CC_EDIT_SHORT = 46
CC_EDIT_LONG = 47
CC_COMPACT_CUTOFF = 118
CC_WIDGETS = 119
CC_DIAL = 124        # 127: firmware can draw a smoothed dial
CC_CUTOFF = 125      # 127: firmware can composite the cutoff pane
CC_REDRAW = 126      # pedal dropped drawing frames: redraw everything
CC_STATE = 127       # 127 = pedal entered DAW mode, 0 = left


def capture(send):
    """Wrap ``send`` so every frame is also appended to the file named by the
    NEXFOOT_CAPTURE environment variable. Replay a capture with
    ``daw/tools/nexfoot_screen.py replay``. Without the variable, or where the
    DAW's Python has no ``os`` module, ``send`` is returned unchanged."""
    try:
        import os
        path = os.environ.get("NEXFOOT_CAPTURE")
    except Exception:
        path = None
    if not path:
        return send
    start = time.time()

    def recorded(data):
        send(data)
        try:
            with open(path, "a") as out:
                out.write("%.3f %s\n" % (time.time() - start, " ".join("%02X" % b for b in data)))
        except Exception:
            pass
    return recorded


def relative_delta(value):
    """Decode a relative rotate value: 1..63 is +n, 65..127 is -(128-n)."""
    value &= 0x7F
    if value == 0 or value == 64:
        return 0
    return value if value < 64 else value - 128


def decode_cc(cc, value):
    """Map an incoming DAW-mode CC to (kind, index, value).

    kind is one of: rotate, press, short, long, combo_short, combo_long,
    exp, edit_short, edit_long, native_dial, native_cutoff, compact_cutoff,
    redraw, state; or None otherwise.
    For rotate, value is the signed delta. For combos, index 0..2 is
    1+2, 2+3, 3+4.
    """
    if CC_ROTATE <= cc < CC_ROTATE + 4:
        return "rotate", cc - CC_ROTATE, relative_delta(value)
    if CC_PRESS <= cc < CC_PRESS + 4:
        return "press", cc - CC_PRESS, 1 if value else 0
    if CC_SHORT <= cc < CC_SHORT + 4:
        return "short", cc - CC_SHORT, value
    if CC_LONG <= cc < CC_LONG + 4:
        return "long", cc - CC_LONG, value
    if CC_COMBO_SHORT <= cc < CC_COMBO_SHORT + 3:
        return "combo_short", cc - CC_COMBO_SHORT, value
    if CC_COMBO_LONG <= cc < CC_COMBO_LONG + 3:
        return "combo_long", cc - CC_COMBO_LONG, value
    if cc == CC_EXP:
        return "exp", 0, value
    if cc == CC_EDIT_SHORT:
        return "edit_short", 0, value
    if cc == CC_EDIT_LONG:
        return "edit_long", 0, value
    if cc == CC_WIDGETS:
        return "widgets", 0, value
    if cc == CC_COMPACT_CUTOFF:
        return "compact_cutoff", 0, value
    if cc == CC_DIAL:
        return "native_dial", 0, value
    if cc == CC_CUTOFF:
        return "native_cutoff", 0, value
    if cc == CC_REDRAW:
        return "redraw", 0, value
    if cc == CC_STATE:
        return "state", 0, 1 if value else 0
    return None, 0, value


# ---------------------------------------------------------------------------
# Frame building
# ---------------------------------------------------------------------------
def checksum(data):
    return (0x80 - (sum(b & 0x7F for b in data) & 0x7F)) & 0x7F


def frame(command, seq, payload=()):
    """Return a complete NanoBridge SysEx frame (tuple of ints, F0..F7)."""
    body = [VERSION, command & 0x7F, seq & 0x7F] + [b & 0x7F for b in payload]
    return tuple([0xF0] + list(PREFIX) + body + [checksum(body), 0xF7])


def mode_frame(enable, host_id=HOST_GENERIC, seq=1):
    return frame(CMD_DAW_MODE, seq, (1 if enable else 0, host_id))


def ascii7(text, limit):
    out = []
    for ch in str(text)[:limit]:
        code = ord(ch)
        out.append(code if 0x20 <= code <= 0x7E else 0x20)
    return out


def rgb565_parts(rgb):
    """0xRRGGBB -> (r5, g6, b5)."""
    r = (rgb >> 16) & 0xFF
    g = (rgb >> 8) & 0xFF
    b = rgb & 0xFF
    return r >> 3, g >> 2, b >> 3


def op_text(field, text, limit):
    data = ascii7(text, limit)
    return [OP_TEXT, field, len(data)] + data


def op_card(card_index, meter, rgb, flags):
    r, g, b = rgb565_parts(rgb)
    meter = max(0, min(127, int(meter)))
    return [OP_CARD, card_index + 1, meter, r, g, b, flags & 0x7F]


def op_leds(mask):
    return [OP_LEDS, mask & 0x0F]


def op_accent(rgb):
    r, g, b = rgb565_parts(rgb)
    return [OP_ACCENT, r, g, b]


def pack_ops(ops, max_frames=None, payload_limit=PAYLOAD_LIMIT):
    """Pack op lists into DAW_UPDATE frames (SEQ 0, no reply).

    Returns (frames, packed_count): the frames and how many leading ops they
    carry. Ops are never split across frames; ops past ``max_frames`` stay
    unsent for the caller to retry.
    """
    frames = []
    current = []
    current_count = 0
    packed = 0
    for op in ops:
        if len(op) > payload_limit:
            raise ValueError("op too large for one frame")
        if len(current) + len(op) > payload_limit:
            frames.append(frame(CMD_DAW_UPDATE, 0, current))
            packed += current_count
            current = []
            current_count = 0
            if max_frames is not None and len(frames) >= max_frames:
                return frames, packed
        current.extend(op)
        current_count += 1
    if current and (max_frames is None or len(frames) < max_frames):
        frames.append(frame(CMD_DAW_UPDATE, 0, current))
        packed += current_count
    return frames, packed


def fit(text, limit):
    """Squeeze text for a small field: drop spaces first, then truncate."""
    text = str(text).strip()
    if len(text) > limit:
        text = text.replace(" ", "")
    return text[:limit]


# ---------------------------------------------------------------------------
# View model with diffing, rate limiting and self-healing resend
# ---------------------------------------------------------------------------
class Card(object):
    __slots__ = ("label", "value", "meter", "color", "flags")

    def __init__(self):
        self.label = ""
        self.value = ""
        self.meter = 0
        self.color = 0x808080
        self.flags = 0


class View(object):
    """Desired pedal screen state. Call ``flush`` regularly to send changes.

    Only changed fields are sent. Every ``refresh_every`` flushes one part of
    the screen is resent anyway, so a dropped frame heals itself and the pedal
    keeps seeing host traffic (its DAW-mode watchdog is 10 s).
    """

    def __init__(self, send, max_frames_per_flush=2, refresh_every=10, payload_limit=PAYLOAD_LIMIT):
        self._send = send
        self._max_frames = max_frames_per_flush
        self._payload_limit = payload_limit
        self._refresh_every = refresh_every
        self._tick = 0
        self._refresh_part = 0
        self.title = ""
        self.status = ""
        self.accent = 0x00FFFF
        self.leds = 0
        self.cards = [Card() for _ in range(4)]
        self._sent = None
        self._carry = []
        self.invalidate()

    def invalidate(self):
        """Forget what the pedal shows; the next flushes resend everything."""
        self._sent = {}
        self._carry = []

    def _desired(self):
        d = {
            ("text", FIELD_TITLE): fit(self.title, TITLE_MAX),
            ("text", FIELD_STATUS): fit(self.status, STATUS_MAX),
            ("accent",): self.accent & 0xFFFFFF,
            ("leds",): self.leds & 0x0F,
        }
        for i, card in enumerate(self.cards):
            d[("text", FIELD_LABEL + i)] = fit(card.label, LABEL_MAX)
            d[("text", FIELD_VALUE + i)] = fit(card.value, VALUE_MAX)
            d[("card", i)] = (max(0, min(127, int(card.meter))),
                              card.color & 0xFFFFFF, card.flags & 0x7F)
        return d

    @staticmethod
    def _op(key, value):
        if key[0] == "text":
            limit = {FIELD_TITLE: TITLE_MAX, FIELD_STATUS: STATUS_MAX}.get(
                key[1], LABEL_MAX if key[1] < FIELD_VALUE else VALUE_MAX)
            return op_text(key[1], value, limit)
        if key[0] == "card":
            meter, color, flags = value
            return op_card(key[1], meter, color, flags)
        if key[0] == "accent":
            return op_accent(value)
        return op_leds(value)

    def _refresh_keys(self):
        part = self._refresh_part
        self._refresh_part = (self._refresh_part + 1) % 5
        if part == 4:
            return [("text", FIELD_TITLE), ("text", FIELD_STATUS), ("accent",), ("leds",)]
        return [("text", FIELD_LABEL + part), ("text", FIELD_VALUE + part), ("card", part)]

    def flush(self, max_frames=None):
        """Send pending changes. Returns the number of frames sent."""
        if max_frames is None:
            max_frames = self._max_frames
        desired = self._desired()
        self._tick += 1
        if self._refresh_every and self._tick % self._refresh_every == 0:
            for key in self._refresh_keys():
                self._sent.pop(key, None)
        # Title/status/accent first so a page change reads quickly.
        order = sorted(desired.keys(), key=lambda k: (k[0] != "text" or k[1] > 1, k))
        ops = []
        changed = []
        for key in order:
            if self._sent.get(key) != desired[key]:
                ops.append(self._op(key, desired[key]))
                changed.append(key)
        if not ops:
            return 0
        frames, packed = pack_ops(ops, max_frames, self._payload_limit)
        for key in changed[:packed]:
            self._sent[key] = desired[key]
        for f in frames:
            self._send(f)
        return len(frames)


# ---------------------------------------------------------------------------
# Drawing: DAW_DRAW (0x50) primitives and flicker-free widgets
# ---------------------------------------------------------------------------
# The pedal draws exactly what these ops say, in order, on its 320x176 screen
# with its own 5x7 font. Widgets remember what they drew and only send the
# part that changed, so a turning knob costs a few bytes, not a full redraw.
CMD_DAW_DRAW = 0x50
OP_CLEAR = 0x10
OP_RECT = 0x11
OP_TEXTXY = 0x12
OP_ARC = 0x13
OP_WAVE = 0x14
OP_CUTOFF = 0x15
OP_DIAL = 0x16
OP_WIDGET = 0x17
OP_VALUE = 0x18
OP_COMPACT_CUTOFF = 0x19
OP_PADS = 0x05      # in DAW_UPDATE: MASK CH N1..N4, switches play notes

SCREEN_W = 320
SCREEN_H = 176
WAVE_CHUNK = 40     # columns per WAVE op, keeps every op inside one frame

BLACK = 0x000000
WHITE = 0xFFFFFF


def _u14(v):
    v = max(0, min(0x3FFF, int(v)))
    return [(v >> 7) & 0x7F, v & 0x7F]


def _rgb(c):
    return list(rgb565_parts(c & 0xFFFFFF))


def text_width(text, scale):
    """Pixel width of text in the pedal font: 5 px glyphs, 1 px gap, times scale."""
    n = len(str(text))
    return n * 6 * scale - scale if n else 0


def fit_width(text, width, scale):
    """Cut text to fit width at scale."""
    text = str(text)
    while text and text_width(text, scale) > width:
        text = text[:-1]
    return text


def abbreviate(name, limit=6):
    """Hardware-style short name: drop spaces, then inner vowels, then cut."""
    name = str(name).upper()
    if len(name) <= limit:
        return name
    name = name.replace(" ", "").replace("/", "")
    if len(name) > limit:
        name = name[0] + "".join(c for c in name[1:] if c not in "AEIOU")
    return name[:limit]


def draw_clear(color=BLACK):
    return [OP_CLEAR] + _rgb(color)


def draw_rect(x, y, w, h, color):
    return [OP_RECT] + _u14(x) + _u14(y) + _u14(w) + _u14(h) + _rgb(color)


def draw_text(x, y, scale, text, fg=WHITE, bg=BLACK):
    """Text in the pedal font. Firmware 0.99.6 draws lowercase letters; older
    firmware shows them as capitals at the same width."""
    data = ascii7(str(text), 40)
    return [OP_TEXTXY] + _u14(x) + _u14(y) + [scale & 0x7F] + _rgb(fg) + _rgb(bg) + [len(data)] + data


def draw_arc(cx, cy, r, thick, a0, a1, color):
    """Arc on a 270 degree dial: a0..a1 are 0..127 along the sweep."""
    a0 = max(0, min(127, int(a0)))
    a1 = max(0, min(127, int(a1)))
    return [OP_ARC] + _u14(cx) + _u14(cy) + [r & 0x7F, thick & 0x7F, a0, a1] + _rgb(color)


def draw_dial(cx, cy, radius, value, accent, track, background, face, pointer):
    return ([OP_DIAL] + _u14(cx) + _u14(cy) + [radius, value] +
            _rgb(accent) + _rgb(track) + _rgb(background) + _rgb(face) + _rgb(pointer))


def _widget_value(slot, value):
    if slot not in range(4) or value not in range(128):
        raise ValueError("widget slot must be 0..3 and value must be 0..127")
    return [slot, value]


def set_widget_value(slot, value):
    return [OP_VALUE] + _widget_value(slot, value)


def _define_widget(slot, kind, x, y, width, height, value, colors):
    _widget_value(slot, value)
    if x < 0 or y < 0 or x + width > SCREEN_W or y + height > SCREEN_H:
        raise ValueError("widget must fit inside the display")
    geometry = _u14(x) + _u14(y) + _u14(width) + _u14(height)
    return [OP_WIDGET, slot, kind] + geometry + [value] + [part for color in colors for part in _rgb(color)]


def define_knob(slot, cx, cy, radius, value, accent, track, background, face, pointer):
    if radius not in range(18, 49):
        raise ValueError("knob radius must be 18..48")
    size = 2 * radius + 1
    return _define_widget(slot, 1, cx - radius, cy - radius, size, size, value,
                          (accent, track, background, face, pointer))


def define_slider(slot, x, y, width, height, value, accent, track, background, thumb, vertical=False):
    if min(width, height) < 6:
        raise ValueError("slider dimensions must be at least 6 pixels")
    return _define_widget(slot, 3 if vertical else 2, x, y, width, height, value,
                          (accent, track, background, background, thumb))


def draw_wave(x, ymid, half, fg, bg, peaks):
    peaks = [max(0, min(127, int(p))) for p in peaks][:WAVE_CHUNK]
    return [OP_WAVE] + _u14(x) + _u14(ymid) + [half & 0x7F] + _rgb(fg) + _rgb(bg) + [len(peaks)] + peaks


def op_pads(mask, channel, notes):
    notes = (list(notes) + [0, 0, 0, 0])[:4]
    return [OP_PADS, mask & 0x0F, channel & 0x0F] + [n & 0x7F for n in notes]


class Canvas(object):
    """Drawing ops sent as DAW_DRAW frames (SEQ 0, no reply), plus the widgets
    drawn on them.

    Widgets are retained: ``set()`` only records what a widget should show.
    ``flush()`` turns the differences into ops, but only once the previous
    batch has gone out, and sends at most ``max_bytes`` per call. Fast changes
    therefore coalesce into one update per flush instead of queueing up, and
    the pedal's 1.5 KB drawing queue is never overrun. Call ``flush`` from the
    DAW's periodic callback (every 40-100 ms works well).
    """

    def __init__(self, send, max_bytes=360, payload_limit=PAYLOAD_LIMIT):
        self._send = send
        self._ops = []
        self._widgets = []
        self.max_bytes = max_bytes
        self._payload_limit = payload_limit

    def add(self, op):
        if len(op) > self._payload_limit:
            raise ValueError("drawing op too large for one frame")
        self._ops.append(op)

    def pending(self):
        return len(self._ops)

    def drop(self):
        self._ops = []

    def forget_widgets(self):
        """Start a new page: drop queued ops and all widgets."""
        self._ops = []
        self._widgets = []

    def invalidate(self):
        """The pedal lost drawing (CC 126) or rebooted: redraw every widget."""
        self._ops = []
        for w in self._widgets:
            w.invalidate()

    def flush(self, max_bytes=None, max_frames=None):
        budget = self.max_bytes if max_bytes is None else max_bytes
        if sum(len(op) for op in self._ops) < budget:
            for w in self._widgets:
                w.emit()
        sent = 0
        spent = 0
        current = []
        while self._ops:
            op = self._ops[0]
            if current and len(current) + len(op) > self._payload_limit:
                self._send(frame(CMD_DAW_DRAW, 0, current))
                sent += 1
                spent += len(current)
                current = []
                if (max_frames is not None and sent >= max_frames) or spent + len(op) > budget:
                    return sent
            current.extend(op)
            self._ops.pop(0)
        if current:
            self._send(frame(CMD_DAW_DRAW, 0, current))
            sent += 1
        return sent


class _Widget(object):
    """Retained widget: ``set`` records, ``emit`` draws the difference."""

    def _register(self, canvas):
        self._args = None
        canvas._widgets.append(self)

    def set(self, *args, **kwargs):
        self._args = (args, kwargs)

    def emit(self):
        if self._args is not None:
            self._apply(*self._args[0], **self._args[1])


class Label(_Widget):
    """One line of text in a fixed box. Redraws only when text or colour change,
    and overwrites in place: no clear-then-draw flicker."""

    def __init__(self, canvas, x, y, width, scale=2, align="left", fg=WHITE, bg=BLACK, grid=False):
        self.c, self.x, self.y, self.w, self.k = canvas, x, y, width, scale
        self.align, self.fg, self.bg = align, fg, bg
        self.grid = grid
        self._drawn = None  # (text, fg, x0, width)
        self._register(canvas)

    def invalidate(self):
        self._drawn = None

    def _apply(self, text, fg=None):
        fg = self.fg if fg is None else fg
        text = fit_width(text, self.w, self.k)
        if self.grid:
            if not text.strip() and self._drawn is None:
                return
            columns = (self.w + self.k) // (6 * self.k)
            text = text.center(columns)
        if self._drawn and self._drawn[0] == text and self._drawn[1] == fg:
            return
        tw = text_width(text, self.k)
        if self.align == "center":
            x0 = self.x + (self.w - tw) // 2
        elif self.align == "right":
            x0 = self.x + self.w - tw
        else:
            x0 = self.x
        h = 8 * self.k  # lowercase descenders use an eighth row
        if text:
            self.c.add(draw_text(x0, self.y, self.k, text, fg, self.bg))
        if self.grid:
            # The opaque glyph cells cover the old text, including its edges.
            self._drawn = (text, fg, x0, tw)
            return
        if self._drawn is None:
            # first draw: clear whatever is left in the box around the text
            if x0 > self.x:
                self.c.add(draw_rect(self.x, self.y, x0 - self.x, h, self.bg))
            if x0 + tw < self.x + self.w:
                self.c.add(draw_rect(x0 + tw, self.y, self.x + self.w - x0 - tw, h, self.bg))
        else:
            ox, ow = self._drawn[2], self._drawn[3]
            if ox < x0:
                self.c.add(draw_rect(ox, self.y, min(x0, ox + ow) - ox, h, self.bg))
            if ox + ow > x0 + tw:
                start = max(x0 + tw, ox)
                self.c.add(draw_rect(start, self.y, ox + ow - start, h, self.bg))
        self._drawn = (text, fg, x0, tw)


class Dial(_Widget):
    """270 degree ring. Moving the value only paints the arc between the old
    and new positions."""

    def __init__(self, canvas, cx, cy, r, thick, fg=WHITE, track=0x34383C):
        self.c, self.cx, self.cy, self.r, self.t = canvas, cx, cy, r, thick
        self.fg, self.track = fg, track
        self._a = None
        self._fg = None
        self._register(canvas)

    def invalidate(self):
        self._a = None

    def _apply(self, value, fg=None):
        fg = self.fg if fg is None else fg
        a = max(0, min(127, int(round(value * 127))))
        args = (self.cx, self.cy, self.r, self.t)
        if self._a is None or fg != self._fg:
            self.c.add(draw_arc(*args, a0=0, a1=127, color=self.track))
            if a:
                self.c.add(draw_arc(*args, a0=0, a1=a, color=fg))
        elif a > self._a:
            self.c.add(draw_arc(*args, a0=self._a, a1=a, color=fg))
        elif a < self._a:
            self.c.add(draw_arc(*args, a0=a, a1=self._a, color=self.track))
            if a:
                self.c.add(draw_arc(*args, a0=a, a1=a, color=fg))
        self._a, self._fg = a, fg


class Bar(_Widget):
    """Horizontal level line; only the changed part is repainted."""

    def __init__(self, canvas, x, y, w, h=2, fg=WHITE, track=0x34383C, bipolar=False):
        self.c, self.x, self.y, self.w, self.h = canvas, x, y, w, h
        self.fg, self.track, self.bipolar = fg, track, bipolar
        self._span = None
        self._fg = None
        self._register(canvas)

    def invalidate(self):
        self._span = None

    def _target(self, value):
        value = max(0.0, min(1.0, value))
        if not self.bipolar:
            return (0, int(round(self.w * value)))
        mid = self.w // 2
        d = int(round((value - 0.5) * self.w))
        return (min(mid, mid + d), max(mid, mid + d) if d else mid + 1)

    def _apply(self, value, fg=None):
        fg = self.fg if fg is None else fg
        span = self._target(value)
        if span == self._span and fg == self._fg:
            return
        if self._span is None or fg != self._fg:
            self.c.add(draw_rect(self.x, self.y, self.w, self.h, self.track))
            if span[1] > span[0]:
                self.c.add(draw_rect(self.x + span[0], self.y, span[1] - span[0], self.h, fg))
        else:
            # repaint only the difference between the old and new spans
            (o0, o1), (n0, n1) = self._span, span
            for a, b, col in ((min(o0, n0), max(o0, n0), self.track if n0 > o0 else fg),
                              (min(o1, n1), max(o1, n1), fg if n1 > o1 else self.track)):
                if b > a:
                    self.c.add(draw_rect(self.x + a, self.y, b - a, self.h, col))
        self._span, self._fg = span, fg


class Segments(_Widget):
    """A row of blocks that light up to a value: crisp, cheap, and a change only
    repaints the blocks that flip. ``bipolar`` lights from the middle out."""

    def __init__(self, canvas, x, y, w, h, count=32, gap=2, fg=WHITE, track=0x34383C, bipolar=False):
        self.c, self.x, self.y, self.h, self.n, self.gap = canvas, x, y, h, count, gap
        self.fg, self.track, self.bipolar = fg, track, bipolar
        self.bw = (w - gap * (count - 1)) // count
        self._lit = None
        self._register(canvas)

    def invalidate(self):
        self._lit = None

    def _want(self, value, fg):
        value = max(0.0, min(1.0, value))
        if not self.bipolar:
            k = int(round(value * self.n))
            return [fg if i < k else self.track for i in range(self.n)]
        mid = self.n / 2.0
        end = value * self.n
        lo, hi = min(mid, end), max(mid, end)
        return [fg if (i + 1 > lo and i < hi) or (abs(value - 0.5) < 1e-6 and int(mid) - 1 <= i <= int(mid)) else self.track
                for i in range(self.n)]

    def _apply(self, value, fg=None):
        want = self._want(value, self.fg if fg is None else fg)
        i = 0
        while i < self.n:
            if self._lit is not None and self._lit[i] == want[i]:
                i += 1
                continue
            j = i
            while j < self.n and want[j] == want[i] and (self._lit is None or self._lit[j] != want[j]):
                j += 1
            # repaint just the blocks that changed; the gaps between them stay black
            for k in range(i, j):
                self.c.add(draw_rect(self.x + k * (self.bw + self.gap), self.y, self.bw, self.h, want[k]))
            i = j
        self._lit = want


class Wave(_Widget):
    """Mirrored waveform, one column per peak. Columns inside ``region`` use
    ``fg``, the rest ``dim``. Only changed columns are resent."""

    def __init__(self, canvas, x, ymid, half, width, fg=0x8C9196, dim=0x1C1F22, bg=BLACK):
        self.c, self.x, self.ym, self.half, self.w = canvas, x, ymid, half, width
        self.fg, self.dim, self.bg = fg, dim, bg
        self._cols = [None] * width
        self._register(canvas)

    def invalidate(self, start=0, end=None):
        end = self.w if end is None else end
        for i in range(max(0, start), min(self.w, end)):
            self._cols[i] = None

    def _apply(self, peaks, region=(0, 1.0), marks=()):
        """``marks``: (column, colour) pairs drawn as 2 px full-height lines,
        e.g. start/end markers and a playhead. They are part of the diff, so
        moving one repaints just the columns it leaves and enters."""
        lo = int(region[0] * self.w)
        hi = int(region[1] * self.w)
        want = []
        for i in range(self.w):
            p = peaks[i] if i < len(peaks) else 0
            want.append((max(0, min(127, int(p))), self.fg if lo <= i < hi else self.dim))
        for col, color in marks:
            for c in (int(col), int(col) + 1):
                if 0 <= c < self.w:
                    want[c] = (127, color)
        i = 0
        while i < self.w:
            if want[i] == self._cols[i]:
                i += 1
                continue
            col = want[i][1]
            j = i
            while j < self.w and j - i < WAVE_CHUNK and want[j][1] == col and want[j] != self._cols[j]:
                j += 1
            self.c.add(draw_wave(self.x + i, self.ym, self.half, col, self.bg, [w[0] for w in want[i:j]]))
            for k in range(i, j):
                self._cols[k] = want[k]
            i = j


def peaks_from_samples(samples, columns):
    """Reduce a list of sample magnitudes (0..1) to ``columns`` peaks (0..127)."""
    n = len(samples)
    if not n:
        return [0] * columns
    out = []
    for c in range(columns):
        a = c * n // columns
        b = max(a + 1, (c + 1) * n // columns)
        out.append(int(127 * min(1.0, max(abs(s) for s in samples[a:b]))))
    return out


def plugin_heading(name, group, page):
    aliases = {"fruity parametric eq 2": "Parametric EQ 2", "fruity parametric eq": "Parametric EQ"}
    name = aliases.get(name.lower(), name)
    if page == "1/1":
        page = ""
        if group.lower() in ("main", ""):
            return name[:22]
    heading = name + (" / " + group if group else "")
    if len(heading) + len(page) + 1 > 23:
        heading = name
    return (heading[:22 - len(page)].rstrip() + " " + page).rstrip()


def compression_visual(threshold, ratio):
    try:
        threshold_db = float(threshold.strip().lower().replace("db", "").strip())
        ratio_value = float(ratio.lower().replace("∞", "inf").split(":")[0].strip())
    except (ValueError, AttributeError):
        return None
    if not math.isfinite(threshold_db) or not ratio_value > 0:
        return None
    return ("compression", max(-60, min(0, threshold_db)), ratio_value, ratio)


def oscillator_visual(caption, shape):
    shapes = {"sine": "sine", "sin": "sine", "triangle": "triangle", "tri": "triangle",
              "square": "square", "rect": "square", "saw": "saw", "sawtooth": "saw",
              "rounded saw": "rounded saw"}
    wave = shapes.get(str(shape).strip().lower())
    return ("oscillator", wave, caption, wave.upper()) if wave else None


ENVELOPE_STAGES = ("attack", "decay", "sustain", "release")


def envelope_groups(names):
    sections = {}
    for index, name in enumerate(names):
        words = str(name or "").lower()
        for separator in ("_", "-", "/", ":"):
            words = words.replace(separator, " ")
        words = words.split()
        candidates = [stage for stage in ENVELOPE_STAGES if stage in words]
        if len(candidates) != 1:
            continue
        stage = candidates[0]
        section = " ".join(word for word in words if word not in (stage, "time", "level"))
        stages = sections.setdefault(section, {})
        stages[stage] = None if stage in stages else index
    return [(section, tuple(stages[stage] for stage in ENVELOPE_STAGES))
            for section, stages in sections.items()
            if all(stages.get(stage) is not None for stage in ENVELOPE_STAGES)]


def eq_groups(names):
    roles = {"freq": "Freq", "frequency": "Freq", "gain": "Gain", "level": "Gain",
             "q": "Q", "resonance": "Q", "width": "Q", "bandwidth": "Q", "type": "Type"}
    bands = {}
    for index, name in enumerate(names):
        words = str(name or "").lower().replace("_", " ").split()
        if not any(word.isdigit() for word in words):
            continue
        matches = [roles[word] for word in words if word in roles]
        if len(matches) != 1:
            continue
        key = tuple(word for word in words if word not in roles and word not in ("band", "filter"))
        controls = bands.setdefault(key, {})
        role = matches[0]
        controls[role] = None if role in controls else index
    return [(key, [controls[role] for role in ("Freq", "Gain", "Q", "Type") if controls.get(role) is not None])
            for key, controls in bands.items() if controls.get("Freq") is not None and controls.get("Gain") is not None]


def bank_visual(bank, focus, device_name=""):
    if not bank:
        return None
    labels = [label.strip().lower() for label, _, _ in bank]
    focus = min(focus, len(bank) - 1)
    for _, indices in envelope_groups(labels):
        if focus in indices:
            return ("envelope", tuple(bank[i][1] for i in indices),
                    ENVELOPE_STAGES[indices.index(focus)].upper(), bank[focus][2])
    label, _, formatted = bank[focus]
    if any(word in labels[focus].replace("_", " ").split() for word in ("wave", "waveform", "shape")):
        return oscillator_visual(label, formatted)
    if "threshold" in labels and "ratio" in labels:
        threshold = bank[labels.index("threshold")][2]
        ratio = bank[labels.index("ratio")][2]
        if "db" in threshold.lower() and (":" in ratio or "inf" in ratio.lower() or "∞" in ratio):
            return compression_visual(threshold, ratio)
    device_name = device_name.lower()
    if "eq" in device_name:
        bands = eq_groups(labels)
        if labels[:2] == ["freq", "gain"] or (len(bands) == 1 and bands[0][1][:2] == [0, 1]):
            return ("eq_band", bank[0][1], bank[1][1], bank[1][2])
    if "delay" in device_name or "echo" in device_name:
        for name in ("feedback", "feedback level", "feedback amount"):
            if name in labels:
                _, meter, value = bank[labels.index(name)]
                return ("echo", meter, 0, value)
    return None


def effect_columns(visual):
    kind, first, second, _ = visual
    columns = []
    for index in range(48):
        position = index / 47.0
        if kind == "compression":
            input_db = -60 + 60 * position
            output_db = input_db if input_db <= first else first + (input_db - first) / second
            height = 1 + 43 * (output_db + 60) / 60
        elif kind == "eq_band":
            marker = round(first * 47 / 127)
            height = 22 if abs(index - marker) > 1 else 2 + 40 * second / 127
        elif kind == "envelope":
            attack, decay, sustain, release = first
            attack_end = 2 + attack / 127 * 10
            decay_end = attack_end + 2 + decay / 127 * 9
            release_start = 45 - release / 127 * 12
            level = sustain / 127
            if index <= attack_end:
                height = 43 * index / attack_end
            elif index <= decay_end:
                height = 43 * (1 - (1 - level) * (index - attack_end) / (decay_end - attack_end))
            elif index <= release_start:
                height = 43 * level
            else:
                height = 43 * level * (47 - index) / (47 - release_start)
        elif kind == "oscillator":
            phase = (position * 2) % 1
            if first == "sine":
                wave = math.sin(2 * math.pi * phase)
            elif first == "triangle":
                wave = 1 - 4 * abs(phase - 0.5)
            elif first == "square":
                wave = 1 if phase < 0.5 else -1
            else:
                wave = 2 * phase - 1
                if first == "rounded saw":
                    wave = math.sin(wave * math.pi / 2)
            height = 22 + 18 * wave
        elif kind == "echo":
            repeat, offset = divmod(index, 7)
            height = (43 * (first / 127.0) ** repeat * (1 - offset / 3.0)
                      if offset < 3 else 0)
        else:
            return ()
        columns.append(max(0, min(44, round(height))))
    return tuple(columns)


_SHORT_WORDS = {
    "frequency": "Freq", "freq": "Freq", "resonance": "Res", "feedback": "Fdbk",
    "threshold": "Thresh", "release": "Rel", "attack": "Atk", "sustain": "Sus",
    "decay": "Decay", "amount": "Amt", "output": "Out", "input": "In",
    "volume": "Vol", "panning": "Pan", "level": "Lvl", "width": "Width",
    "stereo": "St", "filter": "Flt", "envelope": "Env", "oscillator": "Osc",
    "coarse": "Crs", "pitch": "Pitch", "time": "", "predelay": "PreDly",
    "division": "Div", "modulation": "Mod", "transpose": "Trnsp", "velocity": "Vel",
    "channel": "Chan", "pattern": "Patrn", "master": "Master", "shape": "Shape",
    "cutoff": "Cutoff", "drive": "Drive", "ratio": "Ratio", "knee": "Knee", "makeup": "Makeup",
}
_SHORT_NAMES = {
    "dry/wet": "Mix", "dry wet": "Mix", "decay time": "Decay", "room size": "Size",
    "in filter freq": "InFreq", "in filter width": "InWdth", "reflect level": "Early",
    "diffuse level": "Tail", "stereo width": "Width", "filter type": "Type",
    "early reflection level": "ER", "send a": "Send A", "send b": "Send B",
}


def abbreviate_label(name, limit=6):
    """Hardware-style short name, or None when it would need a blind cut:
    known names first, then word abbreviations, then the words run together."""
    text = str(name or "").strip()
    if len(text) <= limit:
        return text
    known = _SHORT_NAMES.get(text.lower())
    if known:
        return known
    words = [w for w in (_SHORT_WORDS.get(w.lower(), w) for w in text.replace("_", " ").split()) if w]
    for candidate in (" ".join(words), "".join(words)):
        if len(candidate) <= limit:
            return candidate
    return None


def short_label(name, limit=6):
    """Like ``abbreviate_label`` but always returns text of at most ``limit``."""
    short = abbreviate_label(name, limit)
    if short is not None:
        return short
    words = [w for w in (_SHORT_WORDS.get(w.lower(), w) for w in str(name).replace("_", " ").split()) if w]
    joined = "".join(words)
    if len(joined) > limit:
        # Drop inner vowels before cutting: Thresh -> Thrsh.
        joined = joined[0] + "".join(c for c in joined[1:] if c.lower() not in "aeiou")
    return joined[:limit]


# Press-state colours for the encoder strip. They mirror the LEDs.
STATE_ARM = 0xF0463A
STATE_MUTE = 0xF4B21C
STATE_SOLO = 0x3D8BFF
STATE_PLAY = 0x47C45A
STATE_ON = 0x00B6CA

HEAD_H = 26
STRIP_Y = 139
STRIP_H = SCREEN_H - STRIP_Y
STRIP_MARKER_Y = SCREEN_H - 4
CUTOFF_X = 108
CUTOFF_RIGHT = 312
CUTOFF_TOP = 32
CUTOFF_PASS_Y = 70
CUTOFF_RADIUS = 43


class StripColumn(_Widget):
    """One encoder in the bottom strip: what turning it controls, a small
    meter, and a coloured bar while its press function is on."""

    def __init__(self, canvas, index, theme):
        self.c, self.index, self.t = canvas, index, theme
        self.x = index * (SCREEN_W // 4)
        self.w = SCREEN_W // 4
        self._drawn = None
        self._register(canvas)
        self._bar = Bar(canvas, self.x + 12, STRIP_Y + 29, self.w - 24, 3,
                        theme["meter"], theme["meter_track"])

    def invalidate(self):
        self._drawn = None

    def _apply(self, item, enabled):
        t = self.t
        item = tuple(item or ())
        _, meter, state, bipolar = (item + ("", None, None, False)[len(item):])[:4]
        old_state, had_meter = self._drawn or (None, False)
        if state != old_state:
            color = t["strip"] if state is None else state
            self.c.add(draw_rect(self.x + 5, STRIP_Y + 3, self.w - 10, 3, color))
        if self._bar.bipolar != bool(bipolar):
            self._bar.bipolar = bool(bipolar)
            self._bar.invalidate()
        if meter is None:
            if had_meter:
                self.c.add(draw_rect(self._bar.x, self._bar.y, self._bar.w, self._bar.h, t["strip"]))
                self._bar.invalidate()
            self._bar._args = None
        else:
            self._bar.set(max(0, min(127, int(meter))) / 127.0, t["meter"] if enabled else t["strip_dim"])
        self._drawn = (state, meter is not None)


class Transport(_Widget):
    """Play / record / stop icon at the right of the header, with the song
    position beside it while the transport runs."""

    def __init__(self, canvas, theme):
        self.c, self.t = canvas, theme
        self._state = None
        self._register(canvas)
        self._text = Label(canvas, 228, 6, 70, 2, "right", theme["head_ink"], theme["head"])

    def invalidate(self):
        self._state = None

    def _apply(self, state, position=""):
        if state != self._state:
            x, y = 304, 6
            self.c.add(draw_rect(x - 1, y - 1, 14, 16, self.t["head"]))
            if state == "play":
                for row, width in enumerate((2, 5, 8, 10, 8, 5, 2)):
                    self.c.add(draw_rect(x + 1, y + 1 + 2 * row, width, 2, STATE_PLAY))
            elif state == "rec":
                for row, (left, width) in enumerate(((3, 6), (1, 10), (0, 12), (0, 12), (1, 10), (3, 6))):
                    self.c.add(draw_rect(x + left, y + 1 + 2 * row, width, 2, STATE_ARM))
            else:
                self.c.add(draw_rect(x + 1, y + 2, 10, 10, self.t["head_dim"]))
            self._state = state
        self._text.set(position if state in ("play", "rec") else "")


class FocusedView(View):
    """DAW-themed canvas driven by the last control that was turned.

    Every page shares one frame: a header with the page, the track or device
    and the transport; the focused control in the middle; and a strip naming
    what each of the four encoders does. The legacy view supplies LEDs,
    keepalive, and the display on firmware without the canvas.
    """

    def __init__(self, send, host, refresh_every=10, clock=time.monotonic):
        self._canvas = Canvas(send, payload_limit=USB_SAFE_PAYLOAD)
        self._readout = Canvas(send, payload_limit=USB_SAFE_PAYLOAD)
        self._focus_readout = Canvas(lambda data: self._send(data), payload_limit=USB_SAFE_PAYLOAD)
        self._clock = clock
        self._send_after = 0
        self._painting_base = False
        self._native_dial = False
        self._widgets_supported = False
        self._host = host
        self._retry_ticks = None
        self._layout = None
        self._cutoff_knee = None
        self._compact_cutoff = False
        self._compact_cutoff_value = None
        self._marker = None
        self._ableton = host == HOST_ABLETON
        self.page = None
        self.active = False
        self.focus = 0
        self.panel_title = ""
        self.page_name = ""
        self.context = ""
        self.transport = ("stop", "")
        self.strip = ()
        self.strip_enabled = True
        self.filter_meter = None
        self.plugin_strips = ()
        self.knob_bank = ()
        self.text_readout = False
        self.show_knob_bank = False
        self.active_encoder = 0
        self.effect_visual = None
        self.plugin_control = "fader"
        if self._ableton:
            self._body, self._head, self._rule = 0xACADAD, 0xBCBDBD, 0x777979
            self._ink, self._accent, self._track = 0x292C2D, 0x00B6CA, 0x45494A
            self._theme = {"head": self._head, "head_ink": self._ink, "head_dim": 0x6A6D6E,
                           "chip": 0x292C2D, "chip_ink": 0xF4B21C,
                           "strip": 0xBCBDBD, "strip_ink": 0x292C2D, "strip_dim": 0x8A8D8D,
                           "strip_marker": 0xF4B21C,
                           "meter": 0x00A3B6, "meter_track": 0x9A9C9C}
        else:
            self._body, self._head, self._rule = 0x37464D, 0x44545B, 0x222E34
            self._ink, self._accent, self._track = 0xE1E8E9, 0xB0D848, 0x252E34
            self._theme = {"head": self._head, "head_ink": self._ink, "head_dim": 0x8C9A9F,
                           "chip": 0xB0D848, "chip_ink": 0x1B2328,
                           "strip": 0x2B373D, "strip_ink": 0xD5DDDE, "strip_dim": 0x66757B,
                           "strip_marker": 0xB0D848,
                           "meter": 0xB0D848, "meter_track": 0x45545B}
        self._graph_base = 132
        super().__init__(send, max_frames_per_flush=1, refresh_every=refresh_every,
                         payload_limit=USB_SAFE_PAYLOAD)

    def start_session(self):
        self._send(mode_frame(True, self._host))
        self.invalidate()
        if not self.active:
            self._retry_ticks = 6

    def set_active(self, active):
        self.active = bool(active)
        if self.active:
            self._retry_ticks = None
        else:
            self._native_dial = False
            self._widgets_supported = False
            self._compact_cutoff = False
        self.invalidate()

    def set_widgets_supported(self, supported):
        if bool(supported) != self._widgets_supported:
            self._widgets_supported = bool(supported)
            self._marker = None

    def set_native_dial(self, supported):
        supported = bool(supported)
        if self._native_dial != supported:
            self._native_dial = supported
            self._marker = None

    def set_compact_cutoff(self, supported):
        supported = self._ableton and bool(supported)
        if supported != self._compact_cutoff:
            self._compact_cutoff = supported
            if self._layout and self._layout[0] == "cutoff":
                self.invalidate()

    def select_page(self, page):
        if page != self.page:
            self.page = page
            self.focus = 0
            self.active_encoder = 0

    def invalidate(self):
        super().invalidate()
        self._canvas.forget_widgets()
        self._readout.forget_widgets()
        self._focus_readout.forget_widgets()
        self._layout = None
        self._cutoff_knee = None
        self._compact_cutoff_value = None
        self._painting_base = False

    def _box(self, x, y, w, h, fill):
        self._canvas.add(draw_rect(x, y, w, h, fill))

    def _face(self, cx, cy):
        # Three-pixel bands keep the inset disk cheap to transfer.
        for dy in range(-30, 31, 3):
            half = int((31 * 31 - dy * dy) ** 0.5)
            self._box(cx - half, cy + dy, 2 * half + 1, 3, 0x263239)

    def _header(self, c):
        t = self._theme
        self._box(0, 0, SCREEN_W, HEAD_H, self._head)
        self._box(0, HEAD_H - 1, SCREEN_W, 1, self._rule)
        left = 6
        page = str(self.page_name or "").upper()
        if page:
            width = text_width(page, 2) + 10
            self._box(4, 4, width, 18, t["chip"])
            self._canvas.add(draw_text(9, 6, 2, page, t["chip_ink"], t["chip"]))
            left = width + 12
        # The transport emits first: its readout may clear its box, and the
        # name is then drawn over the room it leaves.
        self._transport = Transport(c, t)
        self._heading = Label(c, left, 6, 298 - left, 2, fg=self._ink, bg=self._head)

    def _strip(self, c):
        self._box(0, STRIP_Y, SCREEN_W, STRIP_H, self._theme["strip"])
        self._box(0, STRIP_Y, SCREEN_W, 1, self._rule)
        self._strip_captions = Label(c, 0, STRIP_Y + 10, SCREEN_W, 2, "center",
                                     self._theme["strip_ink"], self._theme["strip"], grid=True)
        self._columns = [StripColumn(c, index, self._theme) for index in range(4)]

    def _set_layout(self, layout):
        c = self._readout
        c.forget_widgets()
        focus = self._focus_readout
        focus.forget_widgets()
        self._canvas.drop()
        self._canvas.add(draw_clear(self._body))
        self._layout = layout
        self._painting_base = True
        self._marker = None
        self._columns = ()
        self._strip_captions = None
        self._strip_focus = None
        kind = layout[0]
        self._header(c)
        if kind in ("dial", "text", "effect", "cutoff"):
            self._strip(c)
        if kind == "text":
            self._name = Label(focus, 14, 46, 292, 2, "center", self._ink, self._body, grid=True)
            self._value = Label(focus, 14, 80, 292, 3, "center", self._ink, self._body, grid=True)
        elif kind == "effect":
            self._graph_kind = self.effect_visual[0]
            self._box(140, 30, 172, 106, 0x252D31)
            self._box(151, self._graph_base, 144, 1, 0x646E70)
            self._box(150, 40, 1, self._graph_base - 39, 0x646E70)
            self._graph_drawn = [-1] * 48
            self._graph_cursor = 0
            self._graph_key = None
            self._name = Label(focus, 5, 34, 130, 2, "center", self._ink, self._body, grid=True)
            self._value = Label(focus, 5, 112, 130, 3, "center", self._ink, self._body, grid=True)
            self._dial = Dial(c, 70, 78, 26, 3, self._accent, self._track)
        elif kind == "bank":
            self._bank_labels = []
            self._strip_values = []
            count = len(self.knob_bank)
            width = SCREEN_W // count - 2
            for index, (caption, meter, _) in enumerate(self.knob_bank):
                center = (2 * index + 1) * SCREEN_W // (2 * count)
                self._canvas.add(draw_text(center - text_width(caption, 2) // 2, 39, 2,
                                           caption, self._ink, self._body))
                radius = min(36, width // 2 - 7)
                self._canvas.add(define_knob(index, center, 94, radius, meter, self._accent,
                                             self._track, self._body, self._body, self._ink))
                self._strip_values.append(meter)
                self._bank_labels.append(Label(c, center - width // 2, 140, width, 2,
                                               "center", self._ink, self._body))
        elif kind == "strips":
            panel = 0x313839 if self._ableton else 0x263239
            full_faders = self._host == HOST_FL_STUDIO and self.plugin_control == "fader"
            panel_x = 162 if full_faders else 8
            self._box(panel_x, 34, 150 if full_faders else 140, 134, panel)
            self._strip_values = []
            spacing = 136 // len(self.plugin_strips)
            for index, (caption, value) in enumerate(self.plugin_strips):
                center = panel_x + 4 + spacing * index + spacing // 2
                if self.plugin_control == "knob":
                    radius = min(28, spacing // 2 - 3)
                    self._canvas.add(define_knob(index, center, 92, radius, value, self._accent,
                                                 0x596164, panel, panel, 0xEEF1E9))
                    caption_y = 132
                else:
                    self._canvas.add(define_slider(index, center - 5, 49, 10, 88, value,
                                                   self._accent, 0x546065, panel, 0xEEF1E9, vertical=True))
                    caption_y = 148
                scale = 2 if full_faders else 1
                self._canvas.add(draw_text(center - text_width(caption, scale) // 2, caption_y, scale,
                                           caption, 0xD8DDDB, panel))
                self._strip_values.append(value)
            readout_x = 2 if full_faders else 156
            self._name = Label(focus, readout_x, 66, 158, 2, "center", self._ink, self._body, grid=True)
            self._value = Label(focus, readout_x, 103, 158, 2, "center", self._ink, self._body, grid=True)
        elif kind == "cutoff":
            self._name = Label(focus, 4, 32, 92, 2, "center", self._ink, self._body, grid=True)
            self._value = Label(focus, 3, 113, 100, 2, "center", self._ink, self._body, grid=True)
            self._dial = Dial(c, 50, 79, 28, 4, self._accent, self._track)
            width = CUTOFF_RIGHT - CUTOFF_X
            self._box(CUTOFF_X, CUTOFF_TOP, width, STRIP_Y - CUTOFF_TOP - 3, 0x202223)
            for y in (49, 127):
                self._box(CUTOFF_X, y, width, 1, 0x434646)
            self._cutoff_knee = None
            self._compact_cutoff_value = None
        else:
            # "dial": the focused control's knob beside its name and value.
            inset = 0x26343B
            if not self._ableton:
                if not (self._native_dial or self._widgets_supported):
                    self._face(64, 82)
                self._box(116, 72, 196, 34, 0x29343A)
                self._box(117, 73, 194, 32, inset)
                self._box(117, 105, 194, 1, 0x516068)
            self._name = Label(focus, 116, 46, 196, 2, "center", self._ink, self._body, grid=True)
            self._value = Label(focus, 118, 79, 192, 3, "center", self._ink,
                                self._body if self._ableton else inset, grid=True)
            self._dial = Dial(c, 64, 82, 40, 3, self._accent, self._track)

    def _pointer(self, meter):
        if self._marker == meter:
            return
        dial = self._dial
        bg = self._body if self._ableton else 0x263239
        color = self._ink if self._ableton else self._accent
        radius = dial.r - 13
        op = []
        if self._marker is not None:
            op.extend(draw_arc(dial.cx, dial.cy, radius, 4, self._marker, self._marker, bg))
        op.extend(draw_arc(dial.cx, dial.cy, radius, 4, meter, meter, color))
        self._readout.add(op)
        self._marker = meter

    def _paint_cutoff(self, meter):
        if self._compact_cutoff:
            if self._compact_cutoff_value == meter:
                return 0
            self._send(frame(CMD_DAW_DRAW, 0, (OP_COMPACT_CUTOFF, meter)))
            self._compact_cutoff_value = meter
            return 1

        # Replace only the moving section of the curve. The two strokes and
        # their background correction fit in one USB drawing frame.
        travel = CUTOFF_RIGHT - CUTOFF_X - CUTOFF_RADIUS - 12
        knee = CUTOFF_X + 7 + meter * travel // 127
        old = self._cutoff_knee
        if old == knee:
            return 0
        thickness = 3
        center_y = CUTOFF_PASS_Y + CUTOFF_RADIUS - thickness // 2
        left = CUTOFF_X
        ops = []
        if old is not None:
            left = min(old, knee) - 4
            right = max(old, knee) + CUTOFF_RADIUS + 2
            ops.extend(draw_rect(left, CUTOFF_PASS_Y - 2, right - left, CUTOFF_RADIUS + 5, 0x202223))
        ops.extend(draw_rect(left, CUTOFF_PASS_Y - 1, knee - left + 2, thickness, self._accent))
        ops.extend(draw_arc(knee, center_y, CUTOFF_RADIUS, thickness, 63, 106, self._accent))
        self._send(frame(CMD_DAW_DRAW, 0, ops))
        self._cutoff_knee = knee
        return 1

    def _layout_key(self, plot):
        if plot:
            kind = ("cutoff",)
        elif self.knob_bank and self._widgets_supported and self.show_knob_bank:
            kind = ("bank", tuple(name for name, _, _ in self.knob_bank))
        elif self._widgets_supported and self.plugin_strips:
            kind = ("strips", self.plugin_control, tuple(label for label, _ in self.plugin_strips))
        elif self.knob_bank and self._widgets_supported and self.effect_visual:
            kind = ("effect",)
        elif self.text_readout:
            kind = ("text",)
        else:
            kind = ("dial",)
        return kind + (str(self.page_name or ""),)

    def flush(self, max_frames=None, paint=True):
        if paint and self._retry_ticks is not None:
            self._retry_ticks -= 1
            if self._retry_ticks == 1:
                # It may have entered DAW mode before the host subscribed to
                # the performance port. Re-arm once on two separate ticks.
                self._send(mode_frame(False, self._host))
                return 1
            if self._retry_ticks == 0:
                self._retry_ticks = None
                self._send(mode_frame(True, self._host))
                return 1
        # Do not send drawing opcodes until Nexfoot reports DAW mode.
        if not self.active:
            return super().flush(max_frames)

        now = self._clock()
        card = self.cards[self.focus]
        label, meter, value = card.label, card.meter, card.value
        if self.knob_bank:
            label, meter, value = self.knob_bank[min(self.focus, len(self.knob_bank) - 1)]
        plot = self._ableton and self.filter_meter is not None
        if now < self._send_after:
            return 0
        layout = self._layout_key(plot)
        graph_changed = layout[0] == "effect" and self._layout[0] == "effect" and \
            self._graph_kind != self.effect_visual[0] if self._layout else False
        if self._layout != layout or graph_changed:
            if not paint:
                return 0
            self._set_layout(layout)
        led_key = ("leds",)
        if self._sent.get(led_key) != self.leds:
            self._send(frame(CMD_DAW_UPDATE, 0, op_leds(self.leds)))
            self._sent[led_key] = self.leds
            self._send_after = now + 0.04
            return 1
        # One paced burst across both queues. Every frame fits a single USB
        # packet, including legacy updates when the canvas is idle.
        if self._painting_base:
            if not paint:
                return 0
            count = self._canvas.flush(max_frames=2)
            self._painting_base = bool(self._canvas.pending())
        else:
            count = self._flush_readout(label, meter, value, plot)
            if paint and layout[0] == "effect" and count < 2:
                count += self._flush_graph(2 - count)
            if paint and plot and count < 2:
                count += self._paint_cutoff(self.filter_meter)
            if not count and paint:
                count = super().flush(max_frames)
        if count:
            self._send_after = now + 0.04
        return count

    def _flush_graph(self, frame_limit):
        key = self.effect_visual[:3]
        if self._graph_key != key:
            heights = effect_columns(self.effect_visual)
            line = self.effect_visual[0] in ("compression", "envelope", "oscillator", "eq_band")
            self._graph_target = (tuple((heights[max(0, i - 1)], height) for i, height in enumerate(heights))
                                  if line else heights)
            self._graph_key = key
        if self._graph_drawn == list(self._graph_target):
            return 0
        base = self._graph_base
        ops = []
        frames = 0
        width = 3
        for _ in range(48):
            index = self._graph_cursor
            self._graph_cursor = (index + 1) % 48
            old, new = self._graph_drawn[index], self._graph_target[index]
            if old == new:
                continue
            kind = self.effect_visual[0]
            line = isinstance(new, tuple)
            if line and old != -1 and len(ops) > 12:
                self._graph_cursor = index
            else:
                x = 151 + index * width
                if line:
                    if old != -1:
                        ops.extend(draw_rect(x, base - 2 - 2 * max(old), width, 2 * abs(old[1] - old[0]) + 2,
                                             0x252D31))
                    ink = 0xF4B21C if self._ableton and kind == "compression" else self._accent
                    ops.extend(draw_rect(x, base - 2 - 2 * max(new), width, 2 * abs(new[1] - new[0]) + 2, ink))
                elif new != max(0, old):
                    old = max(0, old)
                    color = self._accent if new > old else 0x252D31
                    ops.extend(draw_rect(x, base - 2 * max(old, new), width, 2 * abs(new - old), color))
                self._graph_drawn[index] = new
                if len(ops) < 36:
                    continue
            if ops:
                self._send(frame(CMD_DAW_DRAW, 0, ops))
                frames += 1
                ops = []
                if frames == frame_limit:
                    break
        if ops:
            self._send(frame(CMD_DAW_DRAW, 0, ops))
            frames += 1
        return frames

    def _paint_focus(self, kind, label, value, plot):
        label = label or "--"
        value = str(value)
        if plot:
            label = {"frequency": "Freq", "resonance": "Res"}.get(label.lower(), label)
        if kind == "effect" and self.effect_visual[0] == "eq_band":
            for suffix, short in ((" level", " gain"), (" frequency", " freq"), (" width", " bw")):
                if label.lower().endswith(suffix):
                    label = label[:-len(suffix)] + short
                    break
        if kind in ("effect", "dial", "text"):
            label = str(label).strip()
            if label.isupper() and len(label) > 1:
                label = label.title()  # legacy card captions are capitals
            if text_width(label, 2) > self._name.w:
                label = " ".join(_SHORT_WORDS.get(word.lower(), word) if word.lower() in (
                    "frequency", "resonance", "envelope", "feedback") else word for word in label.split())
            value = value.replace("∞", "inf").replace("−", "-")
            if text_width(value, 3) > self._value.w:
                amount, separator, unit = value.rpartition(" ")
                if separator and unit.lower() in ("hz", "khz", "ms", "s", "db", "%"):
                    value = amount + unit
            scale = 3 if text_width(value, 3) <= self._value.w else 2
            if scale != self._value.k:
                self._focus_readout.add(draw_rect(self._value.x, self._value.y, self._value.w, 24, self._value.bg))
                self._value.k = scale
                self._value.invalidate()
        self._name.set(label)
        # Keep the percent sign and normal 0–100% values on a fixed glyph grid.
        # Leading spaces overwrite old digits in the same text command.
        if value.endswith("%"):
            value = value.rjust(5)
        self._value.set(value)

    def _flush_readout(self, label, meter, value, plot):
        kind = self._layout[0]
        sent = 0
        if kind != "bank":
            self._paint_focus(kind, label, value, plot)
            sent = self._focus_readout.flush(max_frames=2)
            if sent == 2:
                return sent
        remaining = 2 - sent
        if self._readout.pending():
            return sent + self._readout.flush(max_frames=remaining)
        state, position = (tuple(self.transport) + ("",))[:2]
        heading = self._heading
        # The name may use the position's room while the transport is stopped.
        width = (224 if state in ("play", "rec") else 298) - heading.x
        if width != heading.w:
            if width < heading.w:
                self._readout.add(draw_rect(heading.x + width, heading.y, heading.w - width, 14, self._head))
            heading.w = width
            heading.invalidate()
        heading.set(self.context or self.panel_title or self.title)
        self._transport.set(state, position)
        if self._strip_captions:
            # One 26-cell row fits one USB frame.
            items = (tuple(self.strip) + (None,) * 4)[:4]
            captions = "".join(short_label(item[0], 6).center(width) if item else " " * width
                               for item, width in zip(items, (6, 7, 7, 6)))
            ink = self._theme["strip_ink"] if self.strip_enabled else self._theme["strip_dim"]
            self._strip_captions.set(captions, ink)
            index = self.active_encoder
            item = items[index] if 0 <= index < len(items) else None
            active = index if item and item[0] else None
            if active != self._strip_focus:
                # Move both markers in one frame, without touching the captions.
                ops = []
                for column, color in ((self._strip_focus, self._theme["strip"]),
                                      (active, self._theme["strip_marker"])):
                    if column is not None:
                        width = SCREEN_W // 4
                        ops.extend(draw_rect(column * width + 12, STRIP_MARKER_Y, width - 24, 2, color))
                self._readout.add(ops)
                self._strip_focus = active
            for column, item in zip(self._columns, items):
                column.set(item, self.strip_enabled)
        if kind == "bank":
            for index, (_, meter, formatted) in enumerate(self.knob_bank):
                if self._strip_values[index] != meter:
                    self._readout.add(set_widget_value(index, meter))
                    self._strip_values[index] = meter
                label = self._bank_labels[index]
                if text_width(formatted, 2) > label.w:
                    formatted = {"triangle": "Tri", "rounded saw": "RndSaw", "sawtooth": "Saw"}.get(
                        formatted.lower(), formatted)
                    number, separator, unit = formatted.rpartition(" ")
                    if separator and unit.lower() in ("hz", "khz", "ms", "s", "db", "%"):
                        formatted = number + unit
                scale = 2 if text_width(formatted, 2) <= label.w else 1
                if label.k != scale:
                    self._readout.add(draw_rect(label.x, label.y, label.w, 16, self._body))
                    label.k = scale
                    label.invalidate()
                label.set(str(formatted)[:USB_SAFE_PAYLOAD - 13])
            return sent + self._readout.flush(max_frames=remaining)
        if kind == "text":
            return sent + self._readout.flush(max_frames=remaining)
        if kind == "strips":
            for index, (_, value) in enumerate(self.plugin_strips):
                if self._strip_values[index] != value:
                    self._readout.add(set_widget_value(index, value))
                    self._strip_values[index] = value
        elif self._native_dial or (kind != "cutoff" and self._widgets_supported):
            if self._marker != meter:
                dial = self._dial
                face = self._body if self._ableton else 0x263239
                pointer = self._ink if self._ableton else self._accent
                args = (dial.cx, dial.cy, dial.r, meter, self._accent, self._track,
                        self._body, face, pointer)
                if self._widgets_supported and kind != "cutoff":
                    op = define_knob(0, *args) if self._marker is None else set_widget_value(0, meter)
                else:
                    op = draw_dial(*args)
                self._readout.add(op)
                self._marker = meter
        else:
            self._dial.set(meter / 127.0)
            self._dial.emit()
            self._pointer(meter)
        return sent + self._readout.flush(max_frames=remaining)
