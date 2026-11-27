# Copyright 2026 Nexfoot contributors. Licensed under the Apache License 2.0.
"""Pixel emulator of the Nexfoot DAW screen.

Feed it the SysEx frames a DAW script sends and save what the pedal's
320x176 LCD would show. It is a port of the DAW renderer in
``apps/spp_and_le/examples/trans_data/md400_display.c`` (legacy cards, the
DAW_DRAW canvas, retained widgets, native dial and cutoff pane, waiting page),
so script layouts can be checked without hardware:

    from nexfoot_screen import Pedal
    pedal = Pedal()
    pedal.receive(frame)          # every frame the script sends
    pedal.save("screen.png")      # needs Pillow

Run ``python daw/tools/nexfoot_screen.py demo out.png`` for a sample, or
``python daw/tools/nexfoot_screen.py replay capture.txt out.png [seconds]`` to
render frames recorded by a script running with NEXFOOT_CAPTURE set.
"""
import math
import sys

W, H = 320, 176
TOP_BAR_H = 28
CARD_W = W // 4
CARD_Y = TOP_BAR_H
CARD_H = H - TOP_BAR_H

_SIN = [int(round(4096 * math.sin(math.radians(d)))) for d in range(360)]


def _sin(deg):
    return _SIN[deg % 360]


_DIGITS = [
    (0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e), (0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e),
    (0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f), (0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e),
    (0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02), (0x1f, 0x10, 0x10, 0x1e, 0x01, 0x01, 0x1e),
    (0x0e, 0x10, 0x10, 0x1e, 0x11, 0x11, 0x0e), (0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08),
    (0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e), (0x0e, 0x11, 0x11, 0x0f, 0x01, 0x01, 0x0e),
]
_LETTERS = [
    (0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11), (0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e),
    (0x0e, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0e), (0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e),
    (0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f), (0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10),
    (0x0e, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0f), (0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11),
    (0x0e, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e), (0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0c),
    (0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11), (0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f),
    (0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11), (0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11),
    (0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e), (0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10),
    (0x0e, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0d), (0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11),
    (0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e), (0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04),
    (0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e), (0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04),
    (0x11, 0x11, 0x11, 0x15, 0x15, 0x1b, 0x11), (0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11),
    (0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04), (0x1f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1f),
]
_PUNCT = {
    ".": (0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c), ":": (0x00, 0x0c, 0x0c, 0x00, 0x0c, 0x0c, 0x00),
    "+": (0x00, 0x04, 0x04, 0x1f, 0x04, 0x04, 0x00), "%": (0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03),
    "#": (0x0a, 0x0a, 0x1f, 0x0a, 0x1f, 0x0a, 0x0a), "<": (0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02),
    ">": (0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08), "!": (0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04),
    "(": (0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02), ")": (0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08),
    "_": (0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f), "=": (0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00),
    "*": (0x00, 0x04, 0x15, 0x0e, 0x15, 0x04, 0x00), "'": (0x04, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00),
    ",": (0x00, 0x00, 0x00, 0x00, 0x0c, 0x04, 0x08), "?": (0x0e, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04),
    "|": (0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04), "[": (0x0e, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0e),
    "]": (0x0e, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0e), "&": (0x0c, 0x12, 0x14, 0x08, 0x15, 0x12, 0x0d),
    "-": (0, 0, 0, 0x1f, 0, 0, 0), "/": (0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10),
}
_LOWER = [
    (0x00, 0x00, 0x0e, 0x01, 0x0f, 0x11, 0x0f), (0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x1e),
    (0x00, 0x00, 0x0e, 0x10, 0x10, 0x11, 0x0e), (0x01, 0x01, 0x0d, 0x13, 0x11, 0x11, 0x0f),
    (0x00, 0x00, 0x0e, 0x11, 0x1f, 0x10, 0x0e), (0x06, 0x09, 0x08, 0x1c, 0x08, 0x08, 0x08),
    (0x00, 0x00, 0x0f, 0x11, 0x11, 0x0f, 0x01), (0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x11),
    (0x04, 0x00, 0x0c, 0x04, 0x04, 0x04, 0x0e), (0x02, 0x00, 0x06, 0x02, 0x02, 0x02, 0x12),
    (0x10, 0x10, 0x12, 0x14, 0x18, 0x14, 0x12), (0x0c, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e),
    (0x00, 0x00, 0x1a, 0x15, 0x15, 0x11, 0x11), (0x00, 0x00, 0x16, 0x19, 0x11, 0x11, 0x11),
    (0x00, 0x00, 0x0e, 0x11, 0x11, 0x11, 0x0e), (0x00, 0x00, 0x1e, 0x11, 0x11, 0x1e, 0x10),
    (0x00, 0x00, 0x0f, 0x11, 0x11, 0x0f, 0x01), (0x00, 0x00, 0x16, 0x19, 0x10, 0x10, 0x10),
    (0x00, 0x00, 0x0f, 0x10, 0x0e, 0x01, 0x1e), (0x08, 0x08, 0x1c, 0x08, 0x08, 0x09, 0x06),
    (0x00, 0x00, 0x11, 0x11, 0x11, 0x13, 0x0d), (0x00, 0x00, 0x11, 0x11, 0x11, 0x0a, 0x04),
    (0x00, 0x00, 0x11, 0x11, 0x15, 0x15, 0x0a), (0x00, 0x00, 0x11, 0x0a, 0x04, 0x0a, 0x11),
    (0x00, 0x00, 0x11, 0x11, 0x11, 0x0f, 0x01), (0x00, 0x00, 0x1f, 0x02, 0x04, 0x08, 0x1f),
]
_BLANK = (0,) * 7
_DESCENDERS = {"g": 0x0e, "j": 0x0c, "p": 0x10, "q": 0x01, "y": 0x0e}

# Firmware 0.99.6 draws DAW text with lowercase letters; set False to see
# how older firmware renders the same frames.
MIXED_CASE = True


def glyph(ch):
    if "0" <= ch <= "9":
        return _DIGITS[ord(ch) - 48]
    if MIXED_CASE and "a" <= ch <= "z":
        return _LOWER[ord(ch) - 97]
    ch = ch.upper()
    if "A" <= ch <= "Z":
        return _LETTERS[ord(ch) - 65]
    return _PUNCT.get(ch, _BLANK)


def rgb565(r5, g6, b5):
    return ((r5 & 0x1F) << 11) | ((g6 & 0x3F) << 5) | (b5 & 0x1F)


def _u14(p, i):
    return ((p[i] & 0x7F) << 7) | (p[i + 1] & 0x7F)


def _rgb(p, i):
    return rgb565(p[i], p[i + 1], p[i + 2])


def _coverage(v):
    return 0 if v < 0 else (16 if v > 16 else v)


def blend(bg, fg, cov):
    if not cov:
        return bg
    if cov >= 16:
        return fg
    inv = 16 - cov
    r = ((((bg >> 11) & 31) * inv + ((fg >> 11) & 31) * cov + 8) >> 4)
    g = ((((bg >> 5) & 63) * inv + ((fg >> 5) & 63) * cov + 8) >> 4)
    b = (((bg & 31) * inv + (fg & 31) * cov + 8) >> 4)
    return (r << 11) | (g << 5) | b


def _cdiv(a, b):
    """C integer division (truncates toward zero)."""
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b >= 0) else -q


def _shr(a, n):
    return a >> n  # arithmetic shift, as on the device


class Widget(object):
    __slots__ = ("kind", "x", "y", "w", "h", "value", "colors", "valid", "painted")

    def __init__(self):
        self.kind = 0
        self.x = self.y = self.w = self.h = 0
        self.value = 0
        self.colors = (0, 0, 0, 0, 0)
        self.valid = False
        self.painted = 0


class Pedal(object):
    """Screen model of one Nexfoot in DAW mode (USB, hardware SPI)."""

    def __init__(self):
        self.fb = [0] * (W * H)
        self.leds = 0
        self.active = False
        self.waiting = False
        self.ops_seen = 0
        self._reset_view()

    # -- primitives ---------------------------------------------------------
    def fill(self, x, y, w, h, color):
        if x < 0:
            w += x
            x = 0
        if y < 0:
            h += y
            y = 0
        w = min(w, W - x)
        h = min(h, H - y)
        if w <= 0 or h <= 0:
            return
        fb = self.fb
        for yy in range(y, y + h):
            base = yy * W
            fb[base + x:base + x + w] = [color] * w

    def put(self, x, y, color):
        if 0 <= x < W and 0 <= y < H:
            self.fb[y * W + x] = color

    def glyph(self, x, y, ch, scale, fg, bg):
        rows = glyph(ch)
        for row in range(7 * scale):
            bits = rows[row // scale]
            for col in range(5 * scale):
                self.put(x + col, y + row, fg if bits & (1 << (4 - col // scale)) else bg)

    def text(self, x, y, text, scale, fg, bg):
        for ch in text:
            self.glyph(x, y, ch, scale, fg, bg)
            if MIXED_CASE and y + 8 * scale <= H:
                bits = _DESCENDERS.get(ch, 0)
                for row in range(scale):
                    for col in range(5 * scale):
                        self.put(x + col, y + 7 * scale + row,
                                 fg if bits & (1 << (4 - col // scale)) else bg)
            x += 6 * scale

    @staticmethod
    def text_width(n, scale):
        return n * 5 * scale + (n - 1) * scale if n else 0

    def text_centered(self, x, box_w, y, box_h, text, max_scale, fg, bg):
        self.fill(x, y, box_w, box_h, bg)
        text = text[:20]
        n = len(text)
        scale = 0
        while n:
            scale = next((s for s in range(max_scale, 0, -1) if self.text_width(n, s) <= box_w), 0)
            if scale:
                break
            n -= 1
        if not n:
            return
        w = self.text_width(n, scale)
        yy = y + ((box_h - 7 * scale) // 2 if box_h > 7 * scale else 0)
        self.text(x + (box_w - w) // 2, yy, text[:n], scale, fg, bg)

    # -- legacy card view ---------------------------------------------------
    def _reset_view(self):
        self.title = "DAW"
        self.status = ""
        self.accent = 0x07FF
        self.labels = [""] * 4
        self.values = [""] * 4
        self.colors = [0x8410] * 4
        self.meters = [0] * 4
        self.flags = [0] * 4
        self.canvas = False
        self.widgets = [Widget() for _ in range(4)]
        self.cutoff_valid = False
        self.cutoff_prev = 0
        self.cutoff_compact = False

    def _legacy(self):
        bg_top = 0
        self.fill(0, 0, W, TOP_BAR_H - 1, bg_top)
        self.fill(0, TOP_BAR_H - 1, W, 1, self.accent)
        self.fill(6, 6, 6, 15, self.accent)
        status = self.status[:8]
        sw = self.text_width(len(status), 2)
        if status:
            self.text(W - 8 - sw, 7, status, 2, self.accent, bg_top)
        room = W - 18 - 8 - (sw + 10 if status else 0)
        title = self.title[:20]
        while title and self.text_width(len(title), 2) > room:
            title = title[:-1]
        if title:
            self.text(18, 7, title, 2, 0xFFFF, bg_top)
        for i in range(4):
            self._card(i)

    def _card(self, i):
        x, y = i * CARD_W, CARD_Y
        color, flags, meter = self.colors[i], self.flags[i], min(127, self.meters[i])
        lit = flags & 1
        bg = (((color >> 11) >> 2) << 11 | ((((color >> 5) & 63) >> 2) << 5) | ((color & 31) >> 2)) if lit else 0x0841
        self.fill(x, y, CARD_W, CARD_H, 0)
        self.fill(x + 2, y + 2, CARD_W - 4, CARD_H - 4, bg)
        self.fill(x + 2, y + 2, CARD_W - 4, 4, color)
        self.text_centered(x + 4, CARD_W - 8, y + 12, 14, self.labels[i], 2, 0xFFFF, bg)
        self.text_centered(x + 2, CARD_W - 4, y + 42, 28, self.values[i], 3, 0xFFFF if lit else color, bg)
        mx, mw = x + 8, CARD_W - 16
        if flags & 2:
            # frame
            self.fill(mx, y + 92, mw, 1, 0x2104)
            self.fill(mx, y + 103, mw, 1, 0x2104)
            self.fill(mx, y + 92, 1, 12, 0x2104)
            self.fill(mx + mw - 1, y + 92, 1, 12, 0x2104)
            mx += 2
            mw -= 4
            self.fill(mx, y + 94, mw, 8, bg)
            if flags & 4:
                mid = mx + mw // 2
                if meter >= 64:
                    f = (meter - 64) * (mw // 2) // 63
                    self.fill(mid, y + 94, f or 1, 8, color)
                else:
                    f = (64 - meter) * (mw // 2) // 64
                    self.fill(mid - f, y + 94, f, 8, color)
                self.fill(mid, y + 93, 1, 10, 0xFFFF)
            elif meter:
                self.fill(mx, y + 94, meter * mw // 127, 8, color)
        else:
            self.fill(mx, y + 92, mw, 12, bg)
        self.text_centered(x + 4, CARD_W - 8, y + 124, 14, str(i + 1), 2, color if lit else 0x8410, bg)

    def _waiting(self):
        body, ink = 0xAD75, 0x2965
        self.fill(0, 0, W, H, body)
        self.fill(0, 0, W, 26, 0xBDF7)
        self.text(12, 6, "ABLETON LIVE", 2, ink, 0xBDF7)
        self.text_centered(0, W, 56, 42, "WAITING FOR LIVE", 3, ink, body)
        self.text_centered(0, W, 145, 21, "DOUBLE EDIT TO EXIT", 2, ink, body)

    # -- canvas -------------------------------------------------------------
    def arc(self, cx, cy, r, th, a0, a1, color):
        d0 = 135 + (a0 * 270) // 127
        d1 = 135 + (a1 * 270) // 127
        rm = r - th // 2
        if th < 1 or rm < 1:
            return
        for d in range(d0, d1 + 1):
            x = cx + _shr(_sin(d + 90) * rm, 12)
            y = cy + _shr(_sin(d) * rm, 12)
            self.fill(x - th // 2, y - th // 2, th, th, color)

    @staticmethod
    def _circle(square, radius):
        limit = radius * radius
        if square <= limit - radius:
            return 16
        if square >= limit + radius:
            return 0
        return _cdiv((limit + radius - square) * 8, radius)

    @staticmethod
    def _sin_q4(angle):
        degree, part = angle >> 4, angle & 15
        a, b = _sin(degree), _sin(degree + 1)
        return a + _shr((b - a) * part, 4)

    def knob(self, cx, cy, r, value, colors):
        accent, track, background, face, pointer = colors
        if r < 18 or r > 48 or cx - r < 0 or cx + r >= W or cy - r < 0 or cy + r >= H:
            return
        angle = 135 * 16 + _cdiv(value * 270 * 16 + 63, 127)
        ex, ey = self._sin_q4(angle + 90 * 16), self._sin_q4(angle)
        for dy in range(-r, r + 1):
            for x in range(-r, r + 1):
                square = x * x + dy * dy
                outer = self._circle(square, r - 1)
                inner = 16 - self._circle(square, r - 4)
                ring = min(outer, inner)
                color = background
                if face != background:
                    color = blend(color, face, self._circle(square, r - 9))
                if ring:
                    start = _coverage(8 + _shr(-2896 * (x + dy), 8))
                    total_end = _coverage(8 + _shr(2896 * (x - dy), 8))
                    total = max(start, total_end)
                    color = blend(color, track, (ring * total + 8) >> 4)
                    if value:
                        end = _coverage(8 + _shr(x * ey - dy * ex, 8))
                        swept = max(start, end) if value * 3 > 254 else min(start, end)
                        color = blend(color, accent, (ring * swept + 8) >> 4)
                elif (r - 18) * (r - 18) <= square <= (r - 9) * (r - 9):
                    along = _shr(x * ex + dy * ey, 8)
                    across = abs(_shr(x * ey - dy * ex, 8))
                    side = _coverage(32 - across)
                    start = _coverage(along - (r - 17) * 16 + 8)
                    end = _coverage((r - 10) * 16 + 8 - along)
                    color = blend(color, pointer, min(side, min(start, end)))
                self.put(cx + x, cy + dy, color)

    def slider(self, wd):
        vertical = wd.kind == 3
        length = wd.h if vertical else wd.w
        thick = wd.w if vertical else wd.h
        position = 2 * 16 + _cdiv((length - 5) * 16 * wd.value + 63, 127)
        accent, track, background, _face, pointer = wd.colors
        for along in range(length):
            for cross in range(thick):
                delta = along * 16 - position
                color = background
                if (thick - 4) // 2 <= cross < (thick - 4) // 2 + 4:
                    color = accent if delta <= 0 else track
                color = blend(color, pointer, _coverage(32 - abs(delta)))
                if vertical:
                    self.put(wd.x + cross, wd.y + length - 1 - along, color)
                else:
                    self.put(wd.x + along, wd.y + cross, color)

    def cutoff(self, value, compact=False):
        X0, CW = 108, 204
        Y0, CH = (32, 104) if compact else (36, 130)
        bgc, grid, trace, mark = 0x2104, 0x4228, 0x05B9, 0xF583
        start = 6 * 16 + _cdiv(value * 133 * 16, 127)
        marker = start + 37 * 16

        def sample(x):
            d = x * 16 - start
            y = 64 * 16
            if d > 0:
                y += _cdiv(d * d, 91 * 16)
            y = min(y, 117 * 16)
            return 38 * 16 + (y - 64 * 16) * 9 // 10 if compact else y

        ys = [sample(x) for x in range(CW + 1)]
        for y in range(CH):
            grid_row = y in ((18, 38, 70) if compact else (32, 64, 96))
            for x in range(CW):
                color = grid if (grid_row or x in (48, 102, 156)) else bgc
                row_top = y << 4
                lower, upper = ys[x], ys[x + 1] + 32
                if row_top < upper and row_top + 16 > lower:
                    hit = min(row_top + 16, upper) - max(row_top, lower)
                    stroke = blend(trace, mark, _coverage(32 - abs(x * 16 + 8 - marker)))
                    color = blend(color, stroke, hit)
                self.put(X0 + x, Y0 + y, color)
        self.cutoff_prev = value
        self.cutoff_compact = compact
        self.cutoff_valid = True

    def paint_widgets(self):
        for wd in self.widgets:
            if not wd.kind or (wd.valid and wd.value == wd.painted):
                continue
            if wd.x + wd.w > W or wd.y + wd.h > H or wd.value > 127:
                continue
            if wd.kind == 1:
                r = wd.w // 2
                if wd.h != wd.w or not (wd.w & 1) or not 18 <= r <= 48:
                    continue
                self.knob(wd.x + r, wd.y + r, r, wd.value, wd.colors)
            elif wd.kind in (2, 3):
                if wd.w < 6 or wd.h < 6:
                    continue
                self.slider(wd)
            else:
                continue
            wd.painted, wd.valid = wd.value, True

    def run_draw(self, p):
        i, n = 0, len(p)
        while i < n:
            op = p[i]
            i += 1
            self.ops_seen += 1
            if op == 0x10:
                if i + 3 > n:
                    return
                self.cutoff_valid = False
                self.widgets = [Widget() for _ in range(4)]
                self.fill(0, 0, W, H, _rgb(p, i))
                i += 3
            elif op == 0x11:
                if i + 11 > n:
                    return
                x, y, w, h = (_u14(p, i + offset) for offset in (0, 2, 4, 6))
                top, bottom = (32, 136) if self.cutoff_compact else (36, 166)
                if x < 312 and x + w > 108 and y < bottom and y + h > top:
                    self.cutoff_valid = False
                self.fill(x, y, w, h, _rgb(p, i + 8))
                i += 11
            elif op == 0x12:
                if i + 12 > n:
                    return
                x, y, scale, cnt = _u14(p, i), _u14(p, i + 2), p[i + 4], p[i + 11]
                if i + 12 + cnt > n:
                    return
                if 1 <= scale <= 8 and x < W and y + 7 * scale <= H:
                    fit = (W - x + scale) // (6 * scale)
                    cnt2 = min(cnt, fit, 48)
                    txt = "".join(chr(c) for c in p[i + 12:i + 12 + cnt2])
                    fg, bg = _rgb(p, i + 5), _rgb(p, i + 8)
                    self.text(x, y, txt, scale, fg, bg)
                    for k in range(1, cnt2):
                        self.fill(x + k * 6 * scale - scale, y, scale, 7 * scale, bg)
                i += 12 + cnt
            elif op == 0x13:
                if i + 11 > n:
                    return
                self.arc(_u14(p, i), _u14(p, i + 2), p[i + 4], p[i + 5], p[i + 6], p[i + 7], _rgb(p, i + 8))
                i += 11
            elif op in (0x15, 0x19):
                if i >= n:
                    return
                compact = op == 0x19
                if not (self.cutoff_valid and p[i] == self.cutoff_prev and compact == self.cutoff_compact):
                    self.cutoff(p[i] & 0x7F, compact)
                i += 1
            elif op == 0x16:
                if i + 21 > n:
                    return
                colors = tuple(_rgb(p, i + 6 + 3 * k) for k in range(5))
                self.knob(_u14(p, i), _u14(p, i + 2), p[i + 4], p[i + 5], colors)
                i += 21
            elif op == 0x17:
                if i + 26 > n:
                    return
                if p[i] < 4:
                    wd = self.widgets[p[i]]
                    wd.kind = p[i + 1]
                    wd.x, wd.y, wd.w, wd.h = _u14(p, i + 2), _u14(p, i + 4), _u14(p, i + 6), _u14(p, i + 8)
                    wd.value = min(127, p[i + 10])
                    wd.colors = tuple(_rgb(p, i + 11 + 3 * k) for k in range(5))
                    wd.valid = False
                i += 26
            elif op == 0x18:
                if i + 2 > n:
                    return
                if p[i] < 4:
                    self.widgets[p[i]].value = min(127, p[i + 1])
                i += 2
            elif op == 0x14:
                if i + 12 > n:
                    return
                x, ym, hh, cnt = _u14(p, i), _u14(p, i + 2), p[i + 4], p[i + 11]
                fg, bg = _rgb(p, i + 5), _rgb(p, i + 8)
                if i + 12 + cnt > n:
                    return
                for k in range(cnt):
                    h = max(1, min(hh, p[i + 12 + k] * hh // 127))
                    self.fill(x + k, ym - hh, 1, hh - h, bg)
                    self.fill(x + k, ym - h, 1, 2 * h, fg)
                    self.fill(x + k, ym + h, 1, hh - h, bg)
                i += 12 + cnt
            else:
                return

    # -- protocol -----------------------------------------------------------
    def receive(self, frame):
        """Apply one NanoBridge frame (F0 7D 4E 42 01 CMD SEQ ... SUM F7)."""
        f = list(frame)
        if len(f) < 9 or f[0] != 0xF0 or f[1:4] != [0x7D, 0x4E, 0x42]:
            return
        cmd, payload = f[5], f[7:-2]
        if cmd == 0x4E:
            if payload and payload[0]:
                self._reset_view()
                self.active, self.waiting = True, False
                self._legacy()
            else:
                self.active = False
        elif cmd == 0x4F and self.active:
            self._update(payload)
            if not self.canvas:
                self._legacy()
        elif cmd == 0x50 and self.active:
            self.canvas = True
            self.run_draw(payload)
            self.paint_widgets()

    def _update(self, p):
        i = 0
        while i < len(p):
            op = p[i]
            i += 1
            if op == 1:
                field, n = p[i], p[i + 1]
                text = "".join(chr(c) if 0x20 <= c <= 0x7E else " " for c in p[i + 2:i + 2 + n])
                i += 2 + n
                if field == 0:
                    self.title = text[:20]
                elif field == 1:
                    self.status = text[:8]
                elif 2 <= field < 6:
                    self.labels[field - 2] = text[:10]
                elif 6 <= field < 10:
                    self.values[field - 6] = text[:8]
            elif op == 2:
                k = p[i] - 1
                if 0 <= k < 4:
                    self.meters[k] = p[i + 1]
                    self.colors[k] = rgb565(p[i + 2], p[i + 3], p[i + 4])
                    self.flags[k] = p[i + 5]
                i += 6
            elif op == 3:
                self.leds = p[i] & 0x0F
                i += 1
            elif op == 4:
                self.accent = rgb565(p[i], p[i + 1], p[i + 2])
                i += 3
            elif op == 5:
                i += 6
            else:
                return

    def set_waiting(self, waiting=True):
        self.waiting = waiting
        if waiting:
            self._waiting()

    # -- output -------------------------------------------------------------
    def rgb_rows(self):
        out = []
        for y in range(H):
            row = []
            for c in self.fb[y * W:(y + 1) * W]:
                r, g, b = (c >> 11) & 31, (c >> 5) & 63, c & 31
                row.append(((r * 527 + 23) >> 6, (g * 259 + 33) >> 6, (b * 527 + 23) >> 6))
            out.append(row)
        return out

    def image(self, scale=2, leds=True):
        from PIL import Image, ImageDraw
        img = Image.new("RGB", (W, H))
        img.putdata([px for row in self.rgb_rows() for px in row])
        img = img.resize((W * scale, H * scale), Image.NEAREST)
        if not leds:
            return img
        pad = 14 * scale
        out = Image.new("RGB", (W * scale, H * scale + pad), (24, 24, 24))
        out.paste(img, (0, 0))
        d = ImageDraw.Draw(out)
        for i in range(4):
            cx = (i * 80 + 40) * scale
            cy = H * scale + pad // 2
            on = self.leds & (1 << i)
            d.ellipse((cx - 4 * scale, cy - 3 * scale, cx + 4 * scale, cy + 3 * scale),
                      fill=(80, 220, 255) if on else (55, 55, 55))
        return out

    def save(self, path, scale=2, leds=True):
        self.image(scale, leds).save(path)


def _demo(path):
    sys.path.insert(0, __file__.rsplit("/", 2)[0] + "/common")
    import nexfoot_daw as nf
    pedal = Pedal()
    view = nf.View(pedal.receive)
    pedal.receive(nf.mode_frame(True, nf.HOST_ABLETON))
    view.title, view.status = "MIX 1-Audio", "STOP"
    for i, (label, value) in enumerate((("TRACK", "1-Audio"), ("VOLUME", "0.0 dB"), ("PAN", "C"), ("SEND A", "-inf"))):
        view.cards[i].label, view.cards[i].value = label, value
        view.cards[i].flags = nf.FLAG_METER
        view.cards[i].meter = 90
        view.cards[i].color = 0x3CDC5A
    while view.flush(max_frames=8):
        pass
    pedal.save(path)


def replay(capture_path, out_path, until=None):
    """Render a NEXFOOT_CAPTURE file (``seconds HEX...`` per line) as of
    ``until`` seconds, or its end."""
    pedal = Pedal()
    with open(capture_path) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if until is not None and float(parts[0]) > until:
                break
            pedal.receive([int(b, 16) for b in parts[1:]])
    pedal.save(out_path)
    return pedal


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "demo":
        _demo(sys.argv[2])
    elif len(sys.argv) in (4, 5) and sys.argv[1] == "replay":
        replay(sys.argv[2], sys.argv[3], float(sys.argv[4]) if len(sys.argv) == 5 else None)
    else:
        print(__doc__)
