# Copyright 2026 Nexfoot contributors. Licensed under the Apache License 2.0.
"""Offline tests for the DAW protocol and display SDK.

    python3 -m unittest discover -s daw/tests -v
"""
import os
import sys
import unittest

DAW = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(DAW, "common"))
import nexfoot_daw as nf  # noqa: E402


def parse_frame(f):
    assert f[0] == 0xF0 and f[-1] == 0xF7
    body = list(f[1:-1])
    assert body[:3] == list(nf.PREFIX)
    assert all(0 <= b <= 0x7F for b in body)
    assert nf.checksum(body[3:-1]) == body[-1]
    return body[4], body[5], body[6:-1]


def parse_ops(payload):
    ops = []
    i = 0
    while i < len(payload):
        op = payload[i]
        if op == nf.OP_TEXT:
            n = payload[i + 2]
            ops.append(("text", payload[i + 1], bytes(payload[i + 3:i + 3 + n]).decode()))
            i += 3 + n
        elif op == nf.OP_CARD:
            ops.append(("card",) + tuple(payload[i + 1:i + 7]))
            i += 7
        elif op == nf.OP_LEDS:
            ops.append(("leds", payload[i + 1]))
            i += 2
        elif op == nf.OP_ACCENT:
            ops.append(("accent",) + tuple(payload[i + 1:i + 4]))
            i += 4
        else:
            raise AssertionError("unknown op %r" % op)
    return ops


def draw_ops(frames):
    result = []
    for frame in frames:
        command, _, payload = parse_frame(frame)
        if command != nf.CMD_DAW_DRAW:
            continue
        while payload:
            op = payload[0]
            if op == nf.OP_CLEAR:
                end = 4
            elif op in (nf.OP_CUTOFF, nf.OP_COMPACT_CUTOFF):
                end = 2
            elif op == nf.OP_WIDGET:
                end = 27
            elif op == nf.OP_VALUE:
                end = 3
            elif op == nf.OP_DIAL:
                end = 22
            elif op in (nf.OP_RECT, nf.OP_ARC):
                end = 12
            elif op == nf.OP_TEXTXY:
                end = 13 + payload[12]
            else:
                raise AssertionError("unknown draw op %r" % op)
            if end > len(payload):
                raise AssertionError("draw command split across frames")
            result.append(payload[:end])
            payload = payload[end:]
    return result


class TestClock:
    def __init__(self):
        self.now = 0

    def __call__(self):
        return self.now

    def tick(self, view, paint=True, step=0.045):
        self.now += step
        return view.flush(paint=paint)


class FocusedDisplayTests(unittest.TestCase):
    def test_value_updates_locally_and_redraw_recovers(self):
        for host in (nf.HOST_ABLETON, nf.HOST_FL_STUDIO):
            with self.subTest(host=host):
                frames = []
                clock = TestClock()
                view = nf.FocusedView(frames.append, host, clock=clock)
                view.panel_title = "Synth"
                clock.tick(view)
                self.assertEqual(draw_ops(frames), [])
                view.set_active(True)
                view.cards[0].label = "Cutoff"
                view.cards[0].value = "3.80 kHz"
                view.cards[0].meter = 80
                view.cards[1].label = "Resonance"
                for _ in range(20):
                    clock.tick(view)
                first = draw_ops(frames)
                self.assertEqual(sum(op[0] == nf.OP_CLEAR for op in first), 1)
                text = [bytes(op[13:]).decode().strip() for op in first if op[0] == nf.OP_TEXTXY]
                self.assertEqual(text, ["Cutoff", "3.80 kHz", "Synth"])
                frames.clear()
                view.cards[1].value = "20%"
                clock.tick(view)
                self.assertEqual(draw_ops(frames), [])
                frames.clear()
                view.cards[0].value = "3.92 kHz"
                view.cards[0].meter = 82
                clock.tick(view, paint=False)
                changes = draw_ops(frames)
                self.assertTrue(any(op[0] == nf.OP_ARC for op in changes))
                self.assertTrue(all(op[0] in (nf.OP_TEXTXY, nf.OP_ARC) for op in changes))
                self.assertEqual([bytes(op[13:]).strip() for op in changes if op[0] == nf.OP_TEXTXY],
                                 [b"3.92 kHz"])
                frames.clear()
                view.focus = 1
                for _ in range(5):
                    clock.tick(view)
                text = [bytes(op[13:]).strip() for op in draw_ops(frames) if op[0] == nf.OP_TEXTXY]
                self.assertEqual(text, [b"Resonance", b"20%"])
                frames.clear()
                view.invalidate()
                for _ in range(20):
                    clock.tick(view)
                self.assertEqual(sum(op[0] == nf.OP_CLEAR for op in draw_ops(frames)), 1)

    def test_startup_retries_missed_state_once_and_keeps_frames_short(self):
        frames = []
        view = nf.FocusedView(frames.append, nf.HOST_ABLETON)
        view.title = "A" * 20
        for card in view.cards:
            card.label = "ABCDEFGH"
            card.value = "9.99 kHz"
        view.start_session()
        for _ in range(22):
            view.flush()
        modes = [parse_frame(frame)[2][0] for frame in frames if frame[5] == nf.CMD_DAW_MODE]
        self.assertEqual(modes, [1, 0, 1])
        self.assertTrue(all(len(frame) <= 48 for frame in frames))
        self.assertEqual(draw_ops(frames), [])
        clock = TestClock()
        view._clock = clock
        view.set_active(True)
        for _ in range(22):
            clock.tick(view)
        self.assertTrue(all(len(frame) <= 48 for frame in frames))
        self.assertTrue(draw_ops(frames))

    def test_freq_readout_updates_before_the_curve_and_other_knobs_keep_panel(self):
        frames = []
        clock = TestClock()
        view = nf.FocusedView(frames.append, nf.HOST_ABLETON, clock=clock)
        view.set_active(True)
        view.filter_meter = 80
        view.panel_title = "Auto Filter"
        view.cards[0].label = "Frequency"
        view.cards[0].value = "3.80 kHz"
        view.cards[0].meter = 80
        view.cards[1].label = "Resonance"
        view.cards[1].value = "2.0 %"
        for _ in range(32):
            clock.tick(view)
        frames.clear()
        view.cards[0].value = "4.40 kHz"
        view.cards[0].meter = view.filter_meter = 84
        clock.tick(view, paint=False)
        commands = draw_ops(frames)
        self.assertEqual([bytes(op[13:]).strip() for op in commands if op[0] == nf.OP_TEXTXY], [b"4.40 kHz"])
        self.assertFalse(any(op[0] in (nf.OP_CLEAR, nf.OP_RECT) for op in commands))
        # Cutoff movement is painted on the regular display tick.
        frames.clear()
        view.cards[0].meter = view.filter_meter = 85
        view.cards[0].value = "4.50 kHz"
        clock.tick(view, paint=False)
        self.assertIn(b"4.50 kHz", [bytes(op[13:]).strip() for op in draw_ops(frames) if op[0] == nf.OP_TEXTXY])
        frames.clear()
        view.focus = 1
        clock.tick(view, paint=False)
        for _ in range(36):
            clock.tick(view)
        commands = draw_ops(frames)
        self.assertIn(b"Res", [bytes(op[13:]).strip() for op in commands if op[0] == nf.OP_TEXTXY])
        self.assertFalse(any(op[0] == nf.OP_CLEAR for op in commands))
        self.assertFalse(any(y == b"Auto Filter" for y in
                             (bytes(op[13:]).strip() for op in commands if op[0] == nf.OP_TEXTXY)))


    def test_cutoff_plot_coalesces_to_the_latest_position_in_one_frame(self):
        frames = []
        clock = TestClock()
        view = nf.FocusedView(frames.append, nf.HOST_ABLETON, clock=clock)
        view.set_active(True)
        view.filter_meter = 64
        for _ in range(24):
            clock.tick(view)
        frames.clear()
        view.filter_meter = 66
        clock.tick(view, paint=False)
        view.filter_meter = 70
        clock.tick(view, paint=False)
        self.assertEqual(frames, [])
        for _ in range(4):
            clock.tick(view)
        self.assertEqual(len([frame for frame in frames if frame[5] == nf.CMD_DAW_DRAW]), 1)
        self.assertEqual([op[0] for op in draw_ops(frames)], [nf.OP_RECT, nf.OP_RECT, nf.OP_ARC])
        self.assertTrue(all(len(frame) <= 48 for frame in frames))

    def test_effect_visuals_follow_parameters_and_coalesce(self):
        self.assertIsNone(nf.compression_visual("50%", "2:1"))
        self.assertIsNone(nf.compression_visual("-24 dB", "0:1"))
        unity = nf.effect_columns(nf.compression_visual("-24 dB", "1:1"))
        compressed = nf.effect_columns(nf.compression_visual("-24 dB", "4:1"))
        self.assertEqual(unity[:20], compressed[:20])
        self.assertGreater(unity[-1], compressed[-1])
        self.assertEqual(nf.effect_columns(("echo", 0, 0, "0%"))[7], 0)
        self.assertEqual(nf.effect_columns(("echo", 127, 0, "100%"))[7], 43)
        for name in ("Live Reverb", "Fruity Reeverb 2"):
            self.assertIsNone(nf.bank_visual((("Decay Time", 65, "2.52 s"), ("Predelay", 30, "18 ms")), 0, name))
        frames = []
        clock = TestClock()
        view = nf.FocusedView(frames.append, nf.HOST_ABLETON, clock=clock)
        view.set_active(True)
        view.set_widgets_supported(True)
        view.knob_bank = (("Feedback", 10, "8%"),)
        for amount in [16] * 25 + [60, 90, 120, 70] + [70] * 35:
            view.effect_visual = ("echo", amount, 0, "Feedback")
            clock.tick(view)
        self.assertEqual(tuple(view._graph_drawn), nf.effect_columns(view.effect_visual))
        self.assertTrue(all(len(packet) <= 48 for packet in frames))
        self.assertEqual(sum(op[0] == nf.OP_CLEAR for op in draw_ops(frames)), 1)

    def test_effect_panel_draws_only_the_focused_control(self):
        frames = []
        clock = TestClock()
        view = nf.FocusedView(frames.append, nf.HOST_ABLETON, clock=clock)
        view.set_active(True)
        view.set_widgets_supported(True)
        view.knob_bank = (("Attack", 23, "75 ms"), ("Release", 86, "1.87 s"))
        view.effect_visual = ("envelope", (23, 64, 80, 86), "Attack", "75 ms")
        for _ in range(40):
            clock.tick(view)
        ops = draw_ops(frames)
        self.assertEqual(sum(op[0] == nf.OP_WIDGET for op in ops), 1)
        self.assertIn(b"Attack", [bytes(op[13:]).strip() for op in ops if op[0] == nf.OP_TEXTXY])
        self.assertNotIn(b"Release", [bytes(op[13:]).strip() for op in ops if op[0] == nf.OP_TEXTXY])
        frames.clear()
        view.focus = 1
        for _ in range(12):
            clock.tick(view)
        ops = draw_ops(frames)
        self.assertIn([nf.OP_VALUE, 0, 86], ops)
        self.assertIn(b"Release", [bytes(op[13:]).strip() for op in ops if op[0] == nf.OP_TEXTXY])
        self.assertFalse(any(op[0] == nf.OP_CLEAR for op in ops))
        for visual in [("echo", 80, 0, "63%"), ("envelope", (23, 64, 80, 86), "Release", "1.87 s")]:
            frames.clear()
            view.effect_visual = visual
            for _ in range(40):
                clock.tick(view)
            self.assertEqual(sum(op[0] == nf.OP_CLEAR for op in draw_ops(frames)), 1)
            self.assertTrue(all(old != -1 for old in view._graph_drawn))

    def test_visuals_are_selected_from_exposed_plugin_controls(self):
        bank = (("Band 7 freq", 81, "2.2 kHz"), ("Band 7 level", 96, "6.0 dB"))
        self.assertEqual(nf.bank_visual(bank, 0, "Parametric EQ"), ("eq_band", 81, 96, "6.0 dB"))
        self.assertIsNone(nf.bank_visual(bank, 0, "FM tone"))
        self.assertEqual(nf.eq_groups(["Band 1 freq", "Band 2 gain", "Band 2 freq"]), [(("2",), [2, 1])])
        self.assertFalse(nf.envelope_groups(["Amp Attack", "Mod Decay", "Amp Sustain", "Amp Release"]))
        self.assertFalse(nf.envelope_groups(["Attack", "Decay", "Sustain", "Release", "Release"]))
        self.assertEqual(nf.bank_visual((("Waveform", 0, "Sine"),), 0)[0], "oscillator")
        self.assertIsNone(nf.bank_visual((("Ratio", 10, "25%"), ("Threshold", 20, "15%")), 0))

    def test_retained_knob_updates_and_redefines_after_redraw(self):
        for host in (nf.HOST_ABLETON, nf.HOST_FL_STUDIO):
            frames = []
            clock = TestClock()
            view = nf.FocusedView(frames.append, host, clock=clock)
            view.set_active(True)
            view.set_widgets_supported(True)
            for _ in range(30):
                clock.tick(view)
            self.assertEqual(sum(op[0] == nf.OP_WIDGET for op in draw_ops(frames)), 1)
            frames.clear()
            view.cards[0].meter = 75
            for _ in range(4):
                clock.tick(view)
            self.assertEqual(draw_ops(frames), [[nf.OP_VALUE, 0, 75]])
            frames.clear()
            view.invalidate()
            for _ in range(30):
                clock.tick(view)
            self.assertEqual(sum(op[0] == nf.OP_WIDGET for op in draw_ops(frames)), 1)
            self.assertTrue(all(len(frame) <= 48 for frame in frames))

    def test_native_dial_sends_one_composite_for_each_new_position(self):
        frames = []
        clock = TestClock()
        view = nf.FocusedView(frames.append, nf.HOST_FL_STUDIO, clock=clock)
        view.set_active(True)
        view.set_native_dial(True)
        view.cards[0].label = "Cutoff"
        view.cards[0].meter = 45
        for _ in range(24):
            clock.tick(view)
        self.assertEqual([op[6] for op in draw_ops(frames) if op[0] == nf.OP_DIAL], [45])
        frames.clear()
        view.cards[0].meter = 64
        clock.tick(view, paint=False)
        clock.tick(view, paint=False)
        self.assertEqual([op[6] for op in draw_ops(frames) if op[0] == nf.OP_DIAL], [64])
        self.assertFalse(any(op[0] == nf.OP_ARC for op in draw_ops(frames)))
        self.assertTrue(all(len(frame) <= 48 for frame in frames))


sys.path.insert(0, os.path.join(DAW, "tools"))
import nexfoot_screen  # noqa: E402


def screen_after(view, clock, ticks=80):
    """Run a FocusedView into the pixel emulator and return the pedal."""
    pedal = nexfoot_screen.Pedal()
    view._send = pedal.receive
    view._canvas._send = view._readout._send = pedal.receive
    pedal.receive(nf.mode_frame(True, nf.HOST_ABLETON))
    for _ in range(ticks):
        clock.tick(view)
    return pedal


def rgb(pedal, x, y):
    c = pedal.fb[y * nexfoot_screen.W + x]
    return ((c >> 11) & 31) << 3, ((c >> 5) & 63) << 2, (c & 31) << 3


class LayoutTests(unittest.TestCase):
    def view(self, host=nf.HOST_ABLETON):
        clock = TestClock()
        view = nf.FocusedView(lambda f: None, host, clock=clock)
        view.set_active(True)
        view.set_widgets_supported(True)
        view.set_native_dial(True)
        return view, clock

    def test_labels_abbreviate_like_hardware(self):
        self.assertEqual(nf.abbreviate_label("Decay Time"), "Decay")
        self.assertEqual(nf.abbreviate_label("Dry/Wet"), "Mix")
        self.assertEqual(nf.abbreviate_label("Threshold"), "Thresh")
        self.assertEqual(nf.abbreviate_label("Param 1"), "Param1")
        self.assertIsNone(nf.abbreviate_label("Glissando"))
        self.assertEqual(nf.short_label("Glissando", 5), "Glssn")
        self.assertEqual(nf.short_label("Threshold", 5), "Thrsh")

    def test_every_page_shares_header_strip_and_highlight(self):
        for host in (nf.HOST_ABLETON, nf.HOST_FL_STUDIO):
            view, clock = self.view(host)
            view.page_name = "MIX"
            view.context = "Drums"
            view.cards[1].label, view.cards[1].value, view.cards[1].meter = "Volume", "0.0 dB", 100
            view.focus = view.active_encoder = 1
            view.strip = (("Track", None, nf.STATE_ARM), ("Volume", 100, None),
                          ("Pan", 64, nf.STATE_SOLO, True), ("Send A", 0, None))
            pedal = screen_after(view, clock)
            theme = view._theme
            self.assertEqual(rgb(pedal, 6, 6), tuple(v & 0xF8 if i != 1 else v & 0xFC
                                                     for i, v in enumerate(((theme["chip"] >> 16) & 255,
                                                                           (theme["chip"] >> 8) & 255,
                                                                           theme["chip"] & 255))))
            # The second column is highlighted; arm and solo bars sit on 1 and 3.
            active = rgb(pedal, 120, nf.STRIP_MARKER_Y)
            idle = rgb(pedal, 40, nf.STRIP_MARKER_Y)
            self.assertNotEqual(active, idle)
            self.assertEqual(rgb(pedal, 40, nf.STRIP_Y + 4)[0] >> 3, (nf.STATE_ARM >> 19) & 31)
            self.assertEqual(rgb(pedal, 200, nf.STRIP_Y + 4)[2] >> 3, (nf.STATE_SOLO >> 3) & 31)

    def test_turning_all_four_encoders_keeps_bottom_captions_in_every_frame(self):
        view, clock = self.view()
        view.page_name = "DEVICE"
        view.context = "Auto Filter"
        view.strip = (("Frequency", 32, None), ("Resonance", 45, None),
                      ("Drive", 63, None), ("Dry/Wet", 95, None))
        initial_values = ("91.2 Hz", "35 %", "50 %", "75 %")
        for card, (label, meter, _), value in zip(view.cards, view.strip, initial_values):
            card.label, card.meter, card.value = label, meter, value
        pedal = screen_after(view, clock)
        start = (nf.STRIP_Y + 10) * nf.SCREEN_W
        end = (nf.STRIP_Y + 26) * nf.SCREEN_W
        captions = pedal.fb[start:end]
        frames = []
        view._send = view._canvas._send = view._readout._send = frames.append
        for index, meter, value in ((2, 65, "51 %"), (3, 127, "100 %"),
                                    (0, 92, "3.80 kHz"), (1, 98, "77 %")):
            with self.subTest(encoder=index + 1):
                frames.clear()
                view.focus = view.active_encoder = index
                control = (view.strip[index][0], meter, None)
                view.strip = (*view.strip[:index], control, *view.strip[index + 1:])
                view.cards[index].value, view.cards[index].meter = value, meter
                for _ in range(10):
                    clock.tick(view)
                for frame in frames:
                    pedal.receive(frame)
                    self.assertEqual(pedal.fb[start:end], captions)
                markers = [rgb(pedal, 40 + column * 80, nf.STRIP_MARKER_Y) for column in range(4)]
                self.assertEqual(markers.count(markers[index]), 1)
                self.assertIn(value.encode(), [bytes(op[13:]).strip() for op in draw_ops(frames)
                                               if op[0] == nf.OP_TEXTXY])

    def test_auto_filter_drive_percentage_overwrites_in_one_draw(self):
        view, clock = self.view()
        view.page_name = "DEVICE"
        view.context = "Auto Filter"
        view.focus = view.active_encoder = 2
        view.strip = (("Freq", 32, None), ("Res", 45, None), ("Drive", 127, None), ("Mix", 127, None))
        card = view.cards[2]
        card.label, card.value, card.meter = "Drive", "100 %", 127
        screen_after(view, clock)
        frames = []
        view._send = view._canvas._send = view._readout._send = frames.append
        view._readout.add(nf.draw_rect(12, nf.STRIP_Y + 29, 20, 3, nf.WHITE))
        origin = None
        for value, meter in (("99 %", 126), ("10 %", 13), ("9.0 %", 11), ("0.0 %", 0), ("100 %", 127)):
            with self.subTest(value=value):
                frames.clear()
                card.value, card.meter = value, meter
                view.strip = (*view.strip[:2], ("Drive", meter, None), view.strip[3])
                for _ in range(10):
                    clock.tick(view)
                ops = draw_ops(frames)
                texts = [op for op in ops if op[0] == nf.OP_TEXTXY]
                self.assertEqual([bytes(op[13:]).decode().strip() for op in texts], [value])
                if origin is None:
                    origin = texts[0][1:6]
                self.assertEqual(texts[0][1:6], origin)
                self.assertEqual(bytes(texts[0][13:]).index(b"%"), 6)
                self.assertFalse(any(op[0] == nf.OP_RECT and op[3:5] == origin[2:4] for op in ops))

    def test_a_new_focus_replaces_both_readouts_while_the_strip_is_busy(self):
        view, clock = self.view()
        view.page_name = "DEVICE"
        view.context = "Auto Filter"
        labels = ("Frequency", "Resonance", "Drive", "Dry/Wet")
        values = ("480 Hz", "56 %", "100 %", "65 %")
        view.strip = tuple((label, 50, None) for label in labels)
        for card, label, value in zip(view.cards, labels, values):
            card.label, card.value = label, value
        screen_after(view, clock)
        frames = []
        view._send = view._canvas._send = view._readout._send = frames.append
        # Emulate rapid clicks in Live, with the selected control going to knob 1.
        for index in (2, 3, 1, 0):
            with self.subTest(control=labels[index]):
                frames.clear()
                view.focus = index
                mapping = list(range(4))
                mapping[0], mapping[index] = mapping[index], mapping[0]
                view.strip = tuple((labels[i], 50, None) for i in mapping)
                clock.tick(view)
                ops = draw_ops(frames)
                self.assertEqual([bytes(op[13:]).decode().strip() for op in ops if op[0] == nf.OP_TEXTXY],
                                 [labels[index], values[index]])
        for _ in range(10):
            clock.tick(view)
        self.assertEqual(view._strip_captions._drawn[0].split(), ["Freq", "Res", "Drive", "Mix"])
        frames.clear()
        view.focus = 2
        view.strip = tuple((labels[i], 50, None) for i in (2, 1, 0, 3))
        for _ in range(5):
            clock.tick(view)
        rows = [op for op in draw_ops(frames) if op[0] == nf.OP_TEXTXY and
                op[3] * 128 + op[4] == nf.STRIP_Y + 10]
        self.assertEqual(len(rows), 1)
        self.assertEqual(bytes(rows[0][13:]).split(), [b"Drive", b"Res", b"Freq", b"Mix"])

    def test_cutoff_knob_arc_and_full_size_caption_row_stay_visible(self):
        view, clock = self.view()
        view.set_native_dial(False)
        view.page_name = "DEVICE"
        view.context = "Auto Filter"
        view.filter_meter = view.cards[0].meter = 80
        view.cards[0].label, view.cards[0].value = "Frequency", "3.80 kHz"
        view.cards[2].label, view.cards[2].value, view.cards[2].meter = "Drive", "50 %", 63
        view.strip = (("Freq", 80, None), ("Res", 45, None), ("Drive", 63, None), ("Mix", 127, None))
        pedal = screen_after(view, clock)
        start = (nf.STRIP_Y + 10) * nf.SCREEN_W
        end = (nf.STRIP_Y + 26) * nf.SCREEN_W
        captions = pedal.fb[start:end]

        def graph_pixels():
            return [pedal.fb[y * nf.SCREEN_W + nf.CUTOFF_X:y * nf.SCREEN_W + nf.CUTOFF_RIGHT]
                    for y in range(nf.CUTOFF_TOP, nf.STRIP_Y)]

        graph = graph_pixels()
        self.assertEqual(view._strip_captions.k, 2)
        self.assertEqual(view._strip_captions._drawn[0].split(), ["Freq", "Res", "Drive", "Mix"])
        frames = []
        view._send = view._canvas._send = view._readout._send = frames.append
        view.filter_meter = view.cards[0].meter = 100
        view.cards[0].value = "7.80 kHz"
        for _ in range(10):
            clock.tick(view)
        ops = draw_ops(frames)
        self.assertTrue(any(op[0] == nf.OP_ARC and op[1:5] == nf._u14(50) + nf._u14(79) for op in ops))
        self.assertFalse(any(op[0] in (nf.OP_CUTOFF, nf.OP_DIAL, nf.OP_WIDGET, nf.OP_CLEAR) for op in ops))
        for frame in frames:
            pedal.receive(frame)
            self.assertEqual(pedal.fb[start:end], captions)
        self.assertNotEqual(graph_pixels(), graph)

        graph = graph_pixels()
        frames.clear()
        view.focus = 2
        view.strip = (view.strip[2], view.strip[1], view.strip[0], view.strip[3])
        for _ in range(10):
            clock.tick(view)
        for frame in frames:
            pedal.receive(frame)
            self.assertEqual(graph_pixels(), graph)
        self.assertEqual(view._strip_captions._drawn[0].split(), ["Drive", "Res", "Freq", "Mix"])

    def test_compact_hardware_cutoff_and_direct_dial_preserve_the_caption_row(self):
        view, clock = self.view()
        view.set_compact_cutoff(True)
        view.page_name = "DEVICE"
        view.context = "Auto Filter"
        view.filter_meter = view.cards[0].meter = 65
        view.cards[0].label, view.cards[0].value = "Frequency", "1.75 kHz"
        view.strip = (("Freq", 65, None), ("Res", 42, None), ("Drive", 55, None), ("Mix", 127, None))
        pedal = screen_after(view, clock)
        start = (nf.STRIP_Y + 10) * nf.SCREEN_W
        end = (nf.STRIP_Y + 26) * nf.SCREEN_W
        captions = pedal.fb[start:end]
        self.assertTrue(pedal.cutoff_valid and pedal.cutoff_compact)

        frames = []
        view._send = view._canvas._send = view._readout._send = frames.append
        view.filter_meter = view.cards[0].meter = 100
        view.cards[0].value = "7.80 kHz"
        for _ in range(10):
            clock.tick(view)
        ops = draw_ops(frames)
        self.assertIn([nf.OP_COMPACT_CUTOFF, 100], ops)
        self.assertEqual([op[6] for op in ops if op[0] == nf.OP_DIAL], [100])
        self.assertFalse(any(op[0] in (nf.OP_CLEAR, nf.OP_ARC, nf.OP_CUTOFF, nf.OP_WIDGET) for op in ops))
        for frame in frames:
            pedal.receive(frame)
            self.assertEqual(pedal.fb[start:end], captions)
        self.assertEqual(pedal.cutoff_prev, 100)

    def test_long_name_survives_the_transport_readout(self):
        frames = []
        view, clock = self.view()
        view._send = view._canvas._send = view._readout._send = frames.append
        view.page_name = "DEVICE"
        view.context = "Compressor / Main"
        view.transport = ("stop", "")
        for _ in range(60):
            clock.tick(view)
        texts = [(i, op) for i, op in enumerate(draw_ops(frames))]
        name = max(i for i, op in texts if op[0] == nf.OP_TEXTXY and bytes(op[13:]) == b"Compressor / Main")
        later = [op for i, op in texts if i > name and op[0] == nf.OP_RECT]
        # Nothing painted after the name may reach into the header row it occupies.
        self.assertFalse([op for op in later if op[3] * 128 + op[4] < 20 and op[1] * 128 + op[2] < 298 and
                          op[1] * 128 + op[2] >= 92 and op[1] * 128 + op[2] < 92 + nf.text_width("x" * 17, 2)])

    def test_transport_shows_icon_and_position(self):
        frames = []
        view, clock = self.view()
        view._send = view._canvas._send = view._readout._send = frames.append
        view.transport = ("play", "12.3")
        for _ in range(60):
            clock.tick(view)
        ops = draw_ops(frames)
        self.assertIn(b"12.3", [bytes(op[13:]).strip() for op in ops if op[0] == nf.OP_TEXTXY])
        green = list(nf.rgb565_parts(nf.STATE_PLAY))
        self.assertTrue(any(op[0] == nf.OP_RECT and op[9:12] == green for op in ops))


class CaptureTests(unittest.TestCase):
    def test_capture_records_frames_only_when_asked(self):
        import tempfile
        sent = []

        def send(data):
            sent.append(data)
        self.assertIs(nf.capture(send), send)
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "capture.txt")
            os.environ["NEXFOOT_CAPTURE"] = path
            try:
                nf.capture(send)(nf.mode_frame(True))
            finally:
                del os.environ["NEXFOOT_CAPTURE"]
            with open(path) as f:
                line = f.read().split()
        self.assertEqual(sent, [nf.mode_frame(True)])
        self.assertEqual(line[1:], ["%02X" % b for b in nf.mode_frame(True)])


class ProtocolTests(unittest.TestCase):
    def test_ping_example_checksum(self):
        # NEXFOOT_SYSEX_API.md 2.4: F0 7D 4E 42 01 01 01 7D F7
        self.assertEqual(nf.frame(0x01, 1), (0xF0, 0x7D, 0x4E, 0x42, 0x01, 0x01, 0x01, 0x7D, 0xF7))

    def test_relative(self):
        self.assertEqual(nf.relative_delta(1), 1)
        self.assertEqual(nf.relative_delta(63), 63)
        self.assertEqual(nf.relative_delta(127), -1)
        self.assertEqual(nf.relative_delta(65), -63)
        self.assertEqual(nf.decode_cc(17, 127), ("rotate", 1, -1))
        self.assertEqual(nf.decode_cc(34, 127), ("combo_short", 2, 127))
        self.assertEqual(nf.decode_cc(37, 127), ("combo_long", 2, 127))
        self.assertEqual(nf.decode_cc(127, 0), ("state", 0, 0))

    def test_full_view_fits_frames(self):
        sent = []
        view = nf.View(sent.append, max_frames_per_flush=None, refresh_every=0)
        view.title = "X" * 40
        view.status = "123.4.1234"
        for i, c in enumerate(view.cards):
            c.label = "L" * 20
            c.value = "-12.50 dB"
            c.meter = 200
            c.flags = nf.FLAG_METER
        view.flush()
        self.assertTrue(sent)
        ops = []
        for f in sent:
            self.assertLessEqual(len(f), nf.FRAME_LIMIT)
            cmd, seq, payload = parse_frame(f)
            self.assertEqual((cmd, seq), (nf.CMD_DAW_UPDATE, 0))
            ops += parse_ops(payload)
        texts = {o[1]: o[2] for o in ops if o[0] == "text"}
        self.assertEqual(texts[nf.FIELD_TITLE], "X" * nf.TITLE_MAX)
        self.assertEqual(texts[nf.FIELD_VALUE], "-12.50dB")  # 9 chars: spaces dropped
        self.assertEqual(len([o for o in ops if o[0] == "card"]), 4)
        self.assertTrue(all(o[2] == 127 for o in ops if o[0] == "card"))
        # Nothing changed: nothing sent.
        del sent[:]
        view.flush()
        self.assertEqual(sent, [])

    def test_rate_limit_carries_over(self):
        sent = []
        view = nf.View(sent.append, max_frames_per_flush=1, refresh_every=0)
        for c in view.cards:
            c.label = "LABEL12345"
            c.value = "VALUE123"
        view.title = "TITLE"
        rounds = 0
        while True:
            n = view.flush()
            rounds += 1
            if not n:
                break
            self.assertEqual(n, 1)
        self.assertGreater(rounds, 2)
        all_ops = sum((parse_ops(parse_frame(f)[2]) for f in sent), [])
        labels = [o for o in all_ops if o[0] == "text" and nf.FIELD_LABEL <= o[1] < nf.FIELD_VALUE]
        self.assertEqual(len(labels), 4)

    def test_refresh_resends(self):
        sent = []
        view = nf.View(sent.append, max_frames_per_flush=2, refresh_every=2)
        view.flush()
        del sent[:]
        view.flush()  # tick 2: refresh part 0
        self.assertEqual(len(sent), 1)

    def test_mode_frame(self):
        cmd, seq, payload = parse_frame(nf.mode_frame(True, nf.HOST_ABLETON))
        self.assertEqual((cmd, seq, payload), (nf.CMD_DAW_MODE, 1, [1, 1]))


if __name__ == "__main__":
    unittest.main()
