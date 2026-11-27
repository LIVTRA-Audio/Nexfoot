#!/usr/bin/env python3
# Copyright 2026 Nexfoot contributors. Licensed under the Apache License 2.0.
"""Drive Nexfoot DAW mode from a computer, without a DAW.

Needs ``pip install mido python-rtmidi``. Useful to check a firmware build and
to see the control map before writing your own DAW integration.

    python daw/tools/nexfoot_daw_demo.py ports
    python daw/tools/nexfoot_daw_demo.py check          # enable, verify replies
    python daw/tools/nexfoot_daw_demo.py demo           # live demo, Ctrl+C quits
    python daw/tools/nexfoot_daw_demo.py off            # leave DAW mode

The demo shows four virtual knobs. Turn the pedal's encoders to change them,
press to toggle a card, use the edit key to change page; every control the
pedal sends is printed.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "common"))
import nexfoot_daw as nf  # noqa: E402

try:
    import mido
except ImportError:  # pragma: no cover
    sys.exit("This tool needs: pip install mido python-rtmidi")

PERFORM_HINTS = ("port 2", "nanocontrol", "midiin2", "midiout2")


def find_port(names, explicit, perform):
    """Pick the pedal's control (NanoBridge) or performance port.

    macOS: "Nexfoot MIDI Port 1" / "Nexfoot MIDI Port 2".
    Windows: "Nexfoot MIDI" / "MIDIIN2 (Nexfoot MIDI)" and "MIDIOUT2 (...)".
    """
    if explicit:
        for name in names:
            if explicit.lower() in name.lower():
                return name
        sys.exit("port not found: %s (have: %s)" % (explicit, ", ".join(names)))
    pedal = [n for n in names if "nexfoot" in n.lower() or "md400" in n.lower()]
    second = [n for n in pedal if any(h in n.lower() for h in PERFORM_HINTS)]
    first = [n for n in pedal if n not in second]
    pick = second if perform else first
    if pick:
        return pick[0]
    sys.exit("Nexfoot ports not found (have: %s). Use --control/--perform." % ", ".join(names))


def open_ports(args):
    control = find_port(mido.get_output_names(), args.control, False)
    control_in = find_port(mido.get_input_names(), args.control, False)
    perform_in = find_port(mido.get_input_names(), args.perform, True)
    print("control: %s   perform: %s" % (control, perform_in))
    return mido.open_output(control), mido.open_input(control_in), mido.open_input(perform_in)


def send(out, data):
    out.send(mido.Message("sysex", data=list(data[1:-1])))


def wait_reply(inp, command, seq, timeout=1.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for msg in inp.iter_pending():
            if msg.type != "sysex":
                continue
            body = list(msg.data)
            if body[:3] == list(nf.PREFIX) and len(body) >= 8 and body[4] == command and body[5] == seq:
                if nf.checksum(body[3:-1]) != body[-1]:
                    raise RuntimeError("bad reply checksum")
                return body[6], body[7:-1]
        time.sleep(0.005)
    raise TimeoutError("no reply to command 0x%02X seq %d" % (command, seq))


def cmd_check(args):
    out, cin, pin = open_ports(args)
    send(out, nf.mode_frame(True, nf.HOST_GENERIC, seq=1))
    status, payload = wait_reply(cin, nf.CMD_DAW_MODE, 1)
    print("DAW_MODE on: status=%d active=%s pending=%s" % (status, payload[:1], payload[1:2]))
    assert status == 0, "DAW_MODE failed"
    ops = (nf.op_text(nf.FIELD_TITLE, "CHECK", nf.TITLE_MAX) +
           nf.op_text(nf.FIELD_STATUS, "1.1", nf.STATUS_MAX) +
           nf.op_card(0, 64, 0x00FF00, nf.FLAG_METER) + nf.op_leds(0x05))
    send(out, nf.frame(nf.CMD_DAW_UPDATE, 2, ops))
    status, payload = wait_reply(cin, nf.CMD_DAW_UPDATE, 2)
    print("DAW_UPDATE: status=%d active=%s" % (status, payload[:1]))
    assert status == 0, "DAW_UPDATE failed"
    send(out, nf.frame(nf.CMD_DAW_UPDATE, 3, [0x7E]))
    status, _ = wait_reply(cin, nf.CMD_DAW_UPDATE, 3)
    print("bad op rejected: status=%d (expect 3 BAD_VALUE)" % status)
    assert status == 3
    send(out, nf.frame(nf.CMD_DAW_UPDATE, 4, [nf.OP_TEXT, 0, 9, 0x41]))
    status, _ = wait_reply(cin, nf.CMD_DAW_UPDATE, 4)
    print("short text rejected: status=%d (expect 2 BAD_LENGTH)" % status)
    assert status == 2
    send(out, nf.frame(nf.CMD_DAW_UPDATE, 0, nf.op_leds(0x0A)))
    time.sleep(0.3)
    stray = [m for m in cin.iter_pending() if m.type == "sysex"]
    print("seq 0 replies: %d (expect 0)" % len(stray))
    assert not stray
    print("check passed")
    out.close()
    cin.close()
    pin.close()


def cmd_off(args):
    out, cin, pin = open_ports(args)
    send(out, nf.mode_frame(False, nf.HOST_GENERIC, seq=1))
    status, payload = wait_reply(cin, nf.CMD_DAW_MODE, 1)
    print("DAW_MODE off: status=%d active=%s" % (status, payload[:1]))


def cmd_demo(args):
    out, cin, pin = open_ports(args)
    view = nf.View(lambda f: send(out, f), max_frames_per_flush=2, refresh_every=10)
    colors = [0x3C78FF, 0x3CDC5A, 0xFF9A00, 0xE040FF]
    pages = ["KNOBS", "PAN", "STEPS"]
    page = 0
    values = [[64] * 4 for _ in pages]
    lit = [False] * 4
    held = [False] * 4
    exp = 0
    start = time.monotonic()
    stop_at = start + args.seconds if args.seconds else None
    send(out, nf.mode_frame(True, nf.HOST_GENERIC, seq=1))
    print("DAW mode on. Ctrl+C to quit.")
    try:
        while stop_at is None or time.monotonic() < stop_at:
            for msg in pin.iter_pending():
                if msg.type != "control_change" or msg.channel != 15:
                    continue
                kind, i, value = nf.decode_cc(msg.control, msg.value)
                print("%-12s %d %d" % (kind, i, value), flush=True)
                if kind == "state" and value:
                    view.invalidate()
                elif kind == "rotate":
                    step = value if not held[i] else (1 if value > 0 else -1)
                    values[page][i] = max(0, min(127, values[page][i] + step))
                elif kind == "press":
                    held[i] = bool(value)
                elif kind == "short":
                    lit[i] = not lit[i]
                elif kind == "long":
                    values[page][i] = 64
                elif kind == "edit_short":
                    page = (page + 1) % len(pages)
                elif kind == "exp":
                    exp = value
            for msg in cin.iter_pending():
                pass
            beat = int((time.monotonic() - start) * 2)
            view.title = "DEMO %s" % pages[page]
            view.status = "%d.%d" % (beat // 4 + 1, beat % 4 + 1)
            view.accent = colors[page % 4]
            for i in range(4):
                card = view.cards[i]
                v = values[page][i]
                card.label = "%s %d" % ("KNOB" if page == 0 else pages[page][:4], i + 1)
                if page == 1:
                    card.value = "C" if v == 64 else "%d%s" % (abs(v - 64), "R" if v > 64 else "L")
                else:
                    card.value = str(v)
                card.meter = v
                card.color = colors[i]
                card.flags = nf.FLAG_METER | (nf.FLAG_BIPOLAR if page == 1 else 0) | \
                    (nf.FLAG_LIT if lit[i] else 0)
            if exp:
                view.cards[3].label = "EXP"
                view.cards[3].value = str(exp)
            view.leds = sum(1 << i for i in range(4) if lit[i])
            view.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        send(out, nf.mode_frame(False, nf.HOST_GENERIC, seq=2))
        out.close()
        # Input ports are left to process exit: on Windows, python-rtmidi can
        # hang forever closing an input that has received messages.
        print("DAW mode off.", flush=True)


def cmd_screens(args):
    """Drawn screens (DAW_DRAW): FOCUS dial and SAMPLER waveform. Edit key switches."""
    import math
    import random
    out, cin, pin = open_ports(args)
    canvas = nf.Canvas(lambda f: send(out, f))
    gray, low, yellow = 0x8C9196, 0x34383C, 0xF5D547
    params = [["FILTER", 0.62], ["RESONANCE", 0.30], ["ATTACK", 0.05], ["RELEASE", 0.40]]
    samp = {"start": 0.0, "end": 1.0, "play": None}
    rnd = random.Random(7)
    peaks = [int(127 * min(1, math.exp(-x / 70.0) * (0.45 + 0.55 * rnd.random()))) for x in range(304)]
    state = {"page": 0, "focus": 0, "dirty": True}
    w = {}

    def build():
        canvas.forget_widgets()
        canvas.add(nf.draw_clear())
        w.clear()
        if state["page"] == 0:
            # FOCUS: parameter name, a big value, a row of blocks, the four switches
            w["device"] = nf.Label(canvas, 12, 10, 220, 2, fg=gray)
            w["bank"] = nf.Label(canvas, 250, 10, 58, 2, "right", fg=gray)
            w["name"] = nf.Label(canvas, 12, 36, 296, 3)
            w["value"] = nf.Label(canvas, 12, 66, 296, 6)
            w["bar"] = nf.Segments(canvas, 12, 120, 296, 14, count=32)
            w["notch"] = [nf.Bar(canvas, 12 + i * 76, 160, 64) for i in range(4)]
            w["nletter"] = [nf.Label(canvas, 12 + i * 76, 146, 8, 1) for i in range(4)]
            w["device"].set("OPERATOR")
            w["bank"].set("1/2")
        else:
            w["title"] = nf.Label(canvas, 12, 10, 200, 2)
            w["wave"] = nf.Wave(canvas, 8, 82, 34, 304)
            w["vals"] = [nf.Label(canvas, 12 + i * 78, 160, 70, 1) for i in range(4)]
            for i, name in enumerate(("START", "END", "LENGTH", "MODE")):
                canvas.add(nf.draw_text(12 + i * 78, 150, 1, name, gray))
            w["title"].set("KICK 909")
        state["dirty"] = True
        w["marks"] = None

    def render():
        if state["page"] == 0:
            a = state["focus"]
            name, v = params[a]
            w["name"].set(name)
            w["value"].set("%d%%" % round(v * 100))
            w["bar"].set(v)
            for i in range(4):
                w["notch"][i].set(params[i][1], fg=0xFFFFFF if i == a else gray)
                w["nletter"][i].set("ABCD"[i], fg=0xFFFFFF if i == a else low)
        else:
            s, e = samp["start"], samp["end"]
            wave = w["wave"]
            sx, ex = int(s * 304), max(0, int(e * 304) - 2)
            marks = (sx, ex)
            if samp["play"] is not None:
                p = (time.monotonic() - samp["play"]) / 0.6
                if p >= 1:
                    samp["play"] = None
                else:
                    marks = (sx, ex, int(sx + (ex - sx) * p))
            wave.set(peaks, (s, e), [(sx, 0xFFFFFF), (ex, 0xFFFFFF)] + ([(marks[2], yellow)] if len(marks) > 2 else []))
            ms = lambda f: "%dMS" % round(f * 520)
            for lab, txt in zip(w["vals"], (ms(s), ms(e), ms(e - s), "CLASSIC")):
                lab.set(txt)

    start = time.monotonic()
    stop_at = start + args.seconds if args.seconds else None
    send(out, nf.mode_frame(True, nf.HOST_GENERIC, seq=1))
    time.sleep(0.5)
    build()
    print("Screens on. Turn switches; tap to play (SAMPLER); EDIT tap switches page.", flush=True)
    last_keep = 0
    try:
        while stop_at is None or time.monotonic() < stop_at:
            for msg in pin.iter_pending():
                if msg.type != "control_change" or msg.channel != 15:
                    continue
                kind, i, value = nf.decode_cc(msg.control, msg.value)
                if kind and kind != "press":
                    print("%-12s %d %d" % (kind, i, value), flush=True)
                if kind in ("state", "redraw") or msg.control == nf.CC_REDRAW:
                    if kind == "state" and not value:
                        continue
                    build()
                elif kind == "edit_short":
                    state["page"] ^= 1
                    build()
                elif kind == "rotate":
                    if state["page"] == 0:
                        state["focus"] = i
                        params[i][1] = max(0.0, min(1.0, params[i][1] + value * 0.01))
                    elif i == 1:
                        samp["start"] = max(0.0, min(samp["end"] - 0.02, samp["start"] + value * 0.005))
                    elif i == 2:
                        samp["end"] = max(samp["start"] + 0.02, min(1.0, samp["end"] + value * 0.005))
                elif kind == "short":
                    if state["page"] == 0:
                        state["focus"] = i
                    else:
                        samp["play"] = time.monotonic()
            for msg in cin.iter_pending():
                pass
            render()
            canvas.flush()
            if time.monotonic() - last_keep > 2:
                send(out, nf.frame(nf.CMD_DAW_UPDATE, 0, nf.op_leds(1 << state["focus"] if state["page"] == 0 else 0)))
                last_keep = time.monotonic()
            time.sleep(0.04)
    except KeyboardInterrupt:
        pass
    finally:
        send(out, nf.mode_frame(False, nf.HOST_GENERIC, seq=2))
        out.close()
        print("DAW mode off.", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("ports", "check", "demo", "screens", "off"))
    parser.add_argument("--control", help="control (NanoBridge) port name substring")
    parser.add_argument("--perform", help="performance port name substring")
    parser.add_argument("--seconds", type=float, default=0, help="stop the demo after this many seconds")
    args = parser.parse_args()
    if args.command == "ports":
        print("inputs: ", mido.get_input_names())
        print("outputs:", mido.get_output_names())
    elif args.command == "check":
        cmd_check(args)
    elif args.command == "off":
        cmd_off(args)
    elif args.command == "screens":
        cmd_screens(args)
    else:
        cmd_demo(args)


if __name__ == "__main__":
    main()
