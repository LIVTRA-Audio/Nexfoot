# Nexfoot display widgets

The widget API provides a rotary knob, horizontal and vertical sliders, and a
fixed-width value readout for the 320 × 176 display. Include `nexfoot_ui.h`
from the firmware application. Store each widget for the lifetime of its page.
The rectangle starts at its top-left pixel. Use a square of odd width 37–97 for
a knob. Both slider dimensions must be at least six pixels. A vertical slider
increases toward the top. C colors are RGB565.

```c
#include "nexfoot_ui.h"

static nexfoot_ui_widget cutoff = {
    .kind = NEXFOOT_UI_KNOB, .x = 18, .y = 54, .width = 65, .height = 65,
    .colors = {.accent = 0x05b9, .track = 0x4208, .background = 0xad55,
               .face = 0xad55, .pointer = 0x2124}
};

static nexfoot_ui_widget mix = {
    .kind = NEXFOOT_UI_HORIZONTAL_SLIDER, .x = 108, .y = 102, .width = 188, .height = 16,
    .colors = {.accent = 0x05b9, .track = 0x4208, .background = 0xad55, .pointer = 0xffff}
};

static nexfoot_ui_readout frequency = {
    .x = 108, .y = 56, .columns = 10, .scale = 2, .ink = 0x2124, .background = 0xad55
};

void paint_filter_page(int cutoff_value, int mix_value, const char *formatted_frequency)
{
    nexfoot_ui_set_value(&cutoff, cutoff_value);
    nexfoot_ui_set_value(&mix, mix_value);
    nexfoot_ui_paint(&cutoff);
    nexfoot_ui_paint(&mix);
    nexfoot_ui_paint_readout(&frequency, formatted_frequency);
}
```

Call the page function from the display refresh task while this page owns the LCD.
Pass the most recent input values each tick. The setter clamps to 0–127. Paint
returns 1 when it draws and 0 when unchanged, outside the display, or unavailable.
A readout accepts up to 16 character columns in the built-in font, truncates longer
strings and blanks trailing cells. Widgets own opaque, non-overlapping rectangles.
After repainting the page background or changing geometry or colors, call
`nexfoot_ui_invalidate` for each control and set the readout's `valid` to zero.
When moving a widget, erase its former rectangle as part of the page repaint.

## DAW scripts

Wait for performance CC 119 value 127 on channel 16. Use the helpers in
`daw/common/nexfoot_daw.py` with a Canvas. Python colors are 24-bit RGB.

```python
canvas.add(define_slider(0, 16, 96, 288, 16, 64, 0x00b6ca, 0x45494a, 0xacadad, 0xffffff))
canvas.flush()
# On a later parameter change
canvas.add(set_widget_value(0, 72))
canvas.flush()
```

Each of the four slots can hold a knob or a slider. Use the existing Label helper for
DAW-formatted values. Define slots again after clearing the canvas, reconnecting,
or receiving a redraw request. When reusing a slot at a new location, repaint its
old background. The Ableton Live and FL Studio integrations use retained
widgets when the firmware advertises them.
