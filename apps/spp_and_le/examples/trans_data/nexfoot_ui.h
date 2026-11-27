#ifndef NEXFOOT_UI_H
#define NEXFOOT_UI_H

#include "typedef.h"

enum nexfoot_ui_kind {
    NEXFOOT_UI_KNOB = 1,
    NEXFOOT_UI_HORIZONTAL_SLIDER,
    NEXFOOT_UI_VERTICAL_SLIDER
};

typedef struct {
    u16 accent, track, background, face, pointer;
} nexfoot_ui_colors;

typedef struct {
    u16 x, y, width, height;
    nexfoot_ui_colors colors;
    u8 kind, value, painted, valid;
} nexfoot_ui_widget;

#define NEXFOOT_UI_READOUT_MAX 16

typedef struct {
    u16 x, y, ink, background;
    u8 columns, scale, valid;
    char painted[NEXFOOT_UI_READOUT_MAX];
} nexfoot_ui_readout;

int nexfoot_ui_paint_readout(nexfoot_ui_readout *field, const char *text);
void nexfoot_ui_set_value(nexfoot_ui_widget *widget, int value);
void nexfoot_ui_invalidate(nexfoot_ui_widget *widget);
int nexfoot_ui_paint(nexfoot_ui_widget *widget);

#endif
