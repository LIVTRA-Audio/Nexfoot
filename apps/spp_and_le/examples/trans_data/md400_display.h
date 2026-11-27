/* Modified for Nexfoot; upstream notices are retained. */
#ifndef MD400_DISPLAY_H
#define MD400_DISPLAY_H

#include "typedef.h"

#define MD400_DISPLAY_SNAPSHOT_BASE_LEN 17
#define MD400_DISPLAY_SNAPSHOT_ROLE_LEN 22
#define MD400_DISPLAY_SNAPSHOT_DIAG_LEN 30
#define MD400_DISPLAY_SNAPSHOT_COUNTER_LEN 28
#define MD400_DISPLAY_LAYOUT_VERSION    5
#define MD400_DISPLAY_PLAY_LABEL_LEN    10
#define MD400_DISPLAY_PEDAL_COLOR_COUNT 8
#define MD400_DISPLAY_MSG_TYPE_SHIFT    8
#define MD400_DISPLAY_MSG_PACK(type, value) \
    ((((int)((type) & 0x7f)) << MD400_DISPLAY_MSG_TYPE_SHIFT) | ((value) & 0x7f))
#define MD400_DISPLAY_MSG_RAW_FLAG      (1L << 30)
#define MD400_DISPLAY_MSG_RAW_LEN_SHIFT 24
#define MD400_DISPLAY_MSG_RAW_PACK(len, b0, b1, b2) \
    ((int)(MD400_DISPLAY_MSG_RAW_FLAG | \
    (((long)((len) & 0x03)) << MD400_DISPLAY_MSG_RAW_LEN_SHIFT) | \
    (((long)((b0) & 0xff)) << 16) | \
    (((long)((b1) & 0xff)) << 8) | ((long)((b2) & 0xff))))
#define MD400_DISPLAY_MSG_RAW_IS(msg)   (((msg) & MD400_DISPLAY_MSG_RAW_FLAG) != 0)
#define MD400_DISPLAY_PAGE_PACK(target, page) \
    ((((int)((target) & 0x7f)) << MD400_DISPLAY_MSG_TYPE_SHIFT) | ((page) & 0x7f))
#define MD400_DISPLAY_PAGE_SUBJECT_PACK(target, page, subject) \
    (((int)((subject) & 0x7f) << 16) | MD400_DISPLAY_PAGE_PACK(target, page))

enum {
    MD400_DISPLAY_PEDAL_CAL_HIDDEN = 0,
    MD400_DISPLAY_PEDAL_CAL_MIN = 1,
    MD400_DISPLAY_PEDAL_CAL_MAX = 2,
    MD400_DISPLAY_PEDAL_CAL_ERROR = 3,
    MD400_DISPLAY_PEDAL_CAL_OK = 4,
};

enum {
    MD400_DISPLAY_ROLE_NONE = 0,
    MD400_DISPLAY_ROLE_ENCODER1 = 1,
    MD400_DISPLAY_ROLE_ENCODER2 = 2,
    MD400_DISPLAY_ROLE_ENCODER3 = 3,
    MD400_DISPLAY_ROLE_ENCODER4 = 4,
    MD400_DISPLAY_ROLE_EDIT_SUBJECT = 0x10,
    MD400_DISPLAY_ROLE_EDIT_TYPE = 0x11,
    MD400_DISPLAY_ROLE_EDIT_DATA1 = 0x12,
    MD400_DISPLAY_ROLE_EDIT_VALUE = 0x13,
    MD400_DISPLAY_ROLE_EDIT_CHANNEL = 0x14,
    MD400_DISPLAY_ROLE_EDIT_BEHAVIOR = 0x16,
    MD400_DISPLAY_ROLE_EDIT_MIN = 0x17,
    MD400_DISPLAY_ROLE_EDIT_MAX = 0x18,
    MD400_DISPLAY_ROLE_EDIT_SYSEX = 0x19,
    MD400_DISPLAY_ROLE_EDIT_PAGE = 0x1a,
    MD400_DISPLAY_ROLE_EDIT_ACTION = 0x1b,
    MD400_DISPLAY_ROLE_EDIT_MESSAGE = 0x1c,
    MD400_DISPLAY_ROLE_EDIT_DETAIL_VALUE = 0x1d,
    MD400_DISPLAY_ROLE_EDIT_DETAIL_RELEASE = 0x1e,
    MD400_DISPLAY_ROLE_PRESET_SELECTED = 0x30,
    MD400_DISPLAY_ROLE_PRESET_ACTIVE = 0x31,
    MD400_DISPLAY_ROLE_PRESET_SLOT = 0x32,
    MD400_DISPLAY_ROLE_PRESET_SEQ = 0x33,
    MD400_DISPLAY_ROLE_BT_TOGGLE = 0x34,
    MD400_DISPLAY_ROLE_PEDAL_MODE = 0x35,
    MD400_DISPLAY_ROLE_PEDAL_CAL = 0x36,
    MD400_DISPLAY_ROLE_PEDAL_SLOT = 0x37,
    MD400_DISPLAY_ROLE_PEDAL_COLOR = 0x38,
    MD400_DISPLAY_ROLE_PEDAL_LABEL_POS = 0x39,
    MD400_DISPLAY_ROLE_PEDAL_LABEL_CHAR = 0x3a,
    MD400_DISPLAY_ROLE_PLAY_MESSAGE = 0x40,
    MD400_DISPLAY_ROLE_PLAY_VALUE = 0x41,
    MD400_DISPLAY_ROLE_PLAY_LABEL = 0x42,
};

void md400_display_test_init(void);
void md400_display_encoder_update(u8 encoder, int value, u8 pressed);
void md400_display_perform_assignments_update(const int *rotate_msg,
                                              const int *press_msg);
void md400_display_ui_update(u8 mode, int row0, int row1, int row2, int row3);
void md400_display_ui_update_roles(u8 mode, int row0, int row1, int row2, int row3,
                                   u8 role0, u8 role1, u8 role2, u8 role3);
void md400_display_play_update(int msg, u8 value, const char *label);
void md400_display_foot_mode_update(u8 pedal_mode, u8 page,
                                    const u8 *color);
void md400_display_pedal_edit_update(u8 selected_slot, u8 style_view);
void md400_display_show_save_check(void);
u16 md400_display_shutdown_start(void);
void md400_display_pedal_cal_update(u8 state, u8 raw8, u8 min8, u8 max8,
                                    u8 value7);
/* top-bar BT indicator: 0 = off (gray), 1 = on/advertising (blinks), 2 = connected (solid) */
void md400_display_bt_state_update(u8 state);
u8 md400_display_snapshot(u8 *payload, u8 max_len);
u8 md400_display_pixel_capture_request(void);

/* DAW view (RAM only), driven by NanoBridge DAW_MODE / DAW_UPDATE. */
#define MD400_DISPLAY_DAW_TITLE_MAX   20
#define MD400_DISPLAY_DAW_STATUS_MAX  8
#define MD400_DISPLAY_DAW_LABEL_MAX   10
#define MD400_DISPLAY_DAW_VALUE_MAX   8
#define MD400_DISPLAY_DAW_FIELD_TITLE   0
#define MD400_DISPLAY_DAW_FIELD_STATUS  1
#define MD400_DISPLAY_DAW_FIELD_LABEL1  2
#define MD400_DISPLAY_DAW_FIELD_VALUE1  6
#define MD400_DISPLAY_DAW_FLAG_LIT      0x01
#define MD400_DISPLAY_DAW_FLAG_METER    0x02
#define MD400_DISPLAY_DAW_FLAG_BIPOLAR  0x04

void md400_display_daw_reset(void);
void md400_display_daw_set_active(u8 active);
void md400_display_daw_app_name(const char *title);
void md400_display_daw_set_waiting(u8 waiting);
u8 md400_display_daw_is_active(void);
u8 md400_display_daw_can_cutoff(void);
u8 md400_display_daw_set_text(u8 field, const u8 *text, u8 len);
u8 md400_display_daw_set_card(u8 card, u8 meter, u16 color, u8 flags);
void md400_display_daw_set_accent(u16 color);
/* DAW canvas: queue one DAW_DRAW payload of drawing ops. Returns 0 if the queue is full. */
u8 md400_display_daw_draw_push(const u8 *ops, u8 len);

#endif
