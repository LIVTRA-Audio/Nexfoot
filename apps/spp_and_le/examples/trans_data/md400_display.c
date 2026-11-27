/* Modified for Nexfoot; upstream notices are retained. */
#include "system/includes.h"
#include "asm/gpio.h"
#include "asm/spi.h"
#include "asm/charge.h"
#include "asm/power_interface.h"
#include "os/os_api.h"
#include "usb/otg.h"

#include "app_config.h"
#include "app_power_manage.h"
#include "md400_display.h"
#include "nexfoot_ui.h"
#include "md400_ms32.h"

#define MD400_LCD_CS        IO_PORTB_05
#define MD400_LCD_CLK       IO_PORTB_06
#define MD400_LCD_MOSI      IO_PORTB_07
#define MD400_LCD_DC        IO_PORTB_08
#define MD400_LCD_LIGHT     IO_PORTB_09
#define MD400_LCD_LIGHT_ON  0
#define MD400_LCD_LIGHT_OFF 1
#define MD400_LCD_LIGHT_PROBE_ENABLE 0
#define MD400_LCD_HW_SPI_ENABLE 1
#define MD400_LCD_SPI_DEV   SPI2
#define MD400_LCD_SPI_BAUD  48000000L

#define MD400_LCD_MADCTL_MY  0x80
#define MD400_LCD_MADCTL_MX  0x40
#define MD400_LCD_MADCTL_MV  0x20
#define MD400_LCD_MADCTL_BGR 0x08
#define MD400_LCD_MADCTL_LANDSCAPE \
    (MD400_LCD_MADCTL_MY | MD400_LCD_MADCTL_MV)
#define MD400_LCD_INVERSION_CMD 0x21
#define MD400_LCD_SOFTWARE_FLIP_Y 0

#define MD400_LCD_WIDTH     320
#define MD400_LCD_HEIGHT    176
#define MD400_LCD_X_OFFSET  0
#define MD400_LCD_Y_OFFSET  32

#define MD400_LCD_TOP_BAR_H 28
#define MD400_LCD_TOP_LABEL_X 8
#define MD400_LCD_TOP_LABEL_Y 7
#define MD400_LCD_TOP_LABEL_MAX 20
#define MD400_LCD_SAVE_CHECK_W 18
#define MD400_LCD_SAVE_CHECK_MS 1500
/* Save check sits left of the BT / battery cluster */
#define MD400_LCD_BT_AREA_W 30
#define MD400_LCD_BT_AREA_X (MD400_LCD_BATTERY_AREA_X - MD400_LCD_BT_AREA_W)
#define MD400_LCD_SAVE_CHECK_X (MD400_LCD_BT_AREA_X - MD400_LCD_SAVE_CHECK_W)
#define MD400_LCD_TOP_LABEL_W (MD400_LCD_SAVE_CHECK_X - MD400_LCD_TOP_LABEL_X - 4)
#define MD400_LCD_BATTERY_AREA_X (MD400_LCD_WIDTH - 44)
#define MD400_LCD_BATTERY_AREA_W 44
#define MD400_LCD_CARD_Y    MD400_LCD_TOP_BAR_H
#define MD400_LCD_CARD_W    (MD400_LCD_WIDTH / 4)
#define MD400_LCD_CARD_H    (MD400_LCD_HEIGHT - MD400_LCD_TOP_BAR_H)
#define MD400_LCD_VALUE_X_PAD 5
#define MD400_LCD_VALUE_Y_PAD 44
#define MD400_LCD_VALUE_W   (MD400_LCD_CARD_W - (MD400_LCD_VALUE_X_PAD * 2))
#define MD400_LCD_VALUE_H   54
#define MD400_LCD_VALUE_MAX_CHARS 8
#define MD400_LCD_VALUE_DIGIT_W 11
#define MD400_LCD_VALUE_DIGIT_H 40
#define MD400_LCD_VALUE_DIGIT_T 3
#define MD400_LCD_VALUE_DIGIT_GAP 3
#define MD400_LCD_VALUE_STYLE_SEGMENT 0
#define MD400_LCD_VALUE_STYLE_FONT    1
#define MD400_LCD_PRESS_Y    (MD400_LCD_CARD_H - 16)
#define MD400_LCD_MODE_EDIT  1
#define MD400_LCD_MODE_PLAY  2
#define MD400_LCD_PLAY_PREFIX_MAX 9
#define MD400_LCD_PLAY_LINE_MAX (MD400_LCD_PLAY_PREFIX_MAX + 4)
#define MD400_LCD_PLAY_LINE_SCALE_MAX 6
#define MD400_LCD_PLAY_LINE_SCALE_MIN 4
#define MD400_LCD_PLAY_LINE_GAP 2
#define MD400_LCD_PLAY_LINE_PAD 8
#define MD400_LCD_PLAY_LINE_AREA_Y 50
#define MD400_LCD_PLAY_LINE_AREA_H 68
#define MD400_LCD_PLAY_LINE_Y 59
#define MD400_LCD_PLAY_LABEL_Y 132
#define MD400_LCD_PLAY_LABEL_SCALE 3
#define MD400_LCD_PLAY_LABEL_GAP 2
#define MD400_LCD_BLACK     0x0000
#define MD400_LCD_WHITE     0xffff
#define MD400_LCD_BLUE      0x001f
#define MD400_LCD_GREEN     0x07e0
#define MD400_LCD_CYAN      0x07ff
#define MD400_LCD_YELLOW    0xffe0
#define MD400_LCD_GREEN_BG  0x0320
#define MD400_LCD_MAGENTA   0xf81f
#define MD400_LCD_ROW_BLUE  0x021f
#define MD400_LCD_ROW_GREEN 0x05c0
#define MD400_LCD_ROW_RED   0xc120
#define MD400_LCD_ROW_GOLD  0xdd80
#define MD400_LCD_GRAY      0x39e7
#define MD400_LCD_ORANGE    0xfd20
#define MD400_LCD_RED       0xf800
#define MD400_LCD_DARK2     0x1082

#define MD400_BOOT_ANIM_DURATION_MS 800
#define MD400_BOOT_ANIM_STEP_MS      30
#define MD400_BOOT_ANIM_STEPS        \
    (MD400_BOOT_ANIM_DURATION_MS / MD400_BOOT_ANIM_STEP_MS)
#define MD400_BOOT_ANIM_STAGGER_STEPS 3
#define MD400_BOOT_ANIM_GROW_STEPS    \
    (MD400_BOOT_ANIM_STEPS - 2 - (3 * MD400_BOOT_ANIM_STAGGER_STEPS))
#define MD400_SHUTDOWN_ANIM_DURATION_MS 800
#define MD400_SHUTDOWN_ANIM_STEP_MS     30
#define MD400_SHUTDOWN_ANIM_STEPS       \
    (MD400_SHUTDOWN_ANIM_DURATION_MS / MD400_SHUTDOWN_ANIM_STEP_MS)
#define MD400_SHUTDOWN_ANIM_STAGGER_STEPS 1
#define MD400_SHUTDOWN_ANIM_COLLAPSE_STEPS \
    (MD400_SHUTDOWN_ANIM_STEPS - 2 - \
     (3 * MD400_SHUTDOWN_ANIM_STAGGER_STEPS))
#define MD400_SHUTDOWN_POWER_DELAY_MS   \
    (MD400_SHUTDOWN_ANIM_DURATION_MS + 50)

#define MD400_BATTERY_TICK_MS            500
#define MD400_BATTERY_CHARGE_STEP_TICKS  (1000 / MD400_BATTERY_TICK_MS)
#define MD400_BATTERY_SHUTDOWN_TICKS     (30000 / MD400_BATTERY_TICK_MS)
#define MD400_BATTERY_AUTO_POWEROFF_ENABLE 1
#define MD400_BATTERY_LOW_ENTER_MV10     340
#define MD400_BATTERY_LOW_EXIT_MV10      345
#define MD400_BATTERY_LEVEL_1_UP_MV10    360
#define MD400_BATTERY_LEVEL_1_DOWN_MV10  355
#define MD400_BATTERY_LEVEL_2_UP_MV10    394
#define MD400_BATTERY_LEVEL_2_DOWN_MV10  389

enum {
    MD400_DISPLAY_MSG_CC = 0,
    MD400_DISPLAY_MSG_PC = 1,
    MD400_DISPLAY_MSG_NOTE = 2,
    MD400_DISPLAY_MSG_PITCH = 3,
    MD400_DISPLAY_MSG_PRESSURE = 4,
    MD400_DISPLAY_MSG_SYSEX = 5,
    MD400_DISPLAY_MSG_OFF = 6,
    MD400_DISPLAY_MSG_NOTE_OFF = 7,
    MD400_DISPLAY_MSG_CUSTOM = 8,
};

static const u8 md400_display_edit_type_order[] = {
    MD400_DISPLAY_MSG_CC,
    MD400_DISPLAY_MSG_PC,
    MD400_DISPLAY_MSG_NOTE,
    MD400_DISPLAY_MSG_NOTE_OFF,
    MD400_DISPLAY_MSG_PITCH,
    MD400_DISPLAY_MSG_PRESSURE,
    MD400_DISPLAY_MSG_SYSEX,
    MD400_DISPLAY_MSG_OFF,
};

static u8 md400_display_edit_type_from_selector(u8 selector)
{
    return md400_display_edit_type_order[
        selector % (sizeof(md400_display_edit_type_order) /
                    sizeof(md400_display_edit_type_order[0]))];
}

enum {
    MD400_DISPLAY_EDIT_SUBJECT_ROTATE = 0,
    MD400_DISPLAY_EDIT_SUBJECT_SHORT = 1,
    MD400_DISPLAY_EDIT_SUBJECT_LONG = 2,
    MD400_DISPLAY_EDIT_SUBJECT_DOUBLE = 3,
    MD400_DISPLAY_EDIT_SUBJECT_PRESS = 4,
    MD400_DISPLAY_EDIT_SUBJECT_COMBO_PRESS = 5,
    MD400_DISPLAY_EDIT_SUBJECT_COMBO_SHORT = 6,
    MD400_DISPLAY_EDIT_SUBJECT_COMBO_LONG = 7,
    MD400_DISPLAY_EDIT_SUBJECT_PEDAL = 8,
    MD400_DISPLAY_EDIT_SUBJECT_PRESET = 9,
};

#define MD400_DISPLAY_MSG_OFF_PACKED MD400_DISPLAY_MSG_PACK(MD400_DISPLAY_MSG_OFF, 0)

static u8 md400_lcd_ready;
#if MD400_LCD_HW_SPI_ENABLE
static u8 md400_lcd_hw_spi_ready;
static u16 md400_lcd_hw_spi_errors;
/* Solid-fill pixels are streamed to the panel via hardware SPI DMA instead of
 * one spi_send_byte() per byte. The buffer is preloaded with the repeated
 * RGB565 pattern and sent in chunks; 4-aligned for the SPI DMA engine. */
#define MD400_LCD_DMA_CHUNK_PIXELS  256
static u8 md400_lcd_dma_buf[MD400_LCD_DMA_CHUNK_PIXELS * 2] __attribute__((aligned(4)));
static u16 md400_lcd_dma_color;
static u8 md400_lcd_dma_color_loaded;
#endif
static u16 md400_display_timer;
static u16 md400_battery_timer;
static u16 md400_lcd_light_probe_timer;
static int md400_encoder_value[4];
static u8 md400_encoder_pressed[4];
static int md400_perform_rotate_msg[4] = {
    MD400_DISPLAY_MSG_OFF_PACKED,
    MD400_DISPLAY_MSG_OFF_PACKED,
    MD400_DISPLAY_MSG_OFF_PACKED,
    MD400_DISPLAY_MSG_OFF_PACKED,
};
static int md400_perform_press_msg[4] = {
    MD400_DISPLAY_MSG_OFF_PACKED,
    MD400_DISPLAY_MSG_OFF_PACKED,
    MD400_DISPLAY_MSG_OFF_PACKED,
    MD400_DISPLAY_MSG_OFF_PACKED,
};
static u8 md400_foot_mode_pedal;
static u8 md400_foot_mode_page;
static u8 md400_pedal_edit_slot = 1;
static u8 md400_pedal_edit_style_view;
static int md400_ui_row_value[4];
static u8 md400_ui_row_role[4] = {
    MD400_DISPLAY_ROLE_ENCODER1,
    MD400_DISPLAY_ROLE_ENCODER2,
    MD400_DISPLAY_ROLE_ENCODER3,
    MD400_DISPLAY_ROLE_ENCODER4,
};
static u8 md400_ui_mode;
static u8 md400_card_full_dirty;
static u8 md400_card_label_dirty;
static u8 md400_card_value_dirty;
static u8 md400_card_press_dirty;
static u8 md400_screen_clear_dirty;
static u8 md400_top_bar_dirty;
static u8 md400_display_drawing;
static u8 md400_lcd_first_frame_pending;
static u8 md400_boot_anim_active;
static u8 md400_boot_anim_last_step;
static u32 md400_boot_anim_start_ms;
static u8 md400_shutdown_anim_active;
static u8 md400_shutdown_anim_last_step;
static u32 md400_shutdown_anim_start_ms;
static u8 md400_lcd_light_probe_level;
static u8 md400_battery_dots = 3;
static u8 md400_battery_red;
static u8 md400_battery_visible = 1;
static u8 md400_battery_level_state = 2;
static u8 md400_battery_critical;
static u8 md400_battery_charging;
static u8 md400_battery_charge_phase;
static u8 md400_battery_charge_tick;
static u16 md400_battery_shutdown_ticks;
static char md400_top_bar_label_cache[MD400_LCD_TOP_LABEL_MAX + 1];
static u8 md400_top_bar_label_valid;
static u8 md400_top_bar_battery_valid;
static u8 md400_top_bar_battery_dots;
static u8 md400_top_bar_battery_red;
static u8 md400_top_bar_battery_visible;
static u8 md400_save_check_visible;
static u8 md400_top_bar_save_check_visible;
static u8 md400_top_bar_save_check_valid;
static u32 md400_save_check_until_ms;
/* BT indicator: 0 = off (gray), 1 = on/advertising (blinks 1Hz), 2 = connected (solid) */
static u8 md400_bt_state = 1;
static u8 md400_bt_blink_on = 1;
static u8 md400_top_bar_bt_valid;
static u8 md400_top_bar_bt_state;
static u8 md400_top_bar_bt_blink;
static u8 md400_value_cache_valid[4];
static u8 md400_value_cache_len[4];
static u16 md400_value_cache_start_x[4];
static u8 md400_value_cache_style[4];
static u8 md400_value_cache_scale[4];
static char md400_value_cache_text[4][MD400_LCD_VALUE_MAX_CHARS];
static int md400_play_msg = MD400_DISPLAY_MSG_PACK(MD400_DISPLAY_MSG_CC, 1);
static u8 md400_play_value;
static char md400_play_label[MD400_DISPLAY_PLAY_LABEL_LEN + 1];
static u8 md400_play_full_dirty;
static u8 md400_play_prefix_dirty;
static u8 md400_play_value_dirty;
static u8 md400_play_label_dirty;
static char md400_play_line_cache[MD400_LCD_PLAY_LINE_MAX + 1];
static u8 md400_play_line_cache_len;
static u16 md400_play_line_cache_x;
static u8 md400_play_line_cache_scale;
static u8 md400_play_line_cache_value_start;
static u8 md400_play_line_cache_valid;
static char md400_play_label_cache[MD400_DISPLAY_PLAY_LABEL_LEN + 1];
static u8 md400_play_label_cache_len;
static u16 md400_play_label_cache_x;
static u8 md400_play_label_cache_valid;
static u8 md400_diag_draw_screen;
static u8 md400_diag_draw_top;
static u8 md400_diag_draw_play_full;
static u8 md400_diag_draw_play_line_full;
static u8 md400_diag_draw_play_value;
static u8 md400_diag_draw_play_label;

/* Temporary factory UI-image transport.  The panel has no readable MISO path
 * and the normal renderer draws directly into ST7789 GRAM, so a requested
 * capture mirrors the exact clipped RGB565 fill/blit operations sent to the
 * panel.  Each UART packet is independently framed, sequenced and checksummed;
 * a host can reject an incomplete frame instead of saving a damaged PNG.
 *
 * Packet: "NFXP", type:u8, sequence:u16le, payload_len:u16le, payload, sum:u16le
 * Types: 1 begin(width,height), 2 fill(x,y,w,h,color),
 *        3 blit(x,y,w,h,rgb565be...), 4 end(operation_count).
 */
#if CONFIG_MD400_UART0_RX_TEST_ENABLE
#define MD400_PIXEL_CAPTURE_BEGIN 1
#define MD400_PIXEL_CAPTURE_FILL  2
#define MD400_PIXEL_CAPTURE_BLIT  3
#define MD400_PIXEL_CAPTURE_END   4

static volatile u8 md400_pixel_capture_requested;
static u8 md400_pixel_capture_active;
static u16 md400_pixel_capture_sequence;
static u16 md400_pixel_capture_operations;

static u16 md400_pixel_capture_sum_byte(u16 sum, u8 value)
{
    return (u16)(sum + value);
}

/* The SDK debug putbyte path expands byte 0x0a into CR/LF for text logs.  Pixel
 * packets must be byte-transparent, so use the same blocking UART0 register
 * sequence as the driver's UT0_putbyte implementation. */
static void md400_pixel_capture_put_raw(u8 value)
{
    if (JL_UART0->CON0 & BIT(0)) {
        JL_UART0->BUF = value;
        __asm__ volatile("csync");
        while ((JL_UART0->CON0 & BIT(15)) == 0);
        JL_UART0->CON0 |= BIT(13);
    }
}

static void md400_pixel_capture_put_u16(u16 value)
{
    md400_pixel_capture_put_raw((u8)value);
    md400_pixel_capture_put_raw((u8)(value >> 8));
}

static void md400_pixel_capture_packet(u8 type, const u8 *head, u16 head_len,
                                       const u8 *body, u16 body_len)
{
    static const u8 magic[4] = {'N', 'F', 'X', 'P'};
    u16 payload_len = head_len + body_len;
    u16 sequence = md400_pixel_capture_sequence++;
    u16 sum = 0;
    u16 i;

    for (i = 0; i < sizeof(magic); i++) {
        md400_pixel_capture_put_raw(magic[i]);
    }
    md400_pixel_capture_put_raw(type);
    md400_pixel_capture_put_u16(sequence);
    md400_pixel_capture_put_u16(payload_len);
    sum = md400_pixel_capture_sum_byte(sum, type);
    sum = md400_pixel_capture_sum_byte(sum, (u8)sequence);
    sum = md400_pixel_capture_sum_byte(sum, (u8)(sequence >> 8));
    sum = md400_pixel_capture_sum_byte(sum, (u8)payload_len);
    sum = md400_pixel_capture_sum_byte(sum, (u8)(payload_len >> 8));
    for (i = 0; i < head_len; i++) {
        md400_pixel_capture_put_raw(head[i]);
        sum = md400_pixel_capture_sum_byte(sum, head[i]);
    }
    for (i = 0; i < body_len; i++) {
        md400_pixel_capture_put_raw(body[i]);
        sum = md400_pixel_capture_sum_byte(sum, body[i]);
    }
    md400_pixel_capture_put_u16(sum);
}

static void md400_pixel_capture_begin(void)
{
    u8 payload[4] = {
        (u8)MD400_LCD_WIDTH, (u8)(MD400_LCD_WIDTH >> 8),
        (u8)MD400_LCD_HEIGHT, (u8)(MD400_LCD_HEIGHT >> 8),
    };

    md400_pixel_capture_sequence = 0;
    md400_pixel_capture_operations = 0;
    md400_pixel_capture_active = 1;
    md400_pixel_capture_packet(MD400_PIXEL_CAPTURE_BEGIN,
                               payload, sizeof(payload), NULL, 0);
}

static void md400_pixel_capture_fill(u16 x, u16 y, u16 w, u16 h, u16 color)
{
    u8 payload[10];
    u16 values[5] = {x, y, w, h, color};
    u8 i;

    if (!md400_pixel_capture_active) {
        return;
    }
    for (i = 0; i < 5; i++) {
        payload[i * 2] = (u8)values[i];
        payload[(i * 2) + 1] = (u8)(values[i] >> 8);
    }
    md400_pixel_capture_packet(MD400_PIXEL_CAPTURE_FILL,
                               payload, sizeof(payload), NULL, 0);
    md400_pixel_capture_operations++;
}

static void md400_pixel_capture_blit(u16 x, u16 y, u16 w, u16 h,
                                     const u8 *pixels)
{
    u8 payload[8];
    u16 values[4] = {x, y, w, h};
    u16 bytes = w * h * 2;
    u8 i;

    if (!md400_pixel_capture_active || !pixels) {
        return;
    }
    for (i = 0; i < 4; i++) {
        payload[i * 2] = (u8)values[i];
        payload[(i * 2) + 1] = (u8)(values[i] >> 8);
    }
    md400_pixel_capture_packet(MD400_PIXEL_CAPTURE_BLIT,
                               payload, sizeof(payload), pixels, bytes);
    md400_pixel_capture_operations++;
}

static void md400_pixel_capture_end(void)
{
    u8 payload[2] = {
        (u8)md400_pixel_capture_operations,
        (u8)(md400_pixel_capture_operations >> 8),
    };

    md400_pixel_capture_packet(MD400_PIXEL_CAPTURE_END,
                               payload, sizeof(payload), NULL, 0);
    md400_pixel_capture_active = 0;
}
#else
#define md400_pixel_capture_fill(x, y, w, h, color) do { } while (0)
#define md400_pixel_capture_blit(x, y, w, h, pixels) do { } while (0)
#endif

static void md400_lcd_delay_ms(u16 ms)
{
    u16 ticks = (ms + 9) / 10;

    if (ticks == 0) {
        ticks = 1;
    }
    os_time_dly(ticks);
}

static void md400_lcd_pin_output(u32 pin, u8 level)
{
    gpio_set_die(pin, 1);
    gpio_set_pull_up(pin, 0);
    gpio_set_pull_down(pin, 0);
    gpio_set_hd(pin, 1);
    gpio_direction_output(pin, level);
}

static void md400_lcd_bus_idle(void)
{
    md400_lcd_pin_output(MD400_LCD_CS, 1);
    md400_lcd_pin_output(MD400_LCD_CLK, 0);
    md400_lcd_pin_output(MD400_LCD_MOSI, 0);
    md400_lcd_pin_output(MD400_LCD_DC, 0);
    md400_lcd_pin_output(MD400_LCD_LIGHT, MD400_LCD_LIGHT_OFF);
}

#if MD400_LCD_HW_SPI_ENABLE
static void md400_lcd_hw_spi_init(void)
{
    int err;

    err = spi_open(MD400_LCD_SPI_DEV);
    md400_lcd_hw_spi_ready = (err == 0);
    md400_lcd_pin_output(MD400_LCD_CS, 1);
    md400_lcd_pin_output(MD400_LCD_DC, 0);
    md400_lcd_pin_output(MD400_LCD_LIGHT, MD400_LCD_LIGHT_OFF);
    printf("[MD400_LCD] hw spi2 %s clk=%lu sck=PB6 mosi=PB7 cs=PB5 gpio\r\n",
           md400_lcd_hw_spi_ready ? "ready" : "open failed",
           (unsigned long)MD400_LCD_SPI_BAUD);
}
#endif

static void md400_lcd_power_cycle(void)
{
    md400_lcd_pin_output(MD400_LCD_LIGHT, MD400_LCD_LIGHT_OFF);
    md400_lcd_delay_ms(30);
    md400_lcd_delay_ms(150);
}

static void md400_lcd_write8(u8 value)
{
    u8 mask;

#if MD400_LCD_HW_SPI_ENABLE
    if (md400_lcd_hw_spi_ready) {
        if (spi_send_byte(MD400_LCD_SPI_DEV, value) == 0) {
            return;
        }
        if (md400_lcd_hw_spi_errors < 5) {
            printf("[MD400_LCD] hw spi2 byte timeout\r\n");
        }
        md400_lcd_hw_spi_errors++;
    }
#endif

    for (mask = 0x80; mask; mask >>= 1) {
        gpio_write(MD400_LCD_MOSI, (value & mask) ? 1 : 0);
        gpio_write(MD400_LCD_CLK, 1);
        gpio_write(MD400_LCD_CLK, 0);
    }
}

static void md400_lcd_cmd_data(u8 cmd, const u8 *data, u8 len)
{
    u8 i;

    gpio_write(MD400_LCD_CS, 0);
    gpio_write(MD400_LCD_DC, 0);
    md400_lcd_write8(cmd);

    if (data && len) {
        gpio_write(MD400_LCD_DC, 1);
        for (i = 0; i < len; i++) {
            md400_lcd_write8(data[i]);
        }
    }

    gpio_write(MD400_LCD_DC, 0);
    gpio_write(MD400_LCD_CS, 1);
}

static void md400_lcd_init_sequence(void)
{
    static const u8 commands[] = {
        0x11, 0,
        0x00, 1, 120,
        0x36, 1, MD400_LCD_MADCTL_LANDSCAPE,
        0x3a, 1, 0x05,
        0xB2, 5, 0x0C, 0x0C, 0x00, 0x33, 0x33,
        0xB7, 1, 0x75,
        0xBB, 1, 0x21,
        0xC0, 1, 0x2C,
        0xC2, 1, 0x01,
        0xC3, 1, 0x13,
        0xC4, 1, 0x20,
        0xC6, 1, 0x0F,
        0xD0, 2, 0xA4, 0xA1,
        0xD6, 1, 0xA1,
        0x13, 0,
        0xE0, 14, 0x70, 0x04, 0x0A, 0x08, 0x07, 0x05, 0x32, 0x32,
                  0x48, 0x38, 0x15, 0x15, 0x2A, 0x2E,
        0xE1, 14, 0x70, 0x07, 0x0D, 0x09, 0x09, 0x16, 0x30, 0x44,
                  0x49, 0x39, 0x16, 0x16, 0x2B, 0x2F,
        MD400_LCD_INVERSION_CMD, 0,
        0x29, 0,
        0x00, 1, 20,
    };
    u16 i = 0;

    while (i < sizeof(commands)) {
        u8 cmd = commands[i++];
        u8 len = commands[i++];

        if ((cmd == 0x00) && (len == 1)) {
            md400_lcd_delay_ms(commands[i]);
        } else {
            md400_lcd_cmd_data(cmd, &commands[i], len);
        }
        i += len;
    }
}

static void md400_lcd_set_window(u16 x, u16 y, u16 w, u16 h)
{
    u16 x0;
    u16 x1;
    u16 y0;
    u16 y1;
    u8 data[4];

#if MD400_LCD_SOFTWARE_FLIP_Y
    y = MD400_LCD_HEIGHT - y - h;
#endif
    x0 = x + MD400_LCD_X_OFFSET;
    x1 = x0 + w - 1;
    y0 = y + MD400_LCD_Y_OFFSET;
    y1 = y0 + h - 1;

    data[0] = x0 >> 8;
    data[1] = x0;
    data[2] = x1 >> 8;
    data[3] = x1;
    md400_lcd_cmd_data(0x2A, data, sizeof(data));

    data[0] = y0 >> 8;
    data[1] = y0;
    data[2] = y1 >> 8;
    data[3] = y1;
    md400_lcd_cmd_data(0x2B, data, sizeof(data));
}

static void md400_lcd_start_pixels(u16 x, u16 y, u16 w, u16 h)
{
    md400_lcd_set_window(x, y, w, h);
    gpio_write(MD400_LCD_CS, 0);
    gpio_write(MD400_LCD_DC, 0);
    md400_lcd_write8(0x2C);
    gpio_write(MD400_LCD_DC, 1);
}

static void md400_lcd_end_pixels(void)
{
    gpio_write(MD400_LCD_DC, 0);
    gpio_write(MD400_LCD_CS, 1);
}

static void md400_lcd_write_color(u16 color)
{
    md400_lcd_write8(color >> 8);
    md400_lcd_write8(color);
}

#if MD400_LCD_HW_SPI_ENABLE
static void md400_lcd_dma_load_color(u16 color)
{
    u16 i;

    if (md400_lcd_dma_color_loaded && (md400_lcd_dma_color == color)) {
        return;
    }
    for (i = 0; i < MD400_LCD_DMA_CHUNK_PIXELS; i++) {
        md400_lcd_dma_buf[i * 2]     = (u8)(color >> 8);
        md400_lcd_dma_buf[i * 2 + 1] = (u8)color;
    }
    md400_lcd_dma_color = color;
    md400_lcd_dma_color_loaded = 1;
}

/* Stream `pixels` copies of `color` to the panel through SPI DMA. Caller must
 * have opened the pixel window (md400_lcd_start_pixels) so CS=0 and DC=1. */
static void md400_lcd_fill_color_dma(u16 color, u32 pixels)
{
    md400_lcd_dma_load_color(color);
    while (pixels) {
        u32 chunk = (pixels > MD400_LCD_DMA_CHUNK_PIXELS) ? MD400_LCD_DMA_CHUNK_PIXELS : pixels;

        if (spi_dma_send(MD400_LCD_SPI_DEV, md400_lcd_dma_buf, chunk * 2) < 0) {
            if (md400_lcd_hw_spi_errors < 5) {
                printf("[MD400_LCD] hw spi2 dma timeout\r\n");
            }
            md400_lcd_hw_spi_errors++;
            break;
        }
        pixels -= chunk;
    }
}
#endif

static void md400_lcd_fill_rect(u16 x, u16 y, u16 w, u16 h, u16 color)
{
    u32 pixels;

    if ((x >= MD400_LCD_WIDTH) || (y >= MD400_LCD_HEIGHT) || !w || !h) {
        return;
    }
    if ((x + w) > MD400_LCD_WIDTH) {
        w = MD400_LCD_WIDTH - x;
    }
    if ((y + h) > MD400_LCD_HEIGHT) {
        h = MD400_LCD_HEIGHT - y;
    }

    md400_pixel_capture_fill(x, y, w, h, color);

    md400_lcd_start_pixels(x, y, w, h);
    pixels = (u32)w * h;
#if MD400_LCD_HW_SPI_ENABLE
    if (md400_lcd_hw_spi_ready) {
        md400_lcd_fill_color_dma(color, pixels);
    } else
#endif
    {
        while (pixels--) {
            md400_lcd_write_color(color);
        }
    }
    md400_lcd_end_pixels();
}

static void md400_lcd_draw_hline(u16 x, u16 y, u16 w, u16 color)
{
    md400_lcd_fill_rect(x, y, w, 1, color);
}

static void md400_lcd_draw_vline(u16 x, u16 y, u16 h, u16 color)
{
    md400_lcd_fill_rect(x, y, 1, h, color);
}

static void md400_lcd_draw_rect(u16 x, u16 y, u16 w, u16 h, u16 color)
{
    if (!w || !h) {
        return;
    }

    md400_lcd_draw_hline(x, y, w, color);
    md400_lcd_draw_hline(x, y + h - 1, w, color);
    md400_lcd_draw_vline(x, y, h, color);
    md400_lcd_draw_vline(x + w - 1, y, h, color);
}

static u8 md400_lcd_char_mask(char ch)
{
    static const u8 digit_mask[10] = {
        0x3f, 0x06, 0x5b, 0x4f, 0x66,
        0x6d, 0x7d, 0x07, 0x7f, 0x6f,
    };

    if ((ch >= '0') && (ch <= '9')) {
        return digit_mask[ch - '0'];
    }
    if (ch == '-') {
        return 0x40;
    }
    if ((ch >= 'a') && (ch <= 'z')) {
        ch -= 32;
    }
    switch (ch) {
    case 'A':
        return 0x77;
    case 'B':
        return 0x7c;
    case 'C':
        return 0x39;
    case 'D':
        return 0x5e;
    case 'E':
        return 0x79;
    case 'F':
        return 0x71;
    case 'G':
        return 0x3d;
    case 'H':
        return 0x76;
    case 'I':
        return 0x06;
    case 'L':
        return 0x38;
    case 'M':
        return 0x37;
    case 'N':
        return 0x54;
    case 'O':
        return 0x3f;
    case 'P':
        return 0x73;
    case 'Q':
        return 0x67;
    case 'R':
        return 0x50;
    case 'S':
        return 0x6d;
    case 'T':
        return 0x78;
    case 'U':
        return 0x3e;
    case 'V':
        return 0x3e;
    case 'X':
        return 0x76;
    case 'Y':
        return 0x6e;
    default:
        break;
    }
    return 0;
}

static void md400_lcd_draw_segment_part(u16 x, u16 y, u16 w, u16 h,
                                        u16 t, u8 segment, u16 color)
{
    u16 half = h / 2;

    if (segment == 0) {
        md400_lcd_fill_rect(x + t, y, w - (t * 2), t, color);
    } else if (segment == 1) {
        md400_lcd_fill_rect(x + w - t, y + t, t, half - t, color);
    } else if (segment == 2) {
        md400_lcd_fill_rect(x + w - t, y + half, t, half - t, color);
    } else if (segment == 3) {
        md400_lcd_fill_rect(x + t, y + h - t, w - (t * 2), t, color);
    } else if (segment == 4) {
        md400_lcd_fill_rect(x, y + half, t, half - t, color);
    } else if (segment == 5) {
        md400_lcd_fill_rect(x, y + t, t, half - t, color);
    } else if (segment == 6) {
        md400_lcd_fill_rect(x + t, y + half - (t / 2), w - (t * 2), t, color);
    }
}

static void md400_lcd_draw_segment_char(u16 x, u16 y, u16 w, u16 h,
                                        u16 t, char ch, u16 color)
{
    u8 mask = md400_lcd_char_mask(ch);
    u8 segment;

    for (segment = 0; segment < 7; segment++) {
        if (mask & BIT(segment)) {
            md400_lcd_draw_segment_part(x, y, w, h, t, segment, color);
        }
    }
}

static void md400_lcd_update_segment_char(u16 x, u16 y, u16 w, u16 h,
                                          u16 t, char old_ch, u16 old_color,
                                          char new_ch, u16 new_color, u16 bg)
{
    u8 old_mask = md400_lcd_char_mask(old_ch);
    u8 new_mask = md400_lcd_char_mask(new_ch);
    u8 segment;
    u8 old_on;
    u8 new_on;

    for (segment = 0; segment < 7; segment++) {
        old_on = !!(old_mask & BIT(segment));
        new_on = !!(new_mask & BIT(segment));
        if (old_on && !new_on) {
            md400_lcd_draw_segment_part(x, y, w, h, t, segment, bg);
        } else if ((!old_on && new_on) ||
                   (old_on && new_on && (old_color != new_color))) {
            md400_lcd_draw_segment_part(x, y, w, h, t, segment, new_color);
        }
    }
}

static void md400_lcd_draw_segment_digit(u16 x, u16 y, u16 w, u16 h,
                                         u16 t, char ch, u16 color)
{
    md400_lcd_draw_segment_char(x, y, w, h, t, ch, color);
}

/* DAW text uses real lowercase glyphs; the normal UI keeps capitals. */
static u8 md400_lcd_font_mixed_case;

static const u8 *md400_lcd_font5x7(char ch)
{
    static const u8 lower[26][7] = {
        {0x00, 0x00, 0x0e, 0x01, 0x0f, 0x11, 0x0f}, /* a */
        {0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x1e}, /* b */
        {0x00, 0x00, 0x0e, 0x10, 0x10, 0x11, 0x0e}, /* c */
        {0x01, 0x01, 0x0d, 0x13, 0x11, 0x11, 0x0f}, /* d */
        {0x00, 0x00, 0x0e, 0x11, 0x1f, 0x10, 0x0e}, /* e */
        {0x06, 0x09, 0x08, 0x1c, 0x08, 0x08, 0x08}, /* f */
        {0x00, 0x00, 0x0f, 0x11, 0x11, 0x0f, 0x01}, /* g */
        {0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x11}, /* h */
        {0x04, 0x00, 0x0c, 0x04, 0x04, 0x04, 0x0e}, /* i */
        {0x02, 0x00, 0x06, 0x02, 0x02, 0x02, 0x12}, /* j */
        {0x10, 0x10, 0x12, 0x14, 0x18, 0x14, 0x12}, /* k */
        {0x0c, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e}, /* l */
        {0x00, 0x00, 0x1a, 0x15, 0x15, 0x11, 0x11}, /* m */
        {0x00, 0x00, 0x16, 0x19, 0x11, 0x11, 0x11}, /* n */
        {0x00, 0x00, 0x0e, 0x11, 0x11, 0x11, 0x0e}, /* o */
        {0x00, 0x00, 0x1e, 0x11, 0x11, 0x1e, 0x10}, /* p */
        {0x00, 0x00, 0x0f, 0x11, 0x11, 0x0f, 0x01}, /* q */
        {0x00, 0x00, 0x16, 0x19, 0x10, 0x10, 0x10}, /* r */
        {0x00, 0x00, 0x0f, 0x10, 0x0e, 0x01, 0x1e}, /* s */
        {0x08, 0x08, 0x1c, 0x08, 0x08, 0x09, 0x06}, /* t */
        {0x00, 0x00, 0x11, 0x11, 0x11, 0x13, 0x0d}, /* u */
        {0x00, 0x00, 0x11, 0x11, 0x11, 0x0a, 0x04}, /* v */
        {0x00, 0x00, 0x11, 0x11, 0x15, 0x15, 0x0a}, /* w */
        {0x00, 0x00, 0x11, 0x0a, 0x04, 0x0a, 0x11}, /* x */
        {0x00, 0x00, 0x11, 0x11, 0x11, 0x0f, 0x01}, /* y */
        {0x00, 0x00, 0x1f, 0x02, 0x04, 0x08, 0x1f}, /* z */
    };
    static const u8 blank[7] = {0, 0, 0, 0, 0, 0, 0};
    static const u8 dash[7] = {0, 0, 0, 0x1f, 0, 0, 0};
    static const u8 slash[7] = {0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10};
    static const u8 digit[10][7] = {
        {0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e},
        {0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e},
        {0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f},
        {0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e},
        {0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02},
        {0x1f, 0x10, 0x10, 0x1e, 0x01, 0x01, 0x1e},
        {0x0e, 0x10, 0x10, 0x1e, 0x11, 0x11, 0x0e},
        {0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
        {0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e},
        {0x0e, 0x11, 0x11, 0x0f, 0x01, 0x01, 0x0e},
    };
    static const u8 letter[26][7] = {
        {0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11},
        {0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e},
        {0x0e, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0e},
        {0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e},
        {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f},
        {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10},
        {0x0e, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0f},
        {0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11},
        {0x0e, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e},
        {0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0c},
        {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11},
        {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f},
        {0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11},
        {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11},
        {0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e},
        {0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10},
        {0x0e, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0d},
        {0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11},
        {0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e},
        {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
        {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e},
        {0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04},
        {0x11, 0x11, 0x11, 0x15, 0x15, 0x1b, 0x11},
        {0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11},
        {0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04},
        {0x1f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1f},
    };

    if ((ch >= '0') && (ch <= '9')) {
        return digit[ch - '0'];
    }
    if ((ch >= 'a') && (ch <= 'z')) {
        if (md400_lcd_font_mixed_case) {
            return lower[ch - 'a'];
        }
        ch -= 32;
    }
    if ((ch >= 'A') && (ch <= 'Z')) {
        return letter[ch - 'A'];
    }
    if (ch == '-') {
        return dash;
    }
    if (ch == '/') {
        return slash;
    }
    {
        /* Extra punctuation used by host (DAW) text: beat positions, dB,
         * tempo, percentages, sharps and arrows. */
        static const char punct_chars[] = ".:+%#<>!()_=*',?|[]&";
        static const u8 punct[][7] = {
            {0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c}, /* . */
            {0x00, 0x0c, 0x0c, 0x00, 0x0c, 0x0c, 0x00}, /* : */
            {0x00, 0x04, 0x04, 0x1f, 0x04, 0x04, 0x00}, /* + */
            {0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03}, /* % */
            {0x0a, 0x0a, 0x1f, 0x0a, 0x1f, 0x0a, 0x0a}, /* # */
            {0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02}, /* < */
            {0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08}, /* > */
            {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04}, /* ! */
            {0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02}, /* ( */
            {0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08}, /* ) */
            {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f}, /* _ */
            {0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00}, /* = */
            {0x00, 0x04, 0x15, 0x0e, 0x15, 0x04, 0x00}, /* * */
            {0x04, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00}, /* ' */
            {0x00, 0x00, 0x00, 0x00, 0x0c, 0x04, 0x08}, /* , */
            {0x0e, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04}, /* ? */
            {0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}, /* | */
            {0x0e, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0e}, /* [ */
            {0x0e, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0e}, /* ] */
            {0x0c, 0x12, 0x14, 0x08, 0x15, 0x12, 0x0d}, /* & */
        };
        u8 p;

        for (p = 0; punct_chars[p]; p++) {
            if (punct_chars[p] == ch) {
                return punct[p];
            }
        }
    }
    return blank;
}

static u8 md400_lcd_text_len(const char *text, u8 max_chars)
{
    u8 len = 0;

    if (!text) {
        return 0;
    }
    while (text[len] && (len < max_chars)) {
        len++;
    }
    return len;
}

static u16 md400_lcd_font_text_width(u8 len, u8 scale, u8 gap)
{
    if (!len) {
        return 0;
    }
    return (u16)(len * 5 * scale) + (u16)((len - 1) * gap);
}

#if MD400_LCD_HW_SPI_ENABLE
/* Blit a w*h RGB565 region (big-endian bytes already in buf) to the panel in a
 * single SPI DMA transfer. */
static void md400_lcd_blit(u16 x, u16 y, u16 w, u16 h, const u8 *buf)
{
    u32 bytes = (u32)w * h * 2;

    md400_pixel_capture_blit(x, y, w, h, buf);
    md400_lcd_start_pixels(x, y, w, h);
    if (md400_lcd_hw_spi_ready) {
        if (spi_dma_send(MD400_LCD_SPI_DEV, buf, bytes) < 0) {
            if (md400_lcd_hw_spi_errors < 5) {
                printf("[MD400_LCD] hw spi2 blit timeout\r\n");
            }
            md400_lcd_hw_spi_errors++;
        }
    } else {
        u32 i;

        for (i = 0; i < bytes; i += 2) {
            md400_lcd_write_color(((u16)buf[i] << 8) | buf[i + 1]);
        }
    }
    md400_lcd_end_pixels();
}

/* Render one 5x7 glyph scaled by `scale` (fg where lit, bg elsewhere) and push it
 * as SPI DMA blits, reusing the shared fill buffer in horizontal row-bands. This
 * replaces ~(5*7) tiny per-pixel fills with a few full-width DMA transfers. Falls
 * back to per-cell fills if hw SPI is down or the glyph is wider than the buffer. */
static void md400_lcd_blit_glyph(u16 x, u16 y, char ch, u8 scale, u16 fg, u16 bg)
{
    const u8 *rows = md400_lcd_font5x7(ch);
    u16 w = (u16)5 * scale;
    u16 h = (u16)7 * scale;
    u16 band_rows;
    u16 sy;

    if (!md400_lcd_hw_spi_ready || (w == 0) ||
        (w > MD400_LCD_DMA_CHUNK_PIXELS)) {
        u8 row;
        u8 col;

        md400_lcd_fill_rect(x, y, w, h, bg);
        for (row = 0; row < 7; row++) {
            for (col = 0; col < 5; col++) {
                if (rows[row] & BIT(4 - col)) {
                    md400_lcd_fill_rect(x + ((u16)col * scale),
                                        y + ((u16)row * scale),
                                        scale, scale, fg);
                }
            }
        }
        return;
    }

    band_rows = MD400_LCD_DMA_CHUNK_PIXELS / w;
    if (band_rows == 0) {
        band_rows = 1;
    }

    for (sy = 0; sy < h; sy += band_rows) {
        u16 rows_this = ((u16)(h - sy) < band_rows) ? (u16)(h - sy) : band_rows;
        u32 px = 0;
        u16 by;
        u16 sx;

        for (by = 0; by < rows_this; by++) {
            u8 frow = (u8)((sy + by) / scale);

            for (sx = 0; sx < w; sx++) {
                u8 col = (u8)(sx / scale);
                u16 c = (rows[frow] & BIT(4 - col)) ? fg : bg;

                md400_lcd_dma_buf[px * 2]     = (u8)(c >> 8);
                md400_lcd_dma_buf[px * 2 + 1] = (u8)c;
                px++;
            }
        }
        md400_lcd_blit(x, y + sy, w, rows_this, md400_lcd_dma_buf);
    }
    /* buffer no longer holds the solid-fill colour pattern */
    md400_lcd_dma_color_loaded = 0;
}
#endif

static void md400_lcd_draw_font_char(u16 x, u16 y, char ch, u8 scale, u16 color)
{
    const u8 *rows = md400_lcd_font5x7(ch);
    u8 row;
    u8 col;

    for (row = 0; row < 7; row++) {
        for (col = 0; col < 5; col++) {
            if (rows[row] & BIT(4 - col)) {
                md400_lcd_fill_rect(x + ((u16)col * scale),
                                    y + ((u16)row * scale),
                                    scale, scale, color);
            }
        }
    }
}

static void md400_lcd_update_font_char(u16 x, u16 y, char old_ch, u16 old_color,
                                       char new_ch, u16 new_color, u8 scale,
                                       u16 bg)
{
    const u8 *old_rows;
    const u8 *new_rows;
    u8 row;
    u8 col;
    u8 old_on;
    u8 new_on;

#if MD400_LCD_HW_SPI_ENABLE
    if (md400_lcd_hw_spi_ready) {
        /* One DMA blit of the whole new cell (bg fills the unlit pixels, which
         * also erases the old glyph) beats per-segment micro-fills. */
        (void)old_ch;
        (void)old_color;
        md400_lcd_blit_glyph(x, y, new_ch, scale, new_color, bg);
        return;
    }
#endif

    old_rows = md400_lcd_font5x7(old_ch);
    new_rows = md400_lcd_font5x7(new_ch);
    for (row = 0; row < 7; row++) {
        for (col = 0; col < 5; col++) {
            old_on = (old_rows[row] & BIT(4 - col)) ? 1 : 0;
            new_on = (new_rows[row] & BIT(4 - col)) ? 1 : 0;
            if ((old_on == new_on) &&
                (!new_on || (old_color == new_color))) {
                continue;
            }
            md400_lcd_fill_rect(x + ((u16)col * scale),
                                y + ((u16)row * scale),
                                scale, scale,
                                new_on ? new_color : bg);
        }
    }
}

static void md400_lcd_draw_font_text(u16 x, u16 y, const char *text,
                                     u8 scale, u8 gap, u16 color, u8 max_chars)
{
    u8 i;

    if (!text) {
        return;
    }
    for (i = 0; text[i] && (i < max_chars); i++) {
        md400_lcd_draw_font_char(x + (i * ((5 * scale) + gap)), y,
                                 text[i], scale, color);
    }
}

static void md400_lcd_draw_font_text_centered(u16 x, u16 box_w, u16 y,
                                              const char *text, u8 scale,
                                              u8 gap, u16 color, u8 max_chars)
{
    u8 len;
    u16 width;
    u16 start_x;

    len = md400_lcd_text_len(text, max_chars);
    if (!len) {
        return;
    }
    width = md400_lcd_font_text_width(len, scale, gap);
    start_x = (width < box_w) ? (x + ((box_w - width) / 2)) : x;
    md400_lcd_draw_font_text(start_x, y, text, scale, gap, color, max_chars);
}

static u8 md400_display_msg_type(int msg)
{
    return (msg >> MD400_DISPLAY_MSG_TYPE_SHIFT) & 0x7f;
}

static const char *md400_display_msg_type_label(u8 type)
{
    switch (type) {
    case MD400_DISPLAY_MSG_CC:
        return "CC";
    case MD400_DISPLAY_MSG_PC:
        return "PC";
    case MD400_DISPLAY_MSG_NOTE:
        return "N-ON";
    case MD400_DISPLAY_MSG_PITCH:
        return "PITCH";
    case MD400_DISPLAY_MSG_PRESSURE:
        return "PRESS";
    case MD400_DISPLAY_MSG_SYSEX:
        return "SYSEX";
    case MD400_DISPLAY_MSG_OFF:
        return "OFF";
    case MD400_DISPLAY_MSG_NOTE_OFF:
        return "N-OFF";
    case MD400_DISPLAY_MSG_CUSTOM:
        return "CUSTOM";
    default:
        break;
    }
    return "MSG";
}

static u8 md400_display_format_msg(int msg, char *out)
{
    const char *label;
    u8 type;
    u8 value;
    u8 len = 0;
    u8 i;

    if (!out) {
        return 0;
    }

    type = md400_display_msg_type(msg);
    value = msg & 0x7f;
    label = md400_display_msg_type_label(type);
    for (i = 0; label[i] && (len < 5); i++) {
        out[len++] = label[i];
    }

    if (type != MD400_DISPLAY_MSG_OFF) {
        if (value >= 100) {
            out[len++] = '0' + (value / 100);
            out[len++] = '0' + ((value / 10) % 10);
            out[len++] = '0' + (value % 10);
        } else {
            out[len++] = '0' + (value / 10);
            out[len++] = '0' + (value % 10);
        }
    }

    out[len] = 0;
    return len;
}

static void md400_display_append_dec2_or_3(char *out, u8 *len, u8 value,
                                           u8 max_len)
{
    if (!out || !len || (*len >= max_len)) {
        return;
    }

    if (value >= 100) {
        if ((*len + 3) >= max_len) {
            return;
        }
        out[(*len)++] = '0' + (value / 100);
        out[(*len)++] = '0' + ((value / 10) % 10);
        out[(*len)++] = '0' + (value % 10);
        return;
    }

    if ((*len + 2) >= max_len) {
        return;
    }
    out[(*len)++] = '0' + (value / 10);
    out[(*len)++] = '0' + (value % 10);
}

static char md400_display_hex_char(u8 value)
{
    value &= 0x0f;
    return (value < 10) ? ('0' + value) : ('A' + value - 10);
}

static u8 md400_display_raw_msg_len(int msg)
{
    return (msg >> MD400_DISPLAY_MSG_RAW_LEN_SHIFT) & 0x03;
}

static u8 md400_display_raw_msg_byte(int msg, u8 index)
{
    if (index == 0) {
        return (msg >> 16) & 0xff;
    }
    if (index == 1) {
        return (msg >> 8) & 0xff;
    }
    return msg & 0xff;
}

static u8 md400_display_format_play_raw(int msg, char *out, u8 max_len,
                                        u8 *value_start)
{
    u8 raw_len;
    u8 len = 0;
    u8 i;
    u8 byte;

    if (!out || !max_len) {
        return 0;
    }

    raw_len = md400_display_raw_msg_len(msg);
    if (raw_len > 3) {
        raw_len = 3;
    }

    for (i = 0; i < raw_len; i++) {
        if (i && ((len + 1) < max_len)) {
            out[len++] = ' ';
        }
        if ((i == 1) && value_start) {
            *value_start = len;
        }
        if ((len + 2) >= max_len) {
            break;
        }
        byte = md400_display_raw_msg_byte(msg, i);
        out[len++] = md400_display_hex_char(byte >> 4);
        out[len++] = md400_display_hex_char(byte);
    }

    if (value_start && (raw_len <= 1)) {
        *value_start = len;
    }
    out[len] = 0;
    return len;
}

static u8 md400_display_format_play_prefix(int msg, char *out, u8 max_len)
{
    const char *label;
    u8 type;
    u8 value;
    u8 len = 0;
    u8 i;

    if (!out || !max_len) {
        return 0;
    }

    if (MD400_DISPLAY_MSG_RAW_IS(msg)) {
        return md400_display_format_play_raw(msg, out, max_len, NULL);
    }

    type = md400_display_msg_type(msg);
    value = msg & 0x7f;
    label = md400_display_msg_type_label(type);
    for (i = 0; label[i] && ((len + 1) < max_len); i++) {
        out[len++] = label[i];
    }

    if ((type != MD400_DISPLAY_MSG_OFF) &&
        (type != MD400_DISPLAY_MSG_SYSEX)) {
        if ((len + 1) < max_len) {
            out[len++] = ' ';
        }
        md400_display_append_dec2_or_3(out, &len, value, max_len);
    }

    out[len] = 0;
    return len;
}

static void md400_display_format_play_value(u8 value, char *out)
{
    if (!out) {
        return;
    }
    value &= 0x7f;
    out[0] = '0' + (value / 100);
    out[1] = '0' + ((value / 10) % 10);
    out[2] = '0' + (value % 10);
    out[3] = 0;
}

static u8 md400_display_format_value(int value, char *out)
{
    u8 len = 0;
    u16 mag;

    if (value > 999) {
        value = 999;
    } else if (value < -999) {
        value = -999;
    }

    if (value < 0) {
        out[len++] = '-';
        mag = (u16)(-value);
    } else {
        mag = (u16)value;
    }

    if (mag >= 100) {
        out[len++] = '0' + (mag / 100);
        out[len++] = '0' + ((mag / 10) % 10);
        out[len++] = '0' + (mag % 10);
    } else if (mag >= 10) {
        out[len++] = '0' + (mag / 10);
        out[len++] = '0' + (mag % 10);
    } else {
        out[len++] = '0' + mag;
    }

    if (len < (MD400_LCD_VALUE_MAX_CHARS + 1)) {
        out[len] = 0;
    }
    return len;
}

static u16 md400_display_value_start_x(u16 x, u16 box_w, u8 len)
{
    u16 total_w;

    if (!len) {
        return x;
    }

    total_w = (len * MD400_LCD_VALUE_DIGIT_W) +
              ((len - 1) * MD400_LCD_VALUE_DIGIT_GAP);
    if (total_w > box_w) {
        return x;
    }
    return x + ((box_w - total_w) / 2);
}

static u16 md400_display_font_value_start_x(u16 x, u16 box_w, u8 len,
                                            u8 scale, u8 gap)
{
    u16 total_w;

    if (!len) {
        return x;
    }

    total_w = md400_lcd_font_text_width(len, scale, gap);
    if (total_w > box_w) {
        return x;
    }
    return x + ((box_w - total_w) / 2);
}

static u16 md400_display_value_color(char ch, u16 color)
{
    return (ch == '-') ? MD400_LCD_RED : color;
}

static u8 md400_display_value_has_changed(u8 index, const char *text, u8 len)
{
    u8 i;

    if ((index >= 4) || !md400_value_cache_valid[index] ||
        (md400_value_cache_len[index] != len)) {
        return 1;
    }

    for (i = 0; i < len; i++) {
        if (md400_value_cache_text[index][i] != text[i]) {
            return 1;
        }
    }

    return 0;
}

static void md400_display_value_cache_store(u8 index, const char *text,
                                            u8 len, u16 start_x, u8 style,
                                            u8 scale)
{
    u8 i;

    if (index >= 4) {
        return;
    }

    md400_value_cache_valid[index] = 1;
    md400_value_cache_len[index] = len;
    md400_value_cache_start_x[index] = start_x;
    md400_value_cache_style[index] = style;
    md400_value_cache_scale[index] = scale;
    for (i = 0; i < MD400_LCD_VALUE_MAX_CHARS; i++) {
        md400_value_cache_text[index][i] =
            ((text != NULL) && (i < len)) ? text[i] : 0;
    }
}

static u8 md400_display_format_role_value(u8 index, int value, char *out);

static u8 md400_display_value_style(u8 index)
{
    if ((index < 4) && (md400_ui_mode == MD400_LCD_MODE_EDIT) &&
        ((md400_ui_row_role[index] == MD400_DISPLAY_ROLE_EDIT_TYPE) ||
         (md400_ui_row_role[index] == MD400_DISPLAY_ROLE_EDIT_BEHAVIOR) ||
         (md400_ui_row_role[index] == MD400_DISPLAY_ROLE_BT_TOGGLE) ||
         (md400_ui_row_role[index] == MD400_DISPLAY_ROLE_PEDAL_MODE) ||
         (md400_ui_row_role[index] == MD400_DISPLAY_ROLE_PEDAL_CAL))) {
        return MD400_LCD_VALUE_STYLE_FONT;
    }
    return MD400_LCD_VALUE_STYLE_SEGMENT;
}

static u8 md400_display_value_is_two_line_note(const char *text, u8 len)
{
    return (len >= 7) &&
           (text[0] == 'N') && (text[1] == 'O') &&
           (text[2] == 'T') && (text[3] == 'E') &&
           (text[4] == '/');
}

static void md400_display_draw_two_line_note_value(u8 index, u16 x, u16 y,
                                                   u16 box_w, const char *text,
                                                   u8 len, u16 color, u16 bg,
                                                   u8 full, u8 style, u8 scale)
{
    u16 top;
    u16 line1_x;
    u16 line2_x;
    u16 line1_y;
    u16 line2_y;
    u8 line2_len;
    u8 i;
    u8 changed;

    top = y - 6;
    line2_len = len - 5;
    changed = full || md400_display_value_has_changed(index, text, len) ||
              (md400_value_cache_style[index] != style) ||
              (md400_value_cache_scale[index] != scale) ||
              (md400_value_cache_start_x[index] != x);

    if (!changed) {
        return;
    }

    md400_lcd_fill_rect(x, top, box_w, MD400_LCD_VALUE_H, bg);

    line1_x = md400_display_font_value_start_x(x, box_w, 4, scale, 0);
    line2_x = md400_display_font_value_start_x(x, box_w, line2_len, scale, 0);
    line1_y = top + ((MD400_LCD_VALUE_H - ((7 * scale * 2) + 4)) / 2);
    line2_y = line1_y + (7 * scale) + 4;

    for (i = 0; i < 4; i++) {
        md400_lcd_draw_font_char(line1_x + (i * (5 * scale)),
                                 line1_y, text[i], scale, color);
    }
    for (i = 0; i < line2_len; i++) {
        md400_lcd_draw_font_char(line2_x + (i * (5 * scale)),
                                 line2_y, text[5 + i], scale, color);
    }

    md400_display_value_cache_store(index, text, len, x, style, scale);
}

static void md400_display_draw_pedal_cal_result(u8 index, u16 x, u16 y,
                                                u16 box_w, u8 ok, u16 bg,
                                                u8 full)
{
    char text[2];
    u16 top;
    u16 mark_x;
    u16 mark_y;
    u16 color;
    u8 changed;
    u8 i;

    text[0] = ok ? 'C' : 'X';
    text[1] = 0;
    top = y - 6;
    changed = full || md400_display_value_has_changed(index, text, 1) ||
              (md400_value_cache_start_x[index] != x) ||
              (md400_value_cache_style[index] != MD400_LCD_VALUE_STYLE_FONT);
    if (!changed) {
        return;
    }

    md400_lcd_fill_rect(x, top, box_w, MD400_LCD_VALUE_H, bg);
    mark_x = x + ((box_w > 44) ? ((box_w - 44) / 2) : 0);
    mark_y = top + ((MD400_LCD_VALUE_H > 36) ?
                    ((MD400_LCD_VALUE_H - 36) / 2) : 0);
    color = ok ? MD400_LCD_GREEN : MD400_LCD_RED;

    if (ok) {
        md400_lcd_fill_rect(mark_x + 2, mark_y + 20, 6, 6, color);
        md400_lcd_fill_rect(mark_x + 8, mark_y + 25, 6, 6, color);
        md400_lcd_fill_rect(mark_x + 14, mark_y + 30, 6, 6, color);
        md400_lcd_fill_rect(mark_x + 20, mark_y + 24, 6, 6, color);
        md400_lcd_fill_rect(mark_x + 26, mark_y + 18, 6, 6, color);
        md400_lcd_fill_rect(mark_x + 32, mark_y + 12, 6, 6, color);
        md400_lcd_fill_rect(mark_x + 38, mark_y + 6, 6, 6, color);
    } else {
        for (i = 0; i < 6; i++) {
            md400_lcd_fill_rect(mark_x + ((u16)i * 7), mark_y + ((u16)i * 6),
                                6, 6, color);
            md400_lcd_fill_rect(mark_x + 35 - ((u16)i * 7),
                                mark_y + ((u16)i * 6), 6, 6, color);
        }
    }
    md400_display_value_cache_store(index, text, 1, x,
                                    MD400_LCD_VALUE_STYLE_FONT, 0);
}

static void md400_display_draw_value_box(u8 index, u16 x, u16 y, u16 box_w,
                                         int value, u16 color, u16 bg, u8 full)
{
    char text[MD400_LCD_VALUE_MAX_CHARS + 1];
    u8 len;
    u8 i;
    u16 start_x;
    u16 old_x;
    u16 char_x;
    u8 layout_changed;
    u16 old_color;
    u16 new_color;
    u8 style;
    u8 scale;
    u8 gap;
    u16 font_y;

    len = md400_display_format_role_value(index, value, text);
    style = md400_display_value_style(index);
    if ((index < 4) &&
        (md400_ui_row_role[index] == MD400_DISPLAY_ROLE_PEDAL_CAL) &&
        (((value & 0x7f) == MD400_DISPLAY_PEDAL_CAL_OK) ||
         ((value & 0x7f) == MD400_DISPLAY_PEDAL_CAL_ERROR))) {
        md400_display_draw_pedal_cal_result(index, x, y, box_w,
                                            (value & 0x7f) ==
                                            MD400_DISPLAY_PEDAL_CAL_OK,
                                            bg, full);
        return;
    }
    scale = 0;
    gap = 0;
    if (style == MD400_LCD_VALUE_STYLE_FONT) {
        scale = 3;
        gap = 0;
        if (md400_display_value_is_two_line_note(text, len)) {
            start_x = x;
        } else {
            start_x = md400_display_font_value_start_x(x, box_w, len, scale, gap);
        }
    } else {
        start_x = md400_display_value_start_x(x, box_w, len);
    }
    layout_changed = full || (index >= 4) ||
                     !md400_value_cache_valid[index] ||
                     (md400_value_cache_len[index] != len) ||
                     (md400_value_cache_start_x[index] != start_x) ||
                     (md400_value_cache_style[index] != style) ||
                     (md400_value_cache_scale[index] != scale);

    if (style == MD400_LCD_VALUE_STYLE_FONT) {
        if (md400_display_value_is_two_line_note(text, len)) {
            md400_display_draw_two_line_note_value(index, x, y, box_w, text,
                                                   len, color, bg, full, style,
                                                   scale);
            return;
        }

        font_y = (u16)(y - 6 + ((MD400_LCD_VALUE_H - (7 * scale)) / 2));
        if (layout_changed) {
            md400_lcd_fill_rect(x, y - 6, box_w, MD400_LCD_VALUE_H, bg);
            for (i = 0; i < len; i++) {
                char_x = start_x + (i * ((5 * scale) + gap));
#if MD400_LCD_HW_SPI_ENABLE
                md400_lcd_blit_glyph(char_x, font_y, text[i], scale, color, bg);
#else
                md400_lcd_draw_font_char(char_x, font_y, text[i], scale, color);
#endif
            }
            md400_display_value_cache_store(index, text, len, start_x,
                                            style, scale);
            return;
        }

        for (i = 0; i < len; i++) {
            if (md400_value_cache_text[index][i] == text[i]) {
                continue;
            }
            char_x = start_x + (i * ((5 * scale) + gap));
            md400_lcd_update_font_char(char_x, font_y,
                                       md400_value_cache_text[index][i],
                                       color, text[i], color, scale, bg);
        }
        md400_display_value_cache_store(index, text, len, start_x,
                                        style, scale);
        return;
    }

    if (layout_changed) {
        md400_lcd_fill_rect(x, y - 6, box_w, MD400_LCD_VALUE_H, bg);
        for (i = 0; i < len; i++) {
            new_color = md400_display_value_color(text[i], color);
            md400_lcd_draw_segment_digit(start_x + (i * (MD400_LCD_VALUE_DIGIT_W +
                                                         MD400_LCD_VALUE_DIGIT_GAP)),
                                         y, MD400_LCD_VALUE_DIGIT_W,
                                         MD400_LCD_VALUE_DIGIT_H,
                                         MD400_LCD_VALUE_DIGIT_T,
                                          text[i], new_color);
        }
        md400_display_value_cache_store(index, text, len, start_x,
                                        style, scale);
        return;
    }

    for (i = 0; i < len; i++) {
        if (md400_value_cache_text[index][i] == text[i]) {
            continue;
        }
        old_x = md400_value_cache_start_x[index] +
                (i * (MD400_LCD_VALUE_DIGIT_W + MD400_LCD_VALUE_DIGIT_GAP));
        char_x = start_x + (i * (MD400_LCD_VALUE_DIGIT_W +
                                 MD400_LCD_VALUE_DIGIT_GAP));
        old_color = md400_display_value_color(md400_value_cache_text[index][i],
                                              color);
        new_color = md400_display_value_color(text[i], color);
        if (old_x != char_x) {
            md400_lcd_fill_rect(old_x, y, MD400_LCD_VALUE_DIGIT_W,
                                MD400_LCD_VALUE_DIGIT_H, bg);
            md400_lcd_draw_segment_digit(char_x, y, MD400_LCD_VALUE_DIGIT_W,
                                         MD400_LCD_VALUE_DIGIT_H,
                                         MD400_LCD_VALUE_DIGIT_T,
                                         text[i], new_color);
        } else {
            md400_lcd_update_segment_char(char_x, y, MD400_LCD_VALUE_DIGIT_W,
                                          MD400_LCD_VALUE_DIGIT_H,
                                          MD400_LCD_VALUE_DIGIT_T,
                                          md400_value_cache_text[index][i],
                                          old_color, text[i], new_color, bg);
        }
    }
    md400_display_value_cache_store(index, text, len, start_x, style, scale);
}

static const u16 md400_midi_main_bg[4] = {
    MD400_LCD_ROW_BLUE,
    MD400_LCD_ROW_GREEN,
    MD400_LCD_ROW_RED,
    MD400_LCD_ROW_GOLD,
};

/*
 * PEDAL owns a deliberately separate palette from MIDI. Keeping the alternate
 * colors at indices 0..3 means existing/default PEDAL actions immediately gain
 * the new identity without rewriting their saved MIDI assignments or labels.
 */
static const u16 md400_pedal_bg[MD400_DISPLAY_PEDAL_COLOR_COUNT] = {
    MD400_LCD_CYAN,
    MD400_LCD_ORANGE,
    MD400_LCD_MAGENTA,
    MD400_LCD_YELLOW,
    MD400_LCD_ROW_BLUE,
    MD400_LCD_ROW_GREEN,
    MD400_LCD_ROW_RED,
    MD400_LCD_ROW_GOLD,
};

static u16 md400_display_mode_card_bg(u8 index)
{
    if (index >= 4) {
        return MD400_LCD_BLACK;
    }
    if (md400_foot_mode_pedal) {
        return md400_pedal_bg[index];
    }
    return md400_midi_main_bg[index];
}

static u16 md400_display_row_bg(u8 index)
{
    static const u16 edit_bg[4] = {
        MD400_LCD_CYAN,
        MD400_LCD_GREEN,
        MD400_LCD_ORANGE,
        MD400_LCD_MAGENTA,
    };
    static const u16 preset_bg[4] = {
        MD400_LCD_YELLOW,
        MD400_LCD_CYAN,
        MD400_LCD_ROW_GREEN,
        MD400_LCD_GRAY,
    };

    if (index >= 4) {
        return MD400_LCD_BLACK;
    }
    if (md400_boot_anim_active || md400_shutdown_anim_active) {
        return md400_display_mode_card_bg(index);
    }
    if (md400_ui_mode == MD400_LCD_MODE_EDIT) {
        return edit_bg[index];
    }
    if (md400_ui_mode == MD400_LCD_MODE_PLAY) {
        return preset_bg[index];
    }
    return md400_display_mode_card_bg(index);
}

static int md400_display_role_render_value(u8 role, int value)
{
    if (role == MD400_DISPLAY_ROLE_EDIT_ACTION) {
        return value % 10;
    }
    if ((role == MD400_DISPLAY_ROLE_EDIT_MESSAGE) ||
        (role == MD400_DISPLAY_ROLE_EDIT_DETAIL_VALUE) ||
        (role == MD400_DISPLAY_ROLE_EDIT_DETAIL_RELEASE) ||
        (role == MD400_DISPLAY_ROLE_EDIT_PAGE)) {
        return value & 0x7f;
    }
    return value;
}

static int md400_display_row_value(u8 index)
{
    u8 role;
    int value;

    if (index >= 4) {
        return 0;
    }
    if (md400_ui_mode == MD400_LCD_MODE_PLAY) {
        return index ? 0 : md400_play_value;
    }
    if (md400_ui_mode) {
        role = md400_ui_row_role[index];
        value = md400_ui_row_value[index];
        return md400_display_role_render_value(role, value);
    }
    return md400_encoder_value[index];
}

static u8 md400_display_clip7(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 0x7f) {
        return 0x7f;
    }
    return value;
}

static u8 md400_display_copy_text(char *out, const char *text)
{
    u8 len = 0;

    if (!out || !text) {
        return 0;
    }
    while (text[len] && (len < MD400_LCD_VALUE_MAX_CHARS)) {
        out[len] = text[len];
        len++;
    }
    out[len] = 0;
    return len;
}

static u8 md400_display_edit_subject_number(void)
{
    if ((md400_ui_mode == 1) &&
        (md400_ui_row_role[3] == MD400_DISPLAY_ROLE_EDIT_PAGE)) {
        return (md400_ui_row_value[3] >> 16) & 0x7f;
    }
    return MD400_DISPLAY_EDIT_SUBJECT_ROTATE;
}

static u8 md400_display_edit_page_number(void)
{
    if ((md400_ui_mode == 1) &&
        (md400_ui_row_role[3] == MD400_DISPLAY_ROLE_EDIT_PAGE)) {
        return md400_ui_row_value[3] & 0x7f;
    }
    return 0;
}

static u8 md400_display_edit_target_number(void)
{
    u8 target;

    if ((md400_ui_mode == 1) &&
        (md400_ui_row_role[3] == MD400_DISPLAY_ROLE_EDIT_PAGE)) {
        target = (md400_ui_row_value[3] >> 8) & 0x7f;
        if ((target >= 1) && (target <= 4)) {
            return target;
        }
    }
    return 1;
}

static u8 md400_display_edit_page_is_rotate(void)
{
    u8 subject = md400_display_edit_subject_number();
    u8 page = md400_display_edit_page_number();

    if (subject == MD400_DISPLAY_EDIT_SUBJECT_PRESET) {
        return 0;
    }
    return (subject == MD400_DISPLAY_EDIT_SUBJECT_ROTATE) ||
           ((subject == MD400_DISPLAY_EDIT_SUBJECT_SHORT) &&
            (page >= 3) && (page <= 4));
}

static u8 md400_display_format_role_value(u8 index, int value, char *out)
{
    u8 role;

    if (!out) {
        return 0;
    }

    if ((index < 4) && (md400_ui_mode == 1)) {
        role = md400_ui_row_role[index];
        if (role == MD400_DISPLAY_ROLE_EDIT_TYPE) {
            static const char *type_names[] = {
                "CC", "PC", "NOTE/ON", "PITCH", "PRESS", "SYSEX", "OFF", "NOTE/OFF",
            };
            value &= 0x7f;
            value = md400_display_edit_type_from_selector(value);
            if (value < sizeof(type_names) / sizeof(type_names[0])) {
                return md400_display_copy_text(out, type_names[value]);
            }
        } else if (role == MD400_DISPLAY_ROLE_EDIT_BEHAVIOR) {
            value &= 0x7f;
            if (md400_display_edit_page_is_rotate()) {
                return md400_display_copy_text(out, value ? "REL" : "ABS");
            }
            switch (value) {
            case 0:
                return md400_display_copy_text(out, "SET");
            case 1:
                return md400_display_copy_text(out, "TOG");
            case 2:
                return md400_display_copy_text(out, "MOM");
            case 3:
                return md400_display_copy_text(out, "PUSH");
            default:
                break;
            }
        } else if (role == MD400_DISPLAY_ROLE_EDIT_PAGE) {
            value &= 0x7f;
            out[0] = 'P';
            if (value >= 100) {
                out[1] = '0' + (value / 100);
                out[2] = '0' + ((value / 10) % 10);
                out[3] = '0' + (value % 10);
                out[4] = 0;
                return 4;
            }
            if (value >= 10) {
                out[1] = '0' + (value / 10);
                out[2] = '0' + (value % 10);
                out[3] = 0;
                return 3;
            }
            out[1] = '0' + value;
            out[2] = 0;
            return 2;
        } else if (role == MD400_DISPLAY_ROLE_BT_TOGGLE) {
            return md400_display_copy_text(out, (value & 0x7f) ? "ON" : "OFF");
        } else if (role == MD400_DISPLAY_ROLE_PEDAL_MODE) {
            return md400_display_copy_text(out, (value & 0x01) ? "SWT" : "EXP");
        } else if (role == MD400_DISPLAY_ROLE_PEDAL_CAL) {
            switch (value & 0x7f) {
            case MD400_DISPLAY_PEDAL_CAL_MIN:
                return md400_display_copy_text(out, "MIN");
            case MD400_DISPLAY_PEDAL_CAL_MAX:
                return md400_display_copy_text(out, "MAX");
            case MD400_DISPLAY_PEDAL_CAL_ERROR:
                return md400_display_copy_text(out, "ERR");
            case MD400_DISPLAY_PEDAL_CAL_OK:
                return md400_display_copy_text(out, "OK");
            default:
                return md400_display_copy_text(out, "SET");
            }
        } else if (role == MD400_DISPLAY_ROLE_PEDAL_COLOR) {
            static const char *color_names[MD400_DISPLAY_PEDAL_COLOR_COUNT] = {
                "CYAN", "ORANGE", "MAGENTA", "YELLOW",
                "BLUE", "GREEN", "RED", "GOLD",
            };
            return md400_display_copy_text(
                out, color_names[(value & 0x7f) %
                                 MD400_DISPLAY_PEDAL_COLOR_COUNT]);
        } else if (role == MD400_DISPLAY_ROLE_PEDAL_LABEL_CHAR) {
            value &= 0x7f;
            out[0] = (value >= 0x20 && value <= 0x7e) ? value : ' ';
            out[1] = 0;
            return 1;
        }
    }

    return md400_display_format_value(value, out);
}

static const char *md400_display_role_label(u8 role, u8 index)
{
    static const char *encoder_label[4] = {"E1", "E2", "E3", "E4"};
    u8 value;

    if ((role >= MD400_DISPLAY_ROLE_ENCODER1) &&
        (role <= MD400_DISPLAY_ROLE_ENCODER4)) {
        return encoder_label[role - MD400_DISPLAY_ROLE_ENCODER1];
    }

    switch (role) {
    case MD400_DISPLAY_ROLE_NONE:
        return "";
    case MD400_DISPLAY_ROLE_EDIT_SUBJECT:
        return "SUB";
    case MD400_DISPLAY_ROLE_EDIT_TYPE:
        return "TYPE";
    case MD400_DISPLAY_ROLE_EDIT_DATA1:
        return "NUM";
    case MD400_DISPLAY_ROLE_EDIT_VALUE:
        return "VAL";
    case MD400_DISPLAY_ROLE_EDIT_CHANNEL:
        return "CH";
    case MD400_DISPLAY_ROLE_EDIT_BEHAVIOR:
        if (md400_display_edit_page_is_rotate()) {
            return "MODE";
        }
        return "BEH";
    case MD400_DISPLAY_ROLE_EDIT_MIN:
        return "MIN";
    case MD400_DISPLAY_ROLE_EDIT_MAX:
        return "MAX";
    case MD400_DISPLAY_ROLE_EDIT_SYSEX:
        return "SYS";
    case MD400_DISPLAY_ROLE_EDIT_PAGE:
        return "PAGE";
    case MD400_DISPLAY_ROLE_EDIT_ACTION:
        value = (u8)(md400_ui_row_value[index] / 10);
        switch (value) {
        case 0:
            return "ROT";
        case 1:
            return "SHT";
        case 2:
            return "LNG";
        case 3:
            return "DBL";
        case 4:
            return "PRS";
        case 5:
            return "CPR";
        case 6:
            return "CSH";
        case 7:
            return "CLG";
        default:
            return "ACT";
        }
    case MD400_DISPLAY_ROLE_EDIT_MESSAGE:
        value = (u8)((md400_ui_row_value[index] >> 8) & 0x7f);
        switch (value) {
        case 0:
            return "CC";
        case 1:
            return "PC";
        case 2:
            return "N-ON";
        case 3:
            return "PITCH";
        case 4:
            return "PRESS";
        case 5:
            return "SYSEX";
        case 6:
            return "OFF";
        case 7:
            return "N-OFF";
        case 8:
            return "CUSTOM";
        default:
            return "MSG";
        }
    case MD400_DISPLAY_ROLE_EDIT_DETAIL_VALUE:
        value = (u8)((md400_ui_row_value[index] >> 8) & 0x7f);
        if (value == 1) {
            return "PRS";
        }
        if (value == 2) {
            return "DEF";
        }
        return "VAL";
    case MD400_DISPLAY_ROLE_EDIT_DETAIL_RELEASE:
        value = (u8)((md400_ui_row_value[index] >> 8) & 0x7f);
        if (value == 1) {
            return "MAX";
        }
        if (value == 2) {
            return "MIN";
        }
        return "RLS";
    case MD400_DISPLAY_ROLE_PRESET_SELECTED:
    case MD400_DISPLAY_ROLE_PRESET_ACTIVE:
        return "";
    case MD400_DISPLAY_ROLE_PRESET_SLOT:
        return "PRE";
    case MD400_DISPLAY_ROLE_PRESET_SEQ:
        return "SEQ";
    case MD400_DISPLAY_ROLE_BT_TOGGLE:
        return "BT";
    case MD400_DISPLAY_ROLE_PEDAL_MODE:
        return "EXP";
    case MD400_DISPLAY_ROLE_PEDAL_CAL:
        return "CAL";
    case MD400_DISPLAY_ROLE_PEDAL_SLOT:
        return "FT";
    case MD400_DISPLAY_ROLE_PEDAL_COLOR:
        return "COLOR";
    case MD400_DISPLAY_ROLE_PEDAL_LABEL_POS:
        return "POS";
    case MD400_DISPLAY_ROLE_PEDAL_LABEL_CHAR:
        return "CHAR";
    default:
        break;
    }

    if (index < 4) {
        return encoder_label[index];
    }
    return "";
}

static u8 md400_display_dirty_mask(void)
{
    return (md400_card_full_dirty | md400_card_label_dirty | md400_card_value_dirty |
            md400_card_press_dirty) & 0x0f;
}

static void md400_display_mark_card_full_nolock(u8 index)
{
    u8 mask;

    if (index >= 4) {
        return;
    }
    mask = BIT(index);
    md400_card_full_dirty |= mask;
    md400_card_label_dirty &= ~mask;
    md400_card_value_dirty &= ~mask;
    md400_card_press_dirty &= ~mask;
}

static void md400_display_mark_card_label_nolock(u8 index)
{
    u8 mask;

    if (index >= 4) {
        return;
    }
    mask = BIT(index);
    if (md400_card_full_dirty & mask) {
        return;
    }
    md400_card_label_dirty |= mask;
}

static void md400_display_mark_full_screen_nolock(void)
{
    md400_screen_clear_dirty = 1;
    md400_card_full_dirty = 0x0f;
    md400_card_label_dirty = 0;
    md400_card_value_dirty = 0;
    md400_card_press_dirty = 0;
    md400_top_bar_dirty = 1;
}

void md400_display_perform_assignments_update(const int *rotate_msg,
                                              const int *press_msg)
{
    u8 i;

    if (!rotate_msg || !press_msg) {
        return;
    }

    local_irq_disable();
    for (i = 0; i < 4; i++) {
        if ((md400_perform_rotate_msg[i] != rotate_msg[i]) ||
            (md400_perform_press_msg[i] != press_msg[i])) {
            md400_perform_rotate_msg[i] = rotate_msg[i];
            md400_perform_press_msg[i] = press_msg[i];
            if (!md400_ui_mode) {
                md400_display_mark_card_label_nolock(i);
            }
        }
    }
    local_irq_enable();
}

static u8 md400_display_first_dirty(u8 dirty)
{
    u8 i;

    for (i = 0; i < 4; i++) {
        if (dirty & BIT(i)) {
            return i;
        }
    }
    return 0xff;
}

u8 md400_display_snapshot(u8 *payload, u8 max_len)
{
    u8 i;
    u8 pressed = 0;

    if (!payload || (max_len < MD400_DISPLAY_SNAPSHOT_BASE_LEN)) {
        return 0;
    }

    local_irq_disable();
    for (i = 0; i < 4; i++) {
        if (md400_encoder_pressed[i]) {
            pressed |= BIT(i);
        }
    }

    payload[0] = md400_lcd_ready ? 1 : 0;
    payload[1] = md400_ui_mode & 0x7f;
    payload[2] = md400_display_dirty_mask();
    payload[3] = pressed & 0x0f;
    for (i = 0; i < 4; i++) {
        payload[4 + i] = md400_display_clip7(md400_display_row_value(i));
        payload[8 + i] = md400_display_clip7(md400_encoder_value[i]);
        payload[12 + i] = md400_display_clip7(
            md400_display_role_render_value(md400_ui_row_role[i],
                                            md400_ui_row_value[i]));
    }
    payload[16] = md400_display_drawing ? 1 : 0;
    if (max_len >= MD400_DISPLAY_SNAPSHOT_ROLE_LEN) {
        payload[17] = MD400_DISPLAY_LAYOUT_VERSION;
        for (i = 0; i < 4; i++) {
            payload[18 + i] = md400_ui_row_role[i];
        }
    }
    if (max_len >= MD400_DISPLAY_SNAPSHOT_COUNTER_LEN) {
        payload[22] = md400_diag_draw_screen & 0x7f;
        payload[23] = md400_diag_draw_top & 0x7f;
        payload[24] = md400_diag_draw_play_full & 0x7f;
        payload[25] = md400_diag_draw_play_line_full & 0x7f;
        payload[26] = md400_diag_draw_play_value & 0x7f;
        payload[27] = md400_diag_draw_play_label & 0x7f;
    }
    if (max_len >= MD400_DISPLAY_SNAPSHOT_DIAG_LEN) {
        payload[28] = md400_lcd_hw_spi_ready ? 1 : 0;
        payload[29] = (md400_lcd_hw_spi_errors > 0x7f) ?
                      0x7f : (md400_lcd_hw_spi_errors & 0x7f);
    }
    local_irq_enable();

    if (max_len >= MD400_DISPLAY_SNAPSHOT_DIAG_LEN) {
        return MD400_DISPLAY_SNAPSHOT_DIAG_LEN;
    }
    if (max_len >= MD400_DISPLAY_SNAPSHOT_COUNTER_LEN) {
        return MD400_DISPLAY_SNAPSHOT_COUNTER_LEN;
    }
    return (max_len >= MD400_DISPLAY_SNAPSHOT_ROLE_LEN) ?
           MD400_DISPLAY_SNAPSHOT_ROLE_LEN : MD400_DISPLAY_SNAPSHOT_BASE_LEN;
}

static u16 md400_display_card_x(u8 index)
{
    return (u16)index * MD400_LCD_CARD_W;
}

static void md400_display_clear_screen(void)
{
    md400_lcd_fill_rect(0, 0, MD400_LCD_WIDTH, MD400_LCD_HEIGHT, MD400_LCD_BLACK);
}

static void md400_display_battery_mark_dirty_nolock(void)
{
    md400_top_bar_dirty = 1;
}

static void md400_display_save_check_expire_nolock(void)
{
    if (md400_save_check_visible &&
        ((s32)(sys_timer_get_ms() - md400_save_check_until_ms) >= 0)) {
        md400_save_check_visible = 0;
        md400_top_bar_dirty = 1;
    }
}

void md400_display_show_save_check(void)
{
    if (!md400_lcd_ready) {
        return;
    }

    local_irq_disable();
    md400_save_check_visible = 1;
    md400_save_check_until_ms = sys_timer_get_ms() + MD400_LCD_SAVE_CHECK_MS;
    md400_top_bar_dirty = 1;
    local_irq_enable();
}

void md400_display_pedal_cal_update(u8 state, u8 raw8, u8 min8, u8 max8,
                                    u8 value7)
{
    (void)state;
    (void)raw8;
    (void)min8;
    (void)max8;
    (void)value7;
}

void md400_display_bt_state_update(u8 state)
{
    if (state > 2) {
        state = 2;
    }

    local_irq_disable();
    if (md400_bt_state != state) {
        md400_bt_state = state;
        md400_bt_blink_on = 1;
        if (md400_lcd_ready) {
            md400_top_bar_dirty = 1;
        }
    }
    local_irq_enable();
}

static u8 md400_display_battery_charge_online(void)
{
    u8 online = get_charge_online_flag() ? 1 : 0;

#if TCFG_PC_ENABLE
    u32 usb_mode = usb_otg_online(USB0);

    if ((usb_mode == PRE_SLAVE_MODE) ||
        (usb_mode == SLAVE_MODE_WAIT_CONFIRMATION) ||
        (usb_mode == SLAVE_MODE) ||
        (usb_mode == CHARGE_MODE)) {
        online = 1;
    }
#endif

    return online;
}

static void md400_display_battery_update_level(u16 mv10)
{
    if (md400_battery_level_state == 0) {
        if (mv10 >= MD400_BATTERY_LEVEL_1_UP_MV10) {
            md400_battery_level_state = 1;
        }
    } else if (md400_battery_level_state == 1) {
        if (mv10 <= MD400_BATTERY_LEVEL_1_DOWN_MV10) {
            md400_battery_level_state = 0;
        } else if (mv10 >= MD400_BATTERY_LEVEL_2_UP_MV10) {
            md400_battery_level_state = 2;
        }
    } else {
        if (mv10 <= MD400_BATTERY_LEVEL_2_DOWN_MV10) {
            md400_battery_level_state = 1;
        }
    }
}

static void md400_display_battery_tick(void *priv)
{
    u16 mv10;
    u8 charging;
    u8 old_dots;
    u8 old_red;
    u8 old_visible;
    u8 old_charging;

    (void)priv;

    mv10 = get_vbat_level();
    charging = md400_display_battery_charge_online();

    old_dots = md400_battery_dots;
    old_red = md400_battery_red;
    old_visible = md400_battery_visible;
    old_charging = md400_battery_charging;

    md400_display_battery_update_level(mv10);

    if (charging) {
        md400_battery_critical = 0;
        md400_battery_shutdown_ticks = 0;
    } else if (md400_battery_critical) {
        if (mv10 >= MD400_BATTERY_LOW_EXIT_MV10) {
            md400_battery_critical = 0;
            md400_battery_shutdown_ticks = 0;
            md400_battery_visible = 1;
        }
    } else if (mv10 <= MD400_BATTERY_LOW_ENTER_MV10) {
        md400_battery_critical = 1;
        md400_battery_shutdown_ticks = 0;
        md400_battery_visible = 1;
    }

    md400_battery_charging = charging;
    if (charging) {
        md400_battery_red = 0;
        md400_battery_visible = 1;
        md400_battery_charge_tick++;
        if (md400_battery_charge_tick >= MD400_BATTERY_CHARGE_STEP_TICKS) {
            md400_battery_charge_tick = 0;
            md400_battery_charge_phase = (md400_battery_charge_phase + 1) % 3;
        }
        md400_battery_dots = md400_battery_charge_phase + 1;
    } else if (md400_battery_critical) {
        md400_battery_dots = 1;
        md400_battery_red = 1;
        md400_battery_visible = 1;
        if (md400_battery_shutdown_ticks < MD400_BATTERY_SHUTDOWN_TICKS) {
            md400_battery_shutdown_ticks++;
        }
#if MD400_BATTERY_AUTO_POWEROFF_ENABLE
        if (md400_battery_shutdown_ticks >= MD400_BATTERY_SHUTDOWN_TICKS) {
            power_set_soft_poweroff();
        }
#endif
    } else {
        md400_battery_visible = 1;
        md400_battery_dots = md400_battery_level_state + 1;
        md400_battery_red = (md400_battery_dots <= 1);
        md400_battery_charge_tick = 0;
        md400_battery_charge_phase = 0;
        md400_battery_shutdown_ticks = 0;
    }

    if ((old_dots != md400_battery_dots) ||
        (old_red != md400_battery_red) ||
        (old_visible != md400_battery_visible) ||
        (old_charging != md400_battery_charging)) {
        local_irq_disable();
        md400_display_battery_mark_dirty_nolock();
        local_irq_enable();
    }

    /* BT indicator blink: 1Hz on the 500ms tick while advertising (state 1) */
    md400_bt_blink_on ^= 1;
    if (md400_bt_state == 1) {
        local_irq_disable();
        md400_top_bar_dirty = 1;
        local_irq_enable();
    }
}

static u8 md400_display_append_chars(char *out, u8 len, u8 max_len,
                                     const char *text)
{
    u8 i;

    if (!out || !text) {
        return len;
    }
    for (i = 0; text[i] && ((len + 1) < max_len); i++) {
        out[len++] = text[i];
    }
    return len;
}

static u8 md400_display_append_pedal_page(char *out, u8 len, u8 max_len)
{
    u8 shown_page;

    if (!md400_foot_mode_pedal || ((len + 6) >= max_len)) {
        return len;
    }
    shown_page = (md400_foot_mode_page % 32) + 1;
    out[len++] = ' ';
    if (shown_page >= 10) {
        out[len++] = '0' + (shown_page / 10);
    }
    out[len++] = '0' + (shown_page % 10);
    out[len++] = '/';
    out[len++] = '3';
    out[len++] = '2';
    return len;
}

static void md400_display_top_bar_label(char *out, u8 max_len)
{
    const char *suffix = md400_foot_mode_pedal ? "PEDAL" : "MIDI";
    u8 len = 0;
    u8 i;
    u8 subject;
    u8 page;

    if (!out || !max_len) {
        return;
    }

    if (md400_ui_mode == MD400_LCD_MODE_PLAY) {
        suffix = md400_foot_mode_pedal ? "PEDAL PLAY" : "MIDI PLAY";
    } else if (md400_ui_mode == MD400_LCD_MODE_EDIT) {
        subject = md400_display_edit_subject_number();
        page = md400_display_edit_page_number();
        if (subject == MD400_DISPLAY_EDIT_SUBJECT_PEDAL) {
            /* This page edits the EXP/SWT input. "PEDAL EDIT" is reserved for
             * the separate PEDAL foot mode and would be ambiguous here. */
            suffix = "EXP EDIT";
            len = md400_display_append_chars(out, len, max_len, suffix);
        } else if ((subject != MD400_DISPLAY_EDIT_SUBJECT_PRESET) &&
                   (page >= 1) && ((len + 8) < max_len)) {
            /* Four FT pages: 1-2 press, 3-4 rotate. */
            out[len++] = 'F';
            out[len++] = 'T';
            out[len++] = '0' + md400_display_edit_target_number();
            out[len++] = ' ';
            len = md400_display_append_chars(
                out, len, max_len,
                md400_display_edit_page_is_rotate() ? "ROT" : "PRESS");
            len = md400_display_append_pedal_page(out, len, max_len);
            out[len] = 0;
            return;
        } else {
            suffix = md400_foot_mode_pedal ? "PEDAL EDIT" : "MIDI EDIT";
            len = md400_display_append_chars(out, len, max_len, suffix);
        }
        len = md400_display_append_pedal_page(out, len, max_len);
        out[len] = 0;
        return;
    }

    for (i = 0; suffix[i] && (len + 1 < max_len); i++) {
        out[len++] = suffix[i];
    }
    len = md400_display_append_pedal_page(out, len, max_len);
    out[len] = 0;
}

static u8 md400_display_text_same(const char *a, const char *b, u8 max_len)
{
    u8 i;

    if (!a || !b) {
        return 0;
    }
    for (i = 0; i < max_len; i++) {
        if (a[i] != b[i]) {
            return 0;
        }
        if (!a[i]) {
            return 1;
        }
    }
    return 1;
}

static void md400_display_text_cache_copy(char *dst, const char *src, u8 max_len)
{
    u8 i;

    if (!dst || !src || !max_len) {
        return;
    }
    for (i = 0; (i + 1) < max_len; i++) {
        dst[i] = src[i];
        if (!src[i]) {
            return;
        }
    }
    dst[i] = 0;
}

static void md400_display_draw_top_bar(u8 full)
{
    u8 dots;
    u8 red;
    u8 visible;
    u8 save_visible;
    u8 bt_state;
    u8 bt_blink;
    u8 i;
    u8 label_changed;
    u8 battery_changed;
    u8 save_changed;
    u8 bt_changed;
    u16 color;
    u16 x;
    char label[MD400_LCD_TOP_LABEL_MAX + 1];

    local_irq_disable();
    md400_display_save_check_expire_nolock();
    dots = md400_battery_dots;
    red = md400_battery_red;
    visible = md400_battery_visible;
    save_visible = md400_save_check_visible;
    bt_state = md400_bt_state;
    bt_blink = md400_bt_blink_on;
    local_irq_enable();

    md400_display_top_bar_label(label, sizeof(label));
    label_changed = full || !md400_top_bar_label_valid ||
                    !md400_display_text_same(label, md400_top_bar_label_cache,
                                             sizeof(label));
    battery_changed = full || !md400_top_bar_battery_valid ||
                      (dots != md400_top_bar_battery_dots) ||
                      (red != md400_top_bar_battery_red) ||
                      (visible != md400_top_bar_battery_visible);
    save_changed = full || !md400_top_bar_save_check_valid ||
                   (save_visible != md400_top_bar_save_check_visible);
    bt_changed = full || !md400_top_bar_bt_valid ||
                 (bt_state != md400_top_bar_bt_state) ||
                 ((bt_state == 1) && (bt_blink != md400_top_bar_bt_blink));

    if (full) {
        md400_lcd_fill_rect(0, 0, MD400_LCD_WIDTH, MD400_LCD_TOP_BAR_H,
                            MD400_LCD_BLACK);
    }
    if (label_changed) {
        u8 first = 0;
        u8 last = 0;
        if (!full && md400_top_bar_label_valid) {
            while (label[first] &&
                   label[first] == md400_top_bar_label_cache[first]) {
                first++;
            }
            last = first;
            while (label[last]) last++;
            i = first;
            while (md400_top_bar_label_cache[i]) i++;
            if (i > last) last = i;
            /* Erase only the changed suffix, including any old trailing digit
             * when a group number shrinks from two digits to one. */
            md400_lcd_fill_rect(MD400_LCD_TOP_LABEL_X + first * 12,
                                MD400_LCD_TOP_LABEL_Y, (last - first) * 12,
                                14, MD400_LCD_BLACK);
        } else if (!full) {
            md400_lcd_fill_rect(0, 0, MD400_LCD_TOP_LABEL_W,
                                MD400_LCD_TOP_BAR_H, MD400_LCD_BLACK);
        }
        md400_lcd_draw_font_text(MD400_LCD_TOP_LABEL_X + first * 12,
                                 MD400_LCD_TOP_LABEL_Y, label + first,
                                 2, 2, MD400_LCD_WHITE,
                                 MD400_LCD_TOP_LABEL_MAX - first);
        md400_display_text_cache_copy(md400_top_bar_label_cache, label,
                                      sizeof(md400_top_bar_label_cache));
        md400_top_bar_label_valid = 1;
    }

    if (save_changed) {
        md400_lcd_fill_rect(MD400_LCD_SAVE_CHECK_X, 0,
                            MD400_LCD_SAVE_CHECK_W,
                            MD400_LCD_TOP_BAR_H, MD400_LCD_BLACK);
        if (save_visible) {
            x = MD400_LCD_SAVE_CHECK_X + 2;
            md400_lcd_fill_rect(x, 16, 3, 3, MD400_LCD_GREEN);
            md400_lcd_fill_rect(x + 3, 18, 3, 3, MD400_LCD_GREEN);
            md400_lcd_fill_rect(x + 6, 15, 3, 3, MD400_LCD_GREEN);
            md400_lcd_fill_rect(x + 9, 12, 3, 3, MD400_LCD_GREEN);
            md400_lcd_fill_rect(x + 12, 9, 3, 3, MD400_LCD_GREEN);
        }
        md400_top_bar_save_check_visible = save_visible;
        md400_top_bar_save_check_valid = 1;
    }

    if (battery_changed) {
        if (!full) {
            md400_lcd_fill_rect(MD400_LCD_BATTERY_AREA_X, 0,
                                MD400_LCD_BATTERY_AREA_W,
                                MD400_LCD_TOP_BAR_H, MD400_LCD_BLACK);
        }
        if (visible) {
            color = red ? MD400_LCD_RED : MD400_LCD_WHITE;
            for (i = 0; i < dots && i < 3; i++) {
                x = MD400_LCD_WIDTH - 34 + ((u16)i * 11);
                md400_lcd_fill_rect(x, 12, 5, 5, color);
            }
        }
        md400_top_bar_battery_dots = dots;
        md400_top_bar_battery_red = red;
        md400_top_bar_battery_visible = visible;
        md400_top_bar_battery_valid = 1;
    }

    if (bt_changed) {
        if (!full) {
            md400_lcd_fill_rect(MD400_LCD_BT_AREA_X, 0, MD400_LCD_BT_AREA_W,
                                MD400_LCD_TOP_BAR_H, MD400_LCD_BLACK);
        }
        /* off = gray solid, advertising = white 1Hz blink, connected = white solid */
        if ((bt_state == 2) || (bt_state == 0) || bt_blink) {
            color = bt_state ? MD400_LCD_WHITE : MD400_LCD_GRAY;
            md400_lcd_draw_font_text(MD400_LCD_BT_AREA_X + 2,
                                     MD400_LCD_TOP_LABEL_Y, "BT", 2, 2,
                                     color, 2);
        }
        md400_top_bar_bt_state = bt_state;
        md400_top_bar_bt_blink = bt_blink;
        md400_top_bar_bt_valid = 1;
    }
}

static u8 md400_display_format_play_line(int msg, u8 value, char *out,
                                         u8 max_len, u8 *value_start)
{
    char prefix[MD400_LCD_PLAY_PREFIX_MAX + 1];
    char value_text[4];
    u8 prefix_len;
    u8 len = 0;
    u8 i;

    if (!out || !max_len) {
        return 0;
    }

    if (MD400_DISPLAY_MSG_RAW_IS(msg)) {
        return md400_display_format_play_raw(msg, out, max_len, value_start);
    }

    prefix_len = md400_display_format_play_prefix(msg, prefix, sizeof(prefix));
    md400_display_format_play_value(value, value_text);

    for (i = 0; (i < prefix_len) && ((len + 1) < max_len); i++) {
        out[len++] = prefix[i];
    }
    if (md400_display_msg_type(msg) == MD400_DISPLAY_MSG_SYSEX) {
        if (value_start) {
            *value_start = len;
        }
        out[len] = 0;
        return len;
    }
    if ((len + 1) < max_len) {
        out[len++] = ' ';
    }
    if (value_start) {
        *value_start = len;
    }
    for (i = 0; (i < 3) && ((len + 1) < max_len); i++) {
        out[len++] = value_text[i];
    }
    out[len] = 0;
    return len;
}

static u8 md400_display_play_line_scale(const char *line)
{
    u8 len = md400_lcd_text_len(line, MD400_LCD_PLAY_LINE_MAX);
    u8 scale;
    u16 width;
    u16 max_w = MD400_LCD_WIDTH - (MD400_LCD_PLAY_LINE_PAD * 2);

    for (scale = MD400_LCD_PLAY_LINE_SCALE_MAX;
         scale > MD400_LCD_PLAY_LINE_SCALE_MIN; scale--) {
        width = md400_lcd_font_text_width(len, scale,
                                          MD400_LCD_PLAY_LINE_GAP);
        if (width <= max_w) {
            return scale;
        }
    }
    return MD400_LCD_PLAY_LINE_SCALE_MIN;
}

static u16 md400_display_play_line_start_x(const char *line, u8 scale)
{
    u8 len = md400_lcd_text_len(line, MD400_LCD_PLAY_LINE_MAX);
    u16 width = md400_lcd_font_text_width(len, scale,
                                          MD400_LCD_PLAY_LINE_GAP);

    if (width >= MD400_LCD_WIDTH) {
        return 0;
    }
    return (MD400_LCD_WIDTH - width) / 2;
}

static u16 md400_display_play_line_color(u8 index, u8 value_start)
{
    return (index >= value_start) ? MD400_LCD_CYAN : MD400_LCD_WHITE;
}

static void md400_display_play_line_cache_store(const char *line, u8 len,
                                                u16 start_x, u8 scale,
                                                u8 value_start)
{
    u8 i;

    for (i = 0; i < MD400_LCD_PLAY_LINE_MAX; i++) {
        md400_play_line_cache[i] = ((line != NULL) && (i < len)) ? line[i] : 0;
    }
    md400_play_line_cache[MD400_LCD_PLAY_LINE_MAX] = 0;
    md400_play_line_cache_len = len;
    md400_play_line_cache_x = start_x;
    md400_play_line_cache_scale = scale;
    md400_play_line_cache_value_start = value_start;
    md400_play_line_cache_valid = 1;
}

static void md400_display_draw_play_line(u8 full)
{
    char line[MD400_LCD_PLAY_LINE_MAX + 1];
    u8 value_start = 0;
    u8 len;
    u8 scale;
    u16 start_x;
    u8 layout_changed;
    u8 i;
    u16 x;
    u16 old_color;
    u16 new_color;

    len = md400_display_format_play_line(md400_play_msg, md400_play_value,
                                         line, sizeof(line), &value_start);
    scale = md400_display_play_line_scale(line);
    start_x = md400_display_play_line_start_x(line, scale);

    layout_changed = full || !md400_play_line_cache_valid ||
                     (md400_play_line_cache_len != len) ||
                     (md400_play_line_cache_x != start_x) ||
                     (md400_play_line_cache_scale != scale) ||
                     (md400_play_line_cache_value_start != value_start);

    if (layout_changed) {
        md400_diag_draw_play_line_full++;
        md400_lcd_fill_rect(0, MD400_LCD_PLAY_LINE_AREA_Y,
                            MD400_LCD_WIDTH, MD400_LCD_PLAY_LINE_AREA_H,
                            MD400_LCD_BLACK);
        for (i = 0; i < len; i++) {
            x = start_x + ((u16)i * ((5 * scale) + MD400_LCD_PLAY_LINE_GAP));
#if MD400_LCD_HW_SPI_ENABLE
            md400_lcd_blit_glyph(x, MD400_LCD_PLAY_LINE_Y, line[i], scale,
                                 md400_display_play_line_color(i, value_start),
                                 MD400_LCD_BLACK);
#else
            md400_lcd_draw_font_char(x, MD400_LCD_PLAY_LINE_Y, line[i],
                                     scale,
                                     md400_display_play_line_color(i,
                                                                   value_start));
#endif
        }
        md400_display_play_line_cache_store(line, len, start_x, scale,
                                            value_start);
        return;
    }

    for (i = 0; i < len; i++) {
        old_color = md400_display_play_line_color(
            i, md400_play_line_cache_value_start);
        new_color = md400_display_play_line_color(i, value_start);
        if ((md400_play_line_cache[i] == line[i]) &&
            (old_color == new_color)) {
            continue;
        }
        x = start_x + ((u16)i * ((5 * scale) + MD400_LCD_PLAY_LINE_GAP));
        md400_lcd_update_font_char(x, MD400_LCD_PLAY_LINE_Y,
                                   md400_play_line_cache[i], old_color,
                                   line[i], new_color, scale,
                                   MD400_LCD_BLACK);
    }
    md400_display_play_line_cache_store(line, len, start_x, scale, value_start);
}

static void md400_display_draw_play_value_only(void)
{
    char line[MD400_LCD_PLAY_LINE_MAX + 1];
    u8 value_start = 0;
    u8 len;
    u8 scale;
    u16 start_x;
    u8 layout_changed;
    u8 i;
    u16 x;
    u16 color;

    len = md400_display_format_play_line(md400_play_msg, md400_play_value,
                                         line, sizeof(line), &value_start);
    scale = md400_display_play_line_scale(line);
    start_x = md400_display_play_line_start_x(line, scale);

    layout_changed = !md400_play_line_cache_valid ||
                     (md400_play_line_cache_len != len) ||
                     (md400_play_line_cache_x != start_x) ||
                     (md400_play_line_cache_scale != scale) ||
                     (md400_play_line_cache_value_start != value_start);
    if (layout_changed) {
        for (i = value_start; (i < len) && (i < (value_start + 3)); i++) {
            x = start_x + ((u16)i * ((5 * scale) + MD400_LCD_PLAY_LINE_GAP));
#if MD400_LCD_HW_SPI_ENABLE
            md400_lcd_blit_glyph(x, MD400_LCD_PLAY_LINE_Y, line[i], scale,
                                 md400_display_play_line_color(i, value_start),
                                 MD400_LCD_BLACK);
#else
            md400_lcd_fill_rect(x, MD400_LCD_PLAY_LINE_Y, 5 * scale,
                                7 * scale, MD400_LCD_BLACK);
            md400_lcd_draw_font_char(x, MD400_LCD_PLAY_LINE_Y, line[i],
                                     scale,
                                     md400_display_play_line_color(i,
                                                                   value_start));
#endif
        }
        md400_diag_draw_play_value++;
        md400_display_play_line_cache_store(line, len, start_x, scale,
                                            value_start);
        return;
    }

    color = md400_display_play_line_color(value_start, value_start);
    md400_diag_draw_play_value++;
    for (i = value_start; (i < len) && (i < (value_start + 3)); i++) {
        if (md400_play_line_cache[i] == line[i]) {
            continue;
        }
        x = start_x + ((u16)i * ((5 * scale) + MD400_LCD_PLAY_LINE_GAP));
        md400_lcd_update_font_char(x, MD400_LCD_PLAY_LINE_Y,
                                   md400_play_line_cache[i], color,
                                   line[i], color, scale,
                                   MD400_LCD_BLACK);
    }
    md400_display_play_line_cache_store(line, len, start_x, scale, value_start);
}

static void md400_display_draw_play_label(u8 full)
{
    u8 len;
    u16 width;
    u16 start_x;
    u8 layout_changed;
    u8 i;
    u16 x;

    len = md400_lcd_text_len(md400_play_label, MD400_DISPLAY_PLAY_LABEL_LEN);
    width = md400_lcd_font_text_width(len, MD400_LCD_PLAY_LABEL_SCALE,
                                      MD400_LCD_PLAY_LABEL_GAP);
    start_x = (width < MD400_LCD_WIDTH) ? ((MD400_LCD_WIDTH - width) / 2) : 0;

    layout_changed = full || !md400_play_label_cache_valid ||
                     (md400_play_label_cache_len != len) ||
                     (md400_play_label_cache_x != start_x);

    if (layout_changed) {
        md400_diag_draw_play_label++;
        md400_lcd_fill_rect(0, MD400_LCD_PLAY_LABEL_Y - 4, MD400_LCD_WIDTH,
                            32, MD400_LCD_BLACK);
        md400_lcd_draw_font_text(start_x, MD400_LCD_PLAY_LABEL_Y,
                                 md400_play_label, MD400_LCD_PLAY_LABEL_SCALE,
                                 MD400_LCD_PLAY_LABEL_GAP, MD400_LCD_YELLOW,
                                 MD400_DISPLAY_PLAY_LABEL_LEN);
    } else {
        for (i = 0; i < len; i++) {
            if (md400_play_label_cache[i] == md400_play_label[i]) {
                continue;
            }
            x = start_x + ((u16)i * ((5 * MD400_LCD_PLAY_LABEL_SCALE) +
                                     MD400_LCD_PLAY_LABEL_GAP));
            md400_lcd_update_font_char(x, MD400_LCD_PLAY_LABEL_Y,
                                       md400_play_label_cache[i],
                                       MD400_LCD_YELLOW,
                                       md400_play_label[i], MD400_LCD_YELLOW,
                                       MD400_LCD_PLAY_LABEL_SCALE,
                                       MD400_LCD_BLACK);
            md400_diag_draw_play_label++;
        }
    }

    if (layout_changed ||
        !md400_display_text_same(md400_play_label, md400_play_label_cache,
                                 sizeof(md400_play_label))) {
        memset(md400_play_label_cache, 0, sizeof(md400_play_label_cache));
        md400_display_text_cache_copy(md400_play_label_cache, md400_play_label,
                                      sizeof(md400_play_label_cache));
        md400_play_label_cache_len = len;
        md400_play_label_cache_x = start_x;
        md400_play_label_cache_valid = 1;
    }
}

static void md400_display_draw_play_full(void)
{
    md400_diag_draw_play_full++;
    md400_lcd_fill_rect(0, MD400_LCD_TOP_BAR_H, MD400_LCD_WIDTH,
                        MD400_LCD_HEIGHT - MD400_LCD_TOP_BAR_H,
                        MD400_LCD_BLACK);
    md400_play_line_cache_valid = 0;
    md400_play_label_cache_valid = 0;
    md400_display_draw_play_line(1);
    md400_display_draw_play_label(1);
}

static void md400_display_draw_card_value(u8 index, u8 full)
{
    u16 x;
    u16 y;
    u16 box_w;
    u16 bg;
    int value;

    if (index >= 4) {
        return;
    }

    x = md400_display_card_x(index) + MD400_LCD_VALUE_X_PAD;
    box_w = MD400_LCD_VALUE_W;
    if (md400_display_value_style(index) == MD400_LCD_VALUE_STYLE_FONT) {
        x = md400_display_card_x(index) + 2;
        box_w = MD400_LCD_CARD_W - 4;
    }
    y = MD400_LCD_CARD_Y + MD400_LCD_VALUE_Y_PAD;
    bg = md400_display_row_bg(index);
    value = md400_display_row_value(index);

    md400_display_draw_value_box(index, x, y + 6, box_w, value,
                                 MD400_LCD_BLACK, bg, full);
}

static void md400_display_draw_card_press(u8 index)
{
    u16 x;
    u16 y;
    u16 bg;
    u16 color;

    if (index >= 4) {
        return;
    }

    x = md400_display_card_x(index);
    y = MD400_LCD_CARD_Y;
    bg = md400_display_row_bg(index);
    color = (!md400_ui_mode && md400_encoder_pressed[index]) ?
            MD400_LCD_WHITE : MD400_LCD_DARK2;

    md400_lcd_draw_rect(x + 1, y + 1, MD400_LCD_CARD_W - 2,
                        MD400_LCD_CARD_H - 2, color);
    md400_lcd_fill_rect(x + 10, y + MD400_LCD_PRESS_Y,
                        MD400_LCD_CARD_W - 20, 10, bg);
    md400_lcd_fill_rect(x + 13, y + MD400_LCD_PRESS_Y + 2,
                        MD400_LCD_CARD_W - 26, 6,
                        (!md400_ui_mode && md400_encoder_pressed[index]) ?
                        MD400_LCD_WHITE : MD400_LCD_GRAY);
}

static void md400_display_draw_perform_assignments(u8 index, u16 x, u16 y)
{
    char text[9];

    md400_display_format_msg(md400_perform_rotate_msg[index], text);
    md400_lcd_draw_font_text_centered(x, MD400_LCD_CARD_W, y + 12,
                                      text, 2, 0, MD400_LCD_BLACK, 8);

    md400_display_format_msg(md400_perform_press_msg[index], text);
    md400_lcd_draw_font_text_centered(x, MD400_LCD_CARD_W,
                                      y + MD400_LCD_PRESS_Y - 28,
                                      text, 2, 0, MD400_LCD_BLACK, 8);
}

static void md400_display_draw_card_label(u8 index)
{
    u16 x;
    u16 y;
    u16 bg;
    const char *label;

    if (index >= 4) {
        return;
    }

    x = md400_display_card_x(index);
    y = MD400_LCD_CARD_Y;
    bg = md400_display_row_bg(index);

    md400_lcd_fill_rect(x + 4, y + 8, MD400_LCD_CARD_W - 8, 28, bg);
    if (!md400_ui_mode) {
        md400_lcd_fill_rect(x + 4, y + MD400_LCD_PRESS_Y - 34,
                            MD400_LCD_CARD_W - 8, 26, bg);
        md400_display_draw_perform_assignments(index, x, y);
        return;
    }

    label = md400_display_role_label(md400_ui_row_role[index], index);
    md400_lcd_draw_font_text_centered(x, MD400_LCD_CARD_W, y + 12,
                                      label, 2, 2, MD400_LCD_BLACK, 6);
}

static void md400_display_draw_card_full(u8 index)
{
    u16 x;
    u16 y;
    u16 bg;

    if (index >= 4) {
        return;
    }

    x = md400_display_card_x(index);
    y = MD400_LCD_CARD_Y;
    bg = md400_display_row_bg(index);

    md400_lcd_fill_rect(x, y, MD400_LCD_CARD_W, MD400_LCD_CARD_H, bg);
    md400_lcd_fill_rect(x + 4, y + 4, MD400_LCD_CARD_W - 8, 3, MD400_LCD_BLACK);
    md400_display_draw_card_label(index);
    md400_display_draw_card_value(index, 1);
    md400_display_draw_card_press(index);
}

void md400_display_foot_mode_update(u8 pedal_mode, u8 page,
                                    const u8 *color)
{
    (void)color; /* Retained for callers; pedal colors are fixed by slot. */
    u8 mode_changed;

    pedal_mode = !!pedal_mode;
    page %= 32;
    local_irq_disable();
    mode_changed = md400_foot_mode_pedal != pedal_mode;
    md400_foot_mode_pedal = pedal_mode;
    if (md400_foot_mode_page != page) {
        md400_foot_mode_page = page;
        if (pedal_mode && md400_lcd_ready) {
            md400_top_bar_dirty = 1;
        }
    }
    if (mode_changed && md400_lcd_ready) {
        md400_display_mark_full_screen_nolock();
    }
    local_irq_enable();
}

void md400_display_pedal_edit_update(u8 selected_slot, u8 style_view)
{
    selected_slot = ((selected_slot - 1) % 4) + 1;
    style_view = !!style_view;
    local_irq_disable();
    if ((md400_pedal_edit_slot != selected_slot) ||
        (md400_pedal_edit_style_view != style_view)) {
        md400_pedal_edit_slot = selected_slot;
        md400_pedal_edit_style_view = style_view;
        if (md400_lcd_ready) {
            md400_top_bar_dirty = 1;
        }
    }
    local_irq_enable();
}

static u16 md400_display_boot_card_height(u8 step, u8 index)
{
    int frame;

    frame = (int)step -
            ((int)index * MD400_BOOT_ANIM_STAGGER_STEPS);
    if (frame <= 0) {
        return 0;
    }
    if (frame >= MD400_BOOT_ANIM_GROW_STEPS) {
        return MD400_LCD_CARD_H;
    }
    return ((u32)MD400_LCD_CARD_H * (u16)frame) /
           MD400_BOOT_ANIM_GROW_STEPS;
}

static void md400_display_draw_boot_animation_step(u8 step)
{
    u8 i;
    u16 x;
    u16 y;
    u16 h;
    u16 old_h;

    if (step == 0) {
        md400_display_clear_screen();
        md400_lcd_pin_output(MD400_LCD_LIGHT, MD400_LCD_LIGHT_ON);
        return;
    }

    /* This is the live UI geometry itself, without a separate splash surface. */
    if (step == (MD400_BOOT_ANIM_STEPS - 1)) {
        md400_display_draw_top_bar(1);
        for (i = 0; i < 4; i++) {
            md400_display_draw_card_full(i);
        }
        return;
    }

    for (i = 0; i < 4; i++) {
        old_h = md400_display_boot_card_height(step - 1, i);
        h = md400_display_boot_card_height(step, i);
        if (h <= old_h) {
            continue;
        }
        x = (u16)i * MD400_LCD_CARD_W;
        y = MD400_LCD_HEIGHT - h;
        md400_lcd_fill_rect(x, y, MD400_LCD_CARD_W, h - old_h,
                            md400_display_row_bg(i));
        if (h == MD400_LCD_CARD_H) {
            md400_lcd_fill_rect(x + 4, y + 4,
                                MD400_LCD_CARD_W - 8, 3,
                                MD400_LCD_BLACK);
        }
    }
}

/* Return 1 while the animation owns the panel. On completion, queue one full
 * live-UI repaint and let the normal dirty-region renderer continue. */
static u8 md400_display_boot_animation_tick(void)
{
    u32 elapsed;
    u8 step;
    u8 next;

    if (!md400_boot_anim_active) {
        return 0;
    }

    elapsed = sys_timer_get_ms() - md400_boot_anim_start_ms;
    if (elapsed >= MD400_BOOT_ANIM_DURATION_MS) {
        local_irq_disable();
        md400_boot_anim_active = 0;
        md400_lcd_first_frame_pending = 0;
        md400_display_mark_full_screen_nolock();
        local_irq_enable();
        printf("[MD400_LCD] boot animation done\r\n");
        return 0;
    }

    step = (u8)(elapsed / MD400_BOOT_ANIM_STEP_MS);
    if (step >= MD400_BOOT_ANIM_STEPS) {
        step = MD400_BOOT_ANIM_STEPS - 1;
    }
    if (step == md400_boot_anim_last_step) {
        return 1;
    }

    next = (md400_boot_anim_last_step == 0xff) ?
           0 : (md400_boot_anim_last_step + 1);
    while (next <= step) {
        md400_display_draw_boot_animation_step(next);
        next++;
    }
    md400_boot_anim_last_step = step;
    return 1;
}

static u16 md400_display_shutdown_card_height(u8 step, u8 index)
{
    int frame;

    frame = (int)step -
            ((int)(3 - index) * MD400_SHUTDOWN_ANIM_STAGGER_STEPS);
    if (frame <= 0) {
        return MD400_LCD_CARD_H;
    }
    if (frame >= MD400_SHUTDOWN_ANIM_COLLAPSE_STEPS) {
        return 0;
    }
    return MD400_LCD_CARD_H -
           (((u32)MD400_LCD_CARD_H * (u16)frame) /
            MD400_SHUTDOWN_ANIM_COLLAPSE_STEPS);
}

static void md400_display_draw_shutdown_animation_step(u8 step)
{
    u8 i;
    u16 x;
    u16 y;
    u16 h;
    u16 old_h;

    if (step == 0) {
        /*
         * If power-off starts from PLAY or EDIT, first restore the four cards
         * to the active MIDI/PEDAL palette. The reverse animation therefore
         * always belongs to the current foot mode, not the transient page.
         */
        if (md400_ui_mode != 0) {
            for (i = 0; i < 4; i++) {
                md400_display_draw_card_full(i);
            }
        }
        /* Remove the top row first; the recolored card contents remain. */
        md400_lcd_fill_rect(0, 0, MD400_LCD_WIDTH, MD400_LCD_TOP_BAR_H,
                            MD400_LCD_BLACK);
        return;
    }

    if (step == (MD400_SHUTDOWN_ANIM_STEPS - 1)) {
        md400_display_clear_screen();
        md400_lcd_pin_output(MD400_LCD_LIGHT, MD400_LCD_LIGHT_OFF);
        return;
    }

    /* Reverse the boot rise: wipe each live card downward, right-to-left.
     * The still-visible lower portion retains its real labels and values. */
    for (i = 0; i < 4; i++) {
        old_h = md400_display_shutdown_card_height(step - 1, i);
        h = md400_display_shutdown_card_height(step, i);
        if (h >= old_h) {
            continue;
        }
        x = (u16)i * MD400_LCD_CARD_W;
        y = MD400_LCD_HEIGHT - old_h;
        md400_lcd_fill_rect(x, y, MD400_LCD_CARD_W,
                            old_h - h, MD400_LCD_BLACK);
    }
}

/* Shutdown keeps ownership of the panel after its last black frame. The board
 * power-key timer releases the latch (or enters USB soft-off) 50 ms later. */
static u8 md400_display_shutdown_animation_tick(void)
{
    u32 elapsed;
    u8 step;
    u8 next;

    if (!md400_shutdown_anim_active) {
        return 0;
    }

    elapsed = sys_timer_get_ms() - md400_shutdown_anim_start_ms;
    step = (u8)(elapsed / MD400_SHUTDOWN_ANIM_STEP_MS);
    if (step >= MD400_SHUTDOWN_ANIM_STEPS) {
        step = MD400_SHUTDOWN_ANIM_STEPS - 1;
    }
    if (step == md400_shutdown_anim_last_step) {
        return 1;
    }

    next = (md400_shutdown_anim_last_step == 0xff) ?
           0 : (md400_shutdown_anim_last_step + 1);
    while (next <= step) {
        md400_display_draw_shutdown_animation_step(next);
        next++;
    }
    md400_shutdown_anim_last_step = step;
    return 1;
}

u16 md400_display_shutdown_start(void)
{
    if (!md400_lcd_ready || !md400_display_timer) {
        return 0;
    }

    local_irq_disable();
    md400_boot_anim_active = 0;
    md400_shutdown_anim_active = 1;
    md400_shutdown_anim_last_step = 0xff;
    md400_shutdown_anim_start_ms = sys_timer_get_ms();
    local_irq_enable();
    printf("[MD400_LCD] shutdown animation start %ums\r\n",
           (unsigned)MD400_SHUTDOWN_ANIM_DURATION_MS);
    return MD400_SHUTDOWN_POWER_DELAY_MS;
}

/* Upper bound on regions drained in a single refresh tick. The worst real case
 * is a full-screen repaint (clear+top bar then 4 cards => 5 draws); the cap just
 * guards against an unexpected dirty-loop and never loses work (leftovers paint
 * on the next tick). */
#define MD400_DISPLAY_MAX_DRAWS_PER_TICK 16

/*
 * DAW view.
 *
 * A host DAW script (Ableton Live remote script, FL Studio device script)
 * drives this view through NanoBridge DAW_UPDATE frames. Everything lives in
 * RAM only; nothing here touches flash. While the view is active the normal
 * Perform/Play renderer is suspended (its dirty flags keep accumulating) and a
 * full repaint of the normal UI is requested when the view closes.
 *
 * Layout (320x176):
 *   top bar 0..27   accent block, title (left), status (right), accent rule
 *   cards 28..175   4 x 80 px: colour strip, label, value, meter, index
 */
#define MD400_DAW_CARD_STRIP_H   4
#define MD400_DAW_LABEL_Y        12
#define MD400_DAW_LABEL_H        14
#define MD400_DAW_VALUE_Y        42
#define MD400_DAW_VALUE_H        28
#define MD400_DAW_METER_X_PAD    8
#define MD400_DAW_METER_Y        92
#define MD400_DAW_METER_H        12
#define MD400_DAW_INDEX_Y        124
#define MD400_DAW_BG             0x0000
#define MD400_DAW_CARD_BG        0x0841
#define MD400_DAW_FRAME          0x2104
#define MD400_DAW_DIM_TEXT       0x8410

static u8 md400_daw_view_active;
static u8 md400_daw_view_waiting;
static char daw_app_name[13];
void md400_display_daw_app_name(const char *title)
{
    memcpy(daw_app_name, title, 12);
}
static u8 md400_daw_waiting_dirty;
static u8 md400_daw_top_dirty;
static u8 md400_daw_card_full_dirty;
static u8 md400_daw_card_value_dirty;
static u8 md400_daw_screen_dirty;
static u16 md400_daw_accent = MD400_LCD_CYAN;
static char md400_daw_title[MD400_DISPLAY_DAW_TITLE_MAX + 1];
static char md400_daw_status[MD400_DISPLAY_DAW_STATUS_MAX + 1];
static char md400_daw_label[4][MD400_DISPLAY_DAW_LABEL_MAX + 1];
static char md400_daw_value[4][MD400_DISPLAY_DAW_VALUE_MAX + 1];
static u16 md400_daw_color[4];
static u8 md400_daw_meter[4];
static u8 md400_daw_flags[4];

static u16 md400_daw_dim(u16 color)
{
    /* Quarter-brightness of an RGB565 colour, used for lit card backgrounds. */
    u16 r = (color >> 11) & 0x1f;
    u16 g = (color >> 5) & 0x3f;
    u16 b = color & 0x1f;

    return (u16)(((r >> 2) << 11) | ((g >> 2) << 5) | (b >> 2));
}

/* Eighth glyph row for DAW text: g j p q y hang below the baseline. Every
 * character paints it, so a replaced descender never leaves pixels behind. */
static void md400_daw_draw_descender(u16 x, u16 y, char ch, u8 scale, u16 fg, u16 bg)
{
    u8 bits = 0;
    u8 col;

    switch (ch) {
    case 'g': bits = 0x0e; break;
    case 'j': bits = 0x0c; break;
    case 'p': bits = 0x10; break;
    case 'q': bits = 0x01; break;
    case 'y': bits = 0x0e; break;
    default: break;
    }
    y += 7 * scale;
    if ((y + scale) > MD400_LCD_HEIGHT) {
        return;
    }
    md400_lcd_fill_rect(x, y, 5 * scale, scale, bg);
    for (col = 0; col < 5; col++) {
        if (bits & BIT(4 - col)) {
            md400_lcd_fill_rect(x + (u16)col * scale, y, scale, scale, fg);
        }
    }
}

static void md400_daw_draw_text(u16 x, u16 y, const char *text, u8 len,
                                u8 scale, u16 fg, u16 bg)
{
    u8 i;
    u16 step = (u16)(5 * scale) + scale;

    md400_lcd_font_mixed_case = 1;
    for (i = 0; i < len; i++) {
#if MD400_LCD_HW_SPI_ENABLE
        md400_lcd_blit_glyph(x, y, text[i], scale, fg, bg);
#else
        md400_lcd_fill_rect(x, y, 5 * scale, 7 * scale, bg);
        md400_lcd_draw_font_char(x, y, text[i], scale, fg);
#endif
        md400_daw_draw_descender(x, y, text[i], scale, fg, bg);
        x += step;
    }
    md400_lcd_font_mixed_case = 0;
}

static u16 md400_daw_text_width(u8 len, u8 scale)
{
    return md400_lcd_font_text_width(len, scale, scale);
}

/* Largest scale (<= max_scale) whose text fits in box_w. 0 if nothing fits. */
static u8 md400_daw_fit_scale(u8 len, u16 box_w, u8 max_scale)
{
    u8 scale;

    for (scale = max_scale; scale >= 1; scale--) {
        if (md400_daw_text_width(len, scale) <= box_w) {
            return scale;
        }
    }
    return 0;
}

static void md400_daw_draw_text_centered(u16 x, u16 box_w, u16 y, u16 box_h,
                                         const char *text, u8 max_scale,
                                         u16 fg, u16 bg)
{
    u8 len = md400_lcd_text_len(text, 20);
    u8 scale;
    u16 w;

    md400_lcd_fill_rect(x, y, box_w, box_h, bg);
    while (len && !(scale = md400_daw_fit_scale(len, box_w, max_scale))) {
        len--;
    }
    if (!len) {
        return;
    }
    w = md400_daw_text_width(len, scale);
    md400_daw_draw_text(x + ((box_w - w) / 2),
                        y + ((box_h > (u16)(7 * scale)) ? ((box_h - (7 * scale)) / 2) : 0),
                        text, len, scale, fg, bg);
}

static void md400_daw_draw_top_bar(void)
{
    char title[MD400_DISPLAY_DAW_TITLE_MAX + 1];
    char status[MD400_DISPLAY_DAW_STATUS_MAX + 1];
    u16 accent;
    u8 title_len;
    u8 status_len;
    u16 status_w;
    u16 title_room;

    local_irq_disable();
    memcpy(title, md400_daw_title, sizeof(title));
    memcpy(status, md400_daw_status, sizeof(status));
    accent = md400_daw_accent;
    local_irq_enable();

    md400_lcd_fill_rect(0, 0, MD400_LCD_WIDTH, MD400_LCD_TOP_BAR_H - 1, MD400_DAW_BG);
    md400_lcd_fill_rect(0, MD400_LCD_TOP_BAR_H - 1, MD400_LCD_WIDTH, 1, accent);
    md400_lcd_fill_rect(6, 6, 6, 15, accent);

    status_len = md400_lcd_text_len(status, MD400_DISPLAY_DAW_STATUS_MAX);
    status_w = md400_daw_text_width(status_len, 2);
    if (status_len) {
        md400_daw_draw_text(MD400_LCD_WIDTH - 8 - status_w, 7, status,
                            status_len, 2, accent, MD400_DAW_BG);
    }

    title_room = MD400_LCD_WIDTH - 18 - 8 - (status_len ? (status_w + 10) : 0);
    title_len = md400_lcd_text_len(title, MD400_DISPLAY_DAW_TITLE_MAX);
    while (title_len && (md400_daw_text_width(title_len, 2) > title_room)) {
        title_len--;
    }
    if (title_len) {
        md400_daw_draw_text(18, 7, title, title_len, 2, MD400_LCD_WHITE, MD400_DAW_BG);
    }
}

static void md400_daw_card_snapshot(u8 index, char *label, char *value,
                                    u16 *color, u8 *meter, u8 *flags)
{
    local_irq_disable();
    memcpy(label, md400_daw_label[index], MD400_DISPLAY_DAW_LABEL_MAX + 1);
    memcpy(value, md400_daw_value[index], MD400_DISPLAY_DAW_VALUE_MAX + 1);
    *color = md400_daw_color[index];
    *meter = md400_daw_meter[index];
    *flags = md400_daw_flags[index];
    local_irq_enable();
}

static u16 md400_daw_card_bg(u16 color, u8 flags)
{
    return (flags & MD400_DISPLAY_DAW_FLAG_LIT) ? md400_daw_dim(color) : MD400_DAW_CARD_BG;
}

static void md400_daw_draw_card_value(u8 index)
{
    char label[MD400_DISPLAY_DAW_LABEL_MAX + 1];
    char value[MD400_DISPLAY_DAW_VALUE_MAX + 1];
    u16 color;
    u16 bg;
    u16 x = (u16)index * MD400_LCD_CARD_W;
    u16 y = MD400_LCD_CARD_Y;
    u16 mx;
    u16 mw;
    u16 fill;
    u8 meter;
    u8 flags;

    md400_daw_card_snapshot(index, label, value, &color, &meter, &flags);
    bg = md400_daw_card_bg(color, flags);

    md400_daw_draw_text_centered(x + 2, MD400_LCD_CARD_W - 4, y + MD400_DAW_VALUE_Y,
                                 MD400_DAW_VALUE_H, value, 3,
                                 (flags & MD400_DISPLAY_DAW_FLAG_LIT) ? MD400_LCD_WHITE : color,
                                 bg);

    mx = x + MD400_DAW_METER_X_PAD;
    mw = MD400_LCD_CARD_W - (2 * MD400_DAW_METER_X_PAD);
    if (!(flags & MD400_DISPLAY_DAW_FLAG_METER)) {
        md400_lcd_fill_rect(mx, y + MD400_DAW_METER_Y, mw, MD400_DAW_METER_H, bg);
        return;
    }
    md400_lcd_draw_rect(mx, y + MD400_DAW_METER_Y, mw, MD400_DAW_METER_H, MD400_DAW_FRAME);
    mx += 2;
    mw -= 4;
    md400_lcd_fill_rect(mx, y + MD400_DAW_METER_Y + 2, mw, MD400_DAW_METER_H - 4, bg);
    if (meter > 127) {
        meter = 127;
    }
    if (flags & MD400_DISPLAY_DAW_FLAG_BIPOLAR) {
        u16 mid = mx + (mw / 2);

        if (meter >= 64) {
            fill = (u16)(((u32)(meter - 64) * (mw / 2)) / 63);
            md400_lcd_fill_rect(mid, y + MD400_DAW_METER_Y + 2, fill ? fill : 1,
                                MD400_DAW_METER_H - 4, color);
        } else {
            fill = (u16)(((u32)(64 - meter) * (mw / 2)) / 64);
            md400_lcd_fill_rect(mid - fill, y + MD400_DAW_METER_Y + 2, fill,
                                MD400_DAW_METER_H - 4, color);
        }
        md400_lcd_fill_rect(mid, y + MD400_DAW_METER_Y + 1, 1, MD400_DAW_METER_H - 2,
                            MD400_LCD_WHITE);
    } else {
        fill = (u16)(((u32)meter * mw) / 127);
        if (fill) {
            md400_lcd_fill_rect(mx, y + MD400_DAW_METER_Y + 2, fill,
                                MD400_DAW_METER_H - 4, color);
        }
    }
}

static void md400_daw_draw_card_full(u8 index)
{
    char label[MD400_DISPLAY_DAW_LABEL_MAX + 1];
    char value[MD400_DISPLAY_DAW_VALUE_MAX + 1];
    char number[2];
    u16 color;
    u16 bg;
    u16 x = (u16)index * MD400_LCD_CARD_W;
    u16 y = MD400_LCD_CARD_Y;
    u8 meter;
    u8 flags;

    md400_daw_card_snapshot(index, label, value, &color, &meter, &flags);
    bg = md400_daw_card_bg(color, flags);

    md400_lcd_fill_rect(x, y, MD400_LCD_CARD_W, MD400_LCD_CARD_H, MD400_DAW_BG);
    md400_lcd_fill_rect(x + 2, y + 2, MD400_LCD_CARD_W - 4, MD400_LCD_CARD_H - 4, bg);
    md400_lcd_fill_rect(x + 2, y + 2, MD400_LCD_CARD_W - 4, MD400_DAW_CARD_STRIP_H, color);
    md400_daw_draw_text_centered(x + 4, MD400_LCD_CARD_W - 8, y + MD400_DAW_LABEL_Y,
                                 MD400_DAW_LABEL_H, label, 2, MD400_LCD_WHITE, bg);
    md400_daw_draw_card_value(index);

    number[0] = (char)('1' + index);
    number[1] = 0;
    md400_daw_draw_text_centered(x + 4, MD400_LCD_CARD_W - 8, y + MD400_DAW_INDEX_Y,
                                 14, number, 2,
                                 (flags & MD400_DISPLAY_DAW_FLAG_LIT) ? color : MD400_DAW_DIM_TEXT,
                                 bg);
}

/*
 * DAW canvas. A host script draws its own screens with a handful of
 * primitives sent in NanoBridge DAW_DRAW (0x50) frames. The USB receive path
 * copies each frame's payload into this ring buffer without going through the
 * request slots, and the refresh tick drains it. If the ring is full the frame
 * is dropped and the host is told to redraw (CC 126 = 127).
 *
 * Coordinates are two 7-bit bytes (hi, lo); colours are R5 G6 B5 bytes.
 *   10 CLEAR  rgb
 *   11 RECT   x y w h rgb
 *   12 TEXT   x y scale fg bg len chars...
 *   13 ARC    cx cy r thick a0 a1 rgb      a0..a1: 0..127 of a 270 deg sweep from 135 deg
 *   14 WAVE   x ymid half fg bg n peaks... one column per peak (0..127), mirrored
 *   15 CUTOFF value   native Ableton cutoff pane (value 0..127)
 *   16 DIAL cx cy r value accent track background face pointer
 */
#define MD400_DAW_RING_SIZE      1536
#define MD400_DAW_DRAWS_PER_TICK 64  /* records per refresh tick ... */
#define MD400_DAW_DRAW_BUDGET_MS 12  /* ... or until this much time is spent */
#define MD400_DAW_OP_CLEAR       0x10
#define MD400_DAW_OP_RECT        0x11
#define MD400_DAW_OP_TEXTXY      0x12
#define MD400_DAW_OP_ARC         0x13
#define MD400_DAW_OP_WAVE        0x14
#define MD400_DAW_OP_CUTOFF      0x15
#define MD400_DAW_OP_DIAL        0x16
#define MD400_DAW_OP_WIDGET      0x17
#define MD400_DAW_OP_VALUE       0x18
#define MD400_DAW_OP_COMPACT_CUTOFF 0x19
#define MD400_DAW_WIDGET_COUNT   4
static nexfoot_ui_widget md400_daw_widgets[MD400_DAW_WIDGET_COUNT];

#if MD400_LCD_HW_SPI_ENABLE
#define MD400_CUTOFF_X           108
#define MD400_CUTOFF_Y            36
#define MD400_CUTOFF_W           204
#define MD400_CUTOFF_H           130
#define MD400_COMPACT_CUTOFF_Y    32
#define MD400_COMPACT_CUTOFF_H   104
#define MD400_CUTOFF_BG       0x2104
#define MD400_CUTOFF_GRID     0x4228
#define MD400_CUTOFF_TRACE    0x05B9
#define MD400_CUTOFF_MARK     0xF583
static u8 md400_cutoff_previous;
static u8 md400_cutoff_valid;
static u8 md400_cutoff_compact;
static u16 md400_cutoff_y[MD400_CUTOFF_W + 1];  /* Q4 pixel coordinates */
#endif

static u8 md400_daw_canvas;
static u8 md400_daw_ring[MD400_DAW_RING_SIZE];
static u16 md400_daw_ring_head;
static u16 md400_daw_ring_tail;
static u16 md400_daw_ring_used;

/* sin(deg) * 4096 for deg 0..359 */
static const s16 md400_daw_sin_q12[360] = {
    0, 71, 143, 214, 286, 357, 428, 499, 570, 641, 711, 782,
    852, 921, 991, 1060, 1129, 1198, 1266, 1334, 1401, 1468, 1534, 1600,
    1666, 1731, 1796, 1860, 1923, 1986, 2048, 2110, 2171, 2231, 2290, 2349,
    2408, 2465, 2522, 2578, 2633, 2687, 2741, 2793, 2845, 2896, 2946, 2996,
    3044, 3091, 3138, 3183, 3228, 3271, 3314, 3355, 3396, 3435, 3474, 3511,
    3547, 3582, 3617, 3650, 3681, 3712, 3742, 3770, 3798, 3824, 3849, 3873,
    3896, 3917, 3937, 3956, 3974, 3991, 4006, 4021, 4034, 4046, 4056, 4065,
    4074, 4080, 4086, 4090, 4094, 4095, 4096, 4095, 4094, 4090, 4086, 4080,
    4074, 4065, 4056, 4046, 4034, 4021, 4006, 3991, 3974, 3956, 3937, 3917,
    3896, 3873, 3849, 3824, 3798, 3770, 3742, 3712, 3681, 3650, 3617, 3582,
    3547, 3511, 3474, 3435, 3396, 3355, 3314, 3271, 3228, 3183, 3138, 3091,
    3044, 2996, 2946, 2896, 2845, 2793, 2741, 2687, 2633, 2578, 2522, 2465,
    2408, 2349, 2290, 2231, 2171, 2110, 2048, 1986, 1923, 1860, 1796, 1731,
    1666, 1600, 1534, 1468, 1401, 1334, 1266, 1198, 1129, 1060, 991, 921,
    852, 782, 711, 641, 570, 499, 428, 357, 286, 214, 143, 71,
    0, -71, -143, -214, -286, -357, -428, -499, -570, -641, -711, -782,
    -852, -921, -991, -1060, -1129, -1198, -1266, -1334, -1401, -1468, -1534, -1600,
    -1666, -1731, -1796, -1860, -1923, -1986, -2048, -2110, -2171, -2231, -2290, -2349,
    -2408, -2465, -2522, -2578, -2633, -2687, -2741, -2793, -2845, -2896, -2946, -2996,
    -3044, -3091, -3138, -3183, -3228, -3271, -3314, -3355, -3396, -3435, -3474, -3511,
    -3547, -3582, -3617, -3650, -3681, -3712, -3742, -3770, -3798, -3824, -3849, -3873,
    -3896, -3917, -3937, -3956, -3974, -3991, -4006, -4021, -4034, -4046, -4056, -4065,
    -4074, -4080, -4086, -4090, -4094, -4095, -4096, -4095, -4094, -4090, -4086, -4080,
    -4074, -4065, -4056, -4046, -4034, -4021, -4006, -3991, -3974, -3956, -3937, -3917,
    -3896, -3873, -3849, -3824, -3798, -3770, -3742, -3712, -3681, -3650, -3617, -3582,
    -3547, -3511, -3474, -3435, -3396, -3355, -3314, -3271, -3228, -3183, -3138, -3091,
    -3044, -2996, -2946, -2896, -2845, -2793, -2741, -2687, -2633, -2578, -2522, -2465,
    -2408, -2349, -2290, -2231, -2171, -2110, -2048, -1986, -1923, -1860, -1796, -1731,
    -1666, -1600, -1534, -1468, -1401, -1334, -1266, -1198, -1129, -1060, -991, -921,
    -852, -782, -711, -641, -570, -499, -428, -357, -286, -214, -143, -71,
};

static s16 md400_daw_sin(int deg)
{
    deg %= 360;
    if (deg < 0) {
        deg += 360;
    }
    return md400_daw_sin_q12[deg];
}

static void md400_daw_ring_reset_nolock(void)
{
    md400_daw_ring_head = 0;
    md400_daw_ring_tail = 0;
    md400_daw_ring_used = 0;
    md400_daw_canvas = 0;
    memset(md400_daw_widgets, 0, sizeof(md400_daw_widgets));
#if MD400_LCD_HW_SPI_ENABLE
    md400_cutoff_valid = 0;
#endif
}

u8 md400_display_daw_draw_push(const u8 *ops, u8 len)
{
    u8 i;

    if (!ops || !len) {
        return 1;
    }
    local_irq_disable();
    if (!md400_daw_view_active || md400_daw_view_waiting ||
        ((u16)len + 1 > (u16)(MD400_DAW_RING_SIZE - md400_daw_ring_used))) {
        local_irq_enable();
        return 0;
    }
    md400_daw_ring[md400_daw_ring_head] = len;
    md400_daw_ring_head = (md400_daw_ring_head + 1) % MD400_DAW_RING_SIZE;
    for (i = 0; i < len; i++) {
        md400_daw_ring[md400_daw_ring_head] = ops[i];
        md400_daw_ring_head = (md400_daw_ring_head + 1) % MD400_DAW_RING_SIZE;
    }
    md400_daw_ring_used += (u16)len + 1;
    md400_daw_canvas = 1;
    local_irq_enable();
    return 1;
}

static u8 md400_daw_ring_pop(u8 *buf)
{
    u8 len;
    u8 i;

    local_irq_disable();
    if (!md400_daw_ring_used) {
        local_irq_enable();
        return 0;
    }
    len = md400_daw_ring[md400_daw_ring_tail];
    md400_daw_ring_tail = (md400_daw_ring_tail + 1) % MD400_DAW_RING_SIZE;
    for (i = 0; i < len; i++) {
        buf[i] = md400_daw_ring[md400_daw_ring_tail];
        md400_daw_ring_tail = (md400_daw_ring_tail + 1) % MD400_DAW_RING_SIZE;
    }
    md400_daw_ring_used -= (u16)len + 1;
    local_irq_enable();
    return len;
}

static int md400_daw_u14(const u8 *p)
{
    return (int)(((u16)(p[0] & 0x7f) << 7) | (p[1] & 0x7f));
}

static u16 md400_daw_rgb(const u8 *p)
{
    return (u16)(((p[0] & 0x1f) << 11) | ((p[1] & 0x3f) << 5) | (p[2] & 0x1f));
}

static void md400_daw_clip_fill(int x, int y, int w, int h, u16 color)
{
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x + w > MD400_LCD_WIDTH) {
        w = MD400_LCD_WIDTH - x;
    }
    if (y + h > MD400_LCD_HEIGHT) {
        h = MD400_LCD_HEIGHT - y;
    }
    if ((w <= 0) || (h <= 0)) {
        return;
    }
    md400_lcd_fill_rect((u16)x, (u16)y, (u16)w, (u16)h, color);
}

static void md400_daw_canvas_arc(int cx, int cy, int r, int th, int a0, int a1, u16 color)
{
    int d0 = 135 + (a0 * 270) / 127;
    int d1 = 135 + (a1 * 270) / 127;
    int rm = r - (th / 2);
    int d;

    if ((th < 1) || (rm < 1)) {
        return;
    }
    for (d = d0; d <= d1; d++) {
        int x = cx + (((int)md400_daw_sin(d + 90) * rm) >> 12);
        int y = cy + (((int)md400_daw_sin(d) * rm) >> 12);

        md400_daw_clip_fill(x - (th / 2), y - (th / 2), th, th, color);
    }
}

u8 md400_display_daw_can_cutoff(void)
{
#if MD400_LCD_HW_SPI_ENABLE
    return md400_lcd_hw_spi_ready;
#else
    return 0;
#endif
}

#if MD400_LCD_HW_SPI_ENABLE
static u8 md400_daw_coverage(int value)
{
    return value < 0 ? 0 : (value > 16 ? 16 : (u8)value);
}

static u16 md400_daw_blend565(u16 background, u16 foreground, u8 coverage)
{
    if (!coverage) {
        return background;
    }
    if (coverage >= 16) {
        return foreground;
    }
    unsigned inv = 16 - coverage;
    unsigned r = ((((background >> 11) & 31) * inv + ((foreground >> 11) & 31) * coverage + 8) >> 4);
    unsigned g = ((((background >> 5) & 63) * inv + ((foreground >> 5) & 63) * coverage + 8) >> 4);
    unsigned b = (((background & 31) * inv + (foreground & 31) * coverage + 8) >> 4);
    return (u16)((r << 11) | (g << 5) | b);
}

static u16 md400_cutoff_sample(int x, int start, u8 compact)
{
    int d = x * 16 - start;
    int y = 64 * 16;
    if (d > 0) {
        y += d * d / (91 * 16);
    }
    if (y > 117 * 16) {
        y = 117 * 16;
    }
    return compact ? (u16)(38 * 16 + (y - 64 * 16) * 9 / 10) : (u16)y;
}

static u8 md400_cutoff_marker(int x, int center)
{
    int distance = x * 16 + 8 - center;
    if (distance < 0) {
        distance = -distance;
    }
    return md400_daw_coverage(32 - distance);
}

static void md400_cutoff_dirty(int x, int y, int w, int h)
{
    int origin_y = md400_cutoff_compact ? MD400_COMPACT_CUTOFF_Y : MD400_CUTOFF_Y;
    int height = md400_cutoff_compact ? MD400_COMPACT_CUTOFF_H : MD400_CUTOFF_H;
    if (x < MD400_CUTOFF_X + MD400_CUTOFF_W && x + w > MD400_CUTOFF_X &&
        y < origin_y + height && y + h > origin_y) {
        md400_cutoff_valid = 0;
    }
}

static void md400_daw_draw_cutoff(u8 value, u8 compact)
{
    int origin_y = compact ? MD400_COMPACT_CUTOFF_Y : MD400_CUTOFF_Y;
    int height = compact ? MD400_COMPACT_CUTOFF_H : MD400_CUTOFF_H;
    int start = 6 * 16 + (value * 133 * 16) / 127;
    int previous_start = 6 * 16 + (md400_cutoff_previous * 133 * 16) / 127;
    int marker = start + 37 * 16;
    int previous_marker = previous_start + 37 * 16;
    int left = MD400_CUTOFF_W, right = 0;
    int top = height, bottom = 0;
    int x, y;

    if (!md400_lcd_hw_spi_ready) {
        return;
    }
    if (md400_cutoff_compact != compact) {
        md400_cutoff_valid = 0;
    }
    if (md400_cutoff_valid && value == md400_cutoff_previous) {
        return;
    }
    for (x = 0; x <= MD400_CUTOFF_W; x++) {
        md400_cutoff_y[x] = md400_cutoff_sample(x, start, compact);
    }
    if (!md400_cutoff_valid) {
        left = 0;
        right = MD400_CUTOFF_W;
        top = 0;
        bottom = height;
    } else {
        for (x = 0; x < MD400_CUTOFF_W; x++) {
            int old_top = md400_cutoff_sample(x, previous_start, compact);
            int old_bottom = md400_cutoff_sample(x + 1, previous_start, compact) + 32;
            int new_top = md400_cutoff_y[x];
            int new_bottom = md400_cutoff_y[x + 1] + 32;
            u8 old_mark = md400_cutoff_marker(x, previous_marker);
            u8 new_mark = md400_cutoff_marker(x, marker);
            if (old_top == new_top && old_bottom == new_bottom && old_mark == new_mark) {
                continue;
            }
            if (x < left) {
                left = x;
            }
            right = x + 1;
            int first_row = (old_top < new_top ? old_top : new_top) >> 4;
            int last_row = ((old_bottom > new_bottom ? old_bottom : new_bottom) + 15) >> 4;
            if (first_row < top) {
                top = first_row;
            }
            if (last_row > bottom) {
                bottom = last_row;
            }
        }
    }
    for (y = top; y < bottom; y++) {
        u16 j = 0;
        u8 grid_row = compact ? (y == 18 || y == 38 || y == 70) : (y == 32 || y == 64 || y == 96);
        for (x = left; x < right; x++) {
            u16 color = (grid_row || x == 48 || x == 102 || x == 156) ? MD400_CUTOFF_GRID : MD400_CUTOFF_BG;
            int row_top = y << 4;
            int lower = md400_cutoff_y[x];
            int upper = md400_cutoff_y[x + 1] + 32;
            if (row_top < upper && row_top + 16 > lower) {
                int hit_top = row_top > lower ? row_top : lower;
                int hit_bottom = row_top + 16 < upper ? row_top + 16 : upper;
                u16 stroke = md400_daw_blend565(MD400_CUTOFF_TRACE, MD400_CUTOFF_MARK,
                                               md400_cutoff_marker(x, marker));
                color = md400_daw_blend565(color, stroke, hit_bottom - hit_top);
            }
            md400_lcd_dma_buf[j++] = color >> 8;
            md400_lcd_dma_buf[j++] = color & 0xff;
        }
        md400_lcd_dma_color_loaded = 0;
        md400_lcd_blit(MD400_CUTOFF_X + left, origin_y + y, right - left, 1, md400_lcd_dma_buf);
    }
    md400_cutoff_previous = value;
    md400_cutoff_compact = compact;
    md400_cutoff_valid = 1;
}
/* Estimate edge coverage from the signed squared-distance error. Only edge
 * pixels use the division; the center and the exterior take the fast paths. */
static u8 md400_dial_circle(int square, int radius)
{
    int limit = radius * radius;
    if (square <= limit - radius) {
        return 16;
    }
    if (square >= limit + radius) {
        return 0;
    }
    return (u8)((limit + radius - square) * 8 / radius);
}

static s16 md400_dial_sin_q4(int angle)
{
    int degree = angle >> 4;
    int part = angle & 15;
    int a = md400_daw_sin(degree);
    int b = md400_daw_sin(degree + 1);
    return a + (((b - a) * part) >> 4);
}

static void nexfoot_ui_draw_knob(int cx, int cy, int r, int value, nexfoot_ui_colors colors)
{
    int angle = 135 * 16 + (value * 270 * 16 + 63) / 127;
    int ex = md400_dial_sin_q4(angle + 90 * 16);
    int ey = md400_dial_sin_q4(angle);
    u16 accent = colors.accent;
    u16 track = colors.track;
    u16 background = colors.background;
    u16 face = colors.face;
    u16 pointer = colors.pointer;
    int width = 2 * r + 1;
    int y;

    if (!md400_lcd_hw_spi_ready || r < 18 || r > 48 ||
        cx - r < 0 || cx + r >= MD400_LCD_WIDTH ||
        cy - r < 0 || cy + r >= MD400_LCD_HEIGHT) {
        return;
    }

    int rows_per_chunk = MD400_LCD_DMA_CHUNK_PIXELS / width;
    for (y = -r; y <= r;) {
        int rows = r - y + 1;
        u16 index = 0;
        int j;
        if (rows > rows_per_chunk) {
            rows = rows_per_chunk;
        }
        for (j = 0; j < rows; j++) {
            int dy = y + j;
            int x;
            for (x = -r; x <= r; x++) {
                int square = x*x + dy*dy;
                int outer = md400_dial_circle(square, r - 1);
                int inner = 16 - md400_dial_circle(square, r - 4);
                int ring = outer < inner ? outer : inner;
                u16 color = background;

                if (face != background) {
                    color = md400_daw_blend565(color, face, md400_dial_circle(square, r - 9));
                }
                if (ring) {
                    int start = md400_daw_coverage(8 + ((-2896 * (x + dy)) >> 8));
                    int total_end = md400_daw_coverage(8 + ((2896 * (x - dy)) >> 8));
                    int total = start > total_end ? start : total_end;
                    color = md400_daw_blend565(color, track, (ring * total + 8) >> 4);
                    if (value) {
                        int end = md400_daw_coverage(8 + ((x * ey - dy * ex) >> 8));
                        int swept = value * 3 > 254
                                     ? (start > end ? start : end)
                                     : (start < end ? start : end);
                        color = md400_daw_blend565(color, accent, (ring * swept + 8) >> 4);
                    }
                } else if (square >= (r - 18) * (r - 18) && square <= (r - 9) * (r - 9)) {
                    int along = (x * ex + dy * ey) >> 8;
                    int across = (x * ey - dy * ex) >> 8;
                    if (across < 0) {
                        across = -across;
                    }
                    int side = md400_daw_coverage(32 - across);
                    int start = md400_daw_coverage(along - (r - 17) * 16 + 8);
                    int end = md400_daw_coverage((r - 10) * 16 + 8 - along);
                    int cap = start < end ? start : end;
                    color = md400_daw_blend565(color, pointer, side < cap ? side : cap);
                }
                md400_lcd_dma_buf[index++] = color >> 8;
                md400_lcd_dma_buf[index++] = color & 0xff;
            }
        }
        md400_lcd_dma_color_loaded = 0;
        md400_lcd_blit(cx - r, cy + y, width, rows, md400_lcd_dma_buf);
        y += rows;
    }
}

static int nexfoot_ui_slider_position(int length, int value)
{
    return 2 * 16 + ((length - 5) * 16 * value + 63) / 127;
}

static void nexfoot_ui_draw_slider(const nexfoot_ui_widget *widget)
{
    int vertical = widget->kind == NEXFOOT_UI_VERTICAL_SLIDER;
    int length = vertical ? widget->height : widget->width;
    int thickness = vertical ? widget->width : widget->height;
    int position = nexfoot_ui_slider_position(length, widget->value);
    int first = 0, last = length - 1;
    if (widget->valid) {
        int previous = nexfoot_ui_slider_position(length, widget->painted);
        first = ((previous < position ? previous : position) >> 4) - 2;
        last = ((previous > position ? previous : position) >> 4) + 2;
    }
    int left = vertical ? 0 : first;
    int right = vertical ? thickness - 1 : last;
    int top = vertical ? length - 1 - last : 0;
    int bottom = vertical ? length - 1 - first : thickness - 1;
    for (int col = left; col <= right;) {
        int width = right - col + 1;
        if (width > MD400_LCD_DMA_CHUNK_PIXELS) width = MD400_LCD_DMA_CHUNK_PIXELS;
        int batch = MD400_LCD_DMA_CHUNK_PIXELS / width;
        for (int row = top; row <= bottom;) {
            int rows = bottom - row + 1;
            if (rows > batch) rows = batch;
            unsigned index = 0;
            for (int dy = 0; dy < rows; dy++) {
                for (int dx = 0; dx < width; dx++) {
                    int along = vertical ? length - 1 - (row + dy) : col + dx;
                    int cross = vertical ? col + dx : row + dy;
                    int delta = along * 16 - position;
                    u16 color = widget->colors.background;
                    if (cross >= (thickness - 4) / 2 && cross < (thickness - 4) / 2 + 4) {
                        color = delta <= 0 ? widget->colors.accent : widget->colors.track;
                    }
                    int distance = delta < 0 ? -delta : delta;
                    color = md400_daw_blend565(color, widget->colors.pointer,
                                               md400_daw_coverage(32 - distance));
                    md400_lcd_dma_buf[index++] = color >> 8;
                    md400_lcd_dma_buf[index++] = color;
                }
            }
            md400_lcd_dma_color_loaded = 0;
            md400_lcd_blit(widget->x + col, widget->y + row, width, rows, md400_lcd_dma_buf);
            row += rows;
        }
        col += width;
    }
}
#endif

void nexfoot_ui_set_value(nexfoot_ui_widget *widget, int value)
{
    widget->value = value < 0 ? 0 : (value > 127 ? 127 : value);
}

void nexfoot_ui_invalidate(nexfoot_ui_widget *widget)
{
    widget->valid = 0;
}

int nexfoot_ui_paint(nexfoot_ui_widget *widget)
{
#if MD400_LCD_HW_SPI_ENABLE
    if (!md400_lcd_hw_spi_ready || widget->x + widget->width > MD400_LCD_WIDTH ||
        widget->y + widget->height > MD400_LCD_HEIGHT || widget->value > 127)
        return 0;
    if (widget->valid && widget->value == widget->painted) return 0;
    if (widget->kind == NEXFOOT_UI_KNOB) {
        int radius = widget->width / 2;
        if (widget->height != widget->width || !(widget->width & 1) || radius < 18 || radius > 48)
            return 0;
        nexfoot_ui_draw_knob(widget->x + radius, widget->y + radius, radius,
                             widget->value, widget->colors);
    } else if (widget->kind == NEXFOOT_UI_HORIZONTAL_SLIDER ||
               widget->kind == NEXFOOT_UI_VERTICAL_SLIDER) {
        if (widget->width < 6 || widget->height < 6) return 0;
        nexfoot_ui_draw_slider(widget);
    } else {
        return 0;
    }
    widget->painted = widget->value;
    widget->valid = 1;
    return 1;
#else
    return 0;
#endif
}

int nexfoot_ui_paint_readout(nexfoot_ui_readout *field, const char *text)
{
    if (!md400_lcd_ready || !text || !field->columns || field->columns > NEXFOOT_UI_READOUT_MAX ||
        !field->scale || field->x + field->columns * 6 * field->scale > MD400_LCD_WIDTH ||
        field->y + 7 * field->scale > MD400_LCD_HEIGHT) return 0;
    int changed = 0;
    for (unsigned col = 0; col < field->columns; col++) {
        char glyph = *text ? *text++ : ' ';
        if (field->valid && field->painted[col] == glyph) continue;
        int x = field->x + col * 6 * field->scale;
        md400_daw_draw_text(x, field->y, &glyph, 1, field->scale, field->ink, field->background);
        if (!field->valid)
            md400_lcd_fill_rect(x + 5 * field->scale, field->y, field->scale,
                                7 * field->scale, field->background);
        field->painted[col] = glyph;
        changed = 1;
    }
    field->valid = 1;
    return changed;
}

static nexfoot_ui_colors md400_daw_widget_colors(const u8 *p)
{
    nexfoot_ui_colors colors = {md400_daw_rgb(p), md400_daw_rgb(p + 3), md400_daw_rgb(p + 6),
                                md400_daw_rgb(p + 9), md400_daw_rgb(p + 12)};
    return colors;
}

static void md400_daw_canvas_run(const u8 *p, u8 len)
{
    u8 i = 0;

    while (i < len) {
        u8 op = p[i++];

        if (op == MD400_DAW_OP_CLEAR) {
            if ((u16)i + 3 > len) {
                return;
            }
#if MD400_LCD_HW_SPI_ENABLE
            md400_cutoff_valid = 0;
#endif
            memset(md400_daw_widgets, 0, sizeof(md400_daw_widgets));
            md400_lcd_fill_rect(0, 0, MD400_LCD_WIDTH, MD400_LCD_HEIGHT, md400_daw_rgb(&p[i]));
            i += 3;
        } else if (op == MD400_DAW_OP_RECT) {
            if ((u16)i + 11 > len) {
                return;
            }
            int x = md400_daw_u14(&p[i]);
            int y = md400_daw_u14(&p[i + 2]);
            int w = md400_daw_u14(&p[i + 4]);
            int h = md400_daw_u14(&p[i + 6]);
#if MD400_LCD_HW_SPI_ENABLE
            md400_cutoff_dirty(x, y, w, h);
#endif
            md400_daw_clip_fill(x, y, w, h, md400_daw_rgb(&p[i + 8]));
            i += 11;
        } else if (op == MD400_DAW_OP_TEXTXY) {
            int x;
            int y;
            u8 scale;
            u8 n;
            u8 fit;
            char text[48];

            if ((u16)i + 12 > len) {
                return;
            }
            x = md400_daw_u14(&p[i]);
            y = md400_daw_u14(&p[i + 2]);
            scale = p[i + 4] & 0x7f;
            n = p[i + 11] & 0x7f;
            if ((u16)i + 12 + n > len) {
                return;
            }
            if ((scale >= 1) && (scale <= 8) && (x < MD400_LCD_WIDTH) &&
                (y + (7 * scale) <= MD400_LCD_HEIGHT)) {
                fit = (u8)((MD400_LCD_WIDTH - x + scale) / (6 * scale));
                if (n > fit) {
                    n = fit;
                }
                if (n > sizeof(text)) {
                    n = sizeof(text);
                }
                memcpy(text, &p[i + 12], n);
                md400_daw_draw_text((u16)x, (u16)y, text, n, scale,
                                    md400_daw_rgb(&p[i + 5]), md400_daw_rgb(&p[i + 8]));
                /* paint the gap between letters too, so moved text leaves nothing behind */
                for (fit = 1; fit < n; fit++) {
                    md400_daw_clip_fill(x + (fit * 6 * scale) - scale, y, scale, 7 * scale,
                                        md400_daw_rgb(&p[i + 8]));
                }
            }
            i += 12 + (p[i + 11] & 0x7f);
        } else if (op == MD400_DAW_OP_ARC) {
            if ((u16)i + 11 > len) {
                return;
            }
            md400_daw_canvas_arc(md400_daw_u14(&p[i]), md400_daw_u14(&p[i + 2]),
                                 p[i + 4] & 0x7f, p[i + 5] & 0x7f,
                                 p[i + 6] & 0x7f, p[i + 7] & 0x7f,
                                 md400_daw_rgb(&p[i + 8]));
            i += 11;
        } else if (op == MD400_DAW_OP_CUTOFF || op == MD400_DAW_OP_COMPACT_CUTOFF) {
            if (i >= len) {
                return;
            }
#if MD400_LCD_HW_SPI_ENABLE
            md400_daw_draw_cutoff(p[i] & 0x7f, op == MD400_DAW_OP_COMPACT_CUTOFF);
#endif
            i++;
        } else if (op == MD400_DAW_OP_DIAL) {
            if ((u16)i + 21 > len) {
                return;
            }
#if MD400_LCD_HW_SPI_ENABLE
            nexfoot_ui_draw_knob(md400_daw_u14(p + i), md400_daw_u14(p + i + 2),
                                 p[i + 4] & 127, p[i + 5] & 127, md400_daw_widget_colors(p + i + 6));
#endif
            i += 21;
        } else if (op == MD400_DAW_OP_WIDGET) {
            if (i + 26 > len) return;
            if (p[i] < MD400_DAW_WIDGET_COUNT) {
                nexfoot_ui_widget *widget = &md400_daw_widgets[p[i]];
                widget->kind = p[i + 1];
                widget->x = md400_daw_u14(p + i + 2);
                widget->y = md400_daw_u14(p + i + 4);
                widget->width = md400_daw_u14(p + i + 6);
                widget->height = md400_daw_u14(p + i + 8);
                nexfoot_ui_set_value(widget, p[i + 10]);
                widget->colors = md400_daw_widget_colors(p + i + 11);
                nexfoot_ui_invalidate(widget);
            }
            i += 26;
        } else if (op == MD400_DAW_OP_VALUE) {
            if (i + 2 > len) return;
            if (p[i] < MD400_DAW_WIDGET_COUNT)
                nexfoot_ui_set_value(&md400_daw_widgets[p[i]], p[i + 1]);
            i += 2;
        } else if (op == MD400_DAW_OP_WAVE) {
            int x;
            int ym;
            int hh;
            u8 n;
            u8 k;
            u16 fg;
            u16 bg;

            if ((u16)i + 12 > len) {
                return;
            }
            x = md400_daw_u14(&p[i]);
            ym = md400_daw_u14(&p[i + 2]);
            hh = p[i + 4] & 0x7f;
            fg = md400_daw_rgb(&p[i + 5]);
            bg = md400_daw_rgb(&p[i + 8]);
            n = p[i + 11] & 0x7f;
            if ((u16)i + 12 + n > len) {
                return;
            }
            for (k = 0; k < n; k++) {
                int h = ((p[i + 12 + k] & 0x7f) * hh) / 127;

                if (h < 1) {
                    h = 1;
                }
                if (h > hh) {
                    h = hh;
                }
                md400_daw_clip_fill(x + k, ym - hh, 1, hh - h, bg);
                md400_daw_clip_fill(x + k, ym - h, 1, 2 * h, fg);
                md400_daw_clip_fill(x + k, ym + h, 1, hh - h, bg);
            }
            i += 12 + n;
        } else {
            return;
        }
    }
}

static void md400_daw_canvas_drain(void)
{
    u8 buf[64];
    u8 n;
    u8 count;
    u32 t0 = sys_timer_get_ms();

    for (count = 0; count < MD400_DAW_DRAWS_PER_TICK; count++) {
        if ((u32)(sys_timer_get_ms() - t0) >= MD400_DAW_DRAW_BUDGET_MS) {
            break;
        }
        n = md400_daw_ring_pop(buf);
        if (!n) {
            break;
        }
        md400_daw_canvas_run(buf, n);
    }
    static u8 next_slot;
    for (u8 checked = 0; checked < MD400_DAW_WIDGET_COUNT; checked++) {
        if ((u32)(sys_timer_get_ms() - t0) >= MD400_DAW_DRAW_BUDGET_MS) break;
        nexfoot_ui_paint(&md400_daw_widgets[next_slot]);
        next_slot = (next_slot + 1) % MD400_DAW_WIDGET_COUNT;
    }
    if (count && md400_lcd_first_frame_pending) {
        md400_lcd_first_frame_pending = 0;
        md400_lcd_pin_output(MD400_LCD_LIGHT, MD400_LCD_LIGHT_ON);
    }
}

/* Draw pending DAW regions. Called from the refresh tick with drawing held. */
static void md400_daw_tick(void)
{
    u8 drained;
    u8 index;
    u8 screen;
    u8 top;

    local_irq_disable();
    u8 waiting = md400_daw_view_waiting;
    u8 draw_waiting = md400_daw_waiting_dirty;
    md400_daw_waiting_dirty = 0;
    local_irq_enable();

    if (waiting) {
        if (draw_waiting) {
            const u16 body = 0xad75;
            const u16 ink = 0x2965;
            md400_lcd_fill_rect(0, 0, MD400_LCD_WIDTH, MD400_LCD_HEIGHT, body);
            md400_lcd_fill_rect(0, 0, MD400_LCD_WIDTH, 26, 0xbdf7);
            md400_daw_draw_text(12, 6, daw_app_name, 12, 2, ink, 0xbdf7);
            md400_daw_draw_text_centered(0, MD400_LCD_WIDTH, 56, 42, "WAITING FOR DAW", 3, ink, body);
            md400_daw_draw_text_centered(0, MD400_LCD_WIDTH, 145, 21, "DOUBLE EDIT TO EXIT", 2, ink, body);
        }
        return;
    }

    if (md400_daw_canvas) {
        md400_daw_canvas_drain();
        return;
    }
    for (drained = 0; drained < MD400_DISPLAY_MAX_DRAWS_PER_TICK; drained++) {
        u8 kind = 0;

        index = 0xff;
        local_irq_disable();
        screen = md400_daw_screen_dirty;
        top = md400_daw_top_dirty;
        if (screen) {
            md400_daw_screen_dirty = 0;
            md400_daw_top_dirty = 0;
            md400_daw_card_full_dirty = 0x0f;
            md400_daw_card_value_dirty = 0;
            kind = 1;
        } else if (top) {
            md400_daw_top_dirty = 0;
            kind = 2;
        } else if (md400_daw_card_full_dirty) {
            index = md400_display_first_dirty(md400_daw_card_full_dirty);
            md400_daw_card_full_dirty &= ~BIT(index);
            md400_daw_card_value_dirty &= ~BIT(index);
            kind = 3;
        } else if (md400_daw_card_value_dirty) {
            index = md400_display_first_dirty(md400_daw_card_value_dirty);
            md400_daw_card_value_dirty &= ~BIT(index);
            kind = 4;
        }
        local_irq_enable();

        if (!kind) {
            break;
        }
        if (kind == 1) {
            md400_daw_draw_top_bar();
            if (md400_lcd_first_frame_pending) {
                md400_lcd_first_frame_pending = 0;
                md400_lcd_pin_output(MD400_LCD_LIGHT, MD400_LCD_LIGHT_ON);
            }
        } else if (kind == 2) {
            md400_daw_draw_top_bar();
        } else if (kind == 3) {
            md400_daw_draw_card_full(index);
        } else {
            md400_daw_draw_card_value(index);
        }
    }
}

void md400_display_daw_reset(void)
{
    u8 i;

    local_irq_disable();
    memset(md400_daw_title, 0, sizeof(md400_daw_title));
    memset(md400_daw_status, 0, sizeof(md400_daw_status));
    memcpy(md400_daw_title, "DAW", 3);
    md400_daw_ring_reset_nolock();
    md400_daw_accent = MD400_LCD_CYAN;
    for (i = 0; i < 4; i++) {
        memset(md400_daw_label[i], 0, sizeof(md400_daw_label[i]));
        memset(md400_daw_value[i], 0, sizeof(md400_daw_value[i]));
        md400_daw_color[i] = MD400_LCD_GRAY;
        md400_daw_meter[i] = 0;
        md400_daw_flags[i] = 0;
    }
    md400_daw_screen_dirty = 1;
    local_irq_enable();
}

void md400_display_daw_set_active(u8 active)
{
    active = !!active;
    local_irq_disable();
    if (md400_daw_view_active == active) {
        local_irq_enable();
        return;
    }
    md400_daw_view_active = active;
    md400_daw_view_waiting = 0;
    md400_daw_waiting_dirty = 0;
    md400_daw_ring_reset_nolock();
    if (active) {
        md400_daw_screen_dirty = 1;
    } else {
        /* Repaint the normal UI from scratch; its caches no longer match GRAM. */
        md400_top_bar_label_valid = 0;
        md400_top_bar_battery_valid = 0;
        md400_top_bar_save_check_valid = 0;
        md400_top_bar_bt_valid = 0;
        memset(md400_value_cache_valid, 0, sizeof(md400_value_cache_valid));
        md400_play_line_cache_valid = 0;
        md400_play_label_cache_valid = 0;
        md400_display_mark_full_screen_nolock();
        if (md400_ui_mode == MD400_LCD_MODE_PLAY) {
            md400_play_full_dirty = 1;
        }
    }
    local_irq_enable();
}

void md400_display_daw_set_waiting(u8 waiting)
{
    local_irq_disable();
    if (md400_daw_view_waiting != waiting) {
        md400_daw_view_waiting = waiting;
        md400_daw_waiting_dirty = waiting;
        md400_daw_ring_reset_nolock();
        md400_daw_screen_dirty = 1;
    }
    local_irq_enable();
}

u8 md400_display_daw_is_active(void)
{
    return md400_daw_view_active;
}

/* Copy printable ASCII into dst (max chars), returning 1 if it changed. */
static u8 md400_daw_store_text(char *dst, u8 max, const u8 *text, u8 len)
{
    char tmp[MD400_DISPLAY_DAW_TITLE_MAX + 1];
    u8 i;

    if (len > max) {
        len = max;
    }
    memset(tmp, 0, sizeof(tmp));
    for (i = 0; i < len; i++) {
        tmp[i] = ((text[i] >= 0x20) && (text[i] <= 0x7e)) ? (char)text[i] : ' ';
    }
    if (!memcmp(dst, tmp, max + 1)) {
        return 0;
    }
    memcpy(dst, tmp, max + 1);
    return 1;
}

u8 md400_display_daw_set_text(u8 field, const u8 *text, u8 len)
{
    u8 changed = 0;

    local_irq_disable();
    if (field == MD400_DISPLAY_DAW_FIELD_TITLE) {
        if (md400_daw_store_text(md400_daw_title, MD400_DISPLAY_DAW_TITLE_MAX, text, len)) {
            md400_daw_top_dirty = 1;
        }
        changed = 1;
    } else if (field == MD400_DISPLAY_DAW_FIELD_STATUS) {
        if (md400_daw_store_text(md400_daw_status, MD400_DISPLAY_DAW_STATUS_MAX, text, len)) {
            md400_daw_top_dirty = 1;
        }
        changed = 1;
    } else if ((field >= MD400_DISPLAY_DAW_FIELD_LABEL1) &&
               (field < MD400_DISPLAY_DAW_FIELD_LABEL1 + 4)) {
        u8 index = field - MD400_DISPLAY_DAW_FIELD_LABEL1;

        if (md400_daw_store_text(md400_daw_label[index], MD400_DISPLAY_DAW_LABEL_MAX, text, len)) {
            md400_daw_card_full_dirty |= BIT(index);
        }
        changed = 1;
    } else if ((field >= MD400_DISPLAY_DAW_FIELD_VALUE1) &&
               (field < MD400_DISPLAY_DAW_FIELD_VALUE1 + 4)) {
        u8 index = field - MD400_DISPLAY_DAW_FIELD_VALUE1;

        if (md400_daw_store_text(md400_daw_value[index], MD400_DISPLAY_DAW_VALUE_MAX, text, len)) {
            md400_daw_card_value_dirty |= BIT(index);
        }
        changed = 1;
    }
    local_irq_enable();
    return changed;
}

u8 md400_display_daw_set_card(u8 card, u8 meter, u16 color, u8 flags)
{
    u8 index;

    if ((card < 1) || (card > 4)) {
        return 0;
    }
    index = card - 1;
    local_irq_disable();
    if ((md400_daw_color[index] != color) ||
        ((md400_daw_flags[index] ^ flags) & MD400_DISPLAY_DAW_FLAG_LIT)) {
        md400_daw_card_full_dirty |= BIT(index);
    } else if ((md400_daw_meter[index] != meter) || (md400_daw_flags[index] != flags)) {
        md400_daw_card_value_dirty |= BIT(index);
    }
    md400_daw_color[index] = color;
    md400_daw_meter[index] = meter;
    md400_daw_flags[index] = flags;
    local_irq_enable();
    return 1;
}

void md400_display_daw_set_accent(u16 color)
{
    local_irq_disable();
    if (md400_daw_accent != color) {
        md400_daw_accent = color;
        md400_daw_top_dirty = 1;
    }
    local_irq_enable();
}

static void md400_display_refresh_tick(void *priv)
{
    u8 i;
    u8 drained;

    (void)priv;

    if (!md400_lcd_ready || md400_display_drawing) {
        return;
    }

    md400_display_drawing = 1;

    if (md400_display_shutdown_animation_tick()) {
        md400_display_drawing = 0;
        return;
    }

    if (md400_daw_view_active && !md400_boot_anim_active) {
        md400_daw_tick();
        md400_display_drawing = 0;
        return;
    }

    if (md400_display_boot_animation_tick()) {
        md400_display_drawing = 0;
        return;
    }

#if CONFIG_MD400_UART0_RX_TEST_ENABLE
    if (md400_pixel_capture_requested) {
        local_irq_disable();
        md400_pixel_capture_requested = 0;
        md400_display_mark_full_screen_nolock();
        local_irq_enable();
        md400_pixel_capture_begin();
    }
#endif

    /* Drain every pending dirty region in this tick so a multi-region update
     * (e.g. a full-screen repaint) paints in one pass instead of one region per
     * 30ms tick. */
    for (drained = 0; drained < MD400_DISPLAY_MAX_DRAWS_PER_TICK; drained++) {
    u8 draw_kind = 0;
    u8 draw_index = 0xff;
    u8 show_first_frame = 0;

    local_irq_disable();
    md400_display_save_check_expire_nolock();
    if (md400_screen_clear_dirty) {
        md400_screen_clear_dirty = 0;
        md400_card_label_dirty = 0;
        md400_card_value_dirty = 0;
        md400_card_press_dirty = 0;
        md400_top_bar_dirty = 0;
        if (md400_ui_mode == MD400_LCD_MODE_PLAY) {
            md400_card_full_dirty = 0;
            md400_play_full_dirty = 0;
            md400_play_prefix_dirty = 0;
            md400_play_value_dirty = 0;
            md400_play_label_dirty = 0;
        } else if (md400_lcd_first_frame_pending) {
            md400_card_full_dirty = 0;
            md400_card_label_dirty = 0;
            md400_card_value_dirty = 0;
            md400_card_press_dirty = 0;
            draw_kind = 11;
        }
        if (!draw_kind) {
            draw_kind = 1;
        }
    } else if (md400_top_bar_dirty) {
        md400_top_bar_dirty = 0;
        draw_kind = 6;
    } else if ((md400_ui_mode == MD400_LCD_MODE_PLAY) && md400_play_full_dirty) {
        md400_play_full_dirty = 0;
        md400_play_prefix_dirty = 0;
        md400_play_value_dirty = 0;
        md400_play_label_dirty = 0;
        draw_kind = 7;
    } else if ((md400_ui_mode == MD400_LCD_MODE_PLAY) && md400_play_prefix_dirty) {
        md400_play_prefix_dirty = 0;
        md400_play_value_dirty = 0;
        draw_kind = 8;
    } else if ((md400_ui_mode == MD400_LCD_MODE_PLAY) && md400_play_value_dirty) {
        md400_play_value_dirty = 0;
        draw_kind = 9;
    } else if ((md400_ui_mode == MD400_LCD_MODE_PLAY) && md400_play_label_dirty) {
        md400_play_label_dirty = 0;
        draw_kind = 10;
    } else if (md400_card_full_dirty) {
        draw_index = md400_display_first_dirty(md400_card_full_dirty);
        md400_card_full_dirty &= ~BIT(draw_index);
        md400_card_label_dirty &= ~BIT(draw_index);
        md400_card_value_dirty &= ~BIT(draw_index);
        md400_card_press_dirty &= ~BIT(draw_index);
        draw_kind = 2;
    } else if (md400_card_label_dirty) {
        draw_index = md400_display_first_dirty(md400_card_label_dirty);
        md400_card_label_dirty &= ~BIT(draw_index);
        draw_kind = 5;
    } else if (md400_card_value_dirty) {
        draw_index = md400_display_first_dirty(md400_card_value_dirty);
        md400_card_value_dirty &= ~BIT(draw_index);
        draw_kind = 3;
    } else if (md400_card_press_dirty) {
        draw_index = md400_display_first_dirty(md400_card_press_dirty);
        md400_card_press_dirty &= ~BIT(draw_index);
        draw_kind = 4;
    }
    local_irq_enable();

    if (!draw_kind) {
        break;
    }

    if ((draw_kind == 1) || (draw_kind == 11)) {
        md400_diag_draw_screen++;
        /* Only blank the whole screen on the very first frame (GRAM is garbage).
         * For page/mode changes the top bar + cards/play-full fully tile the
         * screen and overwrite the old content in place, so a full clear just
         * adds a ~37ms black flash. */
        if (draw_kind == 11) {
            md400_display_clear_screen();
        }
        md400_display_draw_top_bar(1);
        if (md400_ui_mode == MD400_LCD_MODE_PLAY) {
            md400_display_draw_play_full();
        } else if (draw_kind == 11) {
            for (i = 0; i < 4; i++) {
                md400_display_draw_card_full(i);
            }
        }
    } else if (draw_kind == 6) {
        md400_diag_draw_top++;
        md400_display_draw_top_bar(0);
    } else if (draw_kind == 7) {
        md400_display_draw_play_full();
    } else if (draw_kind == 8) {
        md400_display_draw_play_line(0);
    } else if (draw_kind == 9) {
        md400_display_draw_play_value_only();
    } else if (draw_kind == 10) {
        md400_display_draw_play_label(0);
    } else if (draw_kind == 2) {
        md400_display_draw_card_full(draw_index);
    } else if (draw_kind == 5) {
        md400_display_draw_card_label(draw_index);
    } else if (draw_kind == 3) {
        md400_display_draw_card_value(draw_index, 0);
    } else {
        md400_display_draw_card_press(draw_index);
    }
    if (md400_lcd_first_frame_pending &&
        ((draw_kind == 1) || (draw_kind == 11))) {
        local_irq_disable();
        if (md400_lcd_first_frame_pending) {
            md400_lcd_first_frame_pending = 0;
            show_first_frame = 1;
        }
        local_irq_enable();
        if (show_first_frame) {
            md400_lcd_pin_output(MD400_LCD_LIGHT, MD400_LCD_LIGHT_ON);
        }
    }
    }
#if CONFIG_MD400_UART0_RX_TEST_ENABLE
    if (md400_pixel_capture_active) {
        md400_pixel_capture_end();
    }
#endif
    md400_display_drawing = 0;
}

u8 md400_display_pixel_capture_request(void)
{
#if CONFIG_MD400_UART0_RX_TEST_ENABLE
    if (!md400_lcd_ready || md400_pixel_capture_requested ||
        md400_pixel_capture_active) {
        return 0;
    }
    md400_pixel_capture_requested = 1;
    return 1;
#else
    return 0;
#endif
}

void md400_display_encoder_update(u8 encoder, int value, u8 pressed)
{
    u8 index;
    u8 mask;
    u8 value_changed;
    u8 press_changed;

    if ((encoder < 1) || (encoder > 4)) {
        return;
    }

    index = encoder - 1;
    mask = BIT(index);

    local_irq_disable();
    value_changed = (md400_encoder_value[index] != value);
    press_changed = (md400_encoder_pressed[index] != !!pressed);
    md400_encoder_value[index] = value;
    md400_encoder_pressed[index] = !!pressed;
    if (!md400_ui_mode) {
        if (md400_card_full_dirty & mask) {
            md400_card_value_dirty &= ~mask;
            md400_card_press_dirty &= ~mask;
        } else {
            if (value_changed) {
                md400_card_value_dirty |= mask;
            }
            if (press_changed) {
                md400_card_press_dirty |= mask;
            }
        }
    }
    local_irq_enable();
}

static u8 md400_display_role_value_label_changed(u8 role, int old_value,
                                                 int new_value)
{
    if (role == MD400_DISPLAY_ROLE_EDIT_ACTION) {
        return (old_value / 10) != (new_value / 10);
    }
    if ((role == MD400_DISPLAY_ROLE_EDIT_MESSAGE) ||
        (role == MD400_DISPLAY_ROLE_EDIT_DETAIL_VALUE) ||
        (role == MD400_DISPLAY_ROLE_EDIT_DETAIL_RELEASE)) {
        return ((old_value >> 8) & 0x7f) != ((new_value >> 8) & 0x7f);
    }
    return 0;
}

void md400_display_ui_update_roles(u8 mode, int row0, int row1, int row2, int row3,
                                   u8 role0, u8 role1, u8 role2, u8 role3)
{
    u8 label_dirty = 0;
    u8 value_dirty = 0;
    u8 role[4];
    u8 i;
    u8 mode_changed = 0;
    u8 top_bar_changed = 0;
    u8 old_mode;
    u8 old_role3;
    int old_row3;

    if (mode > 2) {
        mode = 0;
    }
    role[0] = role0;
    role[1] = role1;
    role[2] = role2;
    role[3] = role3;

    local_irq_disable();
    old_mode = md400_ui_mode;
    old_role3 = md400_ui_row_role[3];
    old_row3 = md400_ui_row_value[3];
    if (md400_ui_mode != mode) {
        md400_ui_mode = mode;
        mode_changed = 1;
    }
    if (md400_ui_row_value[0] != row0) {
        if (md400_display_role_value_label_changed(role[0],
                                                   md400_ui_row_value[0],
                                                   row0)) {
            label_dirty |= BIT(0);
        } else {
            value_dirty |= BIT(0);
        }
        md400_ui_row_value[0] = row0;
    }
    if (md400_ui_row_value[1] != row1) {
        if (md400_display_role_value_label_changed(role[1],
                                                   md400_ui_row_value[1],
                                                   row1)) {
            label_dirty |= BIT(1);
        } else {
            value_dirty |= BIT(1);
        }
        md400_ui_row_value[1] = row1;
    }
    if (md400_ui_row_value[2] != row2) {
        if (md400_display_role_value_label_changed(role[2],
                                                   md400_ui_row_value[2],
                                                   row2)) {
            label_dirty |= BIT(2);
        } else {
            value_dirty |= BIT(2);
        }
        md400_ui_row_value[2] = row2;
    }
    if (md400_ui_row_value[3] != row3) {
        if (md400_display_role_value_label_changed(role[3],
                                                   md400_ui_row_value[3],
                                                   row3)) {
            label_dirty |= BIT(3);
        } else {
            value_dirty |= BIT(3);
        }
        md400_ui_row_value[3] = row3;
    }
    for (i = 0; i < 4; i++) {
        if (md400_ui_row_role[i] != role[i]) {
            md400_ui_row_role[i] = role[i];
            label_dirty |= BIT(i);
            value_dirty |= BIT(i);
        }
    }

    if (old_mode != md400_ui_mode) {
        top_bar_changed = 1;
    } else if ((md400_ui_mode == 1) &&
               ((old_role3 == MD400_DISPLAY_ROLE_EDIT_PAGE) ||
                (md400_ui_row_role[3] == MD400_DISPLAY_ROLE_EDIT_PAGE)) &&
               ((old_role3 != md400_ui_row_role[3]) ||
                (old_row3 != md400_ui_row_value[3]))) {
        top_bar_changed = 1;
    }
    if (top_bar_changed) {
        md400_top_bar_dirty = 1;
    }
    if (mode_changed) {
        md400_display_mark_full_screen_nolock();
    } else if (md400_ui_mode == MD400_LCD_MODE_PLAY) {
        md400_card_full_dirty = 0;
        md400_card_label_dirty = 0;
        md400_card_value_dirty = 0;
        md400_card_press_dirty = 0;
    } else {
        for (i = 0; i < 4; i++) {
            if (label_dirty & BIT(i)) {
                md400_display_mark_card_label_nolock(i);
            }
        }
        if (md400_ui_mode && value_dirty) {
            md400_card_value_dirty |= value_dirty & ~md400_card_full_dirty;
        }
    }
    local_irq_enable();
}

void md400_display_ui_update(u8 mode, int row0, int row1, int row2, int row3)
{
    md400_display_ui_update_roles(mode, row0, row1, row2, row3,
                                  MD400_DISPLAY_ROLE_ENCODER1,
                                  MD400_DISPLAY_ROLE_ENCODER2,
                                  MD400_DISPLAY_ROLE_ENCODER3,
                                  MD400_DISPLAY_ROLE_ENCODER4);
}

void md400_display_play_update(int msg, u8 value, const char *label)
{
    char prefix[MD400_LCD_PLAY_PREFIX_MAX + 1];
    char old_prefix[MD400_LCD_PLAY_PREFIX_MAX + 1];
    u8 i;
    u8 prefix_changed;
    u8 label_changed;
    u8 value_changed;

    md400_display_format_play_prefix(msg, prefix, sizeof(prefix));

    local_irq_disable();
    md400_display_format_play_prefix(md400_play_msg, old_prefix,
                                     sizeof(old_prefix));
    prefix_changed = !md400_display_text_same(prefix, old_prefix,
                                              sizeof(prefix));
    label_changed = 0;
    for (i = 0; i < MD400_DISPLAY_PLAY_LABEL_LEN; i++) {
        char ch = (label && label[i]) ? label[i] : 0;
        if (md400_play_label[i] != ch) {
            label_changed = 1;
        }
        md400_play_label[i] = ch;
        if (!ch) {
            i++;
            while (i < MD400_DISPLAY_PLAY_LABEL_LEN) {
                if (md400_play_label[i]) {
                    label_changed = 1;
                }
                md400_play_label[i++] = 0;
            }
            break;
        }
    }
    md400_play_label[MD400_DISPLAY_PLAY_LABEL_LEN] = 0;
    value &= 0x7f;
    value_changed = md400_play_value != value;
    md400_play_msg = msg;
    md400_play_value = value;

    md400_ui_row_value[0] = msg;
    md400_ui_row_value[1] = value;
    md400_ui_row_value[2] = 0;
    md400_ui_row_value[3] = 0;
    md400_ui_row_role[0] = MD400_DISPLAY_ROLE_PLAY_MESSAGE;
    md400_ui_row_role[1] = MD400_DISPLAY_ROLE_PLAY_VALUE;
    md400_ui_row_role[2] = MD400_DISPLAY_ROLE_PLAY_LABEL;
    md400_ui_row_role[3] = MD400_DISPLAY_ROLE_NONE;

    if (md400_ui_mode == MD400_LCD_MODE_PLAY) {
        if (prefix_changed) {
            md400_play_prefix_dirty = 1;
        } else if (value_changed) {
            md400_play_value_dirty = 1;
        }
        if (label_changed) {
            md400_play_label_dirty = 1;
        }
    }
    local_irq_enable();
}

static void md400_display_ms32_encoder_event(u8 encoder, u8 event, int value)
{
    (void)event;
    md400_display_encoder_update(encoder, value, md400_ms32_get_encoder_pressed(encoder));
}

#if MD400_LCD_LIGHT_PROBE_ENABLE
static void md400_lcd_light_probe_tick(void *priv)
{
    static u32 count;

    (void)priv;
    md400_lcd_light_probe_level ^= 1;
    md400_lcd_pin_output(MD400_LCD_LIGHT, md400_lcd_light_probe_level);
    if (count < 24) {
        printf("[MD400_LCD] PB9 light probe raw=%u\r\n",
               (unsigned)md400_lcd_light_probe_level);
    } else if (count == 24) {
        printf("[MD400_LCD] PB9 light probe log muted\r\n");
    }
    count++;
}
#endif

void md400_display_test_init(void)
{
    if (md400_lcd_ready) {
        return;
    }
    md400_lcd_ready = 1;

    printf("[MD400_LCD] init ST7789 Mosaic pins PB5-PB9 madctl=0x%02x inv=0x%02x\r\n",
           MD400_LCD_MADCTL_LANDSCAPE, MD400_LCD_INVERSION_CMD);
    md400_lcd_bus_idle();
    md400_lcd_power_cycle();
#if MD400_LCD_HW_SPI_ENABLE
    md400_lcd_hw_spi_init();
#endif
    md400_lcd_init_sequence();
    local_irq_disable();
    md400_lcd_first_frame_pending = 0;
    md400_boot_anim_active = 1;
    md400_boot_anim_last_step = 0xff;
    md400_boot_anim_start_ms = sys_timer_get_ms();
    md400_shutdown_anim_active = 0;
    local_irq_enable();
    printf("[MD400_LCD] boot animation start %ums\r\n",
           (unsigned)MD400_BOOT_ANIM_DURATION_MS);
    md400_ms32_set_encoder_event_cb(md400_display_ms32_encoder_event);
    if (!md400_display_timer) {
        md400_display_timer = sys_timer_add(NULL, md400_display_refresh_tick, 30);
    }
    md400_display_battery_tick(NULL);
    if (!md400_battery_timer) {
        md400_battery_timer =
            sys_timer_add(NULL, md400_display_battery_tick, MD400_BATTERY_TICK_MS);
    }
#if MD400_LCD_LIGHT_PROBE_ENABLE
    md400_lcd_light_probe_level = MD400_LCD_LIGHT_ON;
    if (!md400_lcd_light_probe_timer) {
        md400_lcd_light_probe_timer = sys_timer_add(NULL, md400_lcd_light_probe_tick, 500);
        printf("[MD400_LCD] PB9 light probe enabled, watch for flicker\r\n");
    }
#endif
    printf("[MD400_LCD] encoder monitor ready %ux%u layout=%u\r\n",
           (unsigned)MD400_LCD_WIDTH, (unsigned)MD400_LCD_HEIGHT,
           (unsigned)MD400_DISPLAY_LAYOUT_VERSION);
}
