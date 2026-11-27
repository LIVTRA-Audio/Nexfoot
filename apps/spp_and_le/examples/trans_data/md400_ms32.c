/* Modified for Nexfoot; upstream notices are retained. */
#include "system/includes.h"
#include "app_config.h"
#include "asm/gpio.h"

#include "md400_ms32.h"

#ifndef CONFIG_MD400_MS32_ENCODER_ENABLE
#define CONFIG_MD400_MS32_ENCODER_ENABLE 1
#endif
#ifndef CONFIG_MD400_MS32_ADC_SERIAL_MONITOR_ENABLE
#define CONFIG_MD400_MS32_ADC_SERIAL_MONITOR_ENABLE 0
#endif

#if CONFIG_MD400_MS32_ENCODER_ENABLE

#define MD400_MS32_HEAD0          0xA5
#define MD400_MS32_HEAD1          0x5A
#define MD400_MS32_VER            0x01
#define MD400_MS32_TYPE_ENCODER   0x10
#define MD400_MS32_TYPE_ADC       0x20
#define MD400_MS32_TYPE_PEDAL_MODE_ACK 0x31
#define MD400_MS32_TYPE_UART_STATUS 0x32
#define MD400_MS32_PAYLOAD_MAX    16
#define MD400_MS32_ACCEL_IDLE_MS  180
#define MD400_MS32_ACCEL_FAST_MS  70
#define MD400_MS32_ACCEL_FASTER_MS 35
#define MD400_MS32_ACCEL_FASTEST_MS 18
#define MD400_MS32_ACCEL_STEP_MAX 16
#define MD400_MS32_REVERSE_ROTATION 1

enum {
    MD400_MS32_WAIT_HEAD0 = 0,
    MD400_MS32_WAIT_HEAD1,
    MD400_MS32_READ_VER,
    MD400_MS32_READ_TYPE,
    MD400_MS32_READ_SEQ,
    MD400_MS32_READ_LEN,
    MD400_MS32_READ_PAYLOAD,
    MD400_MS32_READ_CHK,
};

struct md400_ms32_parser {
    u8 state;
    u8 ver;
    u8 type;
    u8 seq;
    u8 len;
    u8 pos;
    u8 chk;
    u8 payload[MD400_MS32_PAYLOAD_MAX];
};

static struct md400_ms32_parser md400_ms32_parser;
static int md400_ms32_encoder_value[4];
static u8 md400_ms32_encoder_pressed[4];
static u32 md400_ms32_encoder_last_ms[4];
static u8 md400_ms32_encoder_last_event[4];
static u8 md400_ms32_led_on[4];
static u8 md400_ms32_adc_channel;
static u8 md400_ms32_adc_value;
static u8 md400_ms32_adc_min = 0xff;
static u8 md400_ms32_adc_max;
static u8 md400_ms32_ready;
static u8 md400_led_ready;
static u32 md400_ms32_frame_count;
static u32 md400_ms32_encoder_frame_count;
static u32 md400_ms32_adc_frame_count;
static u32 md400_ms32_unknown_frame_count;
static u32 md400_ms32_adc_log_count;
static u32 md400_ms32_event_log_count;
static u32 md400_ms32_error_log_count;
static u32 md400_ms32_pedal_mode_ack_count;
static u32 md400_ms32_pedal_mode_ack_log_count;
static u32 md400_ms32_uart_status_count;
static u32 md400_ms32_uart_status_log_count;
static u8 md400_ms32_last_type;
static u8 md400_ms32_last_seq;
static u8 md400_ms32_last_len;
static u8 md400_ms32_pedal_mode_ack = 0x7f;
static u8 md400_ms32_pedal_mode_ack_seq;
static u8 md400_ms32_uart_status_mode = 0x7f;
static u16 md400_ms32_uart_status_rx_bytes;
static u8 md400_ms32_uart_status_frames;
static u8 md400_ms32_uart_status_mode_cmds;
static u8 md400_ms32_uart_status_checksum_errors;
static u8 md400_ms32_uart_status_version_errors;
static u8 md400_ms32_uart_status_length_errors;
static u8 md400_ms32_uart_status_overflows;
static u8 md400_ms32_uart_status_last_type;
static u8 md400_ms32_uart_status_last_seq;
static u8 md400_ms32_uart_status_last_len;
static u8 md400_ms32_uart_status_last_byte;
static u8 md400_ms32_last_encoder_raw_id;
static u8 md400_ms32_last_encoder_id;
static u8 md400_ms32_last_encoder_event;
static int md400_ms32_last_encoder_delta;
#if CONFIG_MD400_MS32_ADC_SERIAL_MONITOR_ENABLE
static u16 md400_ms32_adc_monitor_timer;
#endif
static md400_ms32_encoder_event_cb_t md400_ms32_encoder_cb;
static md400_ms32_adc_event_cb_t md400_ms32_adc_cb;

extern void md400_midi_send_encoder_cc(u8 encoder, u8 value, u8 event);
extern void md400_midi_send_encoder_press(u8 encoder, u8 pressed, u8 value);

static const u32 md400_ms32_led_pins[4] = {
    IO_PORTA_04,
    IO_PORTA_05,
    IO_PORTA_09,
    IO_PORTA_03,
};

static const u8 md400_ms32_encoder_map[4] = {
    2, 1, 4, 3,
};

static void md400_ms32_parser_reset(void)
{
    md400_ms32_parser.state = MD400_MS32_WAIT_HEAD0;
    md400_ms32_parser.ver = 0;
    md400_ms32_parser.type = 0;
    md400_ms32_parser.seq = 0;
    md400_ms32_parser.len = 0;
    md400_ms32_parser.pos = 0;
    md400_ms32_parser.chk = 0;
}

static void md400_ms32_log_event(const char *text, u8 id, int value)
{
    if (md400_ms32_event_log_count < 80) {
        printf("[MD400_MS32] enc%u %s value=%d\r\n", (unsigned)id, text, value);
    } else if (md400_ms32_event_log_count == 80) {
        printf("[MD400_MS32] event log muted\r\n");
    }
    md400_ms32_event_log_count++;
}

static void md400_ms32_log_error(const char *text, u8 a, u8 b)
{
    if (md400_ms32_error_log_count < 20) {
        printf("[MD400_MS32] %s a=0x%02x b=0x%02x\r\n", text, a, b);
    } else if (md400_ms32_error_log_count == 20) {
        printf("[MD400_MS32] error log muted\r\n");
    }
    md400_ms32_error_log_count++;
}

static void md400_ms32_set_led(u8 index, u8 on)
{
    u32 pin;

    if (index >= 4) {
        return;
    }

    pin = md400_ms32_led_pins[index];
    md400_ms32_led_on[index] = !!on;
    gpio_direction_output(pin, md400_ms32_led_on[index] ? 0 : 1);
}

void md400_led_init(void)
{
    u8 i;

    if (md400_led_ready) {
        return;
    }

    for (i = 0; i < 4; i++) {
        gpio_set_die(md400_ms32_led_pins[i], 1);
        gpio_set_pull_up(md400_ms32_led_pins[i], 0);
        gpio_set_pull_down(md400_ms32_led_pins[i], 0);
        md400_ms32_set_led(i, 0);
    }
    md400_led_ready = 1;
    printf("[MD400_LED] init PA4 PA5 PA9 PA3 active-low\r\n");
}

void md400_led_set_mask(u8 mask)
{
    u8 i;

    md400_led_init();
    for (i = 0; i < 4; i++) {
        md400_ms32_set_led(i, (mask & BIT(i)) ? 1 : 0);
    }
}

u8 md400_led_get_mask(void)
{
    u8 i;
    u8 mask = 0;

    for (i = 0; i < 4; i++) {
        if (md400_ms32_led_on[i]) {
            mask |= BIT(i);
        }
    }
    return mask;
}

static u8 md400_ms32_abs_delta(int delta)
{
    if (delta < 0) {
        delta = -delta;
    }
    if (delta <= 0) {
        delta = 1;
    }
    if (delta > MD400_MS32_ACCEL_STEP_MAX) {
        delta = MD400_MS32_ACCEL_STEP_MAX;
    }
    return (u8)delta;
}

static u8 md400_ms32_accel_step(u8 index, u8 event, int delta)
{
    u32 now;
    u32 elapsed;
    u8 base;
    u8 factor = 1;
    u8 step;

    if (index >= 4) {
        return 1;
    }

    base = md400_ms32_abs_delta(delta);
    now = sys_timer_get_ms();
    elapsed = now - md400_ms32_encoder_last_ms[index];

    if (md400_ms32_encoder_last_ms[index] &&
        (md400_ms32_encoder_last_event[index] == event) &&
        (elapsed <= MD400_MS32_ACCEL_IDLE_MS)) {
        if (elapsed <= MD400_MS32_ACCEL_FASTEST_MS) {
            factor = 12;
        } else if (elapsed <= MD400_MS32_ACCEL_FASTER_MS) {
            factor = 6;
        } else if (elapsed <= MD400_MS32_ACCEL_FAST_MS) {
            factor = 3;
        }
    }

    md400_ms32_encoder_last_ms[index] = now;
    md400_ms32_encoder_last_event[index] = event;

    step = base * factor;
    if (step > MD400_MS32_ACCEL_STEP_MAX) {
        step = MD400_MS32_ACCEL_STEP_MAX;
    }
    if (!step) {
        step = 1;
    }
    return step;
}

static u8 md400_ms32_raw_to_logical_event(u8 event)
{
#if MD400_MS32_REVERSE_ROTATION
    if (event == MD400_MS32_ENCODER_LEFT) {
        return MD400_MS32_ENCODER_RIGHT;
    }
    if (event == MD400_MS32_ENCODER_RIGHT) {
        return MD400_MS32_ENCODER_LEFT;
    }
#endif
    return event;
}

static void md400_ms32_apply_encoder_event(u8 id, u8 event, int delta)
{
    u8 index;
    u8 step;

    if ((id < 1) || (id > 4)) {
        md400_ms32_log_error("bad encoder id", id, event);
        return;
    }

    index = id - 1;

    if (event == MD400_MS32_ENCODER_LEFT) {
        step = md400_ms32_accel_step(index, event, delta);
        if (md400_ms32_encoder_value[index] > step) {
            md400_ms32_encoder_value[index] -= step;
        } else {
            md400_ms32_encoder_value[index] = 0;
        }
        md400_midi_send_encoder_cc(id, (u8)md400_ms32_encoder_value[index], event);
        md400_ms32_log_event("left", id, md400_ms32_encoder_value[index]);
    } else if (event == MD400_MS32_ENCODER_RIGHT) {
        step = md400_ms32_accel_step(index, event, delta);
        if (md400_ms32_encoder_value[index] < (0x7f - step)) {
            md400_ms32_encoder_value[index] += step;
        } else {
            md400_ms32_encoder_value[index] = 0x7f;
        }
        md400_midi_send_encoder_cc(id, (u8)md400_ms32_encoder_value[index], event);
        md400_ms32_log_event("right", id, md400_ms32_encoder_value[index]);
    } else if (event == MD400_MS32_ENCODER_PRESS) {
        md400_ms32_encoder_pressed[index] = 1;
        md400_midi_send_encoder_press(id, 1, (u8)md400_ms32_encoder_value[index]);
        md400_ms32_log_event("down", id, md400_ms32_encoder_value[index]);
    } else if (event == MD400_MS32_ENCODER_RELEASE) {
        md400_ms32_encoder_pressed[index] = 0;
        md400_midi_send_encoder_press(id, 0, (u8)md400_ms32_encoder_value[index]);
        md400_ms32_log_event("up", id, md400_ms32_encoder_value[index]);
    } else {
        md400_ms32_log_error("bad encoder event", id, event);
        return;
    }

    if (md400_ms32_encoder_cb) {
        md400_ms32_encoder_cb(id, event, md400_ms32_encoder_value[index]);
    }

    (void)delta;
}

static void md400_ms32_handle_encoder(const u8 *payload, u8 len)
{
    u8 raw_id;
    u8 id;
    u8 event;
    int delta;

    if (len < 3) {
        md400_ms32_log_error("short encoder frame", len, 0);
        return;
    }

    raw_id = payload[0];
    event = md400_ms32_raw_to_logical_event(payload[1]);
    delta = (payload[2] & 0x80) ? ((int)payload[2] - 0x100) : (int)payload[2];

    if ((raw_id < 1) || (raw_id > 4)) {
        md400_ms32_log_error("bad encoder id", raw_id, event);
        return;
    }

    id = md400_ms32_encoder_map[raw_id - 1];
    md400_ms32_encoder_frame_count++;
    md400_ms32_last_encoder_raw_id = raw_id;
    md400_ms32_last_encoder_id = id;
    md400_ms32_last_encoder_event = event;
    md400_ms32_last_encoder_delta = delta;
    md400_ms32_apply_encoder_event(id, event, delta);
}

static void md400_ms32_handle_adc(const u8 *payload, u8 len)
{
    u8 channel;
    u8 value;

    if (len < 2) {
        md400_ms32_log_error("short adc frame", len, 0);
        return;
    }

    channel = payload[0];
    value = payload[1];
    md400_ms32_adc_channel = channel;
    md400_ms32_adc_value = value;
    md400_ms32_adc_frame_count++;
    if (value < md400_ms32_adc_min) {
        md400_ms32_adc_min = value;
    }
    if (value > md400_ms32_adc_max) {
        md400_ms32_adc_max = value;
    }
    if (md400_ms32_adc_log_count < 24) {
        printf("[MD400_MS32] adc ch=%u raw=%u raw7=%u count=%lu\r\n",
               (unsigned)channel, (unsigned)value, (unsigned)(value >> 1),
               (unsigned long)md400_ms32_adc_frame_count);
    } else if (md400_ms32_adc_log_count == 24) {
        printf("[MD400_MS32] adc log muted\r\n");
    }
    md400_ms32_adc_log_count++;

    if (md400_ms32_adc_cb) {
        md400_ms32_adc_cb(channel, value);
    }
}

static void md400_ms32_handle_pedal_mode_ack(const u8 *payload, u8 len)
{
    u8 mode;

    if (len < 1) {
        md400_ms32_log_error("short mode ack", len, 0);
        return;
    }

    mode = payload[0] ? 1 : 0;
    md400_ms32_pedal_mode_ack = mode;
    md400_ms32_pedal_mode_ack_seq = md400_ms32_parser.seq;
    md400_ms32_pedal_mode_ack_count++;

    if (md400_ms32_pedal_mode_ack_log_count < 16) {
        printf("[MD400_MS32] pedal mode ack=%s count=%lu\r\n",
               mode ? "SWT" : "EXP",
               (unsigned long)md400_ms32_pedal_mode_ack_count);
    } else if (md400_ms32_pedal_mode_ack_log_count == 16) {
        printf("[MD400_MS32] pedal mode ack log muted\r\n");
    }
    md400_ms32_pedal_mode_ack_log_count++;
}

static void md400_ms32_handle_uart_status(const u8 *payload, u8 len)
{
    u8 mode;
    u16 rx_bytes;
    u8 frames;
    u8 mode_cmds;
    u8 checksum_errors;
    u8 version_errors;
    u8 length_errors;
    u8 overflows;
    u8 last_type;
    u8 last_seq;
    u8 last_len;
    u8 last_byte;
    u8 changed;

    if (len < 13) {
        md400_ms32_log_error("short uart status", len, 0);
        return;
    }

    mode = payload[0] ? 1 : 0;
    rx_bytes = (u16)payload[1] | ((u16)payload[2] << 8);
    frames = payload[3];
    mode_cmds = payload[4];
    checksum_errors = payload[5];
    version_errors = payload[6];
    length_errors = payload[7];
    overflows = payload[8];
    last_type = payload[9];
    last_seq = payload[10];
    last_len = payload[11];
    last_byte = payload[12];

    changed = (md400_ms32_uart_status_mode != mode) ||
              (md400_ms32_uart_status_rx_bytes != rx_bytes) ||
              (md400_ms32_uart_status_frames != frames) ||
              (md400_ms32_uart_status_mode_cmds != mode_cmds) ||
              (md400_ms32_uart_status_checksum_errors != checksum_errors) ||
              (md400_ms32_uart_status_version_errors != version_errors) ||
              (md400_ms32_uart_status_length_errors != length_errors) ||
              (md400_ms32_uart_status_overflows != overflows) ||
              (md400_ms32_uart_status_last_type != last_type) ||
              (md400_ms32_uart_status_last_seq != last_seq) ||
              (md400_ms32_uart_status_last_len != last_len) ||
              (md400_ms32_uart_status_last_byte != last_byte);

    md400_ms32_uart_status_mode = mode;
    md400_ms32_uart_status_rx_bytes = rx_bytes;
    md400_ms32_uart_status_frames = frames;
    md400_ms32_uart_status_mode_cmds = mode_cmds;
    md400_ms32_uart_status_checksum_errors = checksum_errors;
    md400_ms32_uart_status_version_errors = version_errors;
    md400_ms32_uart_status_length_errors = length_errors;
    md400_ms32_uart_status_overflows = overflows;
    md400_ms32_uart_status_last_type = last_type;
    md400_ms32_uart_status_last_seq = last_seq;
    md400_ms32_uart_status_last_len = last_len;
    md400_ms32_uart_status_last_byte = last_byte;
    md400_ms32_uart_status_count++;

    if (changed || (md400_ms32_uart_status_log_count < 8)) {
        printf("[MD400_MS32] uart status mode=%s rx=%u frames=%u mode_cmd=%u chk=%u ver=%u len=%u ovf=%u last_type=0x%02x seq=%u plen=%u last_byte=0x%02x count=%lu\r\n",
               mode ? "SWT" : "EXP",
               (unsigned)rx_bytes,
               (unsigned)frames,
               (unsigned)mode_cmds,
               (unsigned)checksum_errors,
               (unsigned)version_errors,
               (unsigned)length_errors,
               (unsigned)overflows,
               (unsigned)last_type,
               (unsigned)last_seq,
               (unsigned)last_len,
               (unsigned)last_byte,
               (unsigned long)md400_ms32_uart_status_count);
        md400_ms32_uart_status_log_count++;
    } else if (md400_ms32_uart_status_log_count == 8) {
        printf("[MD400_MS32] uart status log muted until change\r\n");
        md400_ms32_uart_status_log_count++;
    }
}

static void md400_ms32_handle_frame(void)
{
    u8 calc;
    u8 i;

    calc = md400_ms32_parser.ver ^ md400_ms32_parser.type ^
           md400_ms32_parser.seq ^ md400_ms32_parser.len;
    for (i = 0; i < md400_ms32_parser.len; i++) {
        calc ^= md400_ms32_parser.payload[i];
    }

    if (calc != md400_ms32_parser.chk) {
        md400_ms32_log_error("bad checksum", calc, md400_ms32_parser.chk);
        return;
    }

    if (md400_ms32_parser.ver != MD400_MS32_VER) {
        md400_ms32_log_error("bad version", md400_ms32_parser.ver, md400_ms32_parser.type);
        return;
    }

    md400_ms32_frame_count++;
    md400_ms32_last_type = md400_ms32_parser.type;
    md400_ms32_last_seq = md400_ms32_parser.seq;
    md400_ms32_last_len = md400_ms32_parser.len;

    if (md400_ms32_parser.type == MD400_MS32_TYPE_ENCODER) {
        md400_ms32_handle_encoder(md400_ms32_parser.payload, md400_ms32_parser.len);
    } else if (md400_ms32_parser.type == MD400_MS32_TYPE_ADC) {
        md400_ms32_handle_adc(md400_ms32_parser.payload, md400_ms32_parser.len);
    } else if (md400_ms32_parser.type == MD400_MS32_TYPE_PEDAL_MODE_ACK) {
        md400_ms32_handle_pedal_mode_ack(md400_ms32_parser.payload, md400_ms32_parser.len);
    } else if (md400_ms32_parser.type == MD400_MS32_TYPE_UART_STATUS) {
        md400_ms32_handle_uart_status(md400_ms32_parser.payload, md400_ms32_parser.len);
    } else {
        md400_ms32_unknown_frame_count++;
    }
}

static void md400_ms32_rx_byte(u8 byte)
{
    switch (md400_ms32_parser.state) {
    case MD400_MS32_WAIT_HEAD0:
        if (byte == MD400_MS32_HEAD0) {
            md400_ms32_parser.state = MD400_MS32_WAIT_HEAD1;
        }
        break;

    case MD400_MS32_WAIT_HEAD1:
        if (byte == MD400_MS32_HEAD1) {
            md400_ms32_parser.state = MD400_MS32_READ_VER;
        } else if (byte != MD400_MS32_HEAD0) {
            md400_ms32_parser.state = MD400_MS32_WAIT_HEAD0;
        }
        break;

    case MD400_MS32_READ_VER:
        md400_ms32_parser.ver = byte;
        md400_ms32_parser.state = MD400_MS32_READ_TYPE;
        break;

    case MD400_MS32_READ_TYPE:
        md400_ms32_parser.type = byte;
        md400_ms32_parser.state = MD400_MS32_READ_SEQ;
        break;

    case MD400_MS32_READ_SEQ:
        md400_ms32_parser.seq = byte;
        md400_ms32_parser.state = MD400_MS32_READ_LEN;
        break;

    case MD400_MS32_READ_LEN:
        md400_ms32_parser.len = byte;
        md400_ms32_parser.pos = 0;
        if (byte > MD400_MS32_PAYLOAD_MAX) {
            md400_ms32_log_error("payload too long", byte, MD400_MS32_PAYLOAD_MAX);
            md400_ms32_parser_reset();
        } else if (byte == 0) {
            md400_ms32_parser.state = MD400_MS32_READ_CHK;
        } else {
            md400_ms32_parser.state = MD400_MS32_READ_PAYLOAD;
        }
        break;

    case MD400_MS32_READ_PAYLOAD:
        md400_ms32_parser.payload[md400_ms32_parser.pos++] = byte;
        if (md400_ms32_parser.pos >= md400_ms32_parser.len) {
            md400_ms32_parser.state = MD400_MS32_READ_CHK;
        }
        break;

    case MD400_MS32_READ_CHK:
        md400_ms32_parser.chk = byte;
        md400_ms32_handle_frame();
        md400_ms32_parser_reset();
        break;

    default:
        md400_ms32_parser_reset();
        break;
    }
}

#if CONFIG_MD400_MS32_ADC_SERIAL_MONITOR_ENABLE
static void md400_ms32_adc_monitor_tick(void *priv)
{
    (void)priv;

    printf("[MD400_MS32] adc monitor ready=%u frames=%lu enc=%lu adc=%lu last_type=0x%02x adc_ch=%u raw=%u raw7=%u min=%u max=%u\r\n",
           (unsigned)md400_ms32_ready,
           (unsigned long)md400_ms32_frame_count,
           (unsigned long)md400_ms32_encoder_frame_count,
           (unsigned long)md400_ms32_adc_frame_count,
           (unsigned)md400_ms32_last_type,
           (unsigned)md400_ms32_adc_channel,
           (unsigned)md400_ms32_adc_value,
           (unsigned)(md400_ms32_adc_value >> 1),
           (unsigned)(md400_ms32_adc_frame_count ? md400_ms32_adc_min : 0),
           (unsigned)(md400_ms32_adc_frame_count ? md400_ms32_adc_max : 0));
}
#endif

void md400_ms32_init(void)
{
    u8 i;

    if (md400_ms32_ready) {
        return;
    }

    md400_ms32_parser_reset();

    md400_led_init();

    md400_ms32_ready = 1;
    printf("[MD400_MS32] init uart=115200 tx=PB2 rx=PB3\r\n");
#if CONFIG_MD400_MS32_ADC_SERIAL_MONITOR_ENABLE
    if (!md400_ms32_adc_monitor_timer) {
        md400_ms32_adc_monitor_timer = sys_timer_add(NULL,
                                                     md400_ms32_adc_monitor_tick,
                                                     1000);
        printf("[MD400_MS32] adc serial monitor enabled period=1000ms\r\n");
    }
#endif
}

void md400_ms32_rx_bytes(const u8 *data, u16 len)
{
    u16 i;

    if (!data || !len) {
        return;
    }

    for (i = 0; i < len; i++) {
        md400_ms32_rx_byte(data[i]);
    }
}

void md400_ms32_set_encoder_event_cb(md400_ms32_encoder_event_cb_t cb)
{
    md400_ms32_encoder_cb = cb;
}

void md400_ms32_set_adc_event_cb(md400_ms32_adc_event_cb_t cb)
{
    md400_ms32_adc_cb = cb;
}

void md400_ms32_inject_encoder_event(u8 encoder, u8 event)
{
    int delta = 0;

    if (event == MD400_MS32_ENCODER_LEFT) {
        delta = -1;
    } else if (event == MD400_MS32_ENCODER_RIGHT) {
        delta = 1;
    }

    md400_ms32_apply_encoder_event(encoder, event, delta);
}

int md400_ms32_get_encoder_value(u8 encoder)
{
    if ((encoder < 1) || (encoder > 4)) {
        return 0;
    }

    return md400_ms32_encoder_value[encoder - 1];
}

void md400_ms32_set_encoder_value(u8 encoder, u8 value)
{
    if ((encoder < 1) || (encoder > 4)) {
        return;
    }

    md400_ms32_encoder_value[encoder - 1] = value & 0x7f;
    md400_ms32_encoder_last_ms[encoder - 1] = 0;
    md400_ms32_encoder_last_event[encoder - 1] = 0;
}

u8 md400_ms32_get_encoder_pressed(u8 encoder)
{
    if ((encoder < 1) || (encoder > 4)) {
        return 0;
    }

    return md400_ms32_encoder_pressed[encoder - 1];
}

u8 md400_ms32_get_adc_value(void)
{
    return md400_ms32_adc_value;
}

void md400_ms32_get_adc_snapshot(u8 *channel, u8 *value, u32 *count,
                                  u8 *min_value, u8 *max_value)
{
    if (channel) {
        *channel = md400_ms32_adc_channel;
    }
    if (value) {
        *value = md400_ms32_adc_value;
    }
    if (count) {
        *count = md400_ms32_adc_frame_count;
    }
    if (min_value) {
        *min_value = md400_ms32_adc_frame_count ? md400_ms32_adc_min : 0;
    }
    if (max_value) {
        *max_value = md400_ms32_adc_frame_count ? md400_ms32_adc_max : 0;
    }
}

void md400_ms32_get_pedal_mode_ack_snapshot(u8 *mode, u32 *count, u8 *seq)
{
    if (mode) {
        *mode = md400_ms32_pedal_mode_ack;
    }
    if (count) {
        *count = md400_ms32_pedal_mode_ack_count;
    }
    if (seq) {
        *seq = md400_ms32_pedal_mode_ack_seq;
    }
}

void md400_ms32_print_debug_state(void)
{
    printf("[MD400_MS32] state ready=%u frames=%lu enc=%lu adc=%lu unknown=%lu errors=%lu last_type=0x%02x seq=%u len=%u\r\n",
           (unsigned)md400_ms32_ready,
           (unsigned long)md400_ms32_frame_count,
           (unsigned long)md400_ms32_encoder_frame_count,
           (unsigned long)md400_ms32_adc_frame_count,
           (unsigned long)md400_ms32_unknown_frame_count,
           (unsigned long)md400_ms32_error_log_count,
           (unsigned)md400_ms32_last_type,
           (unsigned)md400_ms32_last_seq,
           (unsigned)md400_ms32_last_len);
    printf("[MD400_MS32] last enc_raw=%u enc=%u event=%u delta=%d pressed=%u%u%u%u value=%d,%d,%d,%d\r\n",
           (unsigned)md400_ms32_last_encoder_raw_id,
           (unsigned)md400_ms32_last_encoder_id,
           (unsigned)md400_ms32_last_encoder_event,
           md400_ms32_last_encoder_delta,
           (unsigned)md400_ms32_encoder_pressed[0],
           (unsigned)md400_ms32_encoder_pressed[1],
           (unsigned)md400_ms32_encoder_pressed[2],
           (unsigned)md400_ms32_encoder_pressed[3],
           md400_ms32_encoder_value[0],
           md400_ms32_encoder_value[1],
           md400_ms32_encoder_value[2],
           md400_ms32_encoder_value[3]);
    printf("[MD400_MS32] last adc_ch=%u raw=%u raw7=%u min=%u max=%u\r\n",
           (unsigned)md400_ms32_adc_channel,
           (unsigned)md400_ms32_adc_value,
           (unsigned)(md400_ms32_adc_value >> 1),
           (unsigned)(md400_ms32_adc_frame_count ? md400_ms32_adc_min : 0),
           (unsigned)(md400_ms32_adc_frame_count ? md400_ms32_adc_max : 0));
    printf("[MD400_MS32] pedal mode ack=%s count=%lu seq=%u\r\n",
           (md400_ms32_pedal_mode_ack == 0x7f) ? "none" :
           (md400_ms32_pedal_mode_ack ? "SWT" : "EXP"),
           (unsigned long)md400_ms32_pedal_mode_ack_count,
           (unsigned)md400_ms32_pedal_mode_ack_seq);
    printf("[MD400_MS32] uart status mode=%s rx=%u frames=%u mode_cmd=%u chk=%u ver=%u len=%u ovf=%u last_type=0x%02x seq=%u plen=%u last_byte=0x%02x count=%lu\r\n",
           (md400_ms32_uart_status_mode == 0x7f) ? "none" :
           (md400_ms32_uart_status_mode ? "SWT" : "EXP"),
           (unsigned)md400_ms32_uart_status_rx_bytes,
           (unsigned)md400_ms32_uart_status_frames,
           (unsigned)md400_ms32_uart_status_mode_cmds,
           (unsigned)md400_ms32_uart_status_checksum_errors,
           (unsigned)md400_ms32_uart_status_version_errors,
           (unsigned)md400_ms32_uart_status_length_errors,
           (unsigned)md400_ms32_uart_status_overflows,
           (unsigned)md400_ms32_uart_status_last_type,
           (unsigned)md400_ms32_uart_status_last_seq,
           (unsigned)md400_ms32_uart_status_last_len,
           (unsigned)md400_ms32_uart_status_last_byte,
           (unsigned long)md400_ms32_uart_status_count);
}

#else

void md400_ms32_init(void)
{
}

void md400_ms32_rx_bytes(const u8 *data, u16 len)
{
    (void)data;
    (void)len;
}

void md400_ms32_set_encoder_event_cb(md400_ms32_encoder_event_cb_t cb)
{
    (void)cb;
}

void md400_ms32_set_adc_event_cb(md400_ms32_adc_event_cb_t cb)
{
    (void)cb;
}

void md400_ms32_inject_encoder_event(u8 encoder, u8 event)
{
    (void)encoder;
    (void)event;
}

int md400_ms32_get_encoder_value(u8 encoder)
{
    (void)encoder;
    return 0;
}

u8 md400_ms32_get_encoder_pressed(u8 encoder)
{
    (void)encoder;
    return 0;
}

u8 md400_ms32_get_adc_value(void)
{
    return 0;
}

void md400_ms32_get_adc_snapshot(u8 *channel, u8 *value, u32 *count,
                                  u8 *min_value, u8 *max_value)
{
    if (channel) {
        *channel = 0;
    }
    if (value) {
        *value = 0;
    }
    if (count) {
        *count = 0;
    }
    if (min_value) {
        *min_value = 0;
    }
    if (max_value) {
        *max_value = 0;
    }
}

void md400_ms32_get_pedal_mode_ack_snapshot(u8 *mode, u32 *count, u8 *seq)
{
    if (mode) {
        *mode = 0x7f;
    }
    if (count) {
        *count = 0;
    }
    if (seq) {
        *seq = 0;
    }
}

void md400_ms32_print_debug_state(void)
{
    printf("[MD400_MS32] disabled\r\n");
}

void md400_led_init(void)
{
}

void md400_led_set_mask(u8 mask)
{
    (void)mask;
}

u8 md400_led_get_mask(void)
{
    return 0;
}

#endif
