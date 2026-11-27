/* Modified for Nexfoot; upstream notices are retained. */
#include "system/app_core.h"
#include "system/includes.h"
#include "timer.h"
#include "server/server_core.h"
#include "app_config.h"
#include "app_action.h"
#include "os/os_api.h"
#include "btcontroller_config.h"
#include "btctrler/btctrler_task.h"
#include "config/config_transport.h"
#include "btstack/avctp_user.h"
#include "btstack/btstack_task.h"
#include "bt_common.h"
#include "rcsp_bluetooth.h"
#include "rcsp_user_update.h"
#include "app_charge.h"
#include "app_chargestore.h"
#include "app_power_manage.h"
#include "app_comm_bt.h"
#include "usb/device/cdc.h"
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
#include "usb/host/usb_host.h"
#include "usb/host/audio.h"
#include "usb/otg.h"
#include "usb/device/usb_stack.h"
#endif
#include "ble_trans.h"
#include "md400_ms32.h"
#ifndef CONFIG_MD400_DISPLAY_ENABLE
#define CONFIG_MD400_DISPLAY_ENABLE 1
#endif
#ifndef CONFIG_MD400_UART0_RX_TEST_ENABLE
#define CONFIG_MD400_UART0_RX_TEST_ENABLE 0
#endif
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
extern void md400_board_power_key_scan_start(void);
#endif
#define MD400_UART_LABEL_LEN 8
#if CONFIG_MD400_DISPLAY_ENABLE
#include "md400_display.h"
#endif

#define LOG_TAG_CONST       SPP_AND_LE
#define LOG_TAG             "[SPP_AND_LE]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "debug.h"

extern void trans_disconnect(void);
extern u8 md400_bt_uart_get_user_enable(void);
extern void md400_bt_uart_set_user_enable(u8 en);
extern void md400_bt_uart_disconnect(void);

#if TCFG_AUDIO_ENABLE
#include "tone_player.h"
#include "media/includes.h"
#include "key_event_deal.h"

extern void rtc_alarm_set_timer(u32 seconds);
extern void midi_paly_test(u32 key);
#endif/*TCFG_AUDIO_ENABLE*/


#if CONFIG_APP_SPP_LE

static u8 is_app_spple_active = 0;
static u8 enter_btstack_num = 0;
extern void app_switch(const char *name, int action);

// 修改日期: 2026-02-17, 修改说明: 参考apps\hid的USB初始化流程，PB7动态使能USB并注册OTG事件
static u8 usb_enabled = 0;
#if CONFIG_MD400_DISPLAY_ENABLE
#define MD400_DISPLAY_BOOT_DELAY_MS 100
static u16 md400_display_init_timer;
#endif
// 修改日期: 2026-02-17, 修改说明: PB7动态USB使能：状态记录与防抖
// 修改日期: 2026-02-17, 修改说明: USB主机挂载状态跟踪，防止未挂载时调用unmount导致崩溃
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
u8 usb1_host_mounted = 0;

static u16 usb1_midi_poll_timer_id = 0;
static u16 usb1_recover_timer;
static u16 usb1_holdoff_timer;
static u16 usb1_unmount_timer;
static u8 usb1_holdoff_host_out;
static u8 usb1_seen_device;

// 修改日期: 2026-02-18, 修改说明: 引用ble_trans.c中的USB1 MIDI接收解码接口
extern void usb1_midi_host_rx_decode(const u8 *packet, u32 len);

static void usb1_midi_poll(void *priv)
{
    (void)priv;
    if (!usb1_host_mounted) {
        return;
    }
    u8 temp_buf[64];
    int len = usb_midi_host_receive(1, temp_buf, sizeof(temp_buf));
    if (len > 0) {
        // log_info("USB1 MIDI RX: %d bytes", len);
        usb1_midi_host_rx_decode(temp_buf, len);
    }
}

static void usb1_midi_poll_start(void)
{
    if (usb1_midi_poll_timer_id == 0) {
        usb1_midi_poll_timer_id = usr_timer_add(NULL, usb1_midi_poll, 5, 1);
    }
}

static void usb1_midi_poll_stop(void)
{
    if (usb1_midi_poll_timer_id) {
        usr_timer_del(usb1_midi_poll_timer_id);
        usb1_midi_poll_timer_id = 0;
    }
}

static void md400_usb1_holdoff_clear(void *priv)
{
    (void)priv;
    usb1_holdoff_timer = 0;
    usb1_holdoff_host_out = 0;
}

static void md400_usb1_host_unmount_now(void *priv)
{
    (void)priv;
    usb1_unmount_timer = 0;
    if (!usb1_host_mounted) {
        return;
    }
    usb1_midi_poll_stop();
    usb_host_unmount(1);
    usb1_host_mounted = 0;
    usb1_seen_device = 0;
    printf("[MD400_USB] USB1 host unmount committed\r\n");
}

static void md400_usb1_cancel_deferred_unmount(void)
{
    if (usb1_unmount_timer) {
        sys_timeout_del(usb1_unmount_timer);
        usb1_unmount_timer = 0;
    }
}

void md400_usb1_host_rearm(void)
{
    if (!usb1_host_mounted) {
        return;
    }
    usb_otg_resume(1);
    usb_sie_enable(1);
    usb_h_isr_reg(1, 1, 0);
    printf("[MD400_USB] USB1 host rearm\r\n");
}

static void md400_usb1_host_mac_recover(void *priv)
{
    u32 type = md400_usb_detected_host_type();
    u32 ret;

    (void)priv;
    usb1_recover_timer = 0;

    printf("[MD400_USB] USB1 recover host_type=%u mounted=%u seen=%u\r\n",
           type, usb1_host_mounted, usb1_seen_device);

    if (!usb1_host_mounted && !usb1_seen_device) {
        md400_usb1_holdoff_clear(NULL);
        return;
    }

    /* Windows MIDI does not reset USB0 after attach. Keep the live USB1
     * session. Apple/macOS does a second bus reset; remount USB1 after it. */
    if (type == HOST_TYPE_WINDOWS) {
        md400_usb1_host_rearm();
        md400_usb1_holdoff_clear(NULL);
        printf("[MD400_USB] USB1 rearm only (Windows)\r\n");
        return;
    }

    if (usb1_host_mounted) {
        ret = usb_host_remount(1, 3, 100, 500, 0);
    } else {
        ret = usb_host_mount(1, 3, 100, 500);
    }
    if (ret == 0) {
        usb1_host_mounted = 1;
        usb1_seen_device = 1;
        usb1_midi_poll_start();
        printf("[MD400_USB] USB1 remount ok host_type=%u\r\n", type);
    } else {
        if (usb1_host_mounted) {
            usb1_midi_poll_stop();
            usb1_host_mounted = 0;
        }
        printf("[MD400_USB] USB1 remount fail %x host_type=%u\r\n", ret, type);
    }
    md400_usb1_holdoff_clear(NULL);
}

void md400_usb1_host_note_send_result(int ret)
{
    static u8 fails;

    if (!usb1_host_mounted) {
        fails = 0;
        return;
    }
    if (ret >= 0) {
        fails = 0;
        return;
    }
    fails++;
    if (fails < 2) {
        return;
    }
    fails = 0;
    printf("[MD400_USB] USB1 host send failed, remount\r\n");
    md400_usb1_cancel_deferred_unmount();
    usb_host_remount(1, 3, 100, 500, 0);
    usb1_midi_poll_start();
}

void md400_usb1_host_schedule_recover(void)
{
    usb1_holdoff_host_out = 1;
    md400_usb1_cancel_deferred_unmount();
    if (usb1_recover_timer) {
        sys_timeout_del(usb1_recover_timer);
        usb1_recover_timer = 0;
    }
    /* macOS CoreMIDI resets USB0 again after the class driver attaches.
     * Debounce past that second reset, then remount USB1. Windows does
     * not do that extra reset, so it only rearms. */
    usb1_recover_timer = sys_timeout_add(NULL, md400_usb1_host_mac_recover, 600);
}

void md400_on_usb0_slave_online(void)
{
    md400_usb1_host_rearm();
    md400_usb1_host_schedule_recover();
    printf("[MD400_USB] USB0 slave online, USB1 recover scheduled\r\n");
}

static void usb_event_handler(struct sys_event *event)
{
    if ((u32)event->arg != DEVICE_EVENT_FROM_OTG) {
        return;
    }
    const char *usb_msg = (const char *)event->u.dev.value;
    if (usb_msg[0] == 's') {
        if ((usb_msg[2] - '0') == 0 &&
            event->u.dev.event == DEVICE_EVENT_IN) {
            md400_on_usb0_slave_online();
        }
        return;
    }
    if (usb_msg[0] == 'h') {
        usb_dev usb_id = usb_msg[2] - '0';
        printf("usb_id:%d", usb_id);
        if (event->u.dev.event == DEVICE_EVENT_IN) {
            u32 ret = usb_host_mount(usb_id, 3, 100, 500);
            if (ret == 0) {
                if (usb_id == 1) {
                    usb1_host_mounted = 1;
                    usb1_seen_device = 1;
                    usb1_midi_poll_start();
                    log_info("USB1 MIDI poll started");
                }
                printf("DEVICE_EVENT_IN");
            } else {
                printf("DEVICE_EVENT_IN mount failed: %x", ret);
            }
        } else if (event->u.dev.event == DEVICE_EVENT_OUT) {
            if (usb_id == 1 && usb1_holdoff_host_out) {
                printf("DEVICE_EVENT_OUT ignored (USB0 slave start)\r\n");
                return;
            }
            if (usb_id == 1 && usb1_host_mounted) {
                md400_usb1_cancel_deferred_unmount();
                usb1_unmount_timer = sys_timeout_add(NULL, md400_usb1_host_unmount_now, 300);
                printf("DEVICE_EVENT_OUT deferred\r\n");
            } else {
                printf("DEVICE_EVENT_OUT skip (not mounted)");
            }
        }
    }
}
#endif

// 修改日期: 2026-02-17, 修改说明: 参考apps\hid启用USB0并注册OTG事件
static void usb_dynamic_enable(void)
{
    if (usb_enabled) {
        return;
    }
#if TCFG_PC_ENABLE
    extern void usb_start(void);
    usb_start();
#endif
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
    register_sys_event_handler(SYS_DEVICE_EVENT, DEVICE_EVENT_FROM_OTG, 2, usb_event_handler);
#endif
    usb_enabled = 1;
}

#if CONFIG_MD400_UART0_RX_TEST_ENABLE && (TCFG_UART0_RX_PORT != NO_CONFIG_PORT)
#include "asm/wdt.h"
#define MD400_UART_RX_LINE_MAX 32
#define MD400_UART_RX_QUEUE_DEPTH 4

static u16 md400_uart0_rx_timer;
static volatile u8 md400_uart0_rx_work_len;
static volatile u8 md400_uart0_rx_queue_rp;
static volatile u8 md400_uart0_rx_queue_wp;
static volatile u8 md400_uart0_rx_queue_count;
static volatile u8 md400_uart0_rx_queue_len[MD400_UART_RX_QUEUE_DEPTH];
static char md400_uart0_rx_queue[MD400_UART_RX_QUEUE_DEPTH][MD400_UART_RX_LINE_MAX];
static char md400_uart0_rx_work[MD400_UART_RX_LINE_MAX];

static void md400_uart0_reboot_timeout(void *priv)
{
    (void)priv;
    cpu_reset();
}

static void md400_uart0_wdt_timeout(void *priv)
{
    (void)priv;
    local_irq_disable();
    wdt_init(WDT_16MS);
    while (1) {
    }
}

extern void uart_db_regiest_recieve_callback(void *rx_cb);
extern u8 md400_usb_mask_update_schedule_fast(void);

static int md400_uart0_rx_line_eq(const char *line, const char *text)
{
    while (*line && *text && (*line == *text)) {
        line++;
        text++;
    }
    return (*line == '\0') && (*text == '\0');
}

static u8 md400_uart0_rx_line_len_calc(const char *line)
{
    u8 len = 0;

    while (line[len] && (len < (MD400_UART_RX_LINE_MAX - 1))) {
        len++;
    }
    return len;
}

static u8 md400_uart0_rx_event_from_encoder_op(char op)
{
    switch (op) {
    case 'L':
    case 'l':
        return MD400_CONTROL_EVENT_ENCODER_LEFT;
    case 'R':
    case 'r':
        return MD400_CONTROL_EVENT_ENCODER_RIGHT;
    case 'D':
    case 'd':
        return MD400_CONTROL_EVENT_ENCODER_PRESS;
    case 'U':
    case 'u':
        return MD400_CONTROL_EVENT_ENCODER_RELEASE;
    default:
        return 0;
    }
}

static u8 md400_uart0_rx_event_from_single_gesture_op(char op)
{
    switch (op) {
    case 'P':
    case 'p':
        return MD400_CONTROL_EVENT_ENCODER_PRESS;
    case 'S':
    case 's':
        return MD400_CONTROL_EVENT_ENCODER_SHORT;
    case 'L':
    case 'l':
        return MD400_CONTROL_EVENT_ENCODER_LONG;
    case 'B':
    case 'b':
        return MD400_CONTROL_EVENT_ENCODER_DOUBLE;
    default:
        return 0;
    }
}

static u8 md400_uart0_rx_event_from_combo_gesture_op(char op)
{
    switch (op) {
    case 'P':
    case 'p':
        return MD400_CONTROL_EVENT_COMBO_PRESS;
    case 'S':
    case 's':
        return MD400_CONTROL_EVENT_COMBO_SHORT;
    case 'L':
    case 'l':
        return MD400_CONTROL_EVENT_COMBO_LONG;
    default:
        return 0;
    }
}

static u8 md400_uart0_rx_combo_mask(char a, char b)
{
    u8 ea;
    u8 eb;

    if ((a < '1') || (a > '4') || (b < '1') || (b > '4')) {
        return 0;
    }
    ea = a - '1';
    eb = b - '1';
    if (ea == eb) {
        return 0;
    }
    return BIT(ea) | BIT(eb);
}

static u8 md400_uart0_rx_parse_u7(const char *line, u8 start, u8 len, u8 *value)
{
    u16 out = 0;
    u8 i;

    if (!line || !value || (start >= len)) {
        return 0;
    }

    for (i = start; i < len; i++) {
        if ((line[i] < '0') || (line[i] > '9')) {
            return 0;
        }
        out = (out * 10) + (line[i] - '0');
        if (out > 127) {
            return 0;
        }
    }

    *value = out & 0x7f;
    return 1;
}

static u8 md400_uart0_rx_hex_nibble(char ch, u8 *value)
{
    if (!value) {
        return 0;
    }
    if ((ch >= '0') && (ch <= '9')) {
        *value = ch - '0';
        return 1;
    }
    if ((ch >= 'a') && (ch <= 'f')) {
        *value = 10 + (ch - 'a');
        return 1;
    }
    if ((ch >= 'A') && (ch <= 'F')) {
        *value = 10 + (ch - 'A');
        return 1;
    }
    return 0;
}

static u8 md400_uart0_rx_hex_pair(const char *line, u8 index, u8 *value)
{
    u8 hi;
    u8 lo;

    if (!line || !value ||
        !md400_uart0_rx_hex_nibble(line[index], &hi) ||
        !md400_uart0_rx_hex_nibble(line[index + 1], &lo)) {
        return 0;
    }

    *value = ((hi << 4) | lo) & 0xff;
    return 1;
}

static u16 md400_uart0_rx_u14(const u8 *payload)
{
    if (!payload) {
        return 0;
    }
    return (payload[0] & 0x7f) | ((payload[1] & 0x7f) << 7);
}

static u8 md400_uart0_rx_parse_assignment_fields(const char *line, u8 start, u8 *fields)
{
    u8 value;

    if (!line || !fields) {
        return 0;
    }

    if (!md400_uart0_rx_hex_nibble(line[start], &fields[0])) {
        return 0;
    }
    if (!md400_uart0_rx_hex_nibble(line[start + 1], &value)) {
        return 0;
    }
    fields[1] = value + 1;
    if (!md400_uart0_rx_hex_pair(line, start + 2, &fields[2]) ||
        !md400_uart0_rx_hex_pair(line, start + 4, &fields[3]) ||
        !md400_uart0_rx_hex_pair(line, start + 6, &fields[4]) ||
        !md400_uart0_rx_hex_pair(line, start + 8, &fields[5]) ||
        !md400_uart0_rx_hex_nibble(line[start + 10], &fields[6]) ||
        !md400_uart0_rx_hex_pair(line, start + 11, &fields[7]) ||
        !md400_uart0_rx_hex_pair(line, start + 13, &fields[8])) {
        return 0;
    }

    return 1;
}

static u8 md400_uart0_rx_match_label_prefix(const char *line)
{
    if (!line) {
        return 0;
    }
    return ((line[0] == 'L') || (line[0] == 'l')) &&
           ((line[1] == 'A') || (line[1] == 'a')) &&
           ((line[2] == 'B') || (line[2] == 'b')) &&
           ((line[3] == 'E') || (line[3] == 'e')) &&
           ((line[4] == 'L') || (line[4] == 'l'));
}

static u8 md400_uart0_rx_handle_command(const char *line)
{
    u8 len = md400_uart0_rx_line_len_calc(line);
    u8 event;
    u8 control;

    if (md400_uart0_rx_line_eq(line, "PING") || md400_uart0_rx_line_eq(line, "?")) {
        printf("[MD400_UART] PONG\r\n");
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "FB")) {
        if (!md400_display_pixel_capture_request()) {
            printf("[MD400_UART] fb busy\r\n");
        }
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "HELP")) {
        printf("[MD400_UART] cmds: PING,REBOOT,WDT,FB,INFO,STATE,MS32,PEDAL,ASSIGN,GEST,LATCH,MOUT,UHOST,BLES,BT?,BTON,BTOFF,BTDISC,DISP,LED,KEY,PROTO,EVLOG,SAVE,L<n>,LABEL<n>,LABEL<n>=TEXT,EXIT,FACTORY,JLOTA,A<enc><hex>,D<hex>,B<event><control><hex>,E1L/E1R/E1D/E1U,E1R16,G1P/G1S/G1L/G1B,G12P/G12S/G12L,KS/KL/KB,P0..P127\r\n");
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "REBOOT")) {
        printf("[MD400_UART] reboot scheduled\r\n");
        sys_timeout_add(NULL, md400_uart0_reboot_timeout, 100);
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "WDT")) {
        printf("[MD400_UART] watchdog reset scheduled\r\n");
        sys_timeout_add(NULL, md400_uart0_wdt_timeout, 100);
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "STATE")) {
        u8 payload[12];

        if (md400_runtime_state_snapshot(payload, sizeof(payload)) == sizeof(payload)) {
            printf("[MD400_UART] state mode=%u active=%u selected=%u page=%u target=%u subject=%u foot=%s pedal_active=%u pedal_selected=%u pedal_page=%u pedal_ft=%u pedal_view=%u\r\n",
                   (unsigned)payload[0],
                   (unsigned)payload[1],
                   (unsigned)payload[2],
                   (unsigned)payload[3],
                   (unsigned)payload[4],
                   (unsigned)payload[5],
                   payload[6] ? "PEDAL" : "MIDI",
                   (unsigned)payload[7],
                   (unsigned)payload[8],
                   (unsigned)payload[9],
                   (unsigned)payload[10],
                   (unsigned)payload[11]);
        } else {
            printf("[MD400_UART] state unavailable\r\n");
        }
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "MS32")) {
        md400_ms32_print_debug_state();
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "PEDAL")) {
        u8 payload[16];
        u8 payload_len;

        payload_len = md400_runtime_pedal_state_snapshot(payload, sizeof(payload));
        if (payload_len >= sizeof(payload)) {
            u16 raw8 = (payload[8] & 0x7f) | ((payload[9] & 0x7f) << 7);
            u16 count = (payload[10] & 0x7f) | ((payload[11] & 0x7f) << 7);
            u16 min8 = (payload[12] & 0x7f) | ((payload[13] & 0x7f) << 7);
            u16 max8 = (payload[14] & 0x7f) | ((payload[15] & 0x7f) << 7);

            printf("[MD400_UART] pedal ch=%u adc7=%u current=%u seen=%u mode=%s switch=%u ms32_ch=%u ms32_raw7=%u raw8=%u count=%u min=%u max=%u\r\n",
                   (unsigned)payload[0], (unsigned)payload[1],
                   (unsigned)payload[2], (unsigned)payload[3],
                   payload[4] ? "SWT" : "EXP", (unsigned)payload[5],
                   (unsigned)payload[6], (unsigned)payload[7],
                   (unsigned)raw8, (unsigned)count,
                   (unsigned)min8, (unsigned)max8);
        } else {
            printf("[MD400_UART] pedal unavailable\r\n");
        }
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "ASSIGN")) {
        u8 payload[56];
        u8 payload_len;
        u8 offset;
        u8 i;

        payload_len = md400_runtime_assignment_snapshot(payload, sizeof(payload));
        if (payload_len >= sizeof(payload)) {
            printf("[MD400_UART] assign active=%u len=%u\r\n",
                   (unsigned)payload[0], (unsigned)payload_len);
            offset = 1;
            for (i = 0; i < 4; i++) {
                printf("[MD400_UART] assign enc=%u type=%u ch=%u data1=%u min=%u max=%u def=%u beh=%u outputs=0x%02x sysex=%u value=%u\r\n",
                       (unsigned)payload[offset],
                       (unsigned)payload[offset + 1],
                       (unsigned)payload[offset + 2],
                       (unsigned)payload[offset + 3],
                       (unsigned)payload[offset + 4],
                       (unsigned)payload[offset + 5],
                       (unsigned)payload[offset + 6],
                       (unsigned)payload[offset + 7],
                       (unsigned)payload[offset + 8],
                       (unsigned)payload[offset + 9],
                       (unsigned)payload[offset + 10]);
                offset += 11;
            }
            printf("[MD400_UART] assign pedal type=%u ch=%u data1=%u min=%u max=%u def=%u beh=%u outputs=0x%02x sysex=%u value=%u\r\n",
                   (unsigned)payload[offset + 1],
                   (unsigned)payload[offset + 2],
                   (unsigned)payload[offset + 3],
                   (unsigned)payload[offset + 4],
                   (unsigned)payload[offset + 5],
                   (unsigned)payload[offset + 6],
                   (unsigned)payload[offset + 7],
                   (unsigned)payload[offset + 8],
                   (unsigned)payload[offset + 9],
                   (unsigned)payload[offset + 10]);
        } else {
            printf("[MD400_UART] assign unavailable len=%u\r\n",
                   (unsigned)payload_len);
        }
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "MOUT")) {
        u8 payload[10];
        u8 trace[16];
        char trace_hex[33];
        u8 payload_len;
        u8 trace_len;
        u8 i;
        static const char hex[] = "0123456789ABCDEF";

        payload_len = md400_runtime_midi_out_state_snapshot(payload, sizeof(payload));
        if (payload_len >= sizeof(payload)) {
            trace_len = md400_runtime_midi_out_trace_snapshot(trace, sizeof(trace));
            for (i = 0; i < trace_len; i++) {
                trace_hex[i * 2] = hex[(trace[i] >> 4) & 0x0f];
                trace_hex[(i * 2) + 1] = hex[trace[i] & 0x0f];
            }
            trace_hex[trace_len * 2] = '\0';
            printf("[MD400_UART] mout enabled=%u available=%u queued=%u written=%u dropped=%u open_fail=%u trace_len=%u trace=%s\r\n",
                   (unsigned)payload[0],
                   (unsigned)payload[1],
                   (unsigned)md400_uart0_rx_u14(&payload[2]),
                   (unsigned)md400_uart0_rx_u14(&payload[4]),
                   (unsigned)md400_uart0_rx_u14(&payload[6]),
                   (unsigned)md400_uart0_rx_u14(&payload[8]),
                   (unsigned)trace_len,
                   trace_hex);
        } else {
            printf("[MD400_UART] mout unavailable len=%u\r\n",
                   (unsigned)payload_len);
        }
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "UHOST")) {
        u8 payload[18];
        u8 payload_len;

        payload_len = md400_runtime_usb_host_state_snapshot(payload, sizeof(payload));
        if (payload_len >= sizeof(payload)) {
            printf("[MD400_UART] uhost enabled=%u mounted=%u tx_queued=%u tx_written=%u tx_packet=%u tx_dropped=%u rx_packet=%u rx_queued=%u rx_dropped=%u send_fail=%u\r\n",
                   (unsigned)payload[0],
                   (unsigned)payload[1],
                   (unsigned)md400_uart0_rx_u14(&payload[2]),
                   (unsigned)md400_uart0_rx_u14(&payload[4]),
                   (unsigned)md400_uart0_rx_u14(&payload[6]),
                   (unsigned)md400_uart0_rx_u14(&payload[8]),
                   (unsigned)md400_uart0_rx_u14(&payload[10]),
                   (unsigned)md400_uart0_rx_u14(&payload[12]),
                   (unsigned)md400_uart0_rx_u14(&payload[14]),
                   (unsigned)md400_uart0_rx_u14(&payload[16]));
        } else {
            printf("[MD400_UART] uhost unavailable len=%u\r\n",
                   (unsigned)payload_len);
        }
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "BLES")) {
        u8 payload[40];
        u8 payload_len;

        payload_len = md400_runtime_ble_midi_state_snapshot(payload, sizeof(payload));
        if (payload_len >= 28) {
            printf("[MD400_UART] ble enabled=%u connected=%u notify=%u handle=%u tx_used=%u rx_used=%u tx_queued=%u tx_written=%u tx_packet=%u tx_dropped=%u tx_fail=%u tx_unavail_drop=%u rx_packet=%u rx_queued=%u rx_dropped=%u link_events=%u",
                   (unsigned)payload[0],
                   (unsigned)payload[1],
                   (unsigned)payload[2],
                   (unsigned)md400_uart0_rx_u14(&payload[3]),
                   (unsigned)md400_uart0_rx_u14(&payload[5]),
                   (unsigned)md400_uart0_rx_u14(&payload[7]),
                   (unsigned)md400_uart0_rx_u14(&payload[9]),
                   (unsigned)md400_uart0_rx_u14(&payload[11]),
                   (unsigned)md400_uart0_rx_u14(&payload[13]),
                   (unsigned)md400_uart0_rx_u14(&payload[15]),
                   (unsigned)md400_uart0_rx_u14(&payload[17]),
                   (unsigned)md400_uart0_rx_u14(&payload[19]),
                   (unsigned)md400_uart0_rx_u14(&payload[21]),
                   (unsigned)md400_uart0_rx_u14(&payload[23]),
                   (unsigned)md400_uart0_rx_u14(&payload[25]),
                   (unsigned)payload[27]);
            if (payload_len >= 40) {
                printf(" role=%u server_conn=%u server_ccc=%u client_ready=%u client_conn=%u client_io=%u client_write=%u client_notify=%u",
                       (unsigned)payload[39],
                       (unsigned)payload[28],
                       (unsigned)payload[29],
                       (unsigned)payload[30],
                       (unsigned)md400_uart0_rx_u14(&payload[31]),
                       (unsigned)md400_uart0_rx_u14(&payload[33]),
                       (unsigned)md400_uart0_rx_u14(&payload[35]),
                       (unsigned)md400_uart0_rx_u14(&payload[37]));
            }
            printf("\r\n");
        } else {
            printf("[MD400_UART] ble unavailable len=%u\r\n",
                   (unsigned)payload_len);
        }
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "BT?")) {
        printf("[MD400_UART] bt enable=%u\r\n",
               (unsigned)md400_bt_uart_get_user_enable());
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "BTON")) {
        md400_bt_uart_set_user_enable(1);
        printf("[MD400_UART] bt enable request=1 current=%u\r\n",
               (unsigned)md400_bt_uart_get_user_enable());
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "BTOFF")) {
        md400_bt_uart_set_user_enable(0);
        printf("[MD400_UART] bt enable request=0 current=%u\r\n",
               (unsigned)md400_bt_uart_get_user_enable());
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "BTDISC")) {
        md400_bt_uart_disconnect();
        printf("[MD400_UART] bt disconnect request\r\n");
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "DISP")) {
        u8 payload[30];
        u8 payload_len;

        payload_len = md400_runtime_display_state_snapshot(payload, sizeof(payload));
        if (payload_len >= 17) {
            printf("[MD400_UART] disp ready=%u mode=%u dirty=%u pressed=0x%02x rows=%u,%u,%u,%u enc=%u,%u,%u,%u ui=%u,%u,%u,%u drawing=%u",
                   (unsigned)payload[0],
                   (unsigned)payload[1],
                   (unsigned)payload[2],
                   (unsigned)payload[3],
                   (unsigned)payload[4],
                   (unsigned)payload[5],
                   (unsigned)payload[6],
                   (unsigned)payload[7],
                   (unsigned)payload[8],
                   (unsigned)payload[9],
                   (unsigned)payload[10],
                   (unsigned)payload[11],
                   (unsigned)payload[12],
                   (unsigned)payload[13],
                   (unsigned)payload[14],
                   (unsigned)payload[15],
                   (unsigned)payload[16]);
            if (payload_len >= 22) {
                printf(" layout=%u roles=%02x,%02x,%02x,%02x",
                       (unsigned)payload[17],
                       (unsigned)payload[18],
                       (unsigned)payload[19],
                       (unsigned)payload[20],
                       (unsigned)payload[21]);
            }
            if (payload_len >= 28) {
                printf(" draw=%u,%u,%u,%u,%u,%u",
                       (unsigned)payload[22],
                       (unsigned)payload[23],
                       (unsigned)payload[24],
                       (unsigned)payload[25],
                       (unsigned)payload[26],
                       (unsigned)payload[27]);
            }
            if (payload_len >= 30) {
                printf(" spi=%u err=%u",
                       (unsigned)payload[28],
                       (unsigned)payload[29]);
            }
            printf("\r\n");
        } else {
            printf("[MD400_UART] disp unavailable len=%u\r\n",
                   (unsigned)payload_len);
        }
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "LED")) {
        u8 payload[8];
        u8 payload_len;

        payload_len = md400_runtime_led_state_snapshot(payload, sizeof(payload));
        if (payload_len >= sizeof(payload)) {
            printf("[MD400_UART] led mode=%u mask=0x%02x hw=0x%02x held=0x%02x target=%u selected_pos=%u preset_mode=%u preset_mask=0x%02x\r\n",
                   (unsigned)payload[0],
                   (unsigned)payload[1],
                   (unsigned)payload[2],
                   (unsigned)payload[3],
                   (unsigned)payload[4],
                   (unsigned)payload[5],
                   (unsigned)payload[6],
                   (unsigned)payload[7]);
        } else {
            printf("[MD400_UART] led unavailable len=%u\r\n",
                   (unsigned)payload_len);
        }
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "KEY")) {
        u8 payload[18];
        u8 payload_len;

        payload_len = md400_runtime_edit_key_state_snapshot(payload, sizeof(payload));
        if (payload_len >= sizeof(payload)) {
            printf("[MD400_UART] key enabled=%u raw=%u stable=%u last=%u debounce=%u clicks=%u long_sent=%u ticks=%u short_timer=%u short=%u long=%u double=%u edges=%u\r\n",
                   (unsigned)payload[0],
                   (unsigned)payload[1],
                   (unsigned)payload[2],
                   (unsigned)payload[3],
                   (unsigned)payload[4],
                   (unsigned)payload[5],
                   (unsigned)payload[6],
                   (unsigned)md400_uart0_rx_u14(&payload[7]),
                   (unsigned)payload[9],
                   (unsigned)md400_uart0_rx_u14(&payload[10]),
                   (unsigned)md400_uart0_rx_u14(&payload[12]),
                   (unsigned)md400_uart0_rx_u14(&payload[14]),
                   (unsigned)md400_uart0_rx_u14(&payload[16]));
        } else {
            printf("[MD400_UART] key unavailable len=%u\r\n",
                   (unsigned)payload_len);
        }
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "PROTO")) {
        u8 payload[28];
        u8 payload_len;

        payload_len = md400_runtime_protocol_state_snapshot(payload, sizeof(payload));
        if (payload_len >= 19) {
            printf("[MD400_UART] proto layout=%u valid=%u replies=%u ok=%u bad_checksum=%u bad_length=%u bad_value=%u unsupported=%u busy=%u flash_error=%u\r\n",
                   (unsigned)payload[0],
                   (unsigned)md400_uart0_rx_u14(&payload[1]),
                   (unsigned)md400_uart0_rx_u14(&payload[3]),
                   (unsigned)md400_uart0_rx_u14(&payload[5]),
                   (unsigned)md400_uart0_rx_u14(&payload[7]),
                   (unsigned)md400_uart0_rx_u14(&payload[9]),
                   (unsigned)md400_uart0_rx_u14(&payload[11]),
                   (unsigned)md400_uart0_rx_u14(&payload[13]),
                   (unsigned)md400_uart0_rx_u14(&payload[15]),
                   (unsigned)md400_uart0_rx_u14(&payload[17]));
        } else {
            printf("[MD400_UART] proto unavailable len=%u\r\n",
                   (unsigned)payload_len);
        }
        return 1;
    }

    if ((len == 17) && ((line[0] == 'A') || (line[0] == 'a')) &&
        (line[1] >= '1') && (line[1] <= '4')) {
        u8 fields[9];
        u8 status;

        if (md400_uart0_rx_parse_assignment_fields(line, 2, fields)) {
            control = line[1] - '0';
            status = md400_runtime_set_encoder_assignment(control, fields, sizeof(fields));
            printf("[MD400_UART] assign set enc=%u status=%u\r\n",
                   (unsigned)control, (unsigned)status);
            return 1;
        }
    }

    if ((len == 16) && ((line[0] == 'D') || (line[0] == 'd'))) {
        u8 fields[9];
        u8 status;

        if (md400_uart0_rx_parse_assignment_fields(line, 1, fields)) {
            status = md400_runtime_set_pedal_assignment(fields, sizeof(fields));
            printf("[MD400_UART] assign set pedal status=%u\r\n",
                   (unsigned)status);
            return 1;
        }
    }

    if (md400_uart0_rx_line_eq(line, "GEST")) {
        u8 state[6];
        u8 payload[12];
        u8 count = md400_runtime_gesture_assignment_count();
        u8 active = 0;
        u8 payload_len;
        u8 i;

        if (md400_runtime_state_snapshot(state, sizeof(state)) == sizeof(state)) {
            active = state[1];
        }
        printf("[MD400_UART] gesture active=%u count=%u len=12\r\n",
               (unsigned)active, (unsigned)count);
        for (i = 0; i < count; i++) {
            payload_len = md400_runtime_gesture_assignment_snapshot(i, payload, sizeof(payload));
            if (payload_len >= sizeof(payload)) {
                printf("[MD400_UART] gesture event=%u control=%u type=%u ch=%u data1=%u min=%u max=%u def=%u beh=%u outputs=0x%02x sysex=%u value=%u\r\n",
                       (unsigned)payload[0],
                       (unsigned)payload[1],
                       (unsigned)payload[2],
                       (unsigned)payload[3],
                       (unsigned)payload[4],
                       (unsigned)payload[5],
                       (unsigned)payload[6],
                       (unsigned)payload[7],
                       (unsigned)payload[8],
                       (unsigned)payload[9],
                       (unsigned)payload[10],
                       (unsigned)payload[11]);
            }
        }
        return 1;
    }

    if ((len == 18) && ((line[0] == 'B') || (line[0] == 'b'))) {
        u8 fields[9];
        u8 status;

        if (md400_uart0_rx_hex_nibble(line[1], &event) &&
            md400_uart0_rx_hex_nibble(line[2], &control) &&
            md400_uart0_rx_parse_assignment_fields(line, 3, fields)) {
            status = md400_runtime_set_gesture_assignment(event, control, fields, sizeof(fields));
            printf("[MD400_UART] gesture set event=%u control=%u status=%u\r\n",
                   (unsigned)event, (unsigned)control, (unsigned)status);
            return 1;
        }
    }

    if (md400_uart0_rx_line_eq(line, "EVLOG")) {
        u8 payload[21];
        u8 payload_len;
        u8 count;
        u8 i;

        payload_len = md400_runtime_event_log_snapshot(payload, sizeof(payload));
        count = payload_len ? payload[0] : 0;
        printf("[MD400_UART] evlog count=%u len=%u\r\n",
               (unsigned)count, (unsigned)payload_len);
        for (i = 0; (i < count) && ((1 + (i * 4) + 3) < payload_len); i++) {
            u8 offset = 1 + (i * 4);
            printf("[MD400_UART] ev source=%u event=%u control=%u value=%u\r\n",
                   (unsigned)payload[offset],
                   (unsigned)payload[offset + 1],
                   (unsigned)payload[offset + 2],
                   (unsigned)payload[offset + 3]);
        }
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "LATCH")) {
        u8 payload[12];
        u8 payload_len;

        payload_len = md400_runtime_action_state_snapshot(payload, sizeof(payload));
        if (payload_len >= sizeof(payload)) {
            printf("[MD400_UART] latch active=%u note=0x%02x epress=0x%02x eshort=0x%02x elong=0x%02x edouble=0x%02x cpress=0x%02x cshort=0x%02x clong=0x%02x mpress=0x%02x mlong=0x%02x mclong=0x%02x\r\n",
                   (unsigned)payload[0],
                   (unsigned)payload[1],
                   (unsigned)payload[2],
                   (unsigned)payload[3],
                   (unsigned)payload[4],
                   (unsigned)payload[5],
                   (unsigned)payload[6],
                   (unsigned)payload[7],
                   (unsigned)payload[8],
                   (unsigned)payload[9],
                   (unsigned)payload[10],
                   (unsigned)payload[11]);
        } else {
            printf("[MD400_UART] latch unavailable len=%u\r\n", (unsigned)payload_len);
        }
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "INFO")) {
        u8 major = 0;
        u8 minor = 0;
        u8 patch = 0;

        md400_runtime_fw_version(&major, &minor, &patch);
        printf("[MD400_UART] info firmware=%u.%u.%u preset_store=%u targets=0x%02x features=0x%02x\r\n",
               (unsigned)major,
               (unsigned)minor,
               (unsigned)patch,
               (unsigned)md400_runtime_preset_store_version(),
               (unsigned)md400_runtime_target_mask(),
               (unsigned)md400_runtime_feature_flags());
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "JLOTA")) {
        if (md400_usb_mask_update_schedule()) {
            printf("[MD400_UART] JLOTA mask detach\r\n");
        } else {
            printf("[MD400_UART] JLOTA busy\r\n");
        }
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "EXIT")) {
        u8 status = md400_runtime_exit_edit();

        printf("[MD400_UART] exit status=%u\r\n", (unsigned)status);
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "SAVE")) {
        u8 status = md400_runtime_save_active_preset();

        printf("[MD400_UART] save status=%u\r\n", (unsigned)status);
        return 1;
    }

    if (md400_uart0_rx_line_eq(line, "FACTORY")) {
        u8 status = md400_runtime_factory_reset();

        printf("[MD400_UART] factory reset status=%u presets=%u\r\n",
               (unsigned)status, status ? 0 : 20);
        return 1;
    }

    if ((len >= 2) && (len <= 3) && ((line[0] == 'L') || (line[0] == 'l'))) {
        u8 preset;

        if (md400_uart0_rx_parse_u7(line, 1, len, &preset)) {
            u8 status = md400_runtime_load_preset_from_vm(preset);

            printf("[MD400_UART] load preset=%u status=%u\r\n",
                   (unsigned)preset, (unsigned)status);
            return 1;
        }
    }

    if ((len >= 6) && md400_uart0_rx_match_label_prefix(line)) {
        u8 preset = 0;
        u8 eq = 0;
        u8 i;

        for (i = 5; i < len; i++) {
            if (line[i] == '=') {
                eq = i;
                break;
            }
        }

        if (eq) {
            if (md400_uart0_rx_parse_u7(line, 5, eq, &preset)) {
                u8 label_len = len - eq - 1;
                u8 status;

                if (label_len <= MD400_UART_LABEL_LEN) {
                    status = md400_runtime_set_preset_label(preset, (const u8 *)&line[eq + 1], label_len);
                    printf("[MD400_UART] label preset=%u status=%u len=%u text=%s\r\n",
                           (unsigned)preset, (unsigned)status, (unsigned)label_len, &line[eq + 1]);
                    return 1;
                }
            }
        } else if (md400_uart0_rx_parse_u7(line, 5, len, &preset)) {
            u8 label[MD400_UART_LABEL_LEN + 1];
            u8 label_len = 0;
            u8 status;

            status = md400_runtime_get_preset_label(preset, label, &label_len, MD400_UART_LABEL_LEN);
            label[MD400_UART_LABEL_LEN] = '\0';
            printf("[MD400_UART] label preset=%u status=%u len=%u text=%s\r\n",
                   (unsigned)preset, (unsigned)status, (unsigned)label_len, label);
            return 1;
        }
    }

    if ((len >= 3) && (len <= 5) && ((line[0] == 'E') || (line[0] == 'e')) &&
        (line[1] >= '1') && (line[1] <= '4')) {
        event = md400_uart0_rx_event_from_encoder_op(line[2]);
        if (event) {
            u8 repeat = 1;

            if ((len > 3) && !md400_uart0_rx_parse_u7(line, 3, len, &repeat)) {
                repeat = 0;
            }
            control = line[1] - '0';
            if (repeat) {
                md400_control_inject_event(event, control, repeat);
                printf("[MD400_UART] inject raw event=%u control=%u repeat=%u\r\n",
                       (unsigned)event, (unsigned)control, (unsigned)repeat);
                return 1;
            }
        }
    }

    if ((len == 3) && ((line[0] == 'G') || (line[0] == 'g')) &&
        (line[1] >= '1') && (line[1] <= '4')) {
        event = md400_uart0_rx_event_from_single_gesture_op(line[2]);
        if (event) {
            control = line[1] - '0';
            md400_control_inject_event(event, control, 1);
            printf("[MD400_UART] inject gesture=%u control=%u\r\n",
                   (unsigned)event, (unsigned)control);
            return 1;
        }
    }

    if ((len == 4) && ((line[0] == 'G') || (line[0] == 'g'))) {
        control = md400_uart0_rx_combo_mask(line[1], line[2]);
        event = md400_uart0_rx_event_from_combo_gesture_op(line[3]);
        if (control && event) {
            md400_control_inject_event(event, control, 1);
            printf("[MD400_UART] inject combo=%u control=0x%02x\r\n",
                   (unsigned)event, (unsigned)control);
            return 1;
        }
    }

    if ((len >= 2) && ((line[0] == 'P') || (line[0] == 'p'))) {
        u8 value;

        if (md400_uart0_rx_parse_u7(line, 1, len, &value)) {
            md400_control_inject_event(MD400_CONTROL_EVENT_PEDAL_VALUE, 4, value);
            printf("[MD400_UART] inject pedal value=%u\r\n", (unsigned)value);
            return 1;
        }
    }

    if ((len == 2) && ((line[0] == 'K') || (line[0] == 'k'))) {
        if ((line[1] == 'S') || (line[1] == 's')) {
            event = MD400_CONTROL_EVENT_EDIT_SHORT;
        } else if ((line[1] == 'L') || (line[1] == 'l')) {
            event = MD400_CONTROL_EVENT_EDIT_LONG;
        } else if ((line[1] == 'B') || (line[1] == 'b')) {
            event = MD400_CONTROL_EVENT_EDIT_DOUBLE;
        } else {
            event = 0;
        }
        if (event) {
            md400_control_inject_event(event, 0, 1);
            printf("[MD400_UART] inject key event=%u\r\n", (unsigned)event);
            return 1;
        }
    }

    return 0;
}

static void md400_uart0_rx_publish_line(void)
{
    u8 slot;
    u8 i;
    u8 len = md400_uart0_rx_work_len;

    if (!len) {
        md400_uart0_rx_work_len = 0;
        return;
    }

    if (len >= MD400_UART_RX_LINE_MAX) {
        len = MD400_UART_RX_LINE_MAX - 1;
    }

    if ((len == 5) &&
        (md400_uart0_rx_work[0] == 'J') &&
        (md400_uart0_rx_work[1] == 'L') &&
        (md400_uart0_rx_work[2] == 'O') &&
        (md400_uart0_rx_work[3] == 'T') &&
        (md400_uart0_rx_work[4] == 'A')) {
        if (md400_usb_mask_update_schedule_fast()) {
            printf("[MD400_UART] JLOTA fast mask detach\r\n");
        } else {
            printf("[MD400_UART] JLOTA fast busy\r\n");
        }
        md400_uart0_rx_work_len = 0;
        return;
    }

    if (md400_uart0_rx_queue_count >= MD400_UART_RX_QUEUE_DEPTH) {
        md400_uart0_rx_work_len = 0;
        return;
    }

    slot = md400_uart0_rx_queue_wp;
    for (i = 0; i < len; i++) {
        md400_uart0_rx_queue[slot][i] = md400_uart0_rx_work[i];
    }
    md400_uart0_rx_queue[slot][len] = '\0';
    md400_uart0_rx_queue_len[slot] = len;
    md400_uart0_rx_queue_wp = (md400_uart0_rx_queue_wp + 1) & (MD400_UART_RX_QUEUE_DEPTH - 1);
    md400_uart0_rx_queue_count++;
    md400_uart0_rx_work_len = 0;
}

static void md400_uart0_rx_test_cb(u8 *packet, u32 size)
{
    u32 i;

    for (i = 0; i < size; i++) {
        u8 ch = packet[i];

        if ((ch == '\r') || (ch == '\n')) {
            md400_uart0_rx_publish_line();
        } else if ((ch >= 0x20) && (ch <= 0x7e)) {
            if (md400_uart0_rx_work_len < (MD400_UART_RX_LINE_MAX - 1)) {
                md400_uart0_rx_work[md400_uart0_rx_work_len++] = (char)ch;
            }
        }
    }
}

static void md400_uart0_rx_test_poll(void *priv)
{
    char line[MD400_UART_RX_LINE_MAX];
    u8 i;
    u8 len;
    u8 slot;

    (void)priv;

    while (1) {
        local_irq_disable();
        if (!md400_uart0_rx_queue_count) {
            local_irq_enable();
            return;
        }

        slot = md400_uart0_rx_queue_rp;
        len = md400_uart0_rx_queue_len[slot];
        if (len >= MD400_UART_RX_LINE_MAX) {
            len = MD400_UART_RX_LINE_MAX - 1;
        }
        for (i = 0; i < len; i++) {
            line[i] = md400_uart0_rx_queue[slot][i];
        }
        line[len] = '\0';
        md400_uart0_rx_queue_rp = (md400_uart0_rx_queue_rp + 1) & (MD400_UART_RX_QUEUE_DEPTH - 1);
        md400_uart0_rx_queue_count--;
        local_irq_enable();

        printf("[MD400_UART] rx len=%u text=%s\r\n", (unsigned)len, line);

        if (!md400_uart0_rx_handle_command(line)) {
            printf("[MD400_UART] unknown, send HELP\r\n");
        }
    }
}

static void md400_uart0_rx_test_init(void)
{
    if (md400_uart0_rx_timer) {
        return;
    }

    uart_db_regiest_recieve_callback(md400_uart0_rx_test_cb);
    md400_uart0_rx_timer = sys_timer_add(NULL, md400_uart0_rx_test_poll, 50);
    printf("[MD400_UART] rx test enabled tx=PA1 rx=PA6 baud=1500000\r\n");
}
#endif

#if CONFIG_MD400_DISPLAY_ENABLE
static void md400_display_deferred_init(void *priv)
{
    (void)priv;
    md400_display_init_timer = 0;
    printf("[MD400_LCD] deferred init start\r\n");
    md400_display_boot_state_prepare();
    md400_display_test_init();
}
#endif
// 修改日期: 2026-02-17, 修改说明: 参考apps\hid禁用USB0并卸载USB1 host
#if 0
static void usb_dynamic_disable(void)
{
    if (!usb_enabled) {
        return;
    }
    if (usb1_host_mounted) {
        usb_host_unmount(1);
        usb1_host_mounted = 0;
    }
#if TCFG_PC_ENABLE
    extern void usb_pause(void);
    usb_pause();
#endif
    usb_enabled = 0;
}

// 修改日期: 2026-02-17, 修改说明: PB7动态USB使能检测（带防抖）
static void usb_pb7_poll(void *priv)
{
    (void)priv;
    u8 pb7_current = gpio_read(IO_PORTB_07) ? 1 : 0;
    if (pb7_current != pb7_last_stable_state) {
        pb7_debounce_cnt++;
        if (pb7_debounce_cnt >= PB7_DEBOUNCE_THRESHOLD) {
            pb7_last_stable_state = pb7_current;
            pb7_debounce_cnt = 0;
            if (pb7_current) {
                usb_dynamic_enable();
            } else {
                usb_dynamic_disable();
            }
        }
    } else {
        pb7_debounce_cnt = 0;
    }
}
#endif
//---------------------------------------------------------------------
/*************************************************************************************************/
/*!
 *  \brief      进入软关机
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
void spple_power_event_to_user(u8 event)
{
    struct sys_event e;
    e.type = SYS_DEVICE_EVENT;
    e.arg  = (void *)DEVICE_EVENT_FROM_POWER;
    e.u.dev.event = event;
    e.u.dev.value = 0;
    sys_event_notify(&e);
}

static void spple_set_soft_poweroff(void)
{
    log_info("set_soft_poweroff\n");
    is_app_spple_active = 1;
    //必须先主动断开蓝牙链路,否则要等链路超时断开

#if TCFG_USER_BLE_ENABLE
    btstack_ble_exit(0);
#endif

#if TCFG_USER_EDR_ENABLE
    btstack_edr_exit(0);
#endif

#if (TCFG_USER_EDR_ENABLE || TCFG_USER_BLE_ENABLE)
    //延时300ms，确保BT退出链路断开
    sys_timeout_add(NULL, power_set_soft_poweroff, WAIT_DISCONN_TIME_MS);
#else
    power_set_soft_poweroff();
#endif
}

/* #define LED_GPIO_PIN     IO_PORTB_05//IO_PORTB_01//IO_PORTA_00 */
/* static void led_io_init(void) */
/* { */
/* gpio_set_die(LED_GPIO_PIN, 1); */
/* gpio_set_pull_down(LED_GPIO_PIN, 0); */
/* gpio_set_pull_up(LED_GPIO_PIN, 0); */
/* gpio_direction_output(LED_GPIO_PIN, 0); */
/* } */

// cdc send test
static void usb_cdc_send_test()
{
#if TCFG_USB_SLAVE_CDC_ENABLE
    log_info("-send test cdc data-");
    u8 cdc_test_buf[3] = {0x11, 0x22, 0x33};
    cdc_write_data(USB0, cdc_test_buf, 3);
    /* char test_char[] = "cdc test"; */
    /* cdc_write_data(USB0, test_char, sizeof(test_char)-1); */
#endif
}

extern void mem_stats(void);
static void spple_timer_handle_test(void)
{
    log_info("not_bt");
    //	mem_stats();//see memory
    //sys_timer_dump_time();
}

static const ble_init_cfg_t trans_data_ble_config = {
#if DOUBLE_BT_SAME_MAC
    .same_address = 1,
#else
    .same_address = 0,
#endif
    .appearance = 0,
};

/*************************************************************************************************/
/*!
 *  \brief      app start
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static void spple_app_start()
{
    extern int md400_ota_boot_mode_check(void);
    extern int md400_ota_boot_is_ble(void);
    u8 md400_ota_boot;

    log_info("=======================================");
    log_info("-----------spp_and_le demo-------------");
    log_info("=======================================");
    log_info("app_file: %s", __FILE__);

    /* If a firmware OTA armed the flag, this boot runs the loader-download OTA
     * instead of the normal app. Display and preset/UI init are always skipped
     * to free heap for the JL update task (~99 free pages vs ~19 with the full
     * app). Two transports:
     *   USB mode: btstack also skipped; USB-MIDI + NanoBridge pull is the OTA
     *             transport (the RAM-max path).
     *   BLE mode: btstack KEPT UP so the JL RCSP BLE_APP_UPDATA loader-download
     *             runs over BLE; only display + preset UI + USB-host-MIDI are
     *             trimmed. Free pages sit between the two extremes.
     * A failed/aborted OTA reboots back into this untouched bank. */
    md400_ota_boot = md400_ota_boot_mode_check();
    if (md400_ota_boot) {
        printf("[MD400_OTA] OTA boot (%s): display + preset UI skipped%s\r\n",
               md400_ota_boot_is_ble() ? "BLE" : "USB",
               md400_ota_boot_is_ble() ? "" : " + BLE skipped");
    }
    /* Keep BLE up for BLE-transport OTA; skip it for USB-transport OTA. */
    u8 md400_ota_skip_bt = md400_ota_boot && !md400_ota_boot_is_ble();

    if (enter_btstack_num == 0) {
        enter_btstack_num = 1;
        clk_set("sys", BT_NORMAL_HZ);

//有蓝牙
#if (TCFG_USER_EDR_ENABLE || TCFG_USER_BLE_ENABLE)
        u32 sys_clk =  clk_get("sys");
        bt_pll_para(TCFG_CLOCK_OSC_HZ, sys_clk, 0, 0);

        if (!md400_ota_skip_bt) {
#if TCFG_USER_EDR_ENABLE
            btstack_edr_start_before_init(NULL, 0);
#if USER_SUPPORT_PROFILE_HCRP
            __change_hci_class_type(BD_CLASS_PRINTING);
#endif
#if DOUBLE_BT_SAME_MAC
            //手机自带搜索界面，默认搜索到EDR
            __change_hci_class_type(BD_CLASS_TRANSFER_HEALTH);//
#endif
#endif

#if TCFG_USER_BLE_ENABLE
            btstack_ble_start_before_init(&trans_data_ble_config, 0);
#endif

            btstack_init();
        }

#else
//no bt,to for test
        sys_timer_add(NULL, spple_timer_handle_test, 1000);
#endif
    }
    /* 按键消息使能 */
    sys_key_event_enable();
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    md400_board_power_key_scan_start();
#endif
#if TCFG_SOFTOFF_WAKEUP_KEY_DRIVER_ENABLE
    set_key_wakeup_send_flag(1);
#endif
    // 修改日期: 2026-02-17, 修改说明: 参考apps\hid，PB7下拉输入动态控制USB使能
    /* BLE OTA mode is a minimal BLE-RCSP-only image (like the MD001 IAP): keep
     * USB fully off so USB audio/MIDI/host activity can't disrupt BLE link
     * establishment or consume RAM. USB OTA mode still needs USB as transport. */
    if (md400_ota_boot_is_ble()) {
        printf("[MD400_USB] USB skipped (BLE OTA mode)\r\n");
    } else {
#if CONFIG_MD400_USB_EARLY_START
        usb_enabled = 1;
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
        /* the early-start path skips usb_dynamic_enable(), which was the only
         * place the OTG event handler got registered — without it USB1 host
         * insertions are never mounted and host MIDI TX stays queued forever. */
        register_sys_event_handler(SYS_DEVICE_EVENT, DEVICE_EVENT_FROM_OTG, 2,
                                   usb_event_handler);
#endif
        printf("[MD400_USB] app uses early USB start\r\n");
#else
        usb_dynamic_enable();
#endif
    }
#if CONFIG_MD400_DISPLAY_ENABLE
    if (md400_ota_boot) {
        printf("[MD400_LCD] display init skipped (OTA mode)\r\n");
    } else if (!md400_display_init_timer) {
        printf("[MD400_LCD] display init deferred %ums\r\n",
               (unsigned)MD400_DISPLAY_BOOT_DELAY_MS);
        md400_display_init_timer =
            sys_timeout_add(NULL, md400_display_deferred_init,
                            MD400_DISPLAY_BOOT_DELAY_MS);
    }
#else
    printf("[MD400_LCD] display disabled for USB recovery test\r\n");
#endif
#if CONFIG_MD400_UART0_RX_TEST_ENABLE && (TCFG_UART0_RX_PORT != NO_CONFIG_PORT)
    md400_uart0_rx_test_init();
#endif
}
/*************************************************************************************************/
/*!
 *  \brief      app 状态机处理
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static int spple_state_machine(struct application *app, enum app_state state, struct intent *it)
{
    switch (state) {
    case APP_STA_CREATE:
        break;
    case APP_STA_START:
        if (!it) {
            break;
        }
        switch (it->action) {
        case ACTION_SPPLE_MAIN:
            spple_app_start();
            break;
        }
        break;
    case APP_STA_PAUSE:
        break;
    case APP_STA_RESUME:
        break;
    case APP_STA_STOP:
        break;
    case APP_STA_DESTROY:
        log_info("APP_STA_DESTROY\n");
        break;
    }

    return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      hci 事件处理
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static int spple_bt_hci_event_handler(struct bt_event *bt)
{
    //对应原来的蓝牙连接上断开处理函数  ,bt->value=reason
    log_info("----%s reason %x %x", __FUNCTION__, bt->event, bt->value);

#if TCFG_USER_EDR_ENABLE
    bt_comm_edr_hci_event_handler(bt);
#endif

#if TCFG_USER_BLE_ENABLE
    bt_comm_ble_hci_event_handler(bt);
#endif
    return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      bt 连接状态处理
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static int spple_bt_connction_status_event_handler(struct bt_event *bt)
{
    log_info("----%s %d", __FUNCTION__, bt->event);

    switch (bt->event) {
    case BT_STATUS_INIT_OK:
        break;
    default:
        break;
    }

#if TCFG_USER_EDR_ENABLE
    bt_comm_edr_status_event_handler(bt);
#endif

#if TCFG_USER_BLE_ENABLE
    bt_comm_ble_status_event_handler(bt);
#endif
    return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      按键事件处理
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static void spple_key_event_handler(struct sys_event *event)
{
    /* u16 cpi = 0; */
    u8 event_type = 0;
    u8 key_value = 0;

    if (event->arg == (void *)DEVICE_EVENT_FROM_KEY) {
        event_type = event->u.key.event;
        key_value = event->u.key.value;
        log_info("app_key_evnet: %d,%d\n", event_type, key_value);
        /*Change Case To Idle Demo*/
#if CONFIG_APP_SPP_LE_TO_IDLE
        if (event_type == KEY_EVENT_CLICK && key_value == TCFG_ADKEY_VALUE1) {
            log_info(">>>hold key1:chage case to idle\n");
            //change case to idle test
#if TCFG_USER_BLE_ENABLE
            btstack_ble_exit(0);
#endif

#if TCFG_USER_EDR_ENABLE
            btstack_edr_exit(0);
#endif
            app_switch("idle", ACTION_IDLE_MAIN);//切换case
            return;
        }
#endif  /*Btctrler Task Del Enable */

        /*Audio Test Demo*/
#if TCFG_AUDIO_ENABLE
        if (event_type == KEY_EVENT_CLICK && key_value == TCFG_ADKEY_VALUE0) {
            log_info(">>>key0:mic/encode test\n");
            //AC695N/AC696N mic test
            /* extern int audio_adc_open_demo(void); */
            /* audio_adc_open_demo(); */
            //AD697N/AC897N/AC698N mic test
            /* extern void audio_adc_mic_demo(u8 mic_idx, u8 gain, u8 mic_2_dac); */
            /* audio_adc_mic_demo(1, 1, 1); */


            /*encode test*/
            /* extern int audio_mic_enc_open(int (*mic_output)(void *priv, void *buf, int len), u32 code_type); */
            /* audio_mic_enc_open(NULL, AUDIO_CODING_OPUS);//opus encode test */
            /* audio_mic_enc_open(NULL, AUDIO_CODING_SPEEX);//speex encode test */


            /*
            //AC632N
            编码测试类型：
            AUDIO_CODING_LC3
            AUDIO_CODING_USBC
            */

            /* extern int audio_demo_enc_open(int (*demo_output)(void *priv, void *buf, int len), u32 code_type, u8 ai_type); */
            /* audio_demo_enc_open(NULL, AUDIO_CODING_USBC, 0); */



            /*midi test*/

            /* log_info(">>>key0:open midi\n"); */
            // midi_paly_test(KEY_IR_NUM_0);


        }
        if (event_type == KEY_EVENT_CLICK && key_value == TCFG_ADKEY_VALUE1) {
            log_info(">>>key1:tone/decode test\n");
            //AC695N/AC696N tone play test
            /* tone_play_by_path(TONE_NORMAL, 1); */
            /* tone_play_by_path(TONE_BT_CONN, 1); */
            //AD697N/AC897N/AC698N tone play test
            /* tone_play(TONE_NUM_8, 1); */
            /* tone_play(TONE_SIN_NORMAL, 1); */
            /* log_info(">>>key0:set  midi\n"); */
            // midi_paly_test(KEY_IR_NUM_1);



            /*
            //AC632N
            解码测试类型：(需要在audio_decode.c中配置)
            AUDIO_CODING_LC3
            AUDIO_CODING_USBC
             */

            /* extern void demo_frame_test(void); */
            /* demo_frame_test(); */
        }


        if (event_type == KEY_EVENT_CLICK && key_value == TCFG_ADKEY_VALUE2) {
            /* log_info(">>>key2:play  midi\n"); */
            // midi_paly_test(KEY_IR_NUM_2);
        }

#endif/*TCFG_AUDIO_ENABLE*/

        if (event_type == KEY_EVENT_TRIPLE_CLICK
            && (key_value == TCFG_ADKEY_VALUE3 || key_value == TCFG_ADKEY_VALUE0)) {
            //for test
            spple_power_event_to_user(POWER_EVENT_POWER_SOFTOFF);
            return;
        }

        if (event_type == KEY_EVENT_DOUBLE_CLICK && key_value == TCFG_ADKEY_VALUE0) {
#if TCFG_USER_EDR_ENABLE
            //for test
            static u8 edr_en = 1;
            edr_en = !edr_en;
            bt_comm_edr_mode_enable(edr_en);
#endif
        }

        if (event_type == KEY_EVENT_CLICK && key_value == TCFG_ADKEY_VALUE0) {
#if TCFG_USER_BLE_ENABLE
            log_info(">>>test to disconnect\n");
            trans_disconnect();
#endif
        }

        if (event_type == KEY_EVENT_DOUBLE_CLICK && key_value == TCFG_ADKEY_VALUE1) {
#if TCFG_USB_SLAVE_CDC_ENABLE
            log_info(">>>test to cdc send\n");
            usb_cdc_send_test();
#endif

#if TCFG_RTC_ALARM_ENABLE
            log_info(">>>test to rtc_test\n");
            rtc_alarm_set_timer(60);
            spple_power_event_to_user(POWER_EVENT_POWER_SOFTOFF);
#endif
        }

    }
}

/*************************************************************************************************/
/*!
 *  \brief      app 事件处理
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static int spple_event_handler(struct application *app, struct sys_event *event)
{
    switch (event->type) {
    case SYS_KEY_EVENT:
        spple_key_event_handler(event);
        return 0;

    case SYS_BT_EVENT:
#if (TCFG_USER_EDR_ENABLE || TCFG_USER_BLE_ENABLE)
        if ((u32)event->arg == SYS_BT_EVENT_TYPE_CON_STATUS) {
            spple_bt_connction_status_event_handler(&event->u.bt);
        } else if ((u32)event->arg == SYS_BT_EVENT_TYPE_HCI_STATUS) {
            spple_bt_hci_event_handler(&event->u.bt);
        }
#endif
        return 0;

    case SYS_DEVICE_EVENT:
        if ((u32)event->arg == DEVICE_EVENT_FROM_POWER) {
            return app_power_event_handler(&event->u.dev, spple_set_soft_poweroff);
        }

#if TCFG_CHARGE_ENABLE
        else if ((u32)event->arg == DEVICE_EVENT_FROM_CHARGE) {
            app_charge_event_handler(&event->u.dev);
        }
#endif
        return 0;

    default:
        return FALSE;
    }
    return FALSE;
}

static const struct application_operation app_spple_ops = {
    .state_machine  = spple_state_machine,
    .event_handler 	= spple_event_handler,
};

/*
 * 注册AT Module模式
 */
REGISTER_APPLICATION(app_spple) = {
    .name 	= "spp_le",
    .action	= ACTION_SPPLE_MAIN,
    .ops 	= &app_spple_ops,
    .state  = APP_STA_DESTROY,
};

/*************************************************************************************************/
/*!
 *  \brief      注册控制是否允许系统进入sleep状态
 *
 *  \param      [in]
 *
 *  \return     1--可以进入sleep  0--不允许
 *
 *  \note
 */
/*************************************************************************************************/
static u8 spple_state_idle_query(void)
{
    return !is_app_spple_active;
}

REGISTER_LP_TARGET(spple_state_lp_target) = {
    .name = "spple_state_deal",
    .is_idle = spple_state_idle_query,
};

#endif
