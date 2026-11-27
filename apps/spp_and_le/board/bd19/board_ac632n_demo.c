/* Modified for Nexfoot; upstream notices are retained. */
#include "app_config.h"

#ifdef CONFIG_BOARD_AC632N_DEMO

#include "system/includes.h"
#include "device/key_driver.h"
#include "asm/chargestore.h"
#include "asm/charge.h"
#include "asm/power/p33.h"
#include "asm/power_interface.h"
#include "rtc_alarm.h"
#include "asm/pwm_led.h"
#include "user_cfg.h"
#include "usb/otg.h"
#include "norflash.h"
#include "asm/spi.h"

#define LOG_TAG_CONST       BOARD
#define LOG_TAG             "[BOARD]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "debug.h"

#define AT_UART_PORT_ID        3  // wakeup_param 里面的port id

void board_power_init(void);
#if CONFIG_MD400_DISPLAY_ENABLE
extern u16 md400_display_shutdown_start(void);
#endif

#if CONFIG_MD400_DISPLAY_ENABLE
#define MD400_LCD_SPI_CLK   IO_PORTB_06
#define MD400_LCD_SPI_MOSI  IO_PORTB_07
#define MD400_LCD_SPI_BAUD  48000000L

const struct spi_platform_data spi1_p_data = {
    .port = {
        (u8) -1,
        (u8) -1,
        (u8) -1,
        (u8) -1,
        (u8) -1,
    },
    .mode = SPI_MODE_UNIDIR_1BIT,
    .role = SPI_ROLE_MASTER,
    .clk = 1000000L,
};

const struct spi_platform_data spi2_p_data = {
    .port = {
        MD400_LCD_SPI_CLK,
        MD400_LCD_SPI_MOSI,
        (u8) -1,
        (u8) -1,
        (u8) -1,
    },
    .mode = SPI_MODE_UNIDIR_1BIT,
    .role = SPI_ROLE_MASTER,
    .clk = MD400_LCD_SPI_BAUD,
};
#endif

/************************** LOW POWER config ****************************/
const struct low_power_param power_param = {
    .config         = TCFG_LOWPOWER_LOWPOWER_SEL,          //0：sniff时芯片不进入低功耗  1：sniff时芯片进入powerdown
    .btosc_hz       = TCFG_CLOCK_OSC_HZ,                   //外接晶振频率
    .delay_us       = TCFG_CLOCK_SYS_HZ / 1000000L,        //提供给低功耗模块的延时(不需要需修改)
    .btosc_disable  = TCFG_LOWPOWER_BTOSC_DISABLE,         //进入低功耗时BTOSC是否保持
    .vddiom_lev     = TCFG_LOWPOWER_VDDIOM_LEVEL,          //强VDDIO等级,可选：2.0V  2.2V  2.4V  2.6V  2.8V  3.0V  3.2V  3.6V
    .vddiow_lev     = TCFG_LOWPOWER_VDDIOW_LEVEL,          //弱VDDIO等级,可选：2.1V  2.4V  2.8V  3.2V
    .osc_type       = TCFG_LOWPOWER_OSC_TYPE,
    .lpctmu_en 		= TCFG_LP_TOUCH_KEY_ENABLE,
    .vd13_cap_en    = TCFG_VD13_CAP_EN,
#if TCFG_RTC_ALARM_ENABLE
    .rtc_clk        = 1,
#endif
};

/************************** KEY MSG****************************/
/*各个按键的消息设置，如果USER_CFG中设置了USE_CONFIG_KEY_SETTING为1，则会从配置文件读取对应的配置来填充改结构体*/
/* u8 key_table[KEY_NUM_MAX][KEY_EVENT_MAX] = { */
// SHORT           LONG              HOLD              UP              DOUBLE           TRIPLE
/* }; */


// *INDENT-OFF*
/************************** UART config****************************/
#if TCFG_UART0_ENABLE
UART0_PLATFORM_DATA_BEGIN(uart0_data)
    .tx_pin = TCFG_UART0_TX_PORT,                             //串口打印TX引脚选择
    .rx_pin = TCFG_UART0_RX_PORT,                             //串口打印RX引脚选择
    .baudrate = TCFG_UART0_BAUDRATE,                          //串口波特率

    .flags = UART_DEBUG,                                      //串口用来打印需要把改参数设置为UART_DEBUG
UART0_PLATFORM_DATA_END()
#endif //TCFG_UART0_ENABLE


/************************** CHARGE config****************************/
#if TCFG_CHARGE_ENABLE
CHARGE_PLATFORM_DATA_BEGIN(charge_data)
    .charge_en              = TCFG_CHARGE_ENABLE,              //内置充电使能
    .charge_poweron_en      = TCFG_CHARGE_POWERON_ENABLE,      //是否支持充电开机
    .charge_full_V          = TCFG_CHARGE_FULL_V,              //充电截止电压
    .charge_full_mA			= TCFG_CHARGE_FULL_MA,             //充电截止电流
    .charge_mA				= TCFG_CHARGE_MA,                  //充电电流
/*ldo5v拔出过滤值，过滤时间 = (filter*2 + 20)ms,ldoin<0.6V且时间大于过滤时间才认为拔出
 对于充满直接从5V掉到0V的充电仓，该值必须设置成0，对于充满由5V先掉到0V之后再升压到xV的
 充电仓，需要根据实际情况设置该值大小*/
	.ldo5v_off_filter		= 100,
    .ldo5v_on_filter        = 50,
    .ldo5v_keep_filter      = 220,
    .ldo5v_pulldown_lvl     = CHARGE_PULLDOWN_200K,            //下拉电阻档位选择
    .ldo5v_pulldown_keep    = 1,
//1、对于自动升压充电舱,若充电舱需要更大的负载才能检测到插入时，请将该变量置1,并且根据需求配置下拉电阻档位
//2、对于按键升压,并且是通过上拉电阻去提供维持电压的舱,请将该变量设置1,并且根据舱的上拉配置下拉需要的电阻挡位
//3、对于常5V的舱,可将改变量设为0,省功耗
//4、为LDOIN防止被误触发唤醒,可设置为200k下拉
	.ldo5v_pulldown_en		= 1,
CHARGE_PLATFORM_DATA_END()
#endif//TCFG_CHARGE_ENABLE

    /************************** AD KEY ****************************/
#if TCFG_ADKEY_ENABLE
const struct adkey_platform_data adkey_data = {
    .enable = TCFG_ADKEY_ENABLE,                              //AD按键使能
    .adkey_pin = TCFG_ADKEY_PORT,                             //AD按键对应引脚
    .ad_channel = TCFG_ADKEY_AD_CHANNEL,                      //AD通道值
    .extern_up_en = TCFG_ADKEY_EXTERN_UP_ENABLE,              //是否使用外接上拉电阻
    .ad_value = {                                             //根据电阻算出来的电压值
        TCFG_ADKEY_VOLTAGE0,
        TCFG_ADKEY_VOLTAGE1,
        TCFG_ADKEY_VOLTAGE2,
        TCFG_ADKEY_VOLTAGE3,
        TCFG_ADKEY_VOLTAGE4,
        TCFG_ADKEY_VOLTAGE5,
        TCFG_ADKEY_VOLTAGE6,
        TCFG_ADKEY_VOLTAGE7,
        TCFG_ADKEY_VOLTAGE8,
        TCFG_ADKEY_VOLTAGE9,
    },
    .key_value = {                                             //AD按键各个按键的键值
        TCFG_ADKEY_VALUE0,
        TCFG_ADKEY_VALUE1,
        TCFG_ADKEY_VALUE2,
        TCFG_ADKEY_VALUE3,
        TCFG_ADKEY_VALUE4,
        TCFG_ADKEY_VALUE5,
        TCFG_ADKEY_VALUE6,
        TCFG_ADKEY_VALUE7,
        TCFG_ADKEY_VALUE8,
        TCFG_ADKEY_VALUE9,
    },
};
#endif

#if TCFG_IRKEY_ENABLE
const struct irkey_platform_data irkey_data = {
	    .enable = TCFG_IRKEY_ENABLE,                              //IR按键使能
	    .port = TCFG_IRKEY_PORT,                                       //IR按键口
};
#endif

/************************** IO KEY ****************************/
#if TCFG_IOKEY_ENABLE
const struct iokey_port iokey_list[] = {
	{
		.connect_way = TCFG_IOKEY_POWER_CONNECT_WAY,          //IO按键的连接方式
		.key_type.one_io.port = TCFG_IOKEY_POWER_ONE_PORT,    //IO按键对应的引脚
		.key_value = TCFG_IOKEY_POWER_ONE_PORT_VALUE,         //按键值
	},

	{
		.connect_way = TCFG_IOKEY_PREV_CONNECT_WAY,
		.key_type.one_io.port = TCFG_IOKEY_PREV_ONE_PORT,
		.key_value = TCFG_IOKEY_PREV_ONE_PORT_VALUE,
	},

	{
		.connect_way = TCFG_IOKEY_NEXT_CONNECT_WAY,
		.key_type.one_io.port = TCFG_IOKEY_NEXT_ONE_PORT,
		.key_value = TCFG_IOKEY_NEXT_ONE_PORT_VALUE,
	},
};
const struct iokey_platform_data iokey_data = {
	.enable = TCFG_IOKEY_ENABLE,                              //是否使能IO按键
	.num = ARRAY_SIZE(iokey_list),                            //IO按键的个数
	.port = iokey_list,                                       //IO按键参数表
};

#endif  /* #if TCFG_IO_KEY_ENABLE */

/************************** TOUCH_KEY ****************************/
#if TCFG_TOUCH_KEY_ENABLE
const const struct touch_key_port touch_key_list[] = {
    {
	    .press_delta    = TCFG_TOUCH_KEY0_PRESS_DELTA,
        .port           = TCFG_TOUCH_KEY0_PORT,
        .key_value      = TCFG_TOUCH_KEY0_VALUE,
    },
    {
	    .press_delta    = TCFG_TOUCH_KEY1_PRESS_DELTA,
	    .port           = TCFG_TOUCH_KEY1_PORT,
        .key_value      = TCFG_TOUCH_KEY1_VALUE,
    },
};

const struct touch_key_platform_data touch_key_data = {
    .num = ARRAY_SIZE(touch_key_list),
    .port_list = touch_key_list,
};
#endif  /* #if TCFG_TOUCH_KEY_ENABLE */

#if MULT_KEY_ENABLE
//组合按键消息映射表
//配置注意事项:单个按键按键值需要按照顺序编号,如power:0, prev:1, next:2
//bit_value = BIT(0) | BIT(1) 指按键值为0和按键值为1的两个按键被同时按下,
//remap_value = 3指当这两个按键被同时按下后重新映射的按键值;
const struct key_remap iokey_remap_table[] = {
	{.bit_value = BIT(0) | BIT(1), .remap_value = 3},
	{.bit_value = BIT(0) | BIT(2), .remap_value = 4},
	{.bit_value = BIT(1) | BIT(2), .remap_value = 5},
};

const struct key_remap_data iokey_remap_data = {
	.remap_num = ARRAY_SIZE(iokey_remap_table),
	.table = iokey_remap_table,
};
#endif


#if TCFG_RTC_ALARM_ENABLE
const struct sys_time def_sys_time = {  //初始一下当前时间
    .year = 2024,
    .month = 1,
    .day = 1,
    .hour = 0,
    .min = 0,
    .sec = 0,
};
const struct sys_time def_alarm = {     //初始一下目标时间，即闹钟时间
    .year = 2050,
    .month = 1,
    .day = 1,
    .hour = 0,
    .min = 0,
    .sec = 0,
};

extern void alarm_isr_user_cbfun(u8 index);
RTC_DEV_PLATFORM_DATA_BEGIN(rtc_data)
    .default_sys_time = &def_sys_time,
    .default_alarm = &def_alarm,
    .cbfun = NULL,                      //闹钟中断的回调函数,用户自行定义
    /* .cbfun = alarm_isr_user_cbfun, */
    .clk_sel = CLK_SEL_LRC,
    .trim_t = 1,                        //软关机情况下，1min唤醒一次trim lrc
RTC_DEV_PLATFORM_DATA_END()
#endif

/************************** PWM_LED ****************************/
#if TCFG_PWMLED_ENABLE
LED_PLATFORM_DATA_BEGIN(pwm_led_data)
	.io_mode = TCFG_PWMLED_IOMODE,              //推灯模式设置:支持单个IO推两个灯和两个IO推两个灯
	.io_cfg.one_io.pin = TCFG_PWMLED_PIN,       //单个IO推两个灯的IO口配置
LED_PLATFORM_DATA_END()
#endif

/************************** norflash ****************************/
NORFLASH_DEV_PLATFORM_DATA_BEGIN(norflash_fat_dev_data)
    .spi_hw_num     = TCFG_FLASH_DEV_SPI_HW_NUM,
    .spi_cs_port    = TCFG_FLASH_DEV_SPI_CS_PORT,
    .spi_read_width = 4,
#if (TCFG_FLASH_DEV_SPI_HW_NUM == 1)
    .spi_pdata      = &spi1_p_data,
#elif (TCFG_FLASH_DEV_SPI_HW_NUM == 2)
    .spi_pdata      = &spi2_p_data,
#endif
    .start_addr     = 0,
    .size           = 16*1024*1024,
NORFLASH_DEV_PLATFORM_DATA_END()



/************************** otg data****************************/
#if TCFG_OTG_MODE
#if TCFG_OTG1_ENABLE
static struct otg_dev_data usb1_host_otg = {
    .usb_dev_en = BIT(1),
	.slave_online_cnt = TCFG_OTG_SLAVE_ONLINE_CNT,
	.slave_offline_cnt = TCFG_OTG_SLAVE_OFFLINE_CNT,
	.host_online_cnt = TCFG_OTG_HOST_ONLINE_CNT,
	.host_offline_cnt = TCFG_OTG_HOST_OFFLINE_CNT,
	.detect_mode = OTG_HOST_MODE,
	.detect_time_interval = TCFG_OTG_DET_INTERVAL,
};
struct otg_dev_data otg_data = {
    .usb_dev_en = BIT(0),
	.slave_online_cnt = TCFG_OTG_SLAVE_ONLINE_CNT,
	.slave_offline_cnt = TCFG_OTG_SLAVE_OFFLINE_CNT,
	.host_online_cnt = TCFG_OTG_HOST_ONLINE_CNT,
	.host_offline_cnt = TCFG_OTG_HOST_OFFLINE_CNT,
	.detect_mode = OTG_SLAVE_MODE,
	.detect_time_interval = TCFG_OTG_DET_INTERVAL,
	.otg1 = &usb1_host_otg,
};
#else
struct otg_dev_data otg_data = {
    .usb_dev_en = TCFG_OTG_USB_DEV_EN,
	.slave_online_cnt = TCFG_OTG_SLAVE_ONLINE_CNT,
	.slave_offline_cnt = TCFG_OTG_SLAVE_OFFLINE_CNT,
	.host_online_cnt = TCFG_OTG_HOST_ONLINE_CNT,
	.host_offline_cnt = TCFG_OTG_HOST_OFFLINE_CNT,
	.detect_mode = TCFG_OTG_MODE,
	.detect_time_interval = TCFG_OTG_DET_INTERVAL,
};
#endif
#endif

REGISTER_DEVICES(device_table) = {
#if TCFG_OTG_MODE
    { "otg",     &usb_dev_ops, (void *) &otg_data},
#endif
#if TCFG_CHARGE_ENABLE
    { "charge", &charge_dev_ops, (void *)&charge_data },
#endif

};

void debug_uart_init(const struct uart_platform_data *data)
{
#if TCFG_UART0_ENABLE
    if (data) {
        uart_init(data);
    } else {
        uart_init(&uart0_data);
    }
#endif
}

/*其他封装板级，移该函数*/
u8 get_power_on_status(void)
{
#if TCFG_IOKEY_ENABLE
    struct iokey_port *power_io_list = NULL;
    power_io_list = iokey_data.port;

    if (iokey_data.enable) {
        if (gpio_read(power_io_list->key_type.one_io.port) == power_io_list->connect_way){
            return 1;
        }
    }
#endif

#if TCFG_ADKEY_ENABLE
    if (adkey_data.enable) {
        return 1;
    }
#endif

#if TCFG_LP_TOUCH_KEY_ENABLE
	return lp_touch_key_power_on_status();
#endif

    return 0;
}

static u8 md400_power_latch_reported;
static u8 md400_power_latch_release_reported;
static u8 md400_power_key_reported;
static u8 md400_power_key_wait_release;
static u8 md400_power_key_powering_off;
static u8 md400_power_key_pressed;
static u8 md400_power_usb_softoff;
static u8 md400_power_usb_wait_release;
static u16 md400_power_key_hold_ticks;
static u16 md400_power_usb_wake_ticks;
static u16 md400_power_usb_detach_ticks;
static u16 md400_power_shutdown_ticks;
static u16 md400_power_shutdown_delay_ticks;
static u8 md400_power_shutdown_usb_present;
static u16 md400_power_key_timer;

#define MD400_SOFT_POWER_LONG_TICKS \
    ((CONFIG_MD400_SOFT_POWER_LONG_MS + CONFIG_MD400_SOFT_POWER_SCAN_MS - 1) / CONFIG_MD400_SOFT_POWER_SCAN_MS)
#define MD400_SOFT_POWER_WAKE_TICKS \
    ((CONFIG_MD400_SOFT_POWER_WAKE_MS + CONFIG_MD400_SOFT_POWER_SCAN_MS - 1) / CONFIG_MD400_SOFT_POWER_SCAN_MS)
#define MD400_SOFT_POWER_USB_DETACH_TICKS \
    ((CONFIG_MD400_SOFT_POWER_USB_DETACH_MS + CONFIG_MD400_SOFT_POWER_SCAN_MS - 1) / CONFIG_MD400_SOFT_POWER_SCAN_MS)

static const u32 md400_power_led_pins[4] = {
    IO_PORTA_04,
    IO_PORTA_05,
    IO_PORTA_09,
    IO_PORTA_03,
};

static void md400_board_power_gpio_output(u32 port, u8 level)
{
    gpio_set_pull_up(port, 0);
    gpio_set_pull_down(port, 0);
    gpio_set_die(port, 1);
    gpio_direction_output(port, level ? 1 : 0);
}

static void md400_board_power_latch_drive(u8 level)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    md400_board_power_gpio_output(CONFIG_MD400_SOFT_POWER_LATCH_PORT, level ? 1 : 0);
#endif
}

void md400_board_power_latch_hold(void)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    md400_board_power_latch_drive(1);
    if (!md400_power_latch_reported) {
        md400_power_latch_reported = 1;
        printf("[MD400_POWER] hold PWR_CTL PP0 high\r\n");
    }
#endif
}

static void md400_board_power_latch_release(void)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    md400_board_power_latch_drive(0);
    if (!md400_power_latch_release_reported) {
        md400_power_latch_release_reported = 1;
        printf("[MD400_POWER] release PWR_CTL PP0 low\r\n");
    }
#endif
}

static void md400_board_power_key_config_input(void)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    gpio_set_pull_up(CONFIG_MD400_SOFT_POWER_KEY_PORT, 0);
    gpio_set_pull_down(CONFIG_MD400_SOFT_POWER_KEY_PORT, 0);
    gpio_set_die(CONFIG_MD400_SOFT_POWER_KEY_PORT, 1);
    gpio_set_direction(CONFIG_MD400_SOFT_POWER_KEY_PORT, 1);
#endif
}

static u8 md400_board_power_key_is_pressed(void)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    return (gpio_read(CONFIG_MD400_SOFT_POWER_KEY_PORT) ? 1 : 0) ==
           CONFIG_MD400_SOFT_POWER_KEY_ACTIVE_LEVEL;
#else
    return 0;
#endif
}

static u8 md400_board_usb_check_is_present(void)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    gpio_set_pull_up(CONFIG_MD400_USB_CHECK_PORT, 0);
    gpio_set_pull_down(CONFIG_MD400_USB_CHECK_PORT, 1);
    gpio_set_die(CONFIG_MD400_USB_CHECK_PORT, 1);
    gpio_set_direction(CONFIG_MD400_USB_CHECK_PORT, 1);
    return gpio_read(CONFIG_MD400_USB_CHECK_PORT) ? 1 : 0;
#else
    return 0;
#endif
}

static void md400_board_power_visible_off(void)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    u8 i;

    md400_board_power_gpio_output(CONFIG_MD400_SOFT_POWER_LCD_LIGHT_PORT,
                                  CONFIG_MD400_SOFT_POWER_LCD_LIGHT_OFF);
    for (i = 0; i < 4; i++) {
        md400_board_power_gpio_output(md400_power_led_pins[i], 1);
    }
#endif
}

static u8 md400_board_usb_power_present(u32 *mode_out, u8 *vbus_out)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE && TCFG_PC_ENABLE
    u32 mode = usb_otg_online(USB0);
    u8 vbus = md400_board_usb_check_is_present();

    if (mode_out) {
        *mode_out = mode;
    }
    if (vbus_out) {
        *vbus_out = vbus;
    }
    return vbus || ((mode != IDLE_MODE) && (mode != DISCONN_MODE));
#else
    if (mode_out) {
        *mode_out = IDLE_MODE;
    }
    if (vbus_out) {
        *vbus_out = 0;
    }
    return 0;
#endif
}

static void md400_board_usb_runtime_detach(void)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE && TCFG_PC_ENABLE
    extern void usb_stop(void);
    extern void usb_sie_close_all(void);

    usb_stop();
    usb_sie_close_all();
    usb_otg_suspend(USB0, OTG_UNINSTALL);
    usb_otg_io_suspend(USB0);
    usb_iomode(1);
    gpio_set_pull_up(IO_PORT_DP, 0);
    gpio_set_pull_down(IO_PORT_DP, 0);
    gpio_set_direction(IO_PORT_DP, 1);
    gpio_set_die(IO_PORT_DP, 0);
    gpio_set_dieh(IO_PORT_DP, 0);
    gpio_set_pull_up(IO_PORT_DM, 0);
    gpio_set_pull_down(IO_PORT_DM, 0);
    gpio_set_direction(IO_PORT_DM, 1);
    gpio_set_die(IO_PORT_DM, 0);
    gpio_set_dieh(IO_PORT_DM, 0);
#if USB_MAX_HW_NUM > 1
    usb_otg_io_suspend(USB1);
#endif
    printf("[MD400_POWER] USB soft-off forced DP/DM disconnect\r\n");
#endif
}

static void md400_board_power_enter_usb_softoff(void)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    printf("[MD400_POWER] USB soft-off: release latch, drop USB, wait key\r\n");
    md400_power_usb_softoff = 1;
    md400_power_usb_wait_release = 1;
    md400_power_usb_wake_ticks = 0;
    md400_power_usb_detach_ticks = 0;
    md400_power_key_pressed = 0;
    md400_power_key_hold_ticks = 0;

    md400_board_power_visible_off();
    md400_board_power_latch_release();
    md400_board_usb_runtime_detach();
#endif
}

static void md400_board_power_finish_shutdown(void)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    printf("[MD400_POWER] shutdown animation done\r\n");
    if (md400_power_shutdown_usb_present) {
        md400_power_key_powering_off = 0;
        md400_board_power_enter_usb_softoff();
    } else {
        power_set_soft_poweroff();
    }
#endif
}

static void md400_board_power_begin_shutdown(u8 usb_power_present)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    u16 delay_ms = 0;

#if CONFIG_MD400_DISPLAY_ENABLE
    delay_ms = md400_display_shutdown_start();
#endif
    md400_power_shutdown_usb_present = usb_power_present;
    md400_power_shutdown_ticks = 0;
    md400_power_shutdown_delay_ticks =
        (delay_ms + CONFIG_MD400_SOFT_POWER_SCAN_MS - 1) /
        CONFIG_MD400_SOFT_POWER_SCAN_MS;
    md400_power_key_powering_off = 1;
    printf("[MD400_POWER] shutdown delay %ums\r\n", (unsigned)delay_ms);
    if (!md400_power_shutdown_delay_ticks) {
        md400_board_power_finish_shutdown();
    }
#endif
}

static void md400_board_power_usb_softoff_scan(u8 pressed)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    if (md400_power_usb_detach_ticks < MD400_SOFT_POWER_USB_DETACH_TICKS) {
        md400_power_usb_detach_ticks++;
    }

    if (md400_power_usb_wait_release) {
        if (!pressed) {
            md400_power_usb_wait_release = 0;
            md400_power_usb_wake_ticks = 0;
            printf("[MD400_POWER] USB soft-off key released\r\n");
        }
        return;
    }

    if (md400_power_usb_detach_ticks < MD400_SOFT_POWER_USB_DETACH_TICKS) {
        md400_power_usb_wake_ticks = 0;
        return;
    }

    if (pressed) {
        if (md400_power_usb_wake_ticks < 0xffff) {
            md400_power_usb_wake_ticks++;
        }
        if (md400_power_usb_wake_ticks >= MD400_SOFT_POWER_WAKE_TICKS) {
            printf("[MD400_POWER] USB soft-off wake -> reset\r\n");
            md400_board_power_latch_hold();
            cpu_reset();
        }
    } else {
        md400_power_usb_wake_ticks = 0;
    }
#endif
}

static void md400_board_power_key_scan(void *priv)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    u8 pressed;

    (void)priv;
    if (md400_power_key_powering_off) {
        if (md400_power_shutdown_ticks < md400_power_shutdown_delay_ticks) {
            md400_power_shutdown_ticks++;
        }
        if (md400_power_shutdown_ticks >= md400_power_shutdown_delay_ticks) {
            md400_board_power_finish_shutdown();
        }
        return;
    }

    pressed = md400_board_power_key_is_pressed();
    if (md400_power_usb_softoff) {
        md400_board_power_usb_softoff_scan(pressed);
        return;
    }

    if (md400_power_key_wait_release) {
        if (!pressed) {
            md400_power_key_wait_release = 0;
            md400_power_key_pressed = 0;
            md400_power_key_hold_ticks = 0;
            printf("[MD400_POWER] boot hold released\r\n");
        }
        return;
    }

    if (pressed) {
        if (!md400_power_key_pressed) {
            md400_power_key_pressed = 1;
            md400_power_key_hold_ticks = 0;
            printf("[MD400_POWER] key down\r\n");
        } else if (md400_power_key_hold_ticks < 0xffff) {
            md400_power_key_hold_ticks++;
        }

        if (md400_power_key_hold_ticks >= MD400_SOFT_POWER_LONG_TICKS) {
            u32 usb_mode = IDLE_MODE;
            u8 usb_vbus = 0;
            u8 usb_power_present = md400_board_usb_power_present(&usb_mode, &usb_vbus);

            printf("[MD400_POWER] long press %ums -> soft poweroff usb_mode=%lu vbus=%u\r\n",
                   CONFIG_MD400_SOFT_POWER_LONG_MS,
                   (unsigned long)usb_mode,
                   (unsigned)usb_vbus);
            md400_board_power_begin_shutdown(usb_power_present);
        }
    } else {
        if (md400_power_key_pressed) {
            printf("[MD400_POWER] key up %ums\r\n",
                   (u32)md400_power_key_hold_ticks * CONFIG_MD400_SOFT_POWER_SCAN_MS);
        }
        md400_power_key_pressed = 0;
        md400_power_key_hold_ticks = 0;
    }
#endif
}

static u8 md400_board_reset_is_cold_poweron(u32 rst)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    const u32 cold_mask = BIT(MSYS_DVDD_POR_RST) |
                          BIT(P11_PVDD_POR_RST) |
                          BIT(P33_VDDIO_POR_RST) |
                          BIT(P33_PPINR_RST);
    const u32 soft_mask = BIT(MSYS_SOFT_RST) |
                          BIT(P11_SOFT_RST) |
                          BIT(P33_SOFT_RST) |
                          BIT(P33_EXCEPTION_SOFT_RST) |
                          BIT(P33_ASSERT_SOFT_RST) |
                          BIT(MSYS_POWER_RETURN) |
                          BIT(P11_POWER_RETURN) |
                          BIT(P33_POWER_RETURN);

    if (rst & soft_mask) {
        return 0;
    }
    if (rst & cold_mask) {
        return 1;
    }
    return (rst == MSYS_DVDD_POR_RST) ||
           (rst == P11_PVDD_POR_RST) ||
           (rst == P33_VDDIO_POR_RST) ||
           (rst == P33_PPINR_RST);
#else
    return 0;
#endif
}

#define MD400_WDT_BOOT_GUARD_LIMIT       3
#define MD400_WDT_BOOT_STABLE_MS         30000

/* JL clears normal/non-volatile RAM, PMU soft flags, and the P11 mailboxes on
 * this watchdog path. The VM is initialized before app_main(), and this boot
 * gate runs on app_core (the same safe owner used by product persistence), so
 * reserve one user VM ID for the consecutive-reset guard. */
#define MD400_VM_WDT_BOOT_GUARD_ID       249
#define MD400_WDT_VM_RECORD_VERSION      1

typedef struct {
    u8 magic[4];
    u8 version;
    u8 count;
    u8 count_inv;
    u8 checksum;
} md400_wdt_vm_record_t;

static const u8 md400_wdt_vm_magic[4] = {'N', 'F', 'W', 'D'};

static u8 md400_board_wdt_boots;
static u8 md400_board_wdt_uboot_pending;
static u32 md400_board_boot_reset_source;

u8 md400_board_wdt_boot_guard_count(void)
{
    return md400_board_wdt_boots;
}

u32 md400_board_boot_reset_source_get(void)
{
    return md400_board_boot_reset_source;
}

static u8 md400_board_reset_is_wdt(u32 rst)
{
    return (rst == P11_WDT_RST) || !!(rst & BIT(P11_WDT_RST));
}

static u8 md400_board_wdt_vm_checksum(const md400_wdt_vm_record_t *record)
{
    const u8 *data = (const u8 *)record;
    u8 checksum = 0;
    u8 i;

    for (i = 0; i < sizeof(*record) - 1; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

static u8 md400_board_wdt_vm_read(u8 *count)
{
    md400_wdt_vm_record_t record;
    int ret = syscfg_read(MD400_VM_WDT_BOOT_GUARD_ID, &record,
                          sizeof(record));

    if ((ret != sizeof(record)) ||
        memcmp(record.magic, md400_wdt_vm_magic,
               sizeof(md400_wdt_vm_magic)) ||
        (record.version != MD400_WDT_VM_RECORD_VERSION) ||
        ((u8)(record.count ^ record.count_inv) != 0xff) ||
        (record.count > MD400_WDT_BOOT_GUARD_LIMIT) ||
        (record.checksum != md400_board_wdt_vm_checksum(&record))) {
        return 0;
    }
    *count = record.count;
    return 1;
}

static int md400_board_wdt_vm_write(u8 count)
{
    md400_wdt_vm_record_t record;

    memcpy(record.magic, md400_wdt_vm_magic, sizeof(record.magic));
    record.version = MD400_WDT_VM_RECORD_VERSION;
    record.count = count;
    record.count_inv = ~count;
    record.checksum = md400_board_wdt_vm_checksum(&record);
    return syscfg_write(MD400_VM_WDT_BOOT_GUARD_ID, &record,
                        sizeof(record));
}

static u8 md400_board_wdt_boot_count(u32 rst)
{
    u8 count = 0;
    u8 valid = md400_board_wdt_vm_read(&count);
    int ret;

    if (md400_board_reset_is_wdt(rst)) {
        if (!valid) {
            count = 0;
        }
        if (count < MD400_WDT_BOOT_GUARD_LIMIT) {
            count++;
        }
        /* Keep the count-2 record until the live USB stack schedules UBOOT.
         * A crash before that point therefore retries the escape next boot. */
        if (count < MD400_WDT_BOOT_GUARD_LIMIT) {
            ret = md400_board_wdt_vm_write(count);
            if (ret != sizeof(md400_wdt_vm_record_t)) {
                printf("[MD400_BOOT] watchdog count write failed ret=%d\r\n",
                       ret);
            }
        }
    } else if (valid && count) {
        ret = md400_board_wdt_vm_write(0);
        if (ret != sizeof(md400_wdt_vm_record_t)) {
            printf("[MD400_BOOT] watchdog count clear failed ret=%d\r\n",
                   ret);
        }
        count = 0;
    }
    md400_board_wdt_boots = count;
    return count;
}

static void md400_board_wdt_boot_stable(void *priv)
{
    extern int md400_persist_wdt_guard_clear_async(void);

    (void)priv;
    if (!md400_board_wdt_boots) {
        return;
    }
    if (md400_persist_wdt_guard_clear_async()) {
        sys_timeout_add(NULL, md400_board_wdt_boot_stable, 100);
    }
}

void md400_board_wdt_boot_guard_clear_now(void)
{
    int ret;

    if (!md400_board_wdt_boots) {
        return;
    }
    printf("[MD400_BOOT] stable %ums -> clear watchdog count=%u\r\n",
           (unsigned)MD400_WDT_BOOT_STABLE_MS,
           (unsigned)md400_board_wdt_boots);
    ret = md400_board_wdt_vm_write(0);
    if (ret == sizeof(md400_wdt_vm_record_t)) {
        md400_board_wdt_boots = 0;
    } else {
        printf("[MD400_BOOT] stable clear failed ret=%d\r\n", ret);
    }
}

/* Clear the third-WDT record only when app_core has actually entered the UBOOT
 * handoff. If timer/task posting fails first, the count stays armed and the
 * next watchdog boot retries recovery instead of returning to a dead loop. */
void md400_board_wdt_uboot_handoff_commit(void)
{
    int ret;

    if (!md400_board_wdt_uboot_pending) {
        return;
    }
    ret = md400_board_wdt_vm_write(0);
    if (ret == sizeof(md400_wdt_vm_record_t)) {
        md400_board_wdt_boots = 0;
        md400_board_wdt_uboot_pending = 0;
        printf("[MD400_BOOT] watchdog UBOOT handoff committed\r\n");
    } else {
        printf("[MD400_BOOT] watchdog UBOOT commit failed ret=%d\r\n", ret);
    }
}

static void md400_board_wdt_uboot_retry(void *priv)
{
    extern u8 md400_usb_mask_update_schedule_fast(void);

    (void)priv;
    if (!md400_board_wdt_uboot_pending) {
        return;
    }
    if (md400_usb_mask_update_schedule_fast()) {
        printf("[MD400_BOOT] live-USB UBOOT handoff scheduled\r\n");
    } else {
        printf("[MD400_BOOT] live-USB UBOOT handoff schedule retry\r\n");
        sys_timeout_add(NULL, md400_board_wdt_uboot_retry, 100);
    }
}

void md400_board_power_usb_plug_boot_gate(void)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE && CONFIG_MD400_SOFT_POWER_USB_GATE_ENABLE
    extern void clr_wdt(void);
    u16 held_ms = 0;
    u32 rst = get_reset_source_value();
    u8 wdt_boots = md400_board_wdt_boot_count(rst);

    md400_board_boot_reset_source = rst;

    if (wdt_boots >= MD400_WDT_BOOT_GUARD_LIMIT) {
        md400_board_wdt_uboot_pending = 1;
        printf("[MD400_BOOT] watchdog loop rst=0x%lx count=%u -> schedule UBOOT\r\n",
               (unsigned long)rst, (unsigned)wdt_boots);
    }
    if (wdt_boots) {
        printf("[MD400_BOOT] watchdog recovery rst=0x%lx count=%u/%u\r\n",
               (unsigned long)rst, (unsigned)wdt_boots,
               (unsigned)MD400_WDT_BOOT_GUARD_LIMIT);
    }

    md400_board_power_key_config_input();
    if (wdt_boots || !md400_board_reset_is_cold_poweron(rst) ||
        md400_board_power_key_is_pressed()) {
        printf("[MD400_POWER] boot gate pass rst=0x%lx key=%u\r\n",
               (unsigned long)rst,
               (unsigned)md400_board_power_key_is_pressed());
        return;
    }

    printf("[MD400_POWER] cold boot without key -> USB/off wait rst=0x%lx\r\n",
           (unsigned long)rst);
    md400_board_power_visible_off();
    md400_board_power_latch_hold();

    while (1) {
        clr_wdt();
        if (md400_board_power_key_is_pressed()) {
            held_ms += 10;
            if (held_ms >= CONFIG_MD400_SOFT_POWER_WAKE_MS) {
                printf("[MD400_POWER] USB/off key boot\r\n");
                md400_board_power_latch_hold();
                md400_power_key_wait_release = 1;
                return;
            }
        } else {
            held_ms = 0;
        }
        os_time_dly(1);
    }
#endif
}

u8 md400_board_power_is_softoff(void)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    return md400_power_usb_softoff;
#else
    return 0;
#endif
}

void md400_board_power_key_scan_start(void)
{
#if CONFIG_MD400_SOFT_POWER_LATCH_ENABLE
    if (md400_power_key_timer) {
        return;
    }

    md400_board_power_key_config_input();
    md400_power_key_wait_release = md400_board_power_key_is_pressed();
    md400_power_key_timer = sys_timer_add(NULL, md400_board_power_key_scan,
                                          CONFIG_MD400_SOFT_POWER_SCAN_MS);
    if (md400_board_wdt_uboot_pending) {
        md400_board_wdt_uboot_retry(NULL);
    }
    if (md400_board_wdt_boots && !md400_board_wdt_uboot_pending) {
        sys_timeout_add(NULL, md400_board_wdt_boot_stable,
                        MD400_WDT_BOOT_STABLE_MS);
    }
    if (!md400_power_key_reported) {
        md400_power_key_reported = 1;
        printf("[MD400_POWER] key scan ON/OFF PB1 active=%u wait_release=%u\r\n",
               CONFIG_MD400_SOFT_POWER_KEY_ACTIVE_LEVEL,
               md400_power_key_wait_release);
    }
#endif
}


static void board_devices_init(void)
{
    md400_board_power_latch_hold();

#if TCFG_PWMLED_ENABLE
    pwm_led_init(&pwm_led_data);
#endif

#if (TCFG_IOKEY_ENABLE || TCFG_ADKEY_ENABLE || TCFG_IRKEY_ENABLE || TCFG_TOUCH_KEY_ENABLE)
	key_driver_init();
#endif

#if (!TCFG_CHARGE_ENABLE)
    CHARGE_EN(0);
#endif

#if TCFG_RTC_ALARM_ENABLE
    alarm_init(&rtc_data);
#endif

}

//maskrom 使用到的io
static void mask_io_cfg_with_ldoin(u8 ldoin_stage)
{
    struct boot_soft_flag_t boot_soft_flag = {0};
    boot_soft_flag.flag0.boot_ctrl.wdt_dis = 0;
    boot_soft_flag.flag0.boot_ctrl.poweroff = 0;
    boot_soft_flag.flag0.boot_ctrl.is_port_b = JL_IOMAP->CON0 & BIT(16) ? 1 : 0;

    boot_soft_flag.flag1.misc.usbdm = SOFTFLAG_HIGH_RESISTANCE;
    boot_soft_flag.flag1.misc.usbdp = SOFTFLAG_HIGH_RESISTANCE;

    boot_soft_flag.flag1.misc.uart_key_port = 0;
    boot_soft_flag.flag1.misc.ldoin = ldoin_stage;

    boot_soft_flag.flag2.pa7_pb4.pa7 = SOFTFLAG_HIGH_RESISTANCE;
    boot_soft_flag.flag2.pa7_pb4.pb4 = SOFTFLAG_HIGH_RESISTANCE;

    boot_soft_flag.flag3.pc3_pc5.pc3 = SOFTFLAG_HIGH_RESISTANCE;
    boot_soft_flag.flag3.pc3_pc5.pc5 = SOFTFLAG_HIGH_RESISTANCE;
    mask_softflag_config(&boot_soft_flag);
}

static void mask_io_cfg()
{
    mask_io_cfg_with_ldoin(SOFTFLAG_HIGH_RESISTANCE);
}

static void mask_io_cfg_keep_latch()
{
    mask_io_cfg_with_ldoin(SOFTFLAG_OUT1);
}

void md400_board_prepare_mask_usb_update(void)
{
    mask_io_cfg_keep_latch();

    printf("[MD400_OTA] assert PWR_CTL PP0 high for mask\r\n");
    md400_board_power_latch_hold();
    os_time_dly(2);
}


extern void cfg_file_parse(u8 idx);
void board_init()
{
	board_power_init();
    md400_board_power_latch_hold();
	adc_vbg_init();
	adc_init();
	cfg_file_parse(0);
	devices_init();

	board_devices_init();

    #if TCFG_CHARGE_ENABLE && TCFG_HANDSHAKE_ENABLE
    if(get_charge_online_flag()){
        handshake_app_start(0, NULL);
    }
#endif

	if(get_charge_online_flag()) {
		power_set_mode(PWR_LDO15);
	} else {
		power_set_mode(TCFG_LOWPOWER_POWER_SEL);
	}

	/*close FAST CHARGE */
#if TCFG_UART0_ENABLE
	if (uart0_data.rx_pin < IO_MAX_NUM) {
		gpio_set_die(uart0_data.rx_pin, 1);
	}
#endif

#if USER_UART_UPDATE_ENABLE
        {
#include "uart_update.h"
            uart_update_cfg  update_cfg = {
                .rx = UART_UPDATE_RX_PORT,
                .tx = UART_UPDATE_TX_PORT,
                .output_channel = CH1_UT1_TX,
                .input_channel = INPUT_CH0
            };
            uart_update_init(&update_cfg);
        }
#endif
}

enum {
    PORTA_GROUP = 0,
    PORTB_GROUP,
    PORTC_GROUP,
};

static void port_protect(u16 *port_group, u32 port_num)
{
    if (port_num == NO_CONFIG_PORT) {
        return;
    }
    port_group[port_num / IO_GROUP_NUM] &= ~BIT(port_num % IO_GROUP_NUM);
}

void usb1_iomode(u32 enable);
/*进软关机之前默认将IO口都设置成高阻状态，需要保留原来状态的请修改该函数*/
static void close_gpio(u8 is_softoff)
{
    u16 port_group[] = {
        [PORTA_GROUP] = 0x1ff,
        [PORTB_GROUP] = 0x3ff,//
        [PORTC_GROUP] = 0x3ff,//
    };

	if(P3_ANA_CON2 & BIT(3))
	{
		port_protect(port_group, IO_PORTB_02);	//protect VCM_IO
	}

	if(P3_PINR_CON & BIT(0))
	{
		u8 port_sel = P3_PORT_SEL0;
		if((port_sel >= 1) && (port_sel <= 10)){
			port_sel = IO_GROUP_NUM * 0 + port_sel - 1;
			port_protect(port_group, port_sel);				//protect 长按复位
		}else if((port_sel >= 11) && (port_sel <= 20)){
			port_sel = IO_GROUP_NUM * 1 + port_sel - 11;
			port_protect(port_group, port_sel);				//protect 长按复位
		}else if((port_sel >= 21) && (port_sel <= 25)){
			port_sel = IO_GROUP_NUM * 2 + port_sel - 21;
			port_protect(port_group, port_sel);				//protect 长按复位
		}else if(port_sel == 26){
			port_protect(port_group, IO_PORT_DP);			//protect 长按复位
		}else if(port_sel == 27){
			port_protect(port_group, IO_PORT_DM);			//protect 长按复位
		}else if(port_sel == 28){
			port_protect(port_group, IO_PORT_DP1);			//protect 长按复位
		}else if(port_sel == 29){
			port_protect(port_group, IO_PORT_DM1);			//protect 长按复位
		}
	}

#if TCFG_ADKEY_ENABLE
    port_protect(port_group,TCFG_ADKEY_PORT);
#endif /* */

#if TCFG_IOKEY_ENABLE
    port_protect(port_group, TCFG_IOKEY_POWER_ONE_PORT);
    port_protect(port_group, TCFG_IOKEY_PREV_ONE_PORT);
    port_protect(port_group, TCFG_IOKEY_NEXT_ONE_PORT);
#endif /* TCFG_IOKEY_ENABLE */

#if TCFG_RTC_ALARM_ENABLE
    /* port_protect(port_group, IO_PORTA_01); */
    /* port_protect(port_group, IO_PORTA_02); */
#endif /* TCFG_RTC_ALARM_ENABLE */

#if TCFG_CHARGE_ENABLE && TCFG_HANDSHAKE_ENABLE
    if (is_softoff == 0) {
        port_protect(port_group, TCFG_HANDSHAKE_IO_DATA1);
        port_protect(port_group, TCFG_HANDSHAKE_IO_DATA2);
    }
#endif

#if CONFIG_APP_AT_CHAR_COM || CONFIG_APP_AT_COM
    port_protect(port_group, UART_DB_TX_PIN);
    port_protect(port_group, UART_DB_RX_PIN);

#if FLOW_CONTROL
    port_protect(port_group, UART_DB_RTS_PIN);
    port_protect(port_group, UART_DB_CTS_PIN);
#endif

#endif

    //< close gpio
    gpio_dir(GPIOA, 0, 9, port_group[PORTA_GROUP], GPIO_OR);
    gpio_set_pu(GPIOA, 0, 9, ~port_group[PORTA_GROUP], GPIO_AND);
    gpio_set_pd(GPIOA, 0, 9, ~port_group[PORTA_GROUP], GPIO_AND);
    gpio_die(GPIOA, 0, 9, ~port_group[PORTA_GROUP], GPIO_AND);
    gpio_dieh(GPIOA, 0, 9, ~port_group[PORTA_GROUP], GPIO_AND);

    gpio_dir(GPIOB, 0, 10, port_group[PORTB_GROUP], GPIO_OR);
    gpio_set_pu(GPIOB, 0, 10, ~port_group[PORTB_GROUP], GPIO_AND);
    gpio_set_pd(GPIOB, 0, 10, ~port_group[PORTB_GROUP], GPIO_AND);
    gpio_die(GPIOB, 0, 10, ~port_group[PORTB_GROUP], GPIO_AND);
    gpio_dieh(GPIOB, 0, 10, ~port_group[PORTB_GROUP], GPIO_AND);

    //< close usb io
    usb_iomode(1);
    gpio_set_pull_up(IO_PORT_DP, 0);
    gpio_set_pull_down(IO_PORT_DP, 0);
    gpio_set_direction(IO_PORT_DP, 1);
    gpio_set_die(IO_PORT_DP, 0);
    gpio_set_dieh(IO_PORT_DP, 0);

    gpio_set_pull_up(IO_PORT_DM, 0);
    gpio_set_pull_down(IO_PORT_DM, 0);
    gpio_set_direction(IO_PORT_DM, 1);
    gpio_set_die(IO_PORT_DM, 0);
    gpio_set_dieh(IO_PORT_DM, 0);

#if !(TCFG_OTG_USB_DEV_EN & BIT(1))
    usb1_iomode(1);
    gpio_set_pull_up(IO_PORT_DP1, 0);
    gpio_set_pull_down(IO_PORT_DP1, 0);
    gpio_set_direction(IO_PORT_DP1, 1);
    gpio_set_die(IO_PORT_DP1, 0);
    gpio_set_dieh(IO_PORT_DP1, 0);

    gpio_set_pull_up(IO_PORT_DM1, 0);
    gpio_set_pull_down(IO_PORT_DM1, 0);
    gpio_set_direction(IO_PORT_DM1, 1);
    gpio_set_die(IO_PORT_DM1, 0);
    gpio_set_dieh(IO_PORT_DM1, 0);
#endif

    /* printf("JL_USB_IO->CON0=0x%x\r\n", JL_USB_IO->CON0); */
    /* printf("JL_USB_IO->CON1=0x%x\r\n", JL_USB_IO->CON1); */
    /* printf("JL_USB->CON0=0x%x\r\n", JL_USB->CON0); */
    /*  */
    /* printf("JL_USB1_IO->CON0=0x%x\r\n", JL_USB1_IO->CON0); */
    /* printf("JL_USB1_IO->CON1=0x%x\r\n", JL_USB1_IO->CON1); */
    /* printf("JL_USB1->CON0=0x%x\r\n", JL_USB1->CON0); */
}

/************************** PWR config ****************************/
struct port_wakeup port0 = {
	.pullup_down_enable = ENABLE,                            //配置I/O 内部上下拉是否使能
	.edge               = FALLING_EDGE,                      //唤醒方式选择,可选：上升沿\下降沿
    .both_edge          = 0,

#if TCFG_ADKEY_ENABLE
	.iomap              = TCFG_ADKEY_PORT,                   //唤醒口选择
#else
	.iomap              = TCFG_IOKEY_POWER_ONE_PORT,         //唤醒口选择
#endif
    .filter             = PORT_FLT_2ms,
};

#if TCFG_TEST_BOX_ENABLE
struct port_wakeup port1 = {
    .pullup_down_enable = DISABLE,                            //配置I/O 内部上下拉是否使能
    .edge               = FALLING_EDGE,                      //唤醒方式选择,可选：上升沿\下降沿
    .both_edge          = 1,
    .filter             = PORT_FLT_1ms,
    .iomap              = TCFG_CHARGESTORE_PORT,             //唤醒口选择
};
#endif

#if 0//USER_UART_UPDATE_ENABLE
struct port_wakeup port1 = {
	.pullup_down_enable = ENABLE,                            //配置I/O 内部上下拉是否使能
	.edge               = FALLING_EDGE,                      //唤醒方式选择,可选：上升沿\下降沿
    .both_edge          = 0,
	.iomap              = UART_UPDATE_RX_PORT,               //唤醒口选择
    .filter             = PORT_FLT_2ms,
};
#endif

#if TCFG_CHARGE_ENABLE
struct port_wakeup charge_port = {
    .edge               = RISING_EDGE,                       //唤醒方式选择,可选：上升沿\下降沿\双边沿
    .both_edge          = 0,
    .filter             = PORT_FLT_16ms,
    .iomap              = IO_CHGFL_DET,                      //唤醒口选择
};

struct port_wakeup vbat_port = {
    .edge               = BOTH_EDGE,                      //唤醒方式选择,可选：上升沿\下降沿\双边沿
    .both_edge          = 1,
    .filter             = PORT_FLT_16ms,
    .iomap              = IO_VBTCH_DET,                      //唤醒口选择
};

struct port_wakeup ldoin_port = {
    .edge               = BOTH_EDGE,                      //唤醒方式选择,可选：上升沿\下降沿\双边沿
    .both_edge          = 1,
    .filter             = PORT_FLT_16ms,
    .iomap              = IO_LDOIN_DET,                      //唤醒口选择
};
#endif

struct port_wakeup at_uart_port = {
	.pullup_down_enable = ENABLE,                            //配置I/O 内部上下拉是否使能
	.edge               = FALLING_EDGE,                      //唤醒方式选择,可选：上升沿\下降沿
    .both_edge          = 0,
	.iomap              = UART_DB_RX_PIN,                   //唤醒口选择
    .filter             = PORT_FLT_256us,
};


const struct wakeup_param wk_param = {

#if TCFG_ADKEY_ENABLE || TCFG_IOKEY_ENABLE
	.port[1]    = &port0,
#endif
	/* .sub        = &sub_wkup, */
	/* .charge     = &charge_wkup, */

#if TCFG_TEST_BOX_ENABLE
	.port[2] = &port1,
#endif

#if 0//USER_UART_UPDATE_ENABLE
	.port[2] = &port1,
#endif

#if CONFIG_APP_AT_CHAR_COM || CONFIG_APP_AT_COM
    /*at串口唤醒,至少要发多个00 hex数据,demo只做软关机唤醒*/
    .port[AT_UART_PORT_ID] = &at_uart_port,
#endif

#if TCFG_CHARGE_ENABLE
    .aport[0] = &charge_port,
    .aport[1] = &vbat_port,
    .aport[2] = &ldoin_port,
#endif

};

//-----------------------------------------------


/*进软关机之前默认将IO口都设置成高阻状态，需要保留原来状态的请修改该函数*/
extern void dac_power_off(void);
void board_set_soft_poweroff(void)
{
	log_info("%s",__FUNCTION__);
	mask_io_cfg();
    md400_board_power_latch_release();

#if TCFG_TEST_BOX_ENABLE
	power_wakeup_index_disable(2);
#endif

#if CONFIG_APP_AT_CHAR_COM || CONFIG_APP_AT_COM
	power_wakeup_index_enable(AT_UART_PORT_ID);
#endif

	close_gpio(1);
}

#define     APP_IO_DEBUG_0(i,x)       //{JL_PORT##i->DIR &= ~BIT(x), JL_PORT##i->OUT &= ~BIT(x);}
#define     APP_IO_DEBUG_1(i,x)       //{JL_PORT##i->DIR &= ~BIT(x), JL_PORT##i->OUT |= BIT(x);}


void sleep_exit_callback(u32 usec)
{
	putchar('>');
    APP_IO_DEBUG_0(A, 5);
}

void sleep_enter_callback(u8  step)
{
	/* 此函数禁止添加打印 */
	if (step == 1) {
		putchar('<');
		APP_IO_DEBUG_1(A, 5);
		/*dac_power_off();*/
	} else {
		close_gpio(0);
	}
}

static void wl_audio_clk_on(void)
{
    JL_WL_AUD->CON0 = 1;
}

//-----------------------
//system check go sleep is ok
#if CONFIG_APP_AT_CHAR_COM || CONFIG_APP_AT_COM
static u8 board_is_active;
static u8 board_state_idle_query(void)
{
    return !board_is_active;
}

REGISTER_LP_TARGET(board_lp_target) = {
    .name = "board_state",
    .is_idle = board_state_idle_query,
};

static u8 board_time_to_idle(void)
{
    board_is_active = 0;
}

void board_at_uart_wakeup_enalbe(u8 enalbe)
{
    if(enalbe){
        power_wakeup_index_enable(AT_UART_PORT_ID);
    }
    else{
        power_wakeup_index_disable(AT_UART_PORT_ID);
    }
}
#endif

static void port_wakeup_callback(u8 index, u8 gpio)
{
	/* log_info("%s:%d,%d",__FUNCTION__,index,gpio); */

	switch (index) {
#if TCFG_TEST_BOX_ENABLE
		case 2:
			extern void chargestore_ldo5v_fall_deal(void);
			chargestore_ldo5v_fall_deal();
			break;
#endif

#if CONFIG_APP_AT_CHAR_COM || CONFIG_APP_AT_COM
        case AT_UART_PORT_ID:
            if(!board_is_active){
                board_is_active = 1;
                putchar('W');
                sys_timeout_add(0,board_time_to_idle,10000);//delay 给uart 指令退出低功耗
            }
            break;
#endif
        default:
            break;

    }
}

static void aport_wakeup_callback(u8 index, u8 gpio, u8 edge)
{
#if TCFG_CHARGE_ENABLE
    switch (gpio) {
        case IO_CHGFL_DET://charge port
            charge_wakeup_isr();
            break;
        case IO_VBTCH_DET://vbat port
        case IO_LDOIN_DET://ldoin port
            ldoin_wakeup_isr();
            break;
    }
#endif
}

void board_power_init(void)
{
    log_info("Power init : %s", __FILE__);

    md400_board_power_latch_hold();

    power_init(&power_param);
    md400_board_power_latch_hold();

    gpio_longpress_pin0_reset_config(IO_PORTA_09, 0, 0);
    gpio_shortpress_reset_config(0);//1--enable 0--disable
    //< close short key reset
    /* power_mclr(0); */
    //< close long key reset
    /* power_pin_reset(0); */

    power_set_callback(TCFG_LOWPOWER_LOWPOWER_SEL, sleep_enter_callback, sleep_exit_callback, board_set_soft_poweroff);

//    wl_audio_clk_on();

    power_keep_dacvdd_en(0);

	power_wakeup_init(&wk_param);
    md400_board_power_latch_hold();

    aport_edge_wkup_set_callback(aport_wakeup_callback);
    port_edge_wkup_set_callback(port_wakeup_callback);

#if CONFIG_APP_AT_CHAR_COM || CONFIG_APP_AT_COM
    power_wakeup_index_disable(AT_UART_PORT_ID);
#endif
	/* #if (!TCFG_IOKEY_ENABLE && !TCFG_ADKEY_ENABLE) */
    /* charge_check_and_set_pinr(0); */
/* #endif */
}
#endif
