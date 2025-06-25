/**
 * @file bt_task.c
 * @brief
 * @author JoyLee
 * @version 2.0.0
 * @date 2023-04-06
 *
 * @copyright Copyright (c) 2023 Westberry Technology Corp., Ltd
 */

#include "common/bts_lib.h"
#include "config.h"
#include "gpio.h"
#include "m78.h"
#include "quantum.h"
#include "rgb_matrix.h"
#include "uart.h"
#include "usb_main.h"
#include "common/bt_task.h"

#define NUM_LONG_PRESS_KEYS (sizeof(long_pressed_keys) / sizeof(long_pressed_keys_t))

#ifdef BT_DEBUG_MODE
#    define BT_DEBUG_INFO(fmt, ...) dprintf(fmt, ##__VA_ARGS__)
#else
#    define BT_DEBUG_INFO(fmt, ...)
#endif

// ===========================================
// 函数声明
// ===========================================
static void long_pressed_keys_hook(void);
static void long_pressed_keys_cb(uint16_t keycode);
static bool process_record_other(uint16_t keycode, keyrecord_t *record);
static void bt_scan_mode(void);
static void bt_used_pin_init(void);
static void update_low_voltage_state(void);
static void handle_factory_reset_display(void);
static void handle_blink_effects(void);
static void handle_layer_indication(void);
static void handle_charging_indication(void);
static void handle_low_battery_warning(void);
static void handle_low_battery_shutdow(void);
static void handle_battery_query_display(void);
static void handle_bt_indicate_led(void);
static void handle_usb_indicate_led(void);
#ifdef RGB_MATRIX_ENABLE
static void led_off_standby(void);
static void open_rgb(void);
static void close_rgb(void);
#endif

extern keymap_config_t keymap_config;

// ===========================================
// 结构体定义
// ===========================================

// 低电量警告状态结构体 (20%以下红色闪烁6次)
typedef struct {
    bool     triggered;
    uint8_t  blink_count;
    uint32_t blink_time;
    bool     blink_state;
    bool     completed;
} low_battery_warning_t;

// 充电完成指示状态结构体 (绿色闪烁5次)
typedef struct {
    bool     triggered;
    uint8_t  blink_count;
    uint32_t blink_time;
    bool     blink_state;
    bool     completed;
} charge_complete_warning_t;

// 低电量背光控制状态结构体 (5%以下关闭背光)
typedef struct {
    bool     forced_rgb_off;
    bool     is_low_voltage;
    uint8_t  saved_rgb_mode;
    uint8_t  read_vol_count;
    uint32_t last_normal_vol_time;
    bool     waiting_recovery;
} low_voltage_state_t;

typedef struct {
    uint32_t press_time;
    uint16_t keycode;
    void (*event_cb)(uint16_t);
} long_pressed_keys_t;

// ===========================================
// 全局变量
// ===========================================

static low_battery_warning_t     low_battery_warning     = {0};
static charge_complete_warning_t charge_complete_warning = {0};

static bool is_in_low_power_state  = false;
static bool is_in_full_power_state = false;
bool        low_vol_offed_sleep    = false;

uint32_t   bt_init_time = 0;
dev_info_t dev_info     = {0};
bts_info_t bts_info     = {
        .bt_name        = {"YIGIIX GK81_BT$", "YIGIIX GK81_BT$", "YIGIIX GK81_BT$"},
        .uart_init      = uart_init,
        .uart_read      = uart_read,
        .uart_transmit  = uart_transmit,
        .uart_receive   = uart_receive,
        .uart_available = uart_available,
        .timer_read32   = timer_read32,
};

// 硬件开关状态
static bool    hardware_switch_forced_usb = false;
static uint8_t last_manual_devs           = DEVS_HOST1;
// clang-format off
// 长按键配置
long_pressed_keys_t long_pressed_keys[] = {
    {.keycode = BT_HOST1, .press_time = 0, .event_cb = long_pressed_keys_cb},
    {.keycode = BT_HOST2, .press_time = 0, .event_cb = long_pressed_keys_cb},
    {.keycode = BT_HOST3, .press_time = 0, .event_cb = long_pressed_keys_cb},
    {.keycode = BT_2_4G, .press_time = 0, .event_cb = long_pressed_keys_cb},
    {.keycode = FACTORY_RESET, .press_time = 0, .event_cb = long_pressed_keys_cb},
    {.keycode = RGB_TEST, .press_time = 0, .event_cb = long_pressed_keys_cb},
};
// clang-format on
// 指示器状态
uint8_t indicator_status          = 2;
uint8_t indicator_reset_last_time = false;
uint8_t layer_save;

// RGB控制
static uint32_t key_press_time;
static uint32_t close_rgb_time;
static bool     bak_rgb_toggle;
static bool     sober         = true;
bool            kb_sleep_flag = false;
extern bool     led_inited;
extern void     led_config_all(void);
extern void     led_deconfig_all(void);

// 设备指示配置
static const uint8_t rgb_index_table[]          = {BT_USB_INDEX, BT_HOST1_INDEX, BT_HOST2_INDEX, BT_HOST3_INDEX, BT_2_4G_INDEX};
static const uint8_t rgb_index_color_table[][3] = {
    {100, 100, 100}, {0, 0, 100}, {0, 0, 100}, {0, 0, 100}, {0, 100, 0},
};

static const uint8_t rgb_test_color_table[][3] = {
    {100, 0, 0},
    {0, 100, 0},
    {0, 0, 100},
    {100, 100, 100},
};
static uint8_t  rgb_test_index = 0;
static bool     rgb_test_en    = false;
static uint32_t rgb_test_time  = 0;

static bool rgb_status_save = 1;

const uint32_t sleep_time_table[4] = {0, 10 * 60 * 1000, 30 * 60 * 1000, 60 * 60 * 1000};

// 工厂重置相关
static uint32_t factory_reset_press_time = 0;
static uint8_t  factory_reset_status;
static uint8_t  factory_reset_press_cnt;

// 闪烁效果相关
static uint8_t  all_blink_cnt;
static uint32_t all_blink_time;
static RGB      all_blink_color;
static uint8_t  single_blink_cnt;
static uint8_t  single_blink_index;
static RGB      single_blink_color;
static uint32_t single_blink_time;

// 电量查询
bool query_vol_flag = false;

// USB相关
uint32_t USB_switch_time;
uint8_t  USB_blink_cnt;

uint32_t last_total_time = 0;

static bool is_charging = false;

// ===========================================
// 线程定义
// ===========================================
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
    (void)arg;
    chRegSetThreadName("blinker");
    while (true) {
        bts_task(dev_info.devs);
        chThdSleepMilliseconds(1);
    }
}

// ===========================================
// 初始化函数
// ===========================================
void bt_init(void) {
    bts_init(&bts_info);
    bt_used_pin_init();

    // 读取用户配置
    dev_info.raw = eeconfig_read_user();
    if (!dev_info.raw) {
        dev_info.devs      = DEVS_USB;
        dev_info.last_devs = DEVS_HOST1;
        eeconfig_update_user(dev_info.raw);
    }

    // 初始化手动设备选择记录
    if (dev_info.last_devs != DEVS_USB && dev_info.last_devs <= DEVS_2_4G) {
        last_manual_devs = dev_info.last_devs;
    } else {
        last_manual_devs = DEVS_HOST1;
    }

    // 检查硬件开关初始状态
    bool switch_on = !readPin(BT_MODE_SW_PIN);
    if (switch_on) {
        hardware_switch_forced_usb = true;
        if (dev_info.devs != DEVS_USB) {
            dev_info.devs = DEVS_USB;
        }
    } else {
        hardware_switch_forced_usb = false;
    }

    bt_init_time = timer_read32();
    chThdCreateStatic(waThread1, sizeof(waThread1), HIGHPRIO, Thread1, NULL);

    if (dev_info.devs != DEVS_USB) {
        usbDisconnectBus(&USB_DRIVER);
        usbStop(&USB_DRIVER);
    }

    setPinOutput(A14);
    if (dev_info.devs == DEVS_USB) {
        writePinLow(A14);
    } else {
        writePinHigh(A14);
    }

    rgb_status_save = rgb_matrix_config.enable;
}

// ===========================================
// 蓝牙任务函数
// ===========================================
void bt_task(void) {
    static uint32_t last_time = 0;

    if ((bt_init_time != 0) && (timer_elapsed32(bt_init_time) >= 2000)) {
        bt_init_time = 0;

        bts_send_name(DEVS_HOST1);
        switch (dev_info.devs) {
            case DEVS_HOST1:
                bts_send_vendor(v_host1);
                break;
            case DEVS_HOST2:
                bts_send_vendor(v_host2);
                break;
            case DEVS_HOST3:
                bts_send_vendor(v_host3);
                break;
            case DEVS_2_4G:
                bts_send_vendor(v_2_4g);
                break;
            default:
                bts_send_vendor(v_usb);
                dev_info.devs = DEVS_USB;
                eeconfig_update_user(dev_info.raw);
                break;
        }

        bts_send_vendor(v_en_sleep_bt);
        wait_ms(10);
        bts_send_vendor(v_en_sleep_wl);
        wait_ms(10);
    }

    if (timer_elapsed32(last_time) >= 1) {
        last_time = timer_read32();

        if (dev_info.devs != DEVS_USB) {
            uint8_t keyboard_led_state = 0;
            led_t  *kb_leds            = (led_t *)&keyboard_led_state;
            kb_leds->raw               = bts_info.bt_info.indictor_rgb_s;
            usb_device_state_set_leds(keyboard_led_state);

#ifdef RGB_MATRIX_ENABLE
            close_rgb();
#endif
        }
    }

    long_pressed_keys_hook();
    bt_scan_mode();
}

// ===========================================
// 按键处理函数
// ===========================================
bool process_record_bt(uint16_t keycode, keyrecord_t *record) {
    bool retval = true;

    if (record->event.pressed) {
        BT_DEBUG_INFO("\n\nkeycode = [0x%x], pressed time: [%d]\n\n", keycode, record->event.time);
        BT_DEBUG_INFO("\n devs     = [%d] \
                    \n sleeped       = [%d] \
                    \n low_vol       = [%d] \
                    \n low_vol_offed = [%d] \
                    \n normal_vol    = [%d] \
                    \n pairing       = [%d] \
                    \n paired        = [%d] \
                    \n come_back     = [%d] \
                    \n come_back_err = [%d] \
                    \n mode_switched = [%d] \
                    \n pvol          = [%d]\n\n\n",
                      dev_info.devs, bts_info.bt_info.sleeped, bts_info.bt_info.low_vol, bts_info.bt_info.low_vol_offed, bts_info.bt_info.normal_vol, bts_info.bt_info.pairing, bts_info.bt_info.paired, bts_info.bt_info.come_back, bts_info.bt_info.come_back_err, bts_info.bt_info.mode_switched, bts_info.bt_info.pvol);

        if (!rgb_matrix_config.enable) {
            if (rgb_status_save) {
                rgb_matrix_enable_noeeprom();
            }
        }
    }

    retval = process_record_other(keycode, record);

    if (dev_info.devs != DEVS_USB) {
        if (retval != false) {
            while (bts_is_busy()) {
                wait_ms(1);
            }
            if ((keycode > QK_MODS) && (keycode <= QK_MODS_MAX)) {
                if (QK_MODS_GET_MODS(keycode) & 0x1) {
                    if (QK_MODS_GET_MODS(keycode) & 0x10)
                        bts_process_keys(KC_RCTL, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                    else
                        bts_process_keys(KC_LCTL, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                }
                if (QK_MODS_GET_MODS(keycode) & 0x2) {
                    if (QK_MODS_GET_MODS(keycode) & 0x10)
                        bts_process_keys(KC_RSFT, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                    else
                        bts_process_keys(KC_LSFT, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                }
                if (QK_MODS_GET_MODS(keycode) & 0x4) {
                    if (QK_MODS_GET_MODS(keycode) & 0x10)
                        bts_process_keys(KC_RALT, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                    else
                        bts_process_keys(KC_LALT, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                }
                if (QK_MODS_GET_MODS(keycode) & 0x8) {
                    if (QK_MODS_GET_MODS(keycode) & 0x10)
                        bts_process_keys(KC_RGUI, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                    else
                        bts_process_keys(KC_LGUI, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                }
                retval = bts_process_keys(QK_MODS_GET_BASIC_KEYCODE(keycode), record->event.pressed, dev_info.devs, keymap_config.no_gui);
            } else {
                retval = bts_process_keys(keycode, record->event.pressed, dev_info.devs, keymap_config.no_gui);
            }
        }
    }

#ifdef RGB_MATRIX_ENABLE
    open_rgb();
#endif

    return retval;
}

// ===========================================
// 设备切换函数
// ===========================================
void bt_switch_mode(uint8_t last_mode, uint8_t now_mode, uint8_t reset) {
    if (!rgb_matrix_config.enable) {
        if (rgb_status_save) {
            rgb_matrix_enable_noeeprom();
        }
    }

    bool usb_sws = !!last_mode ? !now_mode : !!now_mode;

    extern uint8_t  indicator_status;
    extern uint8_t  indicator_reset_last_time;
    extern uint32_t last_total_time;

    if (usb_sws) {
        if (!!now_mode) {
            usbDisconnectBus(&USB_DRIVER);
            usbStop(&USB_DRIVER);
        } else {
            init_usb_driver(&USB_DRIVER);
        }
    }

    if (dev_info.devs != dev_info.last_devs) {
        last_total_time = timer_read32();
    }

    dev_info.devs = now_mode;
    if (dev_info.devs != DEVS_USB) {
        dev_info.last_devs = dev_info.devs;
    } else if (dev_info.devs == DEVS_USB) {
        USB_switch_time = timer_read32();
        USB_blink_cnt   = 0;
    }

    if (dev_info.devs == DEVS_USB) {
        writePinLow(A14);
    } else {
        writePinHigh(A14);
    }

    // 重置蓝牙状态
    bts_info.bt_info.pairing        = false;
    bts_info.bt_info.paired         = false;
    bts_info.bt_info.come_back      = false;
    bts_info.bt_info.come_back_err  = false;
    bts_info.bt_info.mode_switched  = false;
    bts_info.bt_info.indictor_rgb_s = 0;

    if (readPin(BT_MODE_SW_PIN)) eeconfig_update_user(dev_info.raw);

    // 发送相应的蓝牙命令
    switch (dev_info.devs) {
        case DEVS_HOST1:
        case DEVS_HOST2:
        case DEVS_HOST3:
        case DEVS_2_4G:
            if (reset != false) {
                indicator_status          = 1;
                indicator_reset_last_time = true;
                bts_send_vendor(v_pair);
            } else {
                indicator_status          = 2;
                indicator_reset_last_time = true;
                uint8_t vendor_cmds[]     = {v_host1, v_host2, v_host3, v_2_4g};
                bts_send_vendor(vendor_cmds[dev_info.devs - 1]);
            }
            break;
        case DEVS_USB:
            indicator_status          = 2;
            indicator_reset_last_time = true;
            bts_send_vendor(v_usb);
            break;
        default:
            break;
    }
}

// ===========================================
// 其他按键处理
// ===========================================
static bool process_record_other(uint16_t keycode, keyrecord_t *record) {
    // 更新长按键时间
    for (uint8_t i = 0; i < NUM_LONG_PRESS_KEYS; i++) {
        if (keycode == long_pressed_keys[i].keycode) {
            if (record->event.pressed) {
                long_pressed_keys[i].press_time = timer_read32();
            } else {
                long_pressed_keys[i].press_time = 0;
            }
            break;
        }
    }

    switch (keycode) {
        case BT_HOST1:
        case BT_HOST2:
        case BT_HOST3:
        case BT_2_4G:
        case BT_USB: {
            if (record->event.pressed) {
                uint8_t target_devs = (keycode == BT_HOST1) ? DEVS_HOST1 : (keycode == BT_HOST2) ? DEVS_HOST2 : (keycode == BT_HOST3) ? DEVS_HOST3 : (keycode == BT_2_4G) ? DEVS_2_4G : DEVS_USB;

                if (hardware_switch_forced_usb && target_devs != DEVS_USB) {
                    last_manual_devs = target_devs;
                    return false;
                }

                if (target_devs != DEVS_USB) {
                    last_manual_devs = target_devs;
                }

                if (dev_info.devs != target_devs) {
                    bt_switch_mode(dev_info.devs, target_devs, false);
                }
            }
        } break;

        case BT_VOL: {
            if (record->event.pressed) {
                bts_send_vendor(v_query_vol);
                query_vol_flag = true;
            } else {
                query_vol_flag = false;
            }
        } break;

        case NK_TOGG: {
            if (record->event.pressed) {
                if (keymap_config.nkro) {
                    single_blink_cnt   = 6;
                    single_blink_index = 0;
                    single_blink_color = (RGB){100, 100, 100};
                } else {
                    single_blink_cnt   = 6;
                    single_blink_index = 0;
                    single_blink_color = (RGB){0, 0, 100};
                }
                single_blink_time = timer_read32();
            }
            return true;
        }

        case SW_OS: {
            if (record->event.pressed) {
                if (get_highest_layer(default_layer_state) == 0) {
                    set_single_persistent_default_layer(2);
                    keymap_config.no_gui = 0;
                    eeconfig_update_keymap(&keymap_config);
                    single_blink_index = 69;
                } else if (get_highest_layer(default_layer_state) == 2) {
                    set_single_persistent_default_layer(0);
                    single_blink_index = 68;
                }
                single_blink_time  = timer_read32();
                single_blink_cnt   = 6;
                single_blink_color = (RGB){100, 100, 100};
            }
        } break;

        case RGB_TEST: {
            if (record->event.pressed) {
                if (rgb_test_en) {
                    rgb_test_en    = false;
                    rgb_test_index = 0;
                }
            }
        } break;

        case FACTORY_RESET:
            break;
        default:
            return true;
    }

    return false;
}

// ===========================================
// 长按键处理
// ===========================================
static void long_pressed_keys_cb(uint16_t keycode) {
    switch (keycode) {
        case BT_HOST1:
        case BT_HOST2:
        case BT_HOST3:
        case BT_2_4G: {
            uint8_t target_dev = keycode - BT_HOST1 + DEVS_HOST1;
            if (dev_info.devs == target_dev) {
                bt_switch_mode(dev_info.devs, target_dev, true);
            }
        } break;

        case FACTORY_RESET: {
            if (!factory_reset_status) {
                factory_reset_status    = 1;
                factory_reset_press_cnt = 1;
                rgb_matrix_enable_noeeprom();
                rgb_status_save          = rgb_matrix_config.enable;
                factory_reset_press_time = timer_read32();
            }
        } break;

        case RGB_TEST: {
            if (rgb_test_en != true) {
                rgb_test_en    = true;
                rgb_test_index = 1;
                rgb_test_time  = timer_read32();
            }
        } break;

        default:
            break;
    }
}

static void long_pressed_keys_hook(void) {
    for (uint8_t i = 0; i < NUM_LONG_PRESS_KEYS; i++) {
        if ((long_pressed_keys[i].press_time != 0) && (timer_elapsed32(long_pressed_keys[i].press_time) >= (3 * 1000))) {
            long_pressed_keys[i].event_cb(long_pressed_keys[i].keycode);
            long_pressed_keys[i].press_time = 0;
        }
    }
}

// ===========================================
// 硬件管理函数
// ===========================================
static void bt_used_pin_init(void) {
#ifdef BT_MODE_SW_PIN
    setPinInputHigh(BT_MODE_SW_PIN);
#endif

#if defined(BT_CABLE_PIN) && defined(BT_CHARGE_PIN)
    setPinInput(BT_CABLE_PIN);
    setPinInput(BT_CHARGE_PIN);
#endif
}

static void bt_scan_mode(void) {
#ifdef BT_MODE_SW_PIN
    static bool mode_sw_status     = 0;
    static bool mode_sw_old_status = 0;

    mode_sw_status = !readPin(BT_MODE_SW_PIN);

    if (mode_sw_old_status != mode_sw_status) {
        mode_sw_old_status = mode_sw_status;

        if (mode_sw_status) {
            last_manual_devs           = dev_info.devs;
            hardware_switch_forced_usb = true;
            if (dev_info.devs != DEVS_USB) {
                bt_switch_mode(dev_info.devs, DEVS_USB, false);
            }
        } else {
            hardware_switch_forced_usb = false;
            uint8_t target_devs        = last_manual_devs;
            bt_switch_mode(DEVS_USB, target_devs, false);
        }
    }
#endif

#ifdef FORCE_USB
    static bool cable_status;
    static bool cable_old_status;

    cable_status = !readPin(BT_CABLE_PIN);

    if (cable_old_status != cable_status) {
        cable_old_status = cable_status;
        if (cable_status) {
            if (dev_info.devs != DEVS_USB) {
                bt_switch_mode(dev_info.devs, DEVS_USB, false);
            }
        } else {
            bt_switch_mode(dev_info.devs, dev_info.last_devs, false);
        }
    }
#endif
}

// ===========================================
// 低电量管理函数
// ===========================================
static void update_low_voltage_state(void) {}

// ===========================================
// RGB控制函数
// ===========================================
static void led_off_standby(void) {
#define led_off_standby_timeout (10 * 60 * 1000)
    if (timer_elapsed32(key_press_time) >= led_off_standby_timeout) {
        rgb_matrix_disable_noeeprom();
    } else {
        rgb_status_save = rgb_matrix_config.enable;
    }
}

static void close_rgb(void) {
    if (!key_press_time) {
        key_press_time = timer_read32();
        return;
    }

    led_off_standby();

    if (sober) {
        if (kb_sleep_flag || (timer_elapsed32(key_press_time) >= sleep_time_table[2])) {
            bak_rgb_toggle = rgb_status_save;
            sober          = false;
            close_rgb_time = timer_read32();
            rgb_matrix_disable_noeeprom();
        }
    } else {
        if (!rgb_matrix_config.enable) {
            if (timer_elapsed32(close_rgb_time) >= ENTRY_STOP_TIMEOUT) {
                if (led_inited) {
                    led_deconfig_all();
                }
#ifdef ENTRY_STOP_MODE
                lp_system_sleep();
#endif
                open_rgb();
            }
        }
    }
}

static void open_rgb(void) {
    key_press_time = timer_read32();
    if (!sober) {
#ifdef WS2812_EN_PIN
        setPinOutput(WS2812_EN_PIN);
        writePinLow(WS2812_EN_PIN);
#endif
        if (bak_rgb_toggle) {
            extern bool low_vol_offed_sleep;
            kb_sleep_flag       = false;
            low_vol_offed_sleep = false;
            rgb_matrix_enable_noeeprom();
        }
        if (!led_inited) {
            led_config_all();
        }
        sober = true;
    }
}

// ===========================================
// 指示灯函数
// ===========================================
static void handle_bt_indicate_led(void) {
    uint8_t         rgb_index      = rgb_index_table[dev_info.devs];
    static uint32_t last_time      = 0;
    static uint32_t last_long_time = 0;
    static uint8_t  last_status    = 0;
    static bool     rgb_flip       = false;
    static RGB      rgb            = {0};

    if (last_status != indicator_status) {
        last_status     = indicator_status;
        last_total_time = timer_read32();
    }

    if (indicator_reset_last_time != false) {
        indicator_reset_last_time = false;
        last_time                 = 0;
    }

    switch (indicator_status) {
        case 1: { // 闪烁模式 5Hz 重置
            if ((last_time == 0) || (timer_elapsed32(last_time) >= 200)) {
                last_time = timer_read32();
                rgb_flip  = !rgb_flip;
                if (rgb_flip) {
                    rgb.r = rgb_index_color_table[dev_info.devs][0];
                    rgb.g = rgb_index_color_table[dev_info.devs][1];
                    rgb.b = rgb_index_color_table[dev_info.devs][2];
                } else {
                    rgb = (RGB){.r = 0, .g = 0, .b = 0};
                }
            }

            if (bts_info.bt_info.paired) {
                last_long_time   = timer_read32();
                indicator_status = 3;
                break;
            }

            if (timer_elapsed32(last_total_time) >= (60 * 1000)) {
                indicator_status = 0;
                kb_sleep_flag    = true;
            }
        } break;

        case 2: { // 闪烁模式 2Hz 回连
            if ((last_time == 0) || (timer_elapsed32(last_time) >= 500)) {
                last_time = timer_read32();
                rgb_flip  = !rgb_flip;
                if (rgb_flip) {
                    rgb.r = rgb_index_color_table[dev_info.devs][0];
                    rgb.g = rgb_index_color_table[dev_info.devs][1];
                    rgb.b = rgb_index_color_table[dev_info.devs][2];
                } else {
                    rgb = (RGB){.r = 0, .g = 0, .b = 0};
                }
            }

            if (bts_info.bt_info.paired) {
                last_long_time   = timer_read32();
                indicator_status = 3;
                break;
            }

            if (timer_elapsed32(last_total_time) >= (60 * 1000)) {
                indicator_status = 0;
                kb_sleep_flag    = true;
            }
        } break;

        case 3: { // 长亮模式
            if ((timer_elapsed32(last_long_time) < (2 * 1000))) {
                rgb.r = rgb_index_color_table[dev_info.devs][0];
                rgb.g = rgb_index_color_table[dev_info.devs][1];
                rgb.b = rgb_index_color_table[dev_info.devs][2];
            } else {
                low_battery_warning.triggered   = true;
                low_battery_warning.blink_count = 0;
                low_battery_warning.blink_time  = timer_read32();
                low_battery_warning.blink_state = false;
                low_battery_warning.completed   = false;
                indicator_status                = 0;
            }
        } break;

        case 4: { // 长灭模式
            rgb = (RGB){.r = 0, .g = 0, .b = 0};
        } break;

        default:
            rgb_flip = false;
            if (!kb_sleep_flag) {
                if (!bts_info.bt_info.paired) {
                    if (!bts_info.bt_info.pairing) {
                        indicator_status = 2;
                        break;
                    }
                    indicator_status = 2;
                    if ((dev_info.devs != DEVS_USB) && (dev_info.devs != DEVS_2_4G)) {
                        bt_switch_mode(DEVS_USB, dev_info.last_devs, false);
                    }
                    if (dev_info.devs == DEVS_2_4G) {
                        bt_switch_mode(DEVS_USB, DEVS_2_4G, false);
                    }
                }
            }
            break;
    }

    if (indicator_status) {
        rgb_matrix_set_color(rgb_index, rgb.r, rgb.g, rgb.b);
    }
}

static void handle_usb_indicate_led(void) {
    static uint16_t USB_blink_time;
    static bool     USB_blink;

    if ((USB_DRIVER.state != USB_ACTIVE)) {
        if (USB_blink_cnt <= 20) {
            if (timer_elapsed(USB_blink_time) >= 500) {
                USB_blink_cnt++;
                USB_blink      = !USB_blink;
                USB_blink_time = timer_read();
            }
            if (USB_blink) {
                rgb_matrix_set_color(rgb_index_table[DEVS_USB], 100, 100, 100);
            } else {
                rgb_matrix_set_color(rgb_index_table[DEVS_USB], 0, 0, 0);
            }
        }
        USB_switch_time = timer_read32();
    } else {
        if (timer_elapsed32(USB_switch_time) < 3000) {
            rgb_matrix_set_color(rgb_index_table[DEVS_USB], 100, 100, 100);
        }
    }
}

// ===========================================
// RGB指示器处理函数
// ===========================================
static void handle_factory_reset_display(void) {
    if (factory_reset_status) {
        if (timer_elapsed32(factory_reset_press_time) >= factory_reset_press_cnt * 500) {
            factory_reset_press_cnt++;
        }

        if (factory_reset_press_cnt >= 7) {
            switch (factory_reset_status) {
                case 1: // factory reset
                    eeconfig_init();
                    keymap_config.no_gui = 0;
                    eeconfig_update_keymap(&keymap_config);
                    if (readPin(BT_MODE_SW_PIN) && (dev_info.devs != DEVS_USB)) {
                        bts_send_vendor(v_clear);
                        bts_info.bt_info.pairing = false;
                        bt_switch_mode(DEVS_HOST1, DEVS_USB, false);
                        last_total_time  = timer_read32();
                        indicator_status = 2;
                    }
                    break;

                default:
                    break;
            }
            factory_reset_press_cnt  = 0;
            factory_reset_status     = 0;
            factory_reset_press_time = 0;
        }

        rgb_matrix_set_color_all(0, 0, 0);
        uint8_t reset_leds[] = {26, 45, 44}; // factory, keyboard, ble
        if (factory_reset_status >= 1 && factory_reset_status <= 3) {
            rgb_matrix_set_color(reset_leds[factory_reset_status - 1], 100, 0, 0);
        }
    }
}

static void handle_blink_effects(void) {
    // 全键闪烁
    if (all_blink_cnt) {
        rgb_matrix_set_color_all(0, 0, 0);
        if (timer_elapsed32(all_blink_time) > 500) {
            all_blink_time = timer_read32();
            all_blink_cnt--;
        }
        if (all_blink_cnt % 2) {
            rgb_matrix_set_color_all(all_blink_color.r, all_blink_color.g, all_blink_color.b);
        } else {
            rgb_matrix_set_color_all(0, 0, 0);
        }
    }

    // 单键闪烁
    if (single_blink_cnt) {
        if (timer_elapsed32(single_blink_time) > 500) {
            single_blink_time = timer_read32();
            single_blink_cnt--;
        }
        if (single_blink_cnt % 2) {
            rgb_matrix_set_color(single_blink_index, single_blink_color.r, single_blink_color.g, single_blink_color.b);
        } else {
            rgb_matrix_set_color(single_blink_index, 0, 0, 0);
        }
    }
}

static void handle_layer_indication(void) {
    // FN 按下时显示当前设备状态
    if ((get_highest_layer(layer_state) == 1) || (get_highest_layer(layer_state) == 3)) {
        rgb_matrix_set_color(rgb_index_table[dev_info.devs], RGB_BLUE);
    }
}

static void handle_charging_indication(void) {
    if (!readPin(BT_CABLE_PIN)) {
        if (readPin(BT_CHARGE_PIN)) {
            // 充满状态
            if (!is_in_full_power_state) {
                is_in_full_power_state = true;
                if (!charge_complete_warning.triggered) {
                    charge_complete_warning.triggered   = true;
                    charge_complete_warning.blink_count = 0;
                    charge_complete_warning.blink_time  = timer_read32();
                    charge_complete_warning.blink_state = false;
                    charge_complete_warning.completed   = false;
                }
            }

            // 只有在未完成闪烁且闪烁次数未达到5次时才显示充电指示
            if (!charge_complete_warning.completed && charge_complete_warning.blink_count < 5) {
                if (timer_elapsed32(charge_complete_warning.blink_time) >= 1000) {
                    charge_complete_warning.blink_time  = timer_read32();
                    charge_complete_warning.blink_state = !charge_complete_warning.blink_state;

                    if (charge_complete_warning.blink_state) {
                        charge_complete_warning.blink_count++;
                        if (charge_complete_warning.blink_count >= 5) {
                            charge_complete_warning.completed   = true;
                            charge_complete_warning.blink_state = false;
                        }
                    }
                }

                // 显示充电完成闪烁
                if (charge_complete_warning.blink_state) {
                    for (uint8_t i = 84; i <= 86; i++) {
                        rgb_matrix_set_color(i, 0, 100, 0);
                    }
                } else {
                    for (uint8_t i = 84; i <= 86; i++) {
                        rgb_matrix_set_color(i, 0, 0, 0);
                    }
                }
            }
        }
    } else {
        // 充电线未接入，重置充电状态
        if (is_in_full_power_state) {
            is_in_full_power_state = false;
            memset(&charge_complete_warning, 0, sizeof(charge_complete_warning_t));
        }
    }
}

static void handle_low_battery_warning(void) {
    // update_low_voltage_state();

    // 低电量警告（电量≤20%）
    if (bts_info.bt_info.pvol <= 20) {
        if (!is_in_low_power_state) {
            is_in_low_power_state = true;

            if (!low_battery_warning.triggered) {
                low_battery_warning.triggered   = true;
                low_battery_warning.blink_count = 0;
                low_battery_warning.blink_time  = timer_read32();
                low_battery_warning.blink_state = false;
                low_battery_warning.completed   = false;
            }
        }

        // 处理闪烁逻辑
        if (!low_battery_warning.completed && !kb_sleep_flag && (indicator_status == 0)) {
            if (timer_elapsed32(low_battery_warning.blink_time) >= 1000) {
                low_battery_warning.blink_time  = timer_read32();
                low_battery_warning.blink_state = !low_battery_warning.blink_state;

                if (low_battery_warning.blink_state) {
                    low_battery_warning.blink_count++;
                    if (low_battery_warning.blink_count >= 6) {
                        low_battery_warning.completed   = true;
                        low_battery_warning.blink_state = false;
                    }
                }
            }

            // 显示闪烁效果
            if (low_battery_warning.blink_state) {
                for (uint8_t i = 84; i <= 86; i++) {
                    rgb_matrix_set_color(i, 100, 0, 0);
                }
            } else {
                for (uint8_t i = 84; i <= 86; i++) {
                    rgb_matrix_set_color(i, 0, 0, 0);
                }
            }
        }
    }

    if (bts_info.bt_info.pvol > 20) {
        if (is_in_low_power_state) {
            is_in_low_power_state = false;
            memset(&low_battery_warning, 0, sizeof(low_battery_warning_t));
        }
    }
}

static void handle_low_battery_shutdow(void) {
    extern bool low_vol_offed_sleep;
    if (bts_info.bt_info.low_vol_offed) {
        kb_sleep_flag       = true;
        low_vol_offed_sleep = true;
    }
}

static void handle_battery_query_display(void) {
    static bool     query_vol_processing = false;
    static uint32_t query_vol_time;

    // 定期查询电量
    if (!kb_sleep_flag && bts_info.bt_info.paired && timer_elapsed32(query_vol_time) >= 10000) {
        query_vol_time = timer_read32();
        bts_send_vendor(v_query_vol);
    }

    if (query_vol_flag) {
        query_vol_processing = true;

        // 清空显示区域
        for (uint8_t i = 0; i < 84; i++) {
            rgb_matrix_set_color(i, 0, 0, 0);
        }

        // 电量显示LED
        uint8_t query_index[10] = {14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
        uint8_t pvol            = bts_info.bt_info.pvol;

        // 计算LED数量（至少2个，最多10个）
        uint8_t led_count = (pvol < 30) ? 2 : ((pvol / 10) > 10 ? 10 : (pvol / 10));

        // 根据电量确定颜色
        RGB color;
        if (pvol < 30) {
            color = (RGB){100, 0, 0}; // 红色
        } else if (pvol < 70) {
            color = (RGB){100, 50, 0}; // 橙色
        } else {
            color = (RGB){0, 100, 0}; // 绿色
        }

        // 点亮LED
        for (uint8_t i = 0; i < led_count; i++) {
            rgb_matrix_set_color(query_index[i], color.r, color.g, color.b);
        }
    } else {
        if (query_vol_processing) {
            query_vol_processing = false;
        }
    }
}

// ===========================================
// 主RGB指示器函数
// ===========================================
bool bt_indicator_rgb(uint8_t led_min, uint8_t led_max) {
    // 工厂重置显示（最高优先级）
    if (factory_reset_status) {
        handle_factory_reset_display();
        return false;
    }

    // 闪烁效果处理
    handle_blink_effects();

    // 图层指示
    handle_layer_indication();

    // 设备状态指示
    if (dev_info.devs != DEVS_USB) {
        handle_bt_indicate_led();
    }
    if (dev_info.devs == DEVS_USB) {
        handle_usb_indicate_led();
    }

// 充电状态指示
#if defined(BT_CABLE_PIN) && defined(BT_CHARGE_PIN)
    is_charging = !readPin(BT_CABLE_PIN);
#endif
    if (is_charging) {
        handle_charging_indication();
    } else {
        // 非充电状态下的其他指示
        if (dev_info.devs != DEVS_USB) {
            handle_low_battery_warning();
            handle_low_battery_shutdow();
            handle_battery_query_display();
        }
    }

    if (dev_info.devs != DEVS_USB) {
        update_low_voltage_state();
    }

    // rgb test
    if (rgb_test_en) {
        if (timer_elapsed32(rgb_test_time) >= 1000) {
            rgb_test_time = timer_read32();
            rgb_test_index++;
            if (rgb_test_index > 4) {
                rgb_test_index = 1;
            }
        }

        for (uint8_t i = 0; i < 87; i++) {
            rgb_matrix_set_color(i, rgb_test_color_table[rgb_test_index - 1][0], rgb_test_color_table[rgb_test_index - 1][1], rgb_test_color_table[rgb_test_index - 1][2]);
        }
    }

    return true;
}
