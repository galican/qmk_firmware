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
#include "m91.h"
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

extern keymap_config_t keymap_config;

void bt_bat_low_indicator(void);

typedef struct {
    uint32_t press_time;
    uint16_t keycode;
    void (*event_cb)(uint16_t);
} long_pressed_keys_t;

uint32_t   bt_init_time = 0;
dev_info_t dev_info     = {0};
bts_info_t bts_info     = {
        .bt_name        = {"Alchemie_$", "Alchemie_$", "Alchemie_$"},
        .uart_init      = uart_init,
        .uart_read      = uart_read,
        .uart_transmit  = uart_transmit,
        .uart_receive   = uart_receive,
        .uart_available = uart_available,
        .timer_read32   = timer_read32,
};

static void long_pressed_keys_hook(void);
static void long_pressed_keys_cb(uint16_t keycode);
static bool process_record_other(uint16_t keycode, keyrecord_t *record);
static void bt_scan_mode(void);
static void bt_used_pin_init(void);
#ifdef RGB_MATRIX_ENABLE
void        open_rgb(void);
static void close_rgb(void);
#endif
void bt_mousekey_task(void);

// clang-format off
 long_pressed_keys_t long_pressed_keys[] = {
     {.keycode = BT_HOST1, .press_time = 0, .event_cb = long_pressed_keys_cb},
     {.keycode = BT_HOST2, .press_time = 0, .event_cb = long_pressed_keys_cb},
     {.keycode = BT_HOST3, .press_time = 0, .event_cb = long_pressed_keys_cb},
     {.keycode = BT_2_4G, .press_time = 0, .event_cb = long_pressed_keys_cb},
     {.keycode = SW_OS, .press_time = 0, .event_cb = long_pressed_keys_cb},
     {.keycode = FACTORY_RESET, .press_time = 0, .event_cb = long_pressed_keys_cb},
     {.keycode = KEYBOARD_RESET, .press_time = 0, .event_cb = long_pressed_keys_cb},
     {.keycode = BLE_RESET, .press_time = 0, .event_cb = long_pressed_keys_cb}
 };
// clang-format on

uint8_t         indicator_status          = 2;
uint8_t         indicator_reset_last_time = false;
uint8_t         layer_save;
static uint16_t Low_power_blink_time;
static uint16_t charge_full_blink_time;

static uint32_t key_press_time;
static uint32_t close_rgb_time;
static bool     bak_rgb_toggle;
static bool     sober         = true;
bool            kb_sleep_flag = false;
extern bool     led_inited;
extern void     led_config_all(void);
extern void     led_deconfig_all(void);

static const uint8_t rgb_index_table[]          = {BT_USB_INDEX, BT_HOST1_INDEX, BT_HOST2_INDEX, BT_HOST3_INDEX, BT_2_4G_INDEX};
static const uint8_t rgb_index_color_table[][3] = {
    {100, 100, 100}, {0, 0, 100}, {0, 0, 100}, {0, 0, 100}, {0, 100, 0},
};

static bool rgb_status_save = 1;

const uint32_t sleep_time_table[4] = {0, 10 * 60 * 1000, 30 * 60 * 1000, 60 * 60 * 1000};

static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
    (void)arg;
    chRegSetThreadName("blinker");
    while (true) {
        bts_task(dev_info.devs);
        chThdSleepMilliseconds(1);
    }
}

/**
 * @brief bluetooth 初始化函数
 * @param None
 * @return None
 */
void bt_init(void) {
    bts_init(&bts_info);
    bt_used_pin_init();

    // Read the user config from EEPROM
    dev_info.raw = eeconfig_read_user();
    if (!dev_info.raw) {
        dev_info.devs      = DEVS_USB;
        dev_info.last_devs = DEVS_HOST1;
        eeconfig_update_user(dev_info.raw);
    }
    bt_init_time = timer_read32();
    // bt_scan_mode();
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

/**
 * @brief bluetooth交互任务
 * @param event 当前事件
 * @return None
 */
void bt_task(void) {
    static uint32_t last_time = 0;

    if ((bt_init_time != 0) && (timer_elapsed32(bt_init_time) >= 2000)) {
        bt_init_time = 0;

        // bts_send_vendor(v_en_sleep_bt);
        bts_send_name(DEVS_HOST1);
        switch (dev_info.devs) {
            case DEVS_HOST1: {
                bts_send_vendor(v_host1);
            } break;
            case DEVS_HOST2: {
                bts_send_vendor(v_host2);
            } break;
            case DEVS_HOST3: {
                bts_send_vendor(v_host3);
            } break;
            case DEVS_2_4G: {
                bts_send_vendor(v_2_4g);
            } break;
            default: {
                bts_send_vendor(v_usb);
                dev_info.devs = DEVS_USB;
                eeconfig_update_user(dev_info.raw);
            } break;
        }
        // dev_info.raw = eeconfig_read_user();
        // if (!dev_info.en_sleep_flag) {
        bts_send_vendor(v_en_sleep_bt);
        bts_send_vendor(v_en_sleep_wl);
        // }
    }

    /* Execute every 1ms */
    if (timer_elapsed32(last_time) >= 1) {
        last_time = timer_read32();

        // 设定指示灯状态
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

bool process_record_bt(uint16_t keycode, keyrecord_t *record) {
    bool retval = true;
    // clang-format off
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
                      dev_info.devs,
                      bts_info.bt_info.sleeped,
                      bts_info.bt_info.low_vol,
                      bts_info.bt_info.low_vol_offed,
                      bts_info.bt_info.normal_vol,
                      bts_info.bt_info.pairing,
                      bts_info.bt_info.paired,
                      bts_info.bt_info.come_back,
                      bts_info.bt_info.come_back_err,
                      bts_info.bt_info.mode_switched,
                      bts_info.bt_info.pvol);

        // clang-format on
        if (!rgb_matrix_config.enable) {
            if (rgb_status_save) {
                rgb_matrix_enable_noeeprom();
            }
        }
    }

    retval = process_record_other(keycode, record);

    if (dev_info.devs != DEVS_USB) {
        if (retval != false) {
            retval = bts_process_keys(keycode, record->event.pressed, dev_info.devs, keymap_config.no_gui);
        }
    }

#ifdef RGB_MATRIX_ENABLE
    open_rgb();
#endif

    return retval;
}

uint32_t USB_switch_time;
uint8_t  USB_blink_cnt;

void bt_switch_mode(uint8_t last_mode, uint8_t now_mode, uint8_t reset) {
    if (!rgb_matrix_config.enable) {
        if (rgb_status_save) {
            rgb_matrix_enable_noeeprom();
        }
    }

    bool usb_sws = !!last_mode ? !now_mode : !!now_mode;

    extern uint8_t indicator_status;
    extern uint8_t indicator_reset_last_time;

    if (usb_sws) {
        if (!!now_mode) {
            usbDisconnectBus(&USB_DRIVER);
            usbStop(&USB_DRIVER);
            // writePinHigh(A12);
        } else {
            init_usb_driver(&USB_DRIVER);
        }
    }

    dev_info.devs = now_mode;
    if ((dev_info.devs != DEVS_USB) && (dev_info.devs != DEVS_2_4G)) {
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
    bts_info.bt_info.pairing       = false;
    bts_info.bt_info.paired        = false;
    bts_info.bt_info.come_back     = false;
    bts_info.bt_info.come_back_err = false;
    bts_info.bt_info.mode_switched = false;
    // 设定指示灯状态
    bts_info.bt_info.indictor_rgb_s = 0;
    eeconfig_update_user(dev_info.raw);

    switch (dev_info.devs) {
        case DEVS_HOST1: {
            if (reset != false) {
                indicator_status          = 1;
                indicator_reset_last_time = true;
                // bts_send_name(DEVS_HOST1);
                // bts_send_vendor(v_host1);
                bts_send_vendor(v_pair);
            } else {
                indicator_status          = 2;
                indicator_reset_last_time = true;
                bts_send_vendor(v_host1);
            }
        } break;
        case DEVS_HOST2: {
            if (reset != false) {
                indicator_status          = 1;
                indicator_reset_last_time = true;
                // bts_send_name(DEVS_HOST2);
                // bts_send_vendor(v_host2);
                bts_send_vendor(v_pair);
            } else {
                indicator_status          = 2;
                indicator_reset_last_time = true;
                bts_send_vendor(v_host2);
            }
        } break;
        case DEVS_HOST3: {
            if (reset != false) {
                indicator_status          = 1;
                indicator_reset_last_time = true;
                // bts_send_name(DEVS_HOST3);
                // bts_send_vendor(v_host3);
                bts_send_vendor(v_pair);
            } else {
                indicator_status          = 2;
                indicator_reset_last_time = true;
                bts_send_vendor(v_host3);
            }
        } break;
        case DEVS_2_4G: {
            if (reset != false) {
                indicator_status          = 1;
                indicator_reset_last_time = true;
                // bts_send_name(DEVS_2_4G);
                // bts_send_vendor(v_2_4g);
                bts_send_vendor(v_pair);
            } else {
                indicator_status          = 2;
                indicator_reset_last_time = true;
                bts_send_vendor(v_2_4g);
            }
        } break;
        case DEVS_USB: {
            indicator_status          = 2;
            indicator_reset_last_time = true;
            bts_send_vendor(v_usb);
        } break;
        default:
            break;
    }
}

static uint32_t factory_reset_press_time = 0;
static uint8_t  factory_reset_status;
static uint8_t  factory_reset_press_cnt;
static uint8_t  all_blink_cnt;
static uint32_t all_blink_time;
static RGB      all_blink_color;
static uint8_t  single_blink_cnt;
static uint8_t  single_blink_index;
static RGB      single_blink_color;
static uint32_t single_blink_time;
// static uint32_t single_light_on_time;
// static uint8_t single_light_on_index;
// static RGB single_light_on_color;
bool query_vol_flag = false;

static bool process_record_other(uint16_t keycode, keyrecord_t *record) {
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
        case BT_HOST1: {
            if (record->event.pressed) {
                if (dev_info.devs != DEVS_HOST1) {
                    bt_switch_mode(dev_info.devs, DEVS_HOST1, false);
                }
            }
        } break;
        case BT_HOST2: {
            if (record->event.pressed) {
                if (dev_info.devs != DEVS_HOST2) {
                    bt_switch_mode(dev_info.devs, DEVS_HOST2, false);
                }
            }
        } break;
        case BT_HOST3: {
            if (record->event.pressed) {
                if (dev_info.devs != DEVS_HOST3) {
                    bt_switch_mode(dev_info.devs, DEVS_HOST3, false);
                }
            }
        } break;
        case BT_2_4G: {
            if (record->event.pressed) {
                if (dev_info.devs != DEVS_2_4G) {
                    bt_switch_mode(dev_info.devs, DEVS_2_4G, false);
                }
            }
        } break;
        case BT_USB: {
            if (record->event.pressed) {
                bt_switch_mode(dev_info.devs, DEVS_USB, false);
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
                if (record->event.pressed) {
                    if (keymap_config.nkro) {
                        single_blink_cnt   = 6;
                        single_blink_index = 0;
                        single_blink_color = (RGB){0, 0, 100};
                        single_blink_time  = timer_read32();
                    } else {
                        single_blink_cnt   = 6;
                        single_blink_index = 0;
                        single_blink_color = (RGB){100, 100, 100};
                        single_blink_time  = timer_read32();
                    }
                }
            }
        }
            return true;
        case FACTORY_RESET:
        case KEYBOARD_RESET:
        case BLE_RESET:
        case SW_OS: // OS switch key
            break;
        default:
            return true;
    }

    return false;
}

static void long_pressed_keys_cb(uint16_t keycode) {
    switch (keycode) {
        case BT_HOST1: {
            if (dev_info.devs == DEVS_HOST1) {
                bt_switch_mode(dev_info.devs, DEVS_HOST1, true);
            }
        } break;
        case BT_HOST2: {
            if (dev_info.devs == DEVS_HOST2) {
                bt_switch_mode(dev_info.devs, DEVS_HOST2, true);
            }
        } break;
        case BT_HOST3: {
            if (dev_info.devs == DEVS_HOST3) {
                bt_switch_mode(dev_info.devs, DEVS_HOST3, true);
            }
        } break;
        case BT_2_4G: {
            if (dev_info.devs == DEVS_2_4G) {
                bt_switch_mode(dev_info.devs, DEVS_2_4G, true);
            }
        } break;
        case SW_OS: {
            if (get_highest_layer(default_layer_state) == 0) { // WIN_BASE
                set_single_persistent_default_layer(2);
                single_blink_time  = timer_read32();
                single_blink_cnt   = 3;
                single_blink_index = 69;
                single_blink_color = (RGB){100, 100, 100};
            } else if (get_highest_layer(default_layer_state) == 2) { // MAC_BASE
                set_single_persistent_default_layer(0);
                keymap_config.no_gui = 0;
                eeconfig_update_keymap(&keymap_config);
                single_blink_time  = timer_read32();
                single_blink_cnt   = 3;
                single_blink_index = 68;
                single_blink_color = (RGB){100, 100, 100};
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
        case KEYBOARD_RESET: {
            if (!factory_reset_status) {
                factory_reset_status    = 2;
                factory_reset_press_cnt = 1;
                rgb_matrix_enable_noeeprom();
                rgb_status_save          = rgb_matrix_config.enable;
                factory_reset_press_time = timer_read32();
            }
        } break;
        case BLE_RESET: {
            if (!factory_reset_status) {
                factory_reset_status    = 3;
                factory_reset_press_cnt = 1;
                rgb_matrix_enable_noeeprom();
                rgb_status_save          = rgb_matrix_config.enable;
                factory_reset_press_time = timer_read32();
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

static void bt_used_pin_init(void) {
#ifdef BT_MODE_SW_PIN
    setPinInputHigh(BT_MODE_SW_PIN);
    setPinInputHigh(RF_MODE_SW_PIN);
#endif

#if defined(BT_CABLE_PIN) && defined(BT_CHARGE_PIN)
    setPinInputHigh(BT_CABLE_PIN);
    setPinInput(BT_CHARGE_PIN);
#endif
}

/**
 * @brief 根据波动开关判断工作模式
 * @param None
 * @return None
 */
static uint8_t current_devs = 0;
static void    bt_scan_mode(void) {
#ifdef BT_MODE_SW_PIN
    if (readPin(RF_MODE_SW_PIN) && !readPin(BT_MODE_SW_PIN)) {
        if ((dev_info.devs == DEVS_USB) || (dev_info.devs == DEVS_2_4G)) bt_switch_mode(dev_info.devs, dev_info.last_devs, false); // BT mode
    }
    if (readPin(BT_MODE_SW_PIN) && !readPin(RF_MODE_SW_PIN)) {
        if (dev_info.devs != DEVS_2_4G) bt_switch_mode(dev_info.devs, DEVS_2_4G, false); // 2_4G mode
    }
    if (readPin(BT_MODE_SW_PIN) && readPin(RF_MODE_SW_PIN)) {
        if (dev_info.devs != DEVS_USB) bt_switch_mode(dev_info.devs, DEVS_USB, false); // usb mode
    }
#endif
    static bool cable_status; // 1为插入，0为断开
    static bool cable_old_status;
    cable_status = !readPin(BT_CABLE_PIN);
    current_devs = dev_info.devs;
    if (cable_old_status != cable_status) {
        cable_old_status = cable_status;
        if (cable_status) { // 当USB从断开到连接状态时，回到有线状态
            if (dev_info.devs != DEVS_USB) bt_switch_mode(dev_info.devs, DEVS_USB, false);
        } else {
            bt_switch_mode(dev_info.devs, dev_info.last_devs, false);
        }
    }
}

void bt_bat_low_indicator(void) {
    if (!kb_sleep_flag && (indicator_status == 0) && bts_info.bt_info.pvol < 20) {
        if (Low_power_blink_time && timer_elapsed(Low_power_blink_time) < (5 * 1000)) {
            for (uint8_t i = 84; i <= 86; i++) {
                rgb_matrix_set_color(i, 100, 0, 0);
            }
        } else {
            Low_power_blink_time = 0;
        }
    }
}

void led_off_standby(void) {
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
        if (kb_sleep_flag || ((timer_elapsed32(key_press_time) >= sleep_time_table[dev_info.sleep_mode]) && (sleep_time_table[dev_info.sleep_mode] != 0))) {
            bak_rgb_toggle = rgb_matrix_config.enable;
            sober          = false;
            close_rgb_time = timer_read32();
            rgb_matrix_disable_noeeprom();
            writePinLow(RGB_DRIVER_SDB_PIN);
        }
    } else {
        if (!rgb_matrix_config.enable) {
            if (timer_elapsed32(close_rgb_time) >= ENTRY_STOP_TIMEOUT) {
                /* Turn off all indicators led */
                if (led_inited) {
                    led_deconfig_all();
                }

#ifdef ENTRY_STOP_MODE
                lp_system_sleep();
#endif
                extern void open_rgb(void);
                open_rgb();
            }
        }
    }
}

void open_rgb(void) {
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

    // bt_bat_low_indicator();
}

// bool entry_low_pow_flag = false;

void bt_led(void) {
    // if (dev_info.devs != DEVS_USB) {
    uint8_t         rgb_index       = rgb_index_table[dev_info.devs];
    static uint32_t last_time       = 0;
    static uint32_t last_long_time  = 0;
    static uint32_t last_total_time = 0;
    static uint8_t  last_status     = 0;
    static bool     rgb_flip        = false;
    static RGB      rgb             = {0};

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

            /* 超时退出 */
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
            if (bts_info.bt_info.come_back_err) {
                last_time        = timer_read32();
                indicator_status = 1;
            }
            /* 超时退出 */
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
                // rgb = (RGB){.r = 0, .g = 0, .b = 0};
                Low_power_blink_time = timer_read32();
                indicator_status     = 4;
            }
            // bt_bat_low_indicator();
        } break;
        case 4: { // 长灭模式
            rgb              = (RGB){.r = 0, .g = 0, .b = 0};
            indicator_status = 0;
        } break;
        default:
            rgb_flip = false;
            if (!kb_sleep_flag) {
                if (!bts_info.bt_info.paired) {
                    indicator_status = 2;
                    if ((dev_info.devs != DEVS_USB) && (dev_info.devs != DEVS_2_4G)) {
                        bt_switch_mode(DEVS_USB, dev_info.last_devs, false);
                    }
                    if (dev_info.devs == DEVS_2_4G) {
                        bt_switch_mode(DEVS_USB, DEVS_2_4G, false);
                    }
                }
            }
            // rgb_matrix_set_color_all(0, 0, 0);
            break;
    }
    if (indicator_status) rgb_matrix_set_color(rgb_index, rgb.r, rgb.g, rgb.b);
}

void usb_led(void) {
    // } else {
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
    // }
}

uint8_t bt_indicator_rgb(uint8_t led_min, uint8_t led_max) {
    // FN 按下时显示当前设备状态
    if ((get_highest_layer(default_layer_state | layer_state) == 1) || (get_highest_layer(default_layer_state | layer_state) == 3)) {
        rgb_matrix_set_color(rgb_index_table[dev_info.devs], RGB_BLUE);
    }

    if (factory_reset_status) {
        if (timer_elapsed32(factory_reset_press_time) >= factory_reset_press_cnt * 500) {
            factory_reset_press_cnt++;
        }
        if (factory_reset_press_cnt >= 7) {
            switch (factory_reset_status) {
                case 1: // factory reset
                    eeconfig_init();
                    eeconfig_update_rgb_matrix_default();
                    rgb_info.ind_brightness  = RGB_MATRIX_DEFAULT_VAL;
                    rgb_info.ind_color_index = 0;
                    rgb_info.rgb_tog_flag    = 0;
                    eeconfig_update_kb(rgb_info.raw);
                    keymap_config.no_gui = 0;
                    eeconfig_update_keymap(&keymap_config);
                    if (dev_info.devs != DEVS_2_4G && dev_info.devs != DEVS_USB) {
                        bts_send_vendor(v_clear);
                    }
                    break;
                case 2: // keyboard reset
                    layer_save = get_highest_layer(default_layer_state);
                    eeconfig_init();
                    set_single_persistent_default_layer(layer_save);
                    break;
                case 3: // ble reset
                    if (dev_info.devs != DEVS_2_4G && dev_info.devs != DEVS_USB) {
                        bts_send_vendor(v_clear);
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
        switch (factory_reset_status) {
            case 1: // factory reset
                rgb_matrix_set_color(29, 100, 0, 0);
                break;
            case 2: // keyboard reset
                rgb_matrix_set_color(52, 100, 0, 0);
                break;
            case 3: // ble reset
                rgb_matrix_set_color(51, 100, 0, 0);
                break;
            default:
                break;
        }
        return false;
    }
    /*************************************************************************************/
    if (all_blink_cnt) { // 全键闪烁
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
    if (single_blink_cnt) { // 单键闪烁
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
    /*************************************************************************************/

    if (dev_info.devs != DEVS_USB) {
        bt_led();
    } else {
        usb_led();
    }

#if defined(BT_CABLE_PIN) && defined(BT_CHARGE_PIN)
    // 充电接入
    if (!readPin(BT_CABLE_PIN)) {
        // 正在充电
        if (readPin(BT_CHARGE_PIN)) {
            // 充满
            charge_full_blink_time = timer_read32();
        }

        if (charge_full_blink_time && timer_elapsed(charge_full_blink_time) < (5 * 1000)) {
            for (uint8_t i = 84; i <= 86; i++) {
                rgb_matrix_set_color(i, 0, 100, 0);
            }
        } else {
            charge_full_blink_time = 0;
        }
    } else {
        if (dev_info.devs != DEVS_USB) {
            bt_bat_low_indicator();

            extern bool low_vol_offed_sleep;
            if (bts_info.bt_info.low_vol_offed) {
                kb_sleep_flag       = true;
                low_vol_offed_sleep = true;
            }

            if (query_vol_flag) {
                // rgb_matrix_set_color_all(0, 0, 0);
                uint8_t query_index[10] = {17, 18, 19, 20, 21, 22, 23, 24, 25, 26};
                if (bts_info.bt_info.pvol < 30) {
                    for (uint8_t i = 0; i < (bts_info.bt_info.pvol / 10); i++) {
                        rgb_matrix_set_color(query_index[i], 100, 0, 0);
                    }
                } else if (bts_info.bt_info.pvol < 40) {
                    for (uint8_t i = 0; i < (bts_info.bt_info.pvol / 10); i++) {
                        rgb_matrix_set_color(query_index[i], 100, 50, 0);
                    }
                } else if (bts_info.bt_info.pvol < 50) {
                    for (uint8_t i = 0; i < (bts_info.bt_info.pvol / 10); i++) {
                        rgb_matrix_set_color(query_index[i], 100, 50, 0);
                    }
                } else if (bts_info.bt_info.pvol < 60) {
                    for (uint8_t i = 0; i < (bts_info.bt_info.pvol / 10); i++) {
                        rgb_matrix_set_color(query_index[i], 100, 50, 0);
                    }
                } else if (bts_info.bt_info.pvol < 70) {
                    for (uint8_t i = 0; i < (bts_info.bt_info.pvol / 10); i++) {
                        rgb_matrix_set_color(query_index[i], 0, 0, 100);
                    }
                } else if (bts_info.bt_info.pvol < 80) {
                    for (uint8_t i = 0; i < (bts_info.bt_info.pvol / 10); i++) {
                        rgb_matrix_set_color(query_index[i], 0, 0, 100);
                    }
                } else if (bts_info.bt_info.pvol < 90) {
                    for (uint8_t i = 0; i < (bts_info.bt_info.pvol / 10); i++) {
                        rgb_matrix_set_color(query_index[i], 0, 0, 100);
                    }
                } else {
                    for (uint8_t i = 0; i < (bts_info.bt_info.pvol / 10); i++) {
                        rgb_matrix_set_color(query_index[i], 0, 0, 100);
                    }
                }
            }
        }
    }
#endif
    return true;
}
