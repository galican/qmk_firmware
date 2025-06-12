/**
 * @file bt_task.c
 * @brief
 * @author JoyLee
 * @version 2.0.0
 * @date 2023-04-06
 *
 * @copyright Copyright (c) 2023 Westberry Technology Corp., Ltd
 */

#include QMK_KEYBOARD_H

#include "quantum.h"
#include "uart.h"
#include "report.h"
#include "usb_main.h"
#include "common/bt_task.h"

#define NUM_LONG_PRESS_KEYS (sizeof(long_pressed_keys) / sizeof(long_pressed_keys_t))

#ifdef BT_DEBUG_MODE
#    define BT_DEBUG_INFO(fmt, ...) dprintf(fmt, ##__VA_ARGS__)
#else
#    define BT_DEBUG_INFO(fmt, ...)
#endif

typedef struct {
    uint32_t press_time;
    uint16_t keycode;
    void (*event_cb)(uint16_t);
} long_pressed_keys_t;

uint32_t   bt_init_time = 0;
dev_info_t dev_info     = {0};
bts_info_t bts_info     = {
        .bt_name        = {"FC220TP_$", "FC220TP_$", "FC220TP_$"},
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
static void bt_mousekey_task(void);
// clang-format off
long_pressed_keys_t long_pressed_keys[] = {
    {.keycode = BT_HOST1, .press_time = 0, .event_cb = long_pressed_keys_cb},
    {.keycode = BT_HOST2, .press_time = 0, .event_cb = long_pressed_keys_cb},
    {.keycode = BT_HOST3, .press_time = 0, .event_cb = long_pressed_keys_cb},
    {.keycode = BT_2_4G, .press_time = 0, .event_cb = long_pressed_keys_cb},

//   {.keycode = BT_PAIR, .press_time = 0, .event_cb = long_pressed_keys_cb},
//   {.keycode = EX_PAIR, .press_time = 0, .event_cb = long_pressed_keys_cb},
//   {.keycode = GU_TOGG, .press_time = 0, .event_cb = long_pressed_keys_cb},
//   {.keycode = SW_OS, .press_time = 0, .event_cb = long_pressed_keys_cb},
//   {.keycode = RGB_TEST, .press_time = 0, .event_cb = long_pressed_keys_cb},

  {.keycode = NL_OFF, .press_time = 0, .event_cb = long_pressed_keys_cb},

  {.keycode = RST_ALL, .press_time = 0, .event_cb = long_pressed_keys_cb},
  {.keycode = RST_LYR, .press_time = 0, .event_cb = long_pressed_keys_cb},
  {.keycode = RST_BLE, .press_time = 0, .event_cb = long_pressed_keys_cb},
};
// clang-format on
void register_mouse(uint8_t mouse_keycode, bool pressed);
/** \brief Utilities for actions. (FIXME: Needs better description)
 *
 * FIXME: Needs documentation.
 */
__attribute__((weak)) void register_code(uint8_t code) {
    if (dev_info.devs) {
        bts_process_keys(code, 1, dev_info.devs, keymap_config.no_gui);
        bts_task(dev_info.devs);
        while (bts_is_busy()) {}
    } else {
        if (code == KC_NO) {
            return;

#ifdef LOCKING_SUPPORT_ENABLE
        } else if (KC_LOCKING_CAPS_LOCK == code) {
#    ifdef LOCKING_RESYNC_ENABLE
            // Resync: ignore if caps lock already is on
            if (host_keyboard_leds() & (1 << USB_LED_CAPS_LOCK)) return;
#    endif
            add_key(KC_CAPS_LOCK);
            send_keyboard_report();
            wait_ms(TAP_HOLD_CAPS_DELAY);
            del_key(KC_CAPS_LOCK);
            send_keyboard_report();

        } else if (KC_LOCKING_NUM_LOCK == code) {
#    ifdef LOCKING_RESYNC_ENABLE
            if (host_keyboard_leds() & (1 << USB_LED_NUM_LOCK)) return;
#    endif
            add_key(KC_NUM_LOCK);
            send_keyboard_report();
            wait_ms(100);
            del_key(KC_NUM_LOCK);
            send_keyboard_report();

        } else if (KC_LOCKING_SCROLL_LOCK == code) {
#    ifdef LOCKING_RESYNC_ENABLE
            if (host_keyboard_leds() & (1 << USB_LED_SCROLL_LOCK)) return;
#    endif
            add_key(KC_SCROLL_LOCK);
            send_keyboard_report();
            wait_ms(100);
            del_key(KC_SCROLL_LOCK);
            send_keyboard_report();
#endif

        } else if (IS_BASIC_KEYCODE(code)) {
            // TODO: should push command_proc out of this block?
            if (command_proc(code)) return;

            // Force a new key press if the key is already pressed
            // without this, keys with the same keycode, but different
            // modifiers will be reported incorrectly, see issue #1708
            if (is_key_pressed(code)) {
                del_key(code);
                send_keyboard_report();
            }
            add_key(code);
            send_keyboard_report();
        } else if (IS_MODIFIER_KEYCODE(code)) {
            add_mods(MOD_BIT(code));
            send_keyboard_report();

#ifdef EXTRAKEY_ENABLE
        } else if (IS_SYSTEM_KEYCODE(code)) {
            host_system_send(KEYCODE2SYSTEM(code));
        } else if (IS_CONSUMER_KEYCODE(code)) {
            host_consumer_send(KEYCODE2CONSUMER(code));
#endif

        } else if (IS_MOUSE_KEYCODE(code)) {
            register_mouse(code, true);
        }
    }
}

/** \brief Utilities for actions. (FIXME: Needs better description)
 *
 * FIXME: Needs documentation.
 */
__attribute__((weak)) void unregister_code(uint8_t code) {
    if (dev_info.devs) {
        bts_process_keys(code, 0, dev_info.devs, keymap_config.no_gui);
        bts_task(dev_info.devs);
        while (bts_is_busy()) {}
    } else {
        if (code == KC_NO) {
            return;

#ifdef LOCKING_SUPPORT_ENABLE
        } else if (KC_LOCKING_CAPS_LOCK == code) {
#    ifdef LOCKING_RESYNC_ENABLE
            // Resync: ignore if caps lock already is off
            if (!(host_keyboard_leds() & (1 << USB_LED_CAPS_LOCK))) return;
#    endif
            add_key(KC_CAPS_LOCK);
            send_keyboard_report();
            del_key(KC_CAPS_LOCK);
            send_keyboard_report();

        } else if (KC_LOCKING_NUM_LOCK == code) {
#    ifdef LOCKING_RESYNC_ENABLE
            if (!(host_keyboard_leds() & (1 << USB_LED_NUM_LOCK))) return;
#    endif
            add_key(KC_NUM_LOCK);
            send_keyboard_report();
            del_key(KC_NUM_LOCK);
            send_keyboard_report();

        } else if (KC_LOCKING_SCROLL_LOCK == code) {
#    ifdef LOCKING_RESYNC_ENABLE
            if (!(host_keyboard_leds() & (1 << USB_LED_SCROLL_LOCK))) return;
#    endif
            add_key(KC_SCROLL_LOCK);
            send_keyboard_report();
            del_key(KC_SCROLL_LOCK);
            send_keyboard_report();
#endif

        } else if (IS_BASIC_KEYCODE(code)) {
            del_key(code);
            send_keyboard_report();
        } else if (IS_MODIFIER_KEYCODE(code)) {
            del_mods(MOD_BIT(code));
            send_keyboard_report();

#ifdef EXTRAKEY_ENABLE
        } else if (IS_SYSTEM_KEYCODE(code)) {
            host_system_send(0);
        } else if (IS_CONSUMER_KEYCODE(code)) {
            host_consumer_send(0);
#endif

        } else if (IS_MOUSE_KEYCODE(code)) {
            register_mouse(code, false);
        }
    }
}
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
    (void)arg;
    chRegSetThreadName("blinker");
    while (true) {
        bts_task(dev_info.devs);
        chThdSleepMilliseconds(1);
    }
}

static bool rgb_status_save = 1;
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
    chThdCreateStatic(waThread1, sizeof(waThread1), HIGHPRIO, Thread1, NULL);
    bt_scan_mode();
    bts_send_name(DEVS_HOST1);
    wait_ms(10);

    if (dev_info.devs != DEVS_USB) {
        usbDisconnectBus(&USB_DRIVER);
        usbStop(&USB_DRIVER);
        writePinHigh(A12);
    }
    if (dev_info.devs == DEVS_USB) {
        writePinLow(A14);
    } else {
        writePinHigh(A14);
    }

    rgb_status_save = rgb_matrix_config.enable;
}
uint16_t bt_mod_tap_pressed_time;
uint16_t bt_mod_tap_released_time;
uint16_t bt_mod_tap_keycode;
void bt_modcode_send(uint16_t keycode, bool pressed){
    if ((keycode > QK_MODS) && (keycode <= QK_MODS_MAX)) {
        if (QK_MODS_GET_MODS(keycode) & 0x1) {
            if(QK_MODS_GET_MODS(keycode) & 0x10)
                bts_process_keys(KC_RCTL, pressed, dev_info.devs, keymap_config.no_gui);
            else
                bts_process_keys(KC_LCTL, pressed, dev_info.devs, keymap_config.no_gui);
        }
        if (QK_MODS_GET_MODS(keycode) & 0x2) {
            if (QK_MODS_GET_MODS(keycode) & 0x10)
                bts_process_keys(KC_RSFT, pressed, dev_info.devs, keymap_config.no_gui);
            else
                bts_process_keys(KC_LSFT, pressed, dev_info.devs, keymap_config.no_gui);
        }
        if (QK_MODS_GET_MODS(keycode) & 0x4) {
            if (QK_MODS_GET_MODS(keycode) & 0x10)
                bts_process_keys(KC_RALT, pressed, dev_info.devs, keymap_config.no_gui);
            else
                bts_process_keys(KC_LALT, pressed, dev_info.devs, keymap_config.no_gui);
        }
        if (QK_MODS_GET_MODS(keycode) & 0x8) {
            if (QK_MODS_GET_MODS(keycode) & 0x10)
                bts_process_keys(KC_RGUI, pressed, dev_info.devs, keymap_config.no_gui);
            else
                bts_process_keys(KC_LGUI, pressed, dev_info.devs, keymap_config.no_gui);
        }
    } else if ((keycode > QK_MOD_TAP) && (keycode <= QK_MOD_TAP_MAX)) {
        if (QK_MOD_TAP_GET_MODS(keycode) & 0x1) {
            if(QK_MOD_TAP_GET_MODS(keycode) & 0x10)
                bts_process_keys(KC_RCTL, pressed, dev_info.devs, keymap_config.no_gui);
            else
                bts_process_keys(KC_LCTL, pressed, dev_info.devs, keymap_config.no_gui);
        }
        if (QK_MOD_TAP_GET_MODS(keycode) & 0x2) {
            if (QK_MOD_TAP_GET_MODS(keycode) & 0x10)
                bts_process_keys(KC_RSFT, pressed, dev_info.devs, keymap_config.no_gui);
            else
                bts_process_keys(KC_LSFT, pressed, dev_info.devs, keymap_config.no_gui);
        }
        if (QK_MOD_TAP_GET_MODS(keycode) & 0x4) {
            if (QK_MOD_TAP_GET_MODS(keycode) & 0x10)
                bts_process_keys(KC_RALT, pressed, dev_info.devs, keymap_config.no_gui);
            else
                bts_process_keys(KC_LALT, pressed, dev_info.devs, keymap_config.no_gui);
        }
        if (QK_MOD_TAP_GET_MODS(keycode) & 0x8) {
            if (QK_MOD_TAP_GET_MODS(keycode) & 0x10)
                bts_process_keys(KC_RGUI, pressed, dev_info.devs, keymap_config.no_gui);
            else
                bts_process_keys(KC_LGUI, pressed, dev_info.devs, keymap_config.no_gui);
        }
    }
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

        switch (dev_info.devs) {
            case DEVS_HOST1: {
                bts_send_vendor(v_host1);
            } break;
            case DEVS_HOST2: {
                bts_send_vendor(v_host2);
                // bts_send_name(DEVS_HOST2);
            } break;
            case DEVS_HOST3: {
                bts_send_vendor(v_host3);
                // bts_send_name(DEVS_HOST3);
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
        bts_send_vendor(v_en_sleep_bt);
    }

    /* Execute every 1ms */
    if (timer_elapsed32(last_time) >= 1) {
        last_time = timer_read32();

        // 设定指示灯状态
        if (dev_info.devs != DEVS_USB) {
            uint8_t keyboard_led_state = 0;
            led_t         *kb_leds = (led_t *)&keyboard_led_state;
            kb_leds->raw           = bts_info.bt_info.indictor_rgb_s;
            usb_device_state_set_leds(keyboard_led_state);

#ifdef RGB_MATRIX_ENABLE
            // if(dev_info.sleep_time_index != 2){
                close_rgb();
            // }
#endif
        }
    }
    if((bt_mod_tap_pressed_time > bt_mod_tap_released_time) && (timer_elapsed(bt_mod_tap_pressed_time) > 100)) {
        if((bt_mod_tap_pressed_time - bt_mod_tap_released_time) > 200)
            bt_modcode_send(bt_mod_tap_keycode, 1);
        bt_mod_tap_pressed_time = 0;
    }
    bt_mousekey_task();
    long_pressed_keys_hook();
    bt_scan_mode();
}

// uint32_t    pressed_time    = 0;
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
        // pressed_time = timer_read32();
        if (!rgb_matrix_config.enable) {
            if (rgb_status_save) {
                rgb_matrix_enable_noeeprom();
            }
        }
    }
    retval = process_record_other(keycode, record);

    if (dev_info.devs != DEVS_USB) {
        if (retval != false) {
            if ((keycode > QK_MODS) && (keycode <= QK_MODS_MAX)) {
                bt_modcode_send(keycode, record->event.pressed);
                retval = bts_process_keys(QK_MODS_GET_BASIC_KEYCODE(keycode), record->event.pressed, dev_info.devs, keymap_config.no_gui);
            } else if ((keycode > QK_MOD_TAP) && (keycode <= QK_MOD_TAP_MAX)) {
                if(record->event.pressed){
                    bt_mod_tap_pressed_time = timer_read();
                    bt_mod_tap_keycode = keycode;
                } else {
                    bt_modcode_send(keycode, 0);
                    bt_mod_tap_released_time = timer_read();
                    bt_mod_tap_pressed_time = 0;
                }
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

void bt_switch_mode(uint8_t last_mode, uint8_t now_mode, uint8_t reset) {
    // pressed_time = timer_read32();
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
                indicator_status = LED_BLE_PAIR;
                indicator_reset_last_time = true;
                // bts_send_name(DEVS_HOST1);
                // bts_send_vendor(v_host1);
                bts_send_vendor(v_pair);
            } else if (last_mode != DEVS_HOST1) {
                indicator_status = LED_BLE_CONN;
                indicator_reset_last_time = true;
                bts_send_vendor(v_host1);
            }
        } break;
        case DEVS_HOST2: {
            if (reset != false) {
                indicator_status = LED_BLE_PAIR;
                indicator_reset_last_time = 0;
                // bts_send_name(DEVS_HOST2);
                // bts_send_vendor(v_host2);
                bts_send_vendor(v_pair);
            } else if (last_mode != DEVS_HOST2) {
                indicator_status = LED_BLE_CONN;
                indicator_reset_last_time = true;
                bts_send_vendor(v_host2);
            }
        } break;
        case DEVS_HOST3: {
            if (reset != false) {
                indicator_status = LED_BLE_PAIR;
                indicator_reset_last_time = true;
                // bts_send_name(DEVS_HOST3);
                // bts_send_vendor(v_host3);
                bts_send_vendor(v_pair);
            } else if (last_mode != DEVS_HOST3) {
                indicator_status = LED_BLE_CONN;
                indicator_reset_last_time = true;
                bts_send_vendor(v_host3);
            }
        } break;
        case DEVS_2_4G: {
            if (reset != false) {
                indicator_status = LED_BLE_PAIR;
                indicator_reset_last_time = true;
                // bts_send_name(DEVS_2_4G);
                // bts_send_vendor(v_2_4g);
                bts_send_vendor(v_pair);
            } else if (last_mode != DEVS_2_4G) {
                indicator_status = LED_BLE_CONN;
                indicator_reset_last_time = true;
                bts_send_vendor(v_2_4g);
            }
        } break;
        case DEVS_USB: {
            bts_send_vendor(v_usb);
        } break;
        default:
            break;
    }
}
typedef struct {
    bool dir;
    bool pressed;
} move_t;
typedef struct {
    bool dir;
    bool pressed;
} wheel_t;
typedef struct {
    uint16_t pressed_time;
    move_t   move_x;
    move_t   move_y;
    wheel_t  wheel_x;
    wheel_t  wheel_y;
    uint8_t  data[5];
} bt_mousekey_t;
static bt_mousekey_t bt_mousekey;

uint16_t bt_mousekey_move_send_time;
uint16_t bt_mousekey_wheel_send_time;
// static uint8_t bt_mousekey_send_speed;

void bt_mousekey_task(void) {
    if (bt_mousekey.move_x.pressed || bt_mousekey.move_y.pressed) {
        if (bt_mousekey.move_x.pressed) {
            if (bt_mousekey.move_x.dir)
                bt_mousekey.data[1] = 0xfe;
            else
                bt_mousekey.data[1] = 0x02;
        } else {
            bt_mousekey.data[1] = 0;
        }
        if (bt_mousekey.move_y.pressed) {
            if (bt_mousekey.move_y.dir)
                bt_mousekey.data[2] = 0xfe;
            else
                bt_mousekey.data[2] = 0x02;
        } else {
            bt_mousekey.data[2] = 0;
        }
        bt_mousekey.data[4] = 0;
        bt_mousekey.data[3] = 0;
        if (timer_elapsed(bt_mousekey_move_send_time) >= 10) {
            bt_mousekey_move_send_time = timer_read();
            bts_send_mouse_report(bt_mousekey.data);
        }
    }
    if (bt_mousekey.wheel_x.pressed || bt_mousekey.wheel_y.pressed) {
        bt_mousekey.data[1] = 0;
        bt_mousekey.data[2] = 0;
        if (bt_mousekey.wheel_x.pressed) {
            if (bt_mousekey.wheel_x.dir)
                bt_mousekey.data[4] = 0xff;
            else
                bt_mousekey.data[4] = 0x01;
        } else {
            bt_mousekey.data[4] = 0;
        }
        if (bt_mousekey.wheel_y.pressed) {
            if (bt_mousekey.wheel_y.dir)
                bt_mousekey.data[3] = 0xfe;
            else
                bt_mousekey.data[3] = 0x01;
        } else {
            bt_mousekey.data[3] = 0;
        }
        if (timer_elapsed(bt_mousekey_wheel_send_time) >= 100) {
            bt_mousekey_wheel_send_time = timer_read();
            bts_send_mouse_report(bt_mousekey.data);
        }
    }
}

static uint16_t bt_tap_time;
static uint32_t EE_CLR_press_cnt  = 0;
static uint32_t EE_CLR_press_time = 0;
static uint8_t  EE_CLR_flag       = false;
bool            query_vol_flag    = false;
static bool     kb_sleep_flag = false;
uint8_t         all_blink_cnt;
uint32_t        all_blink_time;
uint8_t         single_blink_cnt;
uint8_t         single_blink_index;
RGB      single_blink_color;
uint32_t single_blink_time;

bool bt_mode_sw_get(void) {
    if(readPin(BT_MODE_SW_PIN) == BT_MODE_SW_TRUE) {
        return true;
    } else {
        return false;
    }
}

void led_status_set(bool status) {
    // extern bool rgb_status_save;
    // switch (rgb_matrix_get_flags()) {
    //     case LED_FLAG_ALL: {
    //         rgb_matrix_set_flags(LED_FLAG_NONE);
    //         rgb_matrix_set_color_all(RGB_OFF);
    //         // rgb_status_save = 0;
    //     } break;

    //     default: {
    //         rgb_matrix_set_flags(LED_FLAG_ALL);
    //         // rgb_status_save = 1;
    //     } break;
    // }

    // if (dev_info.led_status == false) {
    if (status == false) {
        rgb_matrix_set_flags(LED_FLAG_NONE);
        rgb_matrix_set_color_all(RGB_OFF);
    } else {
        rgb_matrix_set_flags(LED_FLAG_ALL);
    }
}

void led_status_set_sw(void) {
    // static bool init = false;
    static bool bt_mode_sw_prev = false;
    bool bt_mode_sw_curr = bt_mode_sw_get();

    if(bt_mode_sw_prev != bt_mode_sw_curr) {
        bt_mode_sw_prev = bt_mode_sw_curr;

        // if(bt_mode_sw_curr) {
        //     rgb_matrix_set_flags(LED_FLAG_NONE);
        //     rgb_matrix_set_color_all(RGB_OFF);
        // } else {
        //     rgb_matrix_set_flags(LED_FLAG_ALL);
        // }
        led_status_set(!bt_mode_sw_curr);
    }

    // if(!init) {
    //     if(bt_mode_sw_get()) {
    //         // init = true;

    //         rgb_matrix_set_flags(LED_FLAG_NONE);
    //         rgb_matrix_set_color_all(RGB_OFF);
    //     } else {
    //         rgb_matrix_set_flags(LED_FLAG_ALL);
    //     }
    // } else {
    // }
}

void led_status_set_watch(void) {
    static bool led_status_prev = false;

    if(led_status_prev != dev_info.led_status) {
        led_status_prev = dev_info.led_status;

        led_status_set(led_status_prev);
    }
}

// void rgb_tog_off(void) {
//     // extern bool rgb_status_save;
//     switch (rgb_matrix_get_flags()) {
//         case LED_FLAG_ALL: {
//             rgb_matrix_set_flags(LED_FLAG_NONE);
//             rgb_matrix_set_color_all(RGB_OFF);
//             // rgb_status_save = 0;
//         } break;

//         default: {
//             rgb_matrix_set_flags(LED_FLAG_ALL);
//             // rgb_status_save = 1;
//         } break;
//     }
// }

void sleep_time_index_set(void) {
    // dev_info.sleep_time_index = (dev_info.sleep_time_index + 1) % 3;

    // all_blink_time  = timer_read32();
    // all_blink_cnt   = (dev_info.sleep_time_index + 1) * 2;


    // if (dev_info.sleep_time_index )
    dev_info.sleep_time_index++;

    led_single_blink_set((dev_info.sleep_time_index + 1) * 2, LED_SLEEP, RGB_BLUE);
}

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
        // case BT_HOST: {
        //     if (record->event.pressed) {   
        //         // if (readPin(!BT_MODE_SW_PIN)) {
        //         if (bt_mode_sw_get()) {
        //             if (dev_info.devs == DEVS_USB || dev_info.devs == DEVS_2_4G || dev_info.devs == DEVS_HOST3) {
        //                 bt_switch_mode(dev_info.devs, DEVS_HOST1, false);
        //             } else if (dev_info.devs == DEVS_HOST1) {
        //                 bt_switch_mode(dev_info.devs, DEVS_HOST2, false);
        //             } else if (dev_info.devs == DEVS_HOST2) {
        //                 bt_switch_mode(dev_info.devs, DEVS_HOST3, false);
        //             }
        //         } else {
        //         }
        //     }
        // } break;

        // case BT_PAIR: {
        // } break;

        // case EX_PAIR: {
        //     if (record->event.pressed) {
        //         if (dev_info.devs != DEVS_USB) {
        //             bt_switch_mode(DEVS_USB, dev_info.devs, false);
        //         }
        //     }
        // } break;
        
        case BT_HOST1: {
            if (record->event.pressed) {
                // if ((dev_info.devs != DEVS_HOST1) && (dev_info.devs != DEVS_USB) && (dev_info.devs != DEVS_2_4G)) {
                if (bt_mode_sw_get()) {
                    bt_switch_mode(dev_info.devs, DEVS_HOST1, false);
                    // if(!(dev_info.config.first_boot&(0x1<<0))){
                    //     bt_switch_mode(dev_info.devs, DEVS_HOST1, true); // BT mode
                    // }
                }
            }
        } break;

        case BT_HOST2: {
            if (record->event.pressed) {
                // if ((dev_info.devs != DEVS_HOST2) && (dev_info.devs != DEVS_USB) && (dev_info.devs != DEVS_2_4G)) {
                if (bt_mode_sw_get()) {
                    bt_switch_mode(dev_info.devs, DEVS_HOST2, false);
                    // if(!(dev_info.config.first_boot&(0x1<<1))){
                        // bt_switch_mode(dev_info.devs, DEVS_HOST2, true); // BT mode
                    // }
                }
            }
        } break;

        case BT_HOST3: {
            if (record->event.pressed) {
                if (bt_mode_sw_get()) {
                // if ((dev_info.devs != DEVS_HOST3) && (dev_info.devs != DEVS_USB) && (dev_info.devs != DEVS_2_4G)) {
                    bt_switch_mode(dev_info.devs, DEVS_HOST3, false);
                    // if(!(dev_info.config.first_boot&(0x1<<2))){
                    //     bt_switch_mode(dev_info.devs, DEVS_HOST3, true); // BT mode
                    // }
                }
            }
        } break;

        case BT_2_4G: {
            if (record->event.pressed) {
                if (bt_mode_sw_get()) {
                    if (dev_info.devs != DEVS_2_4G) {
                        bt_switch_mode(dev_info.devs, DEVS_2_4G, false);
                    }
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

        case SLEEP_EN: {
            if (record->event.pressed) {
                sleep_time_index_set();
            }
        } break;

        case RGB_TEST: {
            if (record->event.pressed) {
                extern uint8_t rgb_test_en;
                rgb_test_en = !rgb_test_en;
            }
        } break;

        case RGB_TOG:{
            if (record->event.pressed) {
                // rgb_tog_off();
                dev_info.led_status = !dev_info.led_status;
            }
        } return false;

        case EE_CLR: {
        } break;
        // case GU_TOGG:
        // case SW_OS: // OS switch key
        //     if (record->event.pressed) {
        //         if (get_highest_layer(default_layer_state) == 0) { // WIN_BASE
        //             set_single_persistent_default_layer(1);
        //         } else if (get_highest_layer(default_layer_state) == 1) { // MAC_BASE
        //             set_single_persistent_default_layer(0);
        //         }
        //     }
        //     break;
        case QK_GRAVE_ESCAPE:
            if (dev_info.devs) {
                if (record->event.pressed) {
                    if ((get_mods() & MOD_MASK_SG)) {
                        bts_process_keys(KC_GRAVE, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                    } else {
                        bts_process_keys(KC_ESCAPE, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                    }
                } else {
                    bts_process_keys(KC_GRAVE, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                    bts_process_keys(KC_ESCAPE, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                }
            }
            return true; // Skip all further processing of this key
        case LM(1, MOD_LCTL):
        case LM(2, MOD_LCTL):
        case LM(3, MOD_LCTL):
            if (dev_info.devs) {
                bts_process_keys(KC_LCTL, record->event.pressed, dev_info.devs, keymap_config.no_gui);
            }
            return true;
        case LM(1, MOD_LSFT):
        case LM(2, MOD_LSFT):
        case LM(3, MOD_LSFT):
            if (dev_info.devs) {
                bts_process_keys(KC_LSFT, record->event.pressed, dev_info.devs, keymap_config.no_gui);
            }
            return true;
        case LM(1, MOD_LALT):
        case LM(2, MOD_LALT):
        case LM(3, MOD_LALT):
            if (dev_info.devs) {
                bts_process_keys(KC_LALT, record->event.pressed, dev_info.devs, keymap_config.no_gui);
            }
            return true;
        case LM(1, MOD_LGUI):
        case LM(2, MOD_LGUI):
        case LM(3, MOD_LGUI):
            if (dev_info.devs) {
                bts_process_keys(KC_LGUI, record->event.pressed, dev_info.devs, keymap_config.no_gui);
            }
            return true;
        case LM(1, MOD_RCTL):
        case LM(2, MOD_RCTL):
        case LM(3, MOD_RCTL):
            if (dev_info.devs) {
                bts_process_keys(KC_RCTL, record->event.pressed, dev_info.devs, keymap_config.no_gui);
            }
            return true;
        case LM(1, MOD_RSFT):
        case LM(2, MOD_RSFT):
        case LM(3, MOD_RSFT):
            if (dev_info.devs) {
                bts_process_keys(KC_RSFT, record->event.pressed, dev_info.devs, keymap_config.no_gui);
            }
            return true;
        case LM(1, MOD_RALT):
        case LM(2, MOD_RALT):
        case LM(3, MOD_RALT):
            if (dev_info.devs) {
                bts_process_keys(KC_RALT, record->event.pressed, dev_info.devs, keymap_config.no_gui);
            }
            return true;
        case LM(1, MOD_RGUI):
        case LM(2, MOD_RGUI):
        case LM(3, MOD_RGUI):
            if (dev_info.devs) {
                bts_process_keys(KC_RGUI, record->event.pressed, dev_info.devs, keymap_config.no_gui);
            }
            return true;
        case LT(1, KC_SPC):
        case LT(2, KC_SPC):
        case LT(3, KC_SPC):
            if (dev_info.devs) {
                if (record->tap.count && record->event.pressed) {
                    bts_process_keys(KC_SPC, 1, dev_info.devs, keymap_config.no_gui);
                    bts_task(dev_info.devs);
                    for (uint16_t i = TAP_CODE_DELAY; i > 0; i--) {
                        wait_ms(1);
                    }
                    bts_process_keys(KC_SPC, 0, dev_info.devs, keymap_config.no_gui);
                    bts_task(dev_info.devs);
                    return false; // 通过返回false阻止对该键的其它处理
                }
            }
            return true;
        case SC_LCPO:
            if (dev_info.devs) {
                bts_process_keys(KC_LCTL, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                if (record->event.pressed) {
                    bt_tap_time = timer_read();
                } else {
                    if (timer_elapsed(bt_tap_time) <= 100) {
                        bts_process_keys(KC_LSFT, 1, dev_info.devs, keymap_config.no_gui);
                        bts_process_keys(KC_9, 1, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                        // for (uint16_t i = TAP_CODE_DELAY; i > 0; i--) {
                        //     wait_ms(1);
                        // }
                        bts_process_keys(KC_LSFT, 0, dev_info.devs, keymap_config.no_gui);
                        bts_process_keys(KC_9, 0, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                    }
                }
                return false; // 通过返回false阻止对该键的其它处理
            }
            return true;
        case SC_LSPO:
            if (dev_info.devs) {
                bts_process_keys(KC_LSFT, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                if (record->event.pressed) {
                    bt_tap_time = timer_read();
                } else {
                    if (timer_elapsed(bt_tap_time) <= 100) {
                        bts_process_keys(KC_LSFT, 1, dev_info.devs, keymap_config.no_gui);
                        bts_process_keys(KC_9, 1, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                        // for (uint16_t i = TAP_CODE_DELAY; i > 0; i--) {
                        //     wait_ms(1);
                        // }
                        bts_process_keys(KC_LSFT, 0, dev_info.devs, keymap_config.no_gui);
                        bts_process_keys(KC_9, 0, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                    }
                }
                return false; // 通过返回false阻止对该键的其它处理
            }
            return true;
        case SC_LAPO:
            if (dev_info.devs) {
                bts_process_keys(KC_LALT, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                if (record->event.pressed) {
                    bt_tap_time = timer_read();
                } else {
                    if (timer_elapsed(bt_tap_time) <= 100) {
                        bts_process_keys(KC_LSFT, 1, dev_info.devs, keymap_config.no_gui);
                        bts_process_keys(KC_9, 1, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                        // for (uint16_t i = TAP_CODE_DELAY; i > 0; i--) {
                        //     wait_ms(1);
                        // }
                        bts_process_keys(KC_LSFT, 0, dev_info.devs, keymap_config.no_gui);
                        bts_process_keys(KC_9, 0, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                    }
                }
                return false; // 通过返回false阻止对该键的其它处理
            }
            return true;
        case SC_RCPC:
            if (dev_info.devs) {
                bts_process_keys(KC_RCTL, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                if (record->event.pressed) {
                    bt_tap_time = timer_read();
                } else {
                    if (timer_elapsed(bt_tap_time) <= 100) {
                        bts_process_keys(KC_RSFT, 1, dev_info.devs, keymap_config.no_gui);
                        bts_process_keys(KC_0, 1, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                        // for (uint16_t i = TAP_CODE_DELAY; i > 0; i--) {
                        //     wait_ms(1);
                        // }
                        bts_process_keys(KC_RSFT, 0, dev_info.devs, keymap_config.no_gui);
                        bts_process_keys(KC_0, 0, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                    }
                }
                return false; // 通过返回false阻止对该键的其它处理
            }
            return true;
        case SC_RSPC:
            if (dev_info.devs) {
                bts_process_keys(KC_RSFT, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                if (record->event.pressed) {
                    bt_tap_time = timer_read();
                } else {
                    if (timer_elapsed(bt_tap_time) <= 100) {
                        bts_process_keys(KC_RSFT, 1, dev_info.devs, keymap_config.no_gui);
                        bts_process_keys(KC_0, 1, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                        // for (uint16_t i = TAP_CODE_DELAY; i > 0; i--) {
                        //     wait_ms(1);
                        // }
                        bts_process_keys(KC_RSFT, 0, dev_info.devs, keymap_config.no_gui);
                        bts_process_keys(KC_0, 0, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                    }
                }
                return false; // 通过返回false阻止对该键的其它处理
            }
            return true;
        case SC_RAPC:
            if (dev_info.devs) {
                bts_process_keys(KC_RALT, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                if (record->event.pressed) {
                    bt_tap_time = timer_read();
                } else {
                    if (timer_elapsed(bt_tap_time) <= 100) {
                        bts_process_keys(KC_RSFT, 1, dev_info.devs, keymap_config.no_gui);
                        bts_process_keys(KC_0, 1, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                        // for (uint16_t i = TAP_CODE_DELAY; i > 0; i--) {
                        //     wait_ms(1);
                        // }
                        bts_process_keys(KC_RSFT, 0, dev_info.devs, keymap_config.no_gui);
                        bts_process_keys(KC_0, 0, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                    }
                }
                return false; // 通过返回false阻止对该键的其它处理
            }
            return true;
        case SC_SENT:
            if (dev_info.devs) {
                bts_process_keys(KC_RSFT, record->event.pressed, dev_info.devs, keymap_config.no_gui);
                if (record->event.pressed) {
                    bt_tap_time = timer_read();
                } else {
                    if (timer_elapsed(bt_tap_time) <= 100) {
                        bts_process_keys(KC_ENT, 1, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                        // for (uint16_t i = TAP_CODE_DELAY; i > 0; i--) {
                        //     wait_ms(1);
                        // }
                        bts_process_keys(KC_ENT, 0, dev_info.devs, keymap_config.no_gui);
                        bts_task(dev_info.devs);
                    }
                }
                return false; // 通过返回false阻止对该键的其它处理
            }
            return true;
        case KC_MS_U:
            if (dev_info.devs) {
                if (record->event.pressed) {
                    bt_mousekey.move_y.pressed = 1;
                    bt_mousekey.move_y.dir     = 1;
                    // if (!bt_mousekey.pressed_time) bt_mousekey.pressed_time = timer_read();
                } else {
                    if (bt_mousekey.move_y.dir) {
                        bt_mousekey.move_y.pressed = 0;
                        // bt_mousekey.pressed_time   = 0;
                    }
                }
                return false;
            }
            return true;
        case KC_MS_D:
            if (dev_info.devs) {
                if (record->event.pressed) {
                    bt_mousekey.move_y.pressed = 1;
                    bt_mousekey.move_y.dir     = 0;
                    // if (!bt_mousekey.pressed_time) bt_mousekey.pressed_time = timer_read();
                } else {
                    if (!bt_mousekey.move_y.dir) {
                        bt_mousekey.move_y.pressed = 0;
                        // bt_mousekey.pressed_time   = 0;
                    }
                }
                return false;
            }
            return true;
        case KC_MS_L:
            if (dev_info.devs) {
                if (record->event.pressed) {
                    bt_mousekey.move_x.pressed = 1;
                    bt_mousekey.move_x.dir     = 1;
                    // if (!bt_mousekey.pressed_time) bt_mousekey.pressed_time = timer_read();
                } else {
                    if (bt_mousekey.move_x.dir) {
                        bt_mousekey.move_x.pressed = 0;
                        // bt_mousekey.pressed_time   = 0;
                    }
                }
                return false;
            }
            return true;
        case KC_MS_R:
            if (dev_info.devs) {
                if (record->event.pressed) {
                    bt_mousekey.move_x.pressed = 1;
                    bt_mousekey.move_x.dir     = 0;
                    // if (!bt_mousekey.pressed_time) bt_mousekey.pressed_time = timer_read();
                } else {
                    if (!bt_mousekey.move_x.dir) {
                        bt_mousekey.move_x.pressed = 0;
                        // bt_mousekey.pressed_time   = 0;
                    }
                }
                return false;
            }
            return true;
        case KC_WH_U:
            if (dev_info.devs) {
                if (record->event.pressed) {
                    bt_mousekey.wheel_y.pressed = 1;
                    bt_mousekey.wheel_y.dir     = 0;
                    // if (!bt_mousekey.pressed_time) bt_mousekey.pressed_time = timer_read();
                } else {
                    bt_mousekey.wheel_y.pressed = 0;
                    // bt_mousekey.pressed_time   = 0;
                }
                return false;
            }
            return true;
        case KC_WH_D:
            if (dev_info.devs) {
                if (record->event.pressed) {
                    bt_mousekey.wheel_y.pressed = 1;
                    bt_mousekey.wheel_y.dir     = 1;
                    // if (!bt_mousekey.pressed_time) bt_mousekey.pressed_time = timer_read();
                } else {
                    bt_mousekey.wheel_y.pressed = 0;
                    // bt_mousekey.pressed_time   = 0;
                }
                return false;
            }
            return true;
        case KC_WH_L:
            if (dev_info.devs) {
                if (record->event.pressed) {
                    bt_mousekey.wheel_x.pressed = 1;
                    bt_mousekey.wheel_x.dir     = 1;
                    // if (!bt_mousekey.pressed_time) bt_mousekey.pressed_time = timer_read();
                } else {
                    bt_mousekey.wheel_x.pressed = 0;
                    // bt_mousekey.pressed_time   = 0;
                }
                return false;
            }
            return true;
        case KC_WH_R:
            if (dev_info.devs) {
                if (record->event.pressed) {
                    bt_mousekey.wheel_x.pressed = 1;
                    bt_mousekey.wheel_x.dir     = 0;
                    // if (!bt_mousekey.pressed_time) bt_mousekey.pressed_time = timer_read();
                } else {
                    bt_mousekey.wheel_x.pressed = 0;
                    // bt_mousekey.pressed_time   = 0;
                }
                return false;
            }
            return true;
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

        // case BT_PAIR: {
        //     if (dev_info.devs != DEVS_USB) {
        //         bt_switch_mode(DEVS_USB, dev_info.devs, true);
        //     }
        // } break;
        // case BT_2_4G: {
        //     if (dev_info.devs == DEVS_2_4G) {
        //         bt_switch_mode(DEVS_USB, DEVS_2_4G, true);
        //     }
        // } break;
        // case GU_TOGG: {
        //     keymap_config.no_gui = !keymap_config.no_gui;
        // } break;
        // case SW_OS: {
        //     if (get_highest_layer(default_layer_state) == 0) { // WIN_BASE
        //         set_single_persistent_default_layer(1);
        //         // keymap_config.no_gui = 0;
        //         // eeconfig_update_keymap(keymap_config.raw);
        //         all_blink_time  = timer_read32();
        //         all_blink_cnt   = 6;
        //     } else if (get_highest_layer(default_layer_state) == 1) { // MAC_BASE
        //         set_single_persistent_default_layer(0);
        //         all_blink_time  = timer_read32();
        //         all_blink_cnt   = 6;
        //     }
        // } break;
        case NL_OFF: {
            dev_info.num_lock_off = !dev_info.num_lock_off;
            eeconfig_update_user(dev_info.raw);
            all_blink_time  = timer_read32();
            all_blink_cnt   = 2;
        } break;
        // case RGB_TEST: {
        //     if (dev_info.devs == DEVS_USB) {
        //         extern uint8_t rgb_test_en;
        //         extern uint8_t rgb_test_index;

        //         if (rgb_test_en != true) {
        //             rgb_test_en    = true;
        //             rgb_test_index = 1;
        //         }
        //     }
        // } break;

        case RST_ALL: {
            if (!EE_CLR_flag) {
                EE_CLR_flag       = _RST_ALL;
                EE_CLR_press_time = timer_read32();
                EE_CLR_press_cnt  = 1;
                rgb_matrix_enable_noeeprom();
                rgb_status_save = rgb_matrix_config.enable;
            }
        } break;

        case RST_LYR: {
            if (!EE_CLR_flag) {
                EE_CLR_flag       = _RST_LYR;
                EE_CLR_press_time = timer_read32();
                EE_CLR_press_cnt  = 1;
                rgb_matrix_enable_noeeprom();
                rgb_status_save = rgb_matrix_config.enable;
            }
        } break;

        case RST_BLE: {
            if (!EE_CLR_flag) {
                EE_CLR_flag       = _RST_BLE;
                EE_CLR_press_time = timer_read32();
                EE_CLR_press_cnt  = 1;
                rgb_matrix_enable_noeeprom();
                rgb_status_save = rgb_matrix_config.enable;
            }
        } break;

        default:
            break;
    }
}

static void long_pressed_keys_hook(void) {
    for (uint8_t i = 0; i < NUM_LONG_PRESS_KEYS; i++) {
        switch (long_pressed_keys[i].keycode) {
            case NL_OFF:{
                if ((long_pressed_keys[i].press_time != 0) && (timer_elapsed32(long_pressed_keys[i].press_time) >= (1 * 1000))) {
                    long_pressed_keys[i].event_cb(long_pressed_keys[i].keycode);
                    long_pressed_keys[i].press_time = 0;
                }
            } break;

            default:{
                if ((long_pressed_keys[i].press_time != 0) && (timer_elapsed32(long_pressed_keys[i].press_time) >= (3 * 1000))) {
                    long_pressed_keys[i].event_cb(long_pressed_keys[i].keycode);
                    long_pressed_keys[i].press_time = 0;
                }
            } break;
        }
    }
}

static void bt_used_pin_init(void) {
#ifdef BT_MODE_SW_PIN
    setPinInputHigh(BT_MODE_SW_PIN);
    // setPinInputHigh(RF_MODE_SW_PIN);
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
static void bt_scan_mode(void) {
#ifdef BT_MODE_SW_PIN
    // if (readPin(BT_MODE_SW_PIN) && !readPin(RF_MODE_SW_PIN)) {
    //     if ((dev_info.devs == DEVS_USB) || (dev_info.devs == DEVS_2_4G)) {
    //         bt_switch_mode(dev_info.devs, dev_info.last_devs, false); // BT mode
    //         mcu_reset();
    //     }
    // }
    // if (readPin(RF_MODE_SW_PIN) && !readPin(BT_MODE_SW_PIN)) {
    //     if (dev_info.devs != DEVS_2_4G) {
    //         bt_switch_mode(dev_info.devs, DEVS_2_4G, false); // 2_4G mode
    //         mcu_reset();
    //     }
    // }
    // if (readPin(BT_MODE_SW_PIN) && readPin(RF_MODE_SW_PIN)) {
    //     if (dev_info.devs != DEVS_USB) bt_switch_mode(dev_info.devs, DEVS_USB, false); // usb mode
    // }

    // if (readPin(BT_MODE_SW_PIN) == BT_MODE_SW_TRUE) {
    if (bt_mode_sw_get()) {
        // if ((dev_info.devs == DEVS_USB) || (dev_info.devs == DEVS_2_4G)) {
        //     bt_switch_mode(dev_info.devs, dev_info.last_devs, false); // BT mode
        //     mcu_reset();
        // }
    } else {
        if (dev_info.devs != DEVS_USB) {
            bt_switch_mode(dev_info.devs, DEVS_USB, false); // usb mode
        }
    }

    // uint8_t now_mode = false; // 高电平为USB模式
    // // uint8_t now_mode = true; // low电平为USB模式
    // uint8_t usb_sws  = false;

    // now_mode = readPin(BT_MODE_SW_PIN);
    // usb_sws  = !!dev_info.devs ? !now_mode : now_mode;

    // if (usb_sws) {
    //     if (now_mode) {
    //         bt_switch_mode(dev_info.devs, dev_info.last_devs, false); // ble mode
    //     } else {
    //         bt_switch_mode(dev_info.devs, DEVS_USB, false); // usb mode
    //     }
    // }
#endif
}

#ifdef RGB_MATRIX_ENABLE
static uint32_t key_press_time;
static uint32_t close_rgb_time;
static bool     bak_rgb_toggle;
static bool     sober         = true;
extern bool     led_inited;
extern void     led_config_all(void);
extern void     led_deconfig_all(void);


const uint8_t sleep_time_table[4] = {10, 30, 60, 0};

static const uint8_t rgb_index_table[] = {
    LED_USB, LED_BT1, LED_BT2, LED_BT3, LED_2G,
};

static const uint8_t rgb_index_color_table[][3] = {
    {RGB_BLUE}, {RGB_BLUE}, {RGB_BLUE}, {RGB_BLUE}, {RGB_GREEN}
};

void led_off_standby(void) {
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
        if (kb_sleep_flag || ((timer_elapsed32(key_press_time) >= sleep_time_table[dev_info.sleep_time_index] * TIMEOUT_MINUTE) && (sleep_time_table[dev_info.sleep_time_index] != 0))) {
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

#    ifdef ENTRY_STOP_MODE
                lp_system_sleep();
#    endif
                extern void open_rgb(void);
                open_rgb();
            }
        }
    }
}

void open_rgb(void) {
    key_press_time = timer_read32();
    if (!sober) {
        writePinHigh(RGB_DRIVER_SDB_PIN);
        if (bak_rgb_toggle) {
            kb_sleep_flag = false;
            rgb_matrix_enable_noeeprom();
        }
        if (!led_inited) {
            led_config_all();
        }
        sober = true;
    }
}

void led_bat_fn(void) {
    rgb_matrix_set_color(BT_VOL_LED, RGB_OFF);
    if (bts_info.bt_info.pvol > 94) {
        rgb_matrix_set_color(BT_VOL_LED, RGB_GREEN);
    } else if (bts_info.bt_info.pvol > 10) {
        rgb_matrix_set_color(BT_VOL_LED, RGB_YELLOW);
    } else {
        rgb_matrix_set_color(BT_VOL_LED, RGB_RED);
    }
}

// void led_bat_set(void) {
//     static bool  Low_power_blink;
//     static uint32_t Low_power_time;
//     // if ((bts_info.bt_info.low_vol) && readPin(BT_CABLE_PIN)) {
//     if(bts_info.bt_info.pvol < 10) {
//         rgb_matrix_set_color_all(RGB_OFF);
//         if (timer_elapsed32(Low_power_time) >= 500) {
//             Low_power_blink = !Low_power_blink;
//             Low_power_time = timer_read32();
//         }
//         if (Low_power_blink) {
//             rgb_matrix_set_color(LED_NUM, RGB_RED);
//         } else {
//             rgb_matrix_set_color(LED_NUM, RGB_OFF);
//         }
//     }
// }

uint8_t indicator_status = LED_BLE_CONN;
uint8_t indicator_reset_last_time = false;


void led_ee_clr(void) {
    rgb_matrix_set_color_all(RGB_OFF);

    switch (EE_CLR_flag)
    {
        case _RST_ALL:
            rgb_matrix_set_color(LED_RST_ALL, RGB_WHITE);
            break;
        case _RST_LYR:
            rgb_matrix_set_color(LED_RST_LYR, RGB_WHITE);
            break;
        case _RST_BLE:
            // rgb_matrix_set_color(LED_RST_BLE, RGB_WHITE);
            rgb_matrix_set_color(LED_P1, RGB_WHITE);
            rgb_matrix_set_color(LED_P2, RGB_WHITE);
            rgb_matrix_set_color(LED_P3, RGB_WHITE);
            break;
        default:
            break;
    }
}

void kybd_ee_clr_3(void){
    if (EE_CLR_flag) {
        if (timer_elapsed32(EE_CLR_press_time) >= EE_CLR_press_cnt * 500) {
            EE_CLR_press_cnt++;
        }
        if (EE_CLR_press_cnt >= 7) {
            // uint8_t layer_save = get_highest_layer(default_layer_state);
            switch (EE_CLR_flag)
            {
                case _RST_ALL:{
                    eeconfig_init();
                    eeconfig_update_rgb_matrix_default();
                    set_single_persistent_default_layer(0);
                    keymap_config.no_gui = 0;
                    eeconfig_update_keymap(keymap_config.raw);
                    if (dev_info.devs != DEVS_2_4G){
                        bts_send_vendor(v_clear);
                    }
                    dev_info.led_status = false;
                    led_status_set(false);
                } break;

                case _RST_LYR:
                    eeconfig_init();
                    eeconfig_update_rgb_matrix_default();
                    set_single_persistent_default_layer(0);
                    keymap_config.no_gui = 0;
                    eeconfig_update_keymap(keymap_config.raw);
                    break;

                case _RST_BLE:
                    if (dev_info.devs != DEVS_2_4G){
                        bts_send_vendor(v_clear);
                        bt_switch_mode(dev_info.devs, DEVS_HOST1, false);
                    }
                    break;
                default:
                    break;
            }
            EE_CLR_press_time = 0;
            EE_CLR_press_cnt  = 0;
            EE_CLR_flag       = false;
        }

        led_ee_clr();

        // if (EE_CLR_press_cnt & 0x1) {
        //     if (EE_CLR_press_cnt == 1) {
        //         rgb_matrix_set_color_all(RGB_RED);
        //     } else if (EE_CLR_press_cnt == 3) {
        //         rgb_matrix_set_color_all(RGB_BLUE);
        //     } else if (EE_CLR_press_cnt == 5) {
        //         rgb_matrix_set_color_all(RGB_GREEN);
        //     }
        // } else {
        //     rgb_matrix_set_color_all(RGB_OFF);
        // }
        // return false;
    }
}

static uint8_t  pairing_blink_cnt;
static uint32_t pairing_blink_time;
static RGB      pairing_blink_color;

void led_pair_blink(void) {
    if (pairing_blink_cnt) { // Pairing mode
        rgb_matrix_set_color_all(RGB_OFF);
        if (timer_elapsed32(pairing_blink_time) > 1000) {
            pairing_blink_time = timer_read32();
            pairing_blink_cnt--;
        }
        if (pairing_blink_cnt % 2) {
            rgb_matrix_set_color_all(pairing_blink_color.r, pairing_blink_color.g, pairing_blink_color.b);
        } else {
            rgb_matrix_set_color_all(RGB_OFF);
        }
    }
}

static uint16_t Low_power_blink_time;

void led_ble(void) {
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
            case LED_BLE_PAIR: { // 闪烁模式 5Hz 重置
                if ((last_time == 0) || (timer_elapsed32(last_time) >= LED_BLE_PAIR_INTVL_MS)) {
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

                /* 超时60s退出 */
                if (timer_elapsed32(last_total_time) >= BLE_PAIR_TIMEOUT) {
                    indicator_status = 0;
                    kb_sleep_flag    = true;
                }
            } break;
            case LED_BLE_CONN: { // 闪烁模式 2Hz 回连
                if ((last_time == 0) || (timer_elapsed32(last_time) >= LED_BLE_CONN_INTVL_MS)) {
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
                // if (dev_info.devs == DEVS_2_4G) bts_process_keys(KC_LGUI, 0, DEVS_2_4G, 1);
                if (bts_info.bt_info.paired) {
                    last_long_time   = timer_read32();
                    indicator_status = 3;
                    break;
                }

                if(dev_info.devs==DEVS_2_4G){
                    if (timer_elapsed32(last_total_time) >= BLE_CONN_TIMEOUT) {
                    indicator_status = 0;
                    kb_sleep_flag    = true;
                    }
                } else {
                    if (timer_elapsed32(last_total_time) >= BLE_CONN_TIMEOUT) {
                        indicator_status = 0;
                        kb_sleep_flag    = true;
                    }
                }
            } break;
            case LED_BLE_CONNED: { // 长亮模式
                if ((timer_elapsed32(last_long_time) < (3 * 1000))) {
                    rgb.r = rgb_index_color_table[dev_info.devs][0];
                    rgb.g = rgb_index_color_table[dev_info.devs][1];
                    rgb.b = rgb_index_color_table[dev_info.devs][2];
                } else {
                    indicator_status = 0;
                    Low_power_blink_time = timer_read32();
                }
            } break;
            case LED_BLE_OFF: { // 长灭模式
                rgb = (RGB){.r = 0, .g = 0, .b = 0};
            } break;
            default:
                rgb_flip = false;
                if (!kb_sleep_flag) {
                    if (!bts_info.bt_info.paired) {
                        indicator_status = LED_BLE_CONN;
                        if (dev_info.devs == DEVS_2_4G)
                            bt_switch_mode(DEVS_USB, DEVS_2_4G, false);
                        else
                            bt_switch_mode(DEVS_USB, dev_info.last_devs, false);
                    }
                }
                // return true;
                break;
        }

        rgb_matrix_set_color(rgb_index, rgb.r, rgb.g, rgb.b);
    // }
}

void led_bat_gauge(void) {
    if (query_vol_flag) {
        // bts_send_vendor(v_query_vol);

        rgb_matrix_set_color_all(RGB_OFF);

        for (uint8_t i = LED_1; i < LED_1 + (bts_info.bt_info.pvol / 10); i++) {
            if (bts_info.bt_info.pvol < 30) {
                rgb_matrix_set_color(i, RGB_RED);
            } else if (bts_info.bt_info.pvol < 60) {
                rgb_matrix_set_color(i, RGB_ORANGE);
            } else {
                rgb_matrix_set_color(i, RGB_GREEN);
            }
        }
    }
}

bool bt_cable_get(void) {
    if(readPin(BT_CABLE_PIN) == BT_CABLE_TRUE) {
        return true;
    } else {
        return false;
    }
}

bool bt_charge_get(void) {
    if(readPin(BT_CHARGE_PIN) == true) {
        return true;
    } else {
        return false;
    }
}

void led_bat_low_warn_blink(void) {
    if ((bts_info.bt_info.low_vol) && !bt_cable_get()) {

        rgb_matrix_set_color_all(RGB_OFF);

        static bool     Low_power_bink;
        static uint16_t Low_power_time;

        if (timer_elapsed(Low_power_time) >= 300) {
            Low_power_bink = !Low_power_bink;
            Low_power_time = timer_read32();
        }

        if (Low_power_bink) {
            rgb_matrix_set_color(LED_BAT, RGB_RED);
        } else {
            rgb_matrix_set_color(LED_BAT, RGB_OFF);
        }
    }
}

void led_bat_low_warn_blink_5_second(void) {
    static bool once = false;
    if ((bts_info.bt_info.pvol < 20) && !bt_cable_get() && !once) {

        rgb_matrix_set_color_all(RGB_OFF);
        
        led_single_blink_set(LED_BAT_CNT, LED_BAT, RGB_RED);

        once = true;
    }
}

void kybd_bat_low_warn(void) {
    if ((bts_info.bt_info.low_vol) && !bt_cable_get()) {

        rgb_matrix_set_color_all(RGB_OFF);
        // rgb_matrix_disable_noeeprom();
        
        // led_bat_low_warn();
    }
}

void kybd_bat_low_sleep(void) {
    extern bool low_vol_offed_sleep;
    if ((bts_info.bt_info.low_vol_offed) && !bt_cable_get()) {
        low_vol_offed_sleep = true;
        kb_sleep_flag       = true;
    } else {
        low_vol_offed_sleep = false;
    }
}

void bt_bat_low_indicator(void) {
    if (!kb_sleep_flag && (indicator_status == 0) && bts_info.bt_info.pvol < 20) {
        if (Low_power_blink_time && timer_elapsed(Low_power_blink_time) < (5 * TIMEOUT_SECOND)) {
            rgb_matrix_set_color(LED_BAT, RGB_RED);
        } else {
            Low_power_blink_time = 0;
        }
    }
}

void led_bat_charge(void) {
    #if defined(BT_CABLE_PIN) && defined(BT_CHARGE_PIN)
    // 充电接入
    // if (!dev_info.Charge_flag) {
        static uint32_t charging_time;
        static uint32_t charg_full_time;

        if (bt_cable_get()) {
            if (!bt_charge_get()) {
                // 正在充电
                if (timer_elapsed32(charging_time) >= 2000) {
                    // Low_power_bink = 1;
                    // if (dev_info.devs != DEVS_USB) {
                    //     if (timer_elapsed32(charging_time) <= 12000) {
                    //         rgb_matrix_set_color(LED_BAT, RGB_ORANGE);
                    //     }
                    // }
                }
                charg_full_time  = timer_read32();
            } else {
                // 充满
                // Low_power_bink = 0;
                if ((timer_elapsed32(charg_full_time) <= 7 * TIMEOUT_SECOND) && (timer_elapsed32(charg_full_time) >= 2 * TIMEOUT_SECOND)) {
                    // rgb_matrix_set_color(LED_BAT, RGB_GREEN);
                    led_single_blink_set(2, LED_BAT, RGB_GREEN);
                }
                charging_time  = timer_read32();
            }
        } else {
            static uint32_t query_vol_time;

            if (dev_info.devs != DEVS_USB) {
                if  (timer_elapsed32(query_vol_time) >= 10000){
                    query_vol_time  = timer_read32();
                    bts_send_vendor(v_query_vol);
                }
            }
            
            bt_bat_low_indicator();
        }
    // }
    #endif
}

uint8_t         single_blink_cnt;
uint8_t         single_blink_index;
RGB      single_blink_color;
uint32_t single_blink_time;

void led_single_blink_set(uint8_t cnt, uint8_t led, uint8_t r, uint8_t g, uint8_t b) {
    single_blink_time  = timer_read32();
    single_blink_cnt   = cnt;
    single_blink_index = led;
    single_blink_color = (RGB){r, g, b};
}

void led_single_blink(void){
    if (single_blink_cnt) { // 单键闪烁
        if (timer_elapsed32(single_blink_time) > 300) {
            single_blink_time = timer_read32();
            single_blink_cnt--;
        }
        
        if (single_blink_cnt % 2) {
            rgb_matrix_set_color(single_blink_index, single_blink_color.r, single_blink_color.g, single_blink_color.b);
        } else {
            rgb_matrix_set_color(single_blink_index, RGB_OFF);
        }
    }
}

void led_all_blink(void){
    if (all_blink_cnt) { // 全键闪烁
        rgb_matrix_set_color_all(RGB_OFF);
        if (timer_elapsed32(all_blink_time) > 300) {
            all_blink_time = timer_read32();
            all_blink_cnt--;
        }
        if (all_blink_cnt & 0x1) {
            rgb_matrix_set_color_all(140,140,140);
        }
    }
}

void led_fn_curr_device(void){
    // FN 按下时显示当前设备状态
    if (get_highest_layer(layer_state) == 3) {
        rgb_matrix_set_color(rgb_index_table[dev_info.devs], rgb_index_color_table[dev_info.devs][0], rgb_index_color_table[dev_info.devs][1], rgb_index_color_table[dev_info.devs][2]);

        led_bat_fn();
    } else {
        if (!rgb_matrix_get_flags()) {
            // rgb_matrix_set_color(rgb_index_table[dev_info.devs], RGB_OFF);

            for (uint8_t i = 0; i < 5; i++){
                rgb_matrix_set_color(rgb_index_table[i], RGB_OFF);
            }

            rgb_matrix_set_color(BT_VOL_LED, RGB_OFF);
        }
    }
}


uint8_t bt_indicator_rgb(uint8_t led_min, uint8_t led_max) {
    // FN 按下时显示当前设备状态
    // if (get_highest_layer(default_layer_state | layer_state) == 1) {
    //     rgb_matrix_set_color(46, RGB_WHITE);
    // } else if (get_highest_layer(default_layer_state | layer_state) == 3) {
    //     rgb_matrix_set_color(47, RGB_WHITE);
    // }

    led_fn_curr_device();

    kybd_ee_clr_3();

    led_single_blink();
    led_all_blink();

    led_pair_blink();

    if (dev_info.devs != DEVS_USB) {
        // led_bat_low_warn_blink();
        // led_bat_low_warn_blink_5_second();

        led_bat_charge();

        kybd_bat_low_sleep();

        led_ble();
    }

    // led_status_set_sw();
    led_status_set_watch();

    return true;
}
#endif
