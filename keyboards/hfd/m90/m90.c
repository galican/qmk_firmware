// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include "common/bt_task.h"

// clang-format off
#ifdef RGB_MATRIX_ENABLE
const is31fl3733_led_t PROGMEM g_is31fl3733_leds[RGB_MATRIX_LED_COUNT] = {
/* Refer to IS31 manual for these locations
*    driver
*    |   R location
*    |   |          G location
*    |   |          |          B location
*    |   |          |          | */
    {1, SW1_CS1,   SW2_CS1,   SW3_CS1},
    {1, SW1_CS3,   SW2_CS3,   SW3_CS3},
    {1, SW1_CS4,   SW2_CS4,   SW3_CS4},
    {1, SW1_CS5,   SW2_CS5,   SW3_CS5},
    {1, SW1_CS6,   SW2_CS6,   SW3_CS6},
    {1, SW1_CS7,   SW2_CS7,   SW3_CS7},
    {1, SW1_CS8,   SW2_CS8,   SW3_CS8},
    {1, SW1_CS9,   SW2_CS9,   SW3_CS9},
    {1, SW1_CS10,  SW2_CS10,  SW3_CS10},
    {1, SW1_CS11,  SW2_CS11,  SW3_CS11},
    {1, SW1_CS12,  SW2_CS12,  SW3_CS12},
    {1, SW1_CS13,  SW2_CS13,  SW3_CS13},
    {1, SW1_CS14,  SW2_CS14,  SW3_CS14},

    {0, SW1_CS1,   SW2_CS1,   SW3_CS1},
    {0, SW1_CS2,   SW2_CS2,   SW3_CS2},
    {0, SW1_CS3,   SW2_CS3,   SW3_CS3},
    {0, SW1_CS4,   SW2_CS4,   SW3_CS4},
    {0, SW1_CS5,   SW2_CS5,   SW3_CS5},
    {0, SW1_CS6,   SW2_CS6,   SW3_CS6},
    {0, SW1_CS7,   SW2_CS7,   SW3_CS7},
    {0, SW1_CS8,   SW2_CS8,   SW3_CS8},
    {0, SW1_CS9,   SW2_CS9,   SW3_CS9},
    {0, SW1_CS10,  SW2_CS10,  SW3_CS10},
    {0, SW1_CS11,  SW2_CS11,  SW3_CS11},
    {0, SW1_CS12,  SW2_CS12,  SW3_CS12},
    {0, SW1_CS13,  SW2_CS13,  SW3_CS13},
    {0, SW1_CS14,  SW2_CS14,  SW3_CS14},
    {1, SW4_CS1,   SW5_CS1,   SW6_CS1},
    {1, SW4_CS2,   SW5_CS2,   SW6_CS2},
    {1, SW4_CS3,   SW5_CS3,   SW6_CS3},

    {0, SW4_CS1,   SW5_CS1,   SW6_CS1},
    {0, SW4_CS2,   SW5_CS2,   SW6_CS2},
    {0, SW4_CS3,   SW5_CS3,   SW6_CS3},
    {0, SW4_CS4,   SW5_CS4,   SW6_CS4},
    {0, SW4_CS5,   SW5_CS5,   SW6_CS5},
    {0, SW4_CS6,   SW5_CS6,   SW6_CS6},
    {0, SW4_CS7,   SW5_CS7,   SW6_CS7},
    {0, SW4_CS8,   SW5_CS8,   SW6_CS8},
    {0, SW4_CS9,   SW5_CS9,   SW6_CS9},
    {0, SW4_CS10,  SW5_CS10,  SW6_CS10},
    {0, SW4_CS11,  SW5_CS11,  SW6_CS11},
    {0, SW4_CS12,  SW5_CS12,  SW6_CS12},
    {0, SW4_CS13,  SW5_CS13,  SW6_CS13},
    {0, SW4_CS14,  SW5_CS14,  SW6_CS14},
    {1, SW4_CS8,   SW5_CS8,   SW6_CS8},
    {1, SW4_CS9,   SW5_CS9,   SW6_CS9},
    {1, SW4_CS10,  SW5_CS10,  SW6_CS10},

    {0, SW7_CS1,   SW8_CS1,   SW9_CS1},
    {0, SW7_CS2,   SW8_CS2,   SW9_CS2},
    {0, SW7_CS3,   SW8_CS3,   SW9_CS3},
    {0, SW7_CS4,   SW8_CS4,   SW9_CS4},
    {0, SW7_CS5,   SW8_CS5,   SW9_CS5},
    {0, SW7_CS6,   SW8_CS6,   SW9_CS6},
    {0, SW7_CS7,   SW8_CS7,   SW9_CS7},
    {0, SW7_CS8,   SW8_CS8,   SW9_CS8},
    {0, SW7_CS9,   SW8_CS9,   SW9_CS9},
    {0, SW7_CS10,  SW8_CS10,  SW9_CS10},
    {0, SW7_CS11,  SW8_CS11,  SW9_CS11},
    {0, SW7_CS12,  SW8_CS12,  SW9_CS12},
    {0, SW7_CS13,  SW8_CS13,  SW9_CS13},

    {0, SW10_CS1,  SW11_CS1,  SW12_CS1},
    {0, SW10_CS2,  SW11_CS2,  SW12_CS2},
    {0, SW10_CS3,  SW11_CS3,  SW12_CS3},
    {0, SW10_CS4,  SW11_CS4,  SW12_CS4},
    {0, SW10_CS5,  SW11_CS5,  SW12_CS5},
    {0, SW10_CS6,  SW11_CS6,  SW12_CS6},
    {0, SW10_CS7,  SW11_CS7,  SW12_CS7},
    {0, SW10_CS8,  SW11_CS8,  SW12_CS8},
    {0, SW10_CS9,  SW11_CS9,  SW12_CS9},
    {0, SW10_CS10, SW11_CS10, SW12_CS10},
    {0, SW10_CS11, SW11_CS11, SW12_CS11},
    {0, SW10_CS12, SW11_CS12, SW12_CS12},
    {1, SW10_CS4,  SW11_CS4,  SW12_CS4},

    {0, SW10_CS13, SW11_CS13, SW12_CS13},
    {0, SW10_CS14, SW11_CS14, SW12_CS14},
    {0, SW10_CS15, SW11_CS15, SW12_CS15},
    {0, SW10_CS16, SW11_CS16, SW12_CS16},
    {0, SW7_CS15,  SW8_CS15,  SW9_CS15},
    {0, SW7_CS16,  SW8_CS16,  SW9_CS16},
    {1, SW10_CS1,  SW11_CS1,  SW12_CS1},
    {1, SW10_CS2,  SW11_CS2,  SW12_CS2},
    {1, SW10_CS3,  SW11_CS3,  SW12_CS3},
    {1, SW10_CS5,  SW11_CS5,  SW12_CS5},
    {1, SW10_CS6,  SW11_CS6,  SW12_CS6},

    {1, SW4_CS11,  SW5_CS11,  SW6_CS11},
    {1, SW4_CS14,  SW5_CS14,  SW6_CS14},
    {1, SW4_CS15,  SW5_CS15,  SW6_CS15},
};
#endif
// clang-format on

bool led_inited = false;

#define HUE_SET_NUM (9 - 1)
const uint8_t HUE_SET_TABLE[][3] = {
    {HSV_MAGENTA}, {HSV_RED}, {HSV_ORANGE}, {HSV_YELLOW}, {HSV_GREEN}, {HSV_CYAN}, {HSV_BLUE}, {HSV_PURPLE}, {HSV_WHITE},
};

void led_config_all(void) {
    if (!led_inited) {
        setPinOutputPushPull(RGB_DRIVER_SDB_PIN);
        writePinHigh(RGB_DRIVER_SDB_PIN);
        led_inited = true;
    }
}

void led_deconfig_all(void) {
    if (led_inited) {
        setPinOutputPushPull(RGB_DRIVER_SDB_PIN);
        writePinLow(RGB_DRIVER_SDB_PIN);
        led_inited = false;
    }
}

void suspend_power_down_user(void) {
    // code will run multiple times while keyboard is suspended
    led_deconfig_all();
}

void suspend_wakeup_init_user(void) {
    // code will run on keyboard wakeup
    led_config_all();
}

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    if (process_record_user(keycode, record) != true) {
        return false;
    }
    switch (keycode) {
        case RGB_TOG:
            if (record->event.pressed) {
                if (bts_info.bt_info.low_vol) {
                    return false;
                }
                if (rgb_matrix_get_mode() == RGB_MATRIX_CUSTOM_EFFECT_OFF) {
                    rgb_matrix_mode(RGB_MATRIX_DEFAULT_MODE);
                } else {
                    if (rgb_matrix_get_flags() == LED_FLAG_ALL) {
                        rgb_matrix_set_flags(LED_FLAG_NONE);
                        rgb_matrix_set_color_all(0, 0, 0);
                    } else {
                        rgb_matrix_set_flags(LED_FLAG_ALL);
                    }
                }
            }
            return false;
        case RGB_MOD:
            if (record->event.pressed) {
                rgb_matrix_step();
                if (rgb_matrix_get_mode() == RGB_MATRIX_CUSTOM_EFFECT_OFF) {
                    rgb_matrix_step();
                }
            }
            return false;
        case RGB_RMOD:
            if (record->event.pressed) {
                rgb_matrix_step_reverse();
                if (rgb_matrix_get_mode() == RGB_MATRIX_CUSTOM_EFFECT_OFF) {
                    rgb_matrix_step_reverse();
                }
            }
            return false;
        default:
            break;
    }
#ifdef BT_MODE_ENABLE
    if (process_record_bt(keycode, record) != true) {
        return false;
    }
#endif
    return true;
}

void matrix_init_kb(void) {
#ifdef WS2812_EN_PIN
    setPinOutput(WS2812_EN_PIN);
    writePinLow(WS2812_EN_PIN);
#endif

#ifdef BT_MODE_ENABLE
    bt_init(); // 使用新的初始化函数
    led_config_all();
#endif

    matrix_init_user();
}

void keyboard_post_init_kb(void) {
    // This is called after the keyboard matrix is initialized
    // and the RGB matrix is initialized.
    // It can be used to set up additional features or configurations.

    // dev_info.config.raw = eeconfig_read_kb();

    keyboard_post_init_user();
}

void eeconfig_init_kb(void) {
    dev_info.config.sleep_mode = 1;
    // eeconfig_update_user(dev_info.raw);

    // dev_info.config.ind_brightness = RGB_MATRIX_DEFAULT_VAL;
    dev_info.config.ind_brightness = RGB_MATRIX_VAL_STEP * 3;
    // eeconfig_update_kb(dev_info.config.raw);

    // keymap_config.nkro = false;
    // eeconfig_update_keymap(&keymap_config);
    rgb_matrix_config.hsv.h         = 170;
    dev_info.config.smd_color_index = 0;
    eeconfig_update_user(dev_info.raw);

    rgb_matrix_mode(RGB_MATRIX_CUSTOM_EFFECT_OFF);

    eeconfig_init_user();
}

void matrix_scan_kb(void) {
#ifdef BT_MODE_ENABLE
    bt_task();
#endif
    matrix_scan_user();
}

void housekeeping_task_kb(void) {
#ifdef BT_MODE_ENABLE
    extern void housekeeping_task_bt(void);
    housekeeping_task_bt();
#endif

#ifdef CONSOLE_ENABLE
    debug_enable = true;
#endif
}

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
    // if (!rgb_info.rgb_tog_flag) {
    // rgb_matrix_set_flags_noeeprom(LED_FLAG_NONE);
    // rgb_matrix_set_color_all(0, 0, 0);
    // }

    if (rgb_matrix_indicators_advanced_user(led_min, led_max) != true) {
        return false;
    }

#ifdef BT_MODE_ENABLE
    if (bt_indicator_rgb(led_min, led_max) != true) {
        return false;
    }
#endif

    // GUI lock red
    if (keymap_config.no_gui) {
        rgb_matrix_set_color(74, 160, 160, 160);
    }
    return true;
}

#ifdef DIP_SWITCH_ENABLE
bool dip_switch_update_kb(uint8_t index, bool active) {
    if (!dip_switch_update_user(index, active)) {
        return false;
    }

    if (index == 0) {
        if (active) {
            if (dev_info.devs == DEVS_USB) {
                bt_switch_mode(DEVS_USB, dev_info.devs, false); // wireless mode
            }
        } else {
            if (dev_info.devs != DEVS_USB) {
                bt_switch_mode(dev_info.devs, DEVS_USB, false); // usb mode
            }
        }
    }

    return true;
}
#endif // DIP_SWITCH_ENABLE
