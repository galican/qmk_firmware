// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include "common/bt_task.h"
#include <stdlib.h>
// clang-format off

#ifdef RGB_MATRIX_ENABLE
const is31fl3733_led_t PROGMEM g_is31fl3733_leds[RGB_MATRIX_LED_COUNT] = {
/* Refer to IS31 manual for these locations
 *   driver
 *   |   R location
 *   |   |     G location
 *   |   |     |     B location
 *   |   |     |     | */

    {0, SW1_CS1,   SW2_CS1,   SW3_CS1},
    {0, SW1_CS2,   SW2_CS2,   SW3_CS2},
    {0, SW1_CS3,   SW2_CS3,   SW3_CS3},
    {0, SW1_CS4,   SW2_CS4,   SW3_CS4},
    {0, SW1_CS5,   SW2_CS5,   SW3_CS5},
    {0, SW1_CS6,   SW2_CS6,   SW3_CS6},
    {0, SW1_CS7,   SW2_CS7,   SW3_CS7},
    {0, SW1_CS8,   SW2_CS8,   SW3_CS8},

    {0, SW4_CS1,   SW5_CS1,   SW6_CS1},
    {0, SW4_CS2,   SW5_CS2,   SW6_CS2},
    {0, SW4_CS3,   SW5_CS3,   SW6_CS3},
    {0, SW4_CS4,   SW5_CS4,   SW6_CS4},
    {0, SW4_CS5,   SW5_CS5,   SW6_CS5},
    {0, SW4_CS6,   SW5_CS6,   SW6_CS6},
    {0, SW4_CS7,   SW5_CS7,   SW6_CS7},
    {0, SW4_CS8,   SW5_CS8,   SW6_CS8},

    {0, SW7_CS1,   SW8_CS1,   SW9_CS1},
    {0, SW7_CS2,   SW8_CS2,   SW9_CS2},
    {0, SW7_CS3,   SW8_CS3,   SW9_CS3},
    {0, SW7_CS4,   SW8_CS4,   SW9_CS4},
    {0, SW7_CS5,   SW8_CS5,   SW9_CS5},
    {0, SW7_CS6,   SW8_CS6,   SW9_CS6},

    {0, SW1_CS9,   SW2_CS9,   SW3_CS9},
};
#endif
// clang-format on
bool led_inited = false;

void led_config_all(void) {
    if (!led_inited) {
        // Set our LED pins as output
        setPinOutput(A14);
        if (dev_info.devs == DEVS_USB) {
            writePinLow(A14);
        } else {
            writePinHigh(A14);
        }
        setPinOutput(RGB_DRIVER_SDB_PIN);
        writePinHigh(RGB_DRIVER_SDB_PIN);
        led_inited = true;
    }
}

void led_deconfig_all(void) {
    if (led_inited) {
        // Set our LED pins as input
        writePinLow(RGB_DRIVER_SDB_PIN);
        // setPinInput(A14);
        // writePinLow(A14);
        led_inited = false;
    }
}
uint8_t rgb_test_en    = false;

void set_led_state(void) {
    if (led_inited) {
        if (!rgb_test_en) {
        } else {
        }
        // writePin(D2, keymap_config.no_gui);
        // writePin(C11, get_highest_layer(default_layer_state) == 2);
    }
}
// 拨动开关选择系统模式
bool dip_switch_update_kb(uint8_t index, bool active) {
    if (!dip_switch_update_user(index, active)) {
        return false;
    }
    if (index == 0) {
        default_layer_set(1UL << ((active) ? 1 : 0));
        if (!active) {
            layer_off(2);
            // keymap_config.no_gui = 0;
            // eeconfig_update_keymap(keymap_config.raw);
        }
    }
    return true;
}

// #define HSV_NAVY       170, 255, 127
// #define HSV_WARM_WHITE       31, 33, 252

// #define HUE_SET_NUM 11
#define HUE_SET_NUM (9 - 1)
const uint8_t  HUE_SET_TABLE[][3] = {
    // {HSV_MAGENTA}, {HSV_RED}, {HSV_ORANGE}, {HSV_YELLOW}, {HSV_GREEN}, {HSV_CYAN}, {HSV_BLUE}, {HSV_NAVY}, {HSV_PURPLE}, {HSV_WHITE}, {HSV_WARM_WHITE},
    {HSV_MAGENTA}, {HSV_RED}, {HSV_ORANGE}, {HSV_YELLOW}, {HSV_GREEN}, {HSV_CYAN}, {HSV_BLUE}, {HSV_PURPLE}, {HSV_WHITE}, 
};

void nkro_set(void) {
    keymap_config.nkro = !keymap_config.nkro;
    eeconfig_update_keymap(keymap_config.raw);

    // single_blink_time  = timer_read32();
    // single_blink_cnt   = 6;
    // single_blink_index = 13;
    // single_blink_color = (RGB){100,100,100};


    if (keymap_config.nkro) {
        led_single_blink_set(LED_NKRO_CNT, LED_NKRO, RGB_BLUE);
        bts_set_nkro(true);
    } else {
        led_single_blink_set(LED_NKRO_CNT, LED_NKRO, RGB_WHITE);
        bts_set_nkro(false);
    }
}

void led_color_list(void) {
    #define color_list_num (9 - 1)
    
    const uint8_t  color_list[][3] = {
        {HSV_BLUE}, {HSV_PURPLE}, {HSV_WHITE}, {HSV_MAGENTA}, {HSV_RED}, {HSV_ORANGE}, {HSV_YELLOW}, {HSV_GREEN}, {HSV_CYAN}, 
    };

    static uint8_t color_index = 0;

    if (color_index >= color_list_num) {
        color_index = 0;
    } else {
        color_index++;
    }
    rgb_matrix_sethsv(color_list[color_index][0], color_list[color_index][1], rgb_matrix_config.hsv.v);
}

void led_layer_blink(void) {
    static bool status = false;

    status = !status;

    if(status) {
        led_single_blink_set(LED_MAC_CNT, LED_MAC_LYR, RGB_WHITE);
    } else {
        led_single_blink_set(LED_MAC_CNT, LED_WIN_LYR, RGB_BLUE);
    }

    // if (get_highest_layer(default_layer_state) == WIN_B) {
    //     set_single_persistent_default_layer(MAC_B);

    //     led_single_blink_set(LED_MAC_CNT, LED_MAC_LYR, RGB_WHITE);
    // } else if (get_highest_layer(default_layer_state) == MAC_B) {
    //     set_single_persistent_default_layer(WIN_B);
    //     keymap_config.no_gui = 0;
    //     eeconfig_update_keymap(keymap_config.raw);

    //     led_single_blink_set(LED_MAC_CNT, LED_WIN_LYR, RGB_BLUE);
    // }
}

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    if (process_record_user(keycode, record) != true) {
        return false;
    }
    switch (keycode) {
        case RGB_SAI:{
            if (!record->event.pressed && (rgb_matrix_get_sat() == 255)) {
                rgb_matrix_sethsv(rgb_matrix_config.hsv.h, RGB_MATRIX_SAT_STEP, rgb_matrix_config.hsv.v);
            }
        } break;

        case RGB_VAI: {
            if (!record->event.pressed && (rgb_matrix_get_val() == RGB_MATRIX_MAXIMUM_BRIGHTNESS)) {
                rgb_matrix_sethsv(rgb_matrix_config.hsv.h, rgb_matrix_config.hsv.s, RGB_MATRIX_MINIMUM_BRIGHTNESS);
                return false;
            }
        } break;

        case RGB_SPI: {                         //速度循环
            if (!record->event.pressed && (rgb_matrix_get_speed() == 255)) {
                rgb_matrix_set_speed(RGB_MATRIX_MIN_SPD);
                return false;
            }
        } break;

        // case KC_END: {
        //     if (record->event.pressed) {
        //         extern uint8_t rgb_test_en;
        //         if (rgb_test_en) {
        //             rgb_test_en = false;
        //             return false;
        //         }
        //     }
        // } break;
        // case KC_DOWN: {
        //     if (record->event.pressed) {
        //         extern uint8_t rgb_test_en;
        //         extern uint8_t rgb_test_index;
        //         if (rgb_test_en) {
        //             rgb_test_index++;
        //             if (rgb_test_index > 4) rgb_test_index = 1;
        //             return false;
        //         }
        //     }
        // } break;

        case RGB_HUI: {
            if (record->event.pressed) {
                led_color_list();
            }
        } break;

        case NKRO_EN:{
            if (record->event.pressed) {
                nkro_set();
            }
        } break;

        case SW_OS: {
            if (record->event.pressed) {
                led_layer_blink();
            }
        } break;
    }
#ifdef BT_MODE_ENABLE
    if (process_record_bt(keycode, record) != true) {
        return false;
    }
#endif
    return true;
}
void matrix_init_kb(void) {
#ifdef RGB_DRIVER_SDB_PIN
    setPinOutput(RGB_DRIVER_SDB_PIN);
    writePinHigh(RGB_DRIVER_SDB_PIN);
#endif

#ifdef BT_MODE_ENABLE
    bt_init();
    led_config_all();
#endif
    matrix_init_user();
}

void matrix_scan_kb(void) {
#ifdef BT_MODE_ENABLE
    bt_task();
    set_led_state();
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

#ifdef RGB_MATRIX_ENABLE
uint8_t rgb_test_index = 0;

// static const uint8_t rgb_test_color_table[][3] = {
//     {RGB_WHITE},
//     {RGB_RED},
//     {RGB_GREEN},
//     {RGB_BLUE},
// };

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
    if (rgb_matrix_indicators_advanced_user(led_min, led_max) != true) {
        return false;
    }
    if (rgb_test_en) {
        // clang-format off
        rgb_matrix_set_color_all(255, 255, 255);
        // clang-format on
        return false;
    }

#    ifdef BT_MODE_ENABLE
    if (bt_indicator_rgb(led_min, led_max) != true) {
        return false;
    }
#    endif


    // num lock, LED Indicator
    if ((host_keyboard_led_state().num_lock && get_highest_layer(default_layer_state) == 0) || get_highest_layer(default_layer_state) == 1) {
        rgb_matrix_set_color(LED_NUM, RGB_BLUE);
    } else {
        rgb_matrix_set_color(LED_NUM, RGB_OFF);
    }

    // if(dev_info.num_lock_off) {
    //     rgb_matrix_set_color(LED_NUM, 0, 0, 0);
    // } else {
    //     if (bts_info.bt_info.pvol < 10) {
    //         rgb_matrix_set_color(LED_NUM, RGB_RED);
    //     }
    // }

    // GUI lock red
    // if (keymap_config.no_gui) {
    //     RGB_MATRIX_INDICATOR_SET_COLOR(73, 140, 140, 140);
    // }

    return true;
}
#endif
