// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include "common/bt_task.h"
#include <stdlib.h>

// clang-format off
#ifdef RGB_MATRIX_ENABLE
const is31fl3733_led_t PROGMEM g_is31fl3733_leds[RGB_MATRIX_LED_COUNT] = {
/* Refer to IS31 manual for these locations
*    driver
*    |   R location
*    |   |          G location
*    |   |          |          B location
*    |   |          |          | */
    {1, SW2_CS1,   SW1_CS1,   SW3_CS1},
    {1, SW2_CS3,   SW1_CS3,   SW3_CS3},
    {1, SW2_CS4,   SW1_CS4,   SW3_CS4},
    {1, SW2_CS5,   SW1_CS5,   SW3_CS5},
    {1, SW2_CS6,   SW1_CS6,   SW3_CS6},
    {1, SW2_CS7,   SW1_CS7,   SW3_CS7},
    {1, SW2_CS8,   SW1_CS8,   SW3_CS8},
    {1, SW2_CS9,   SW1_CS9,   SW3_CS9},
    {1, SW2_CS10,  SW1_CS10,  SW3_CS10},
    {1, SW2_CS11,  SW1_CS11,  SW3_CS11},
    {1, SW2_CS12,  SW1_CS12,  SW3_CS12},
    {1, SW2_CS13,  SW1_CS13,  SW3_CS13},
    {1, SW2_CS14,  SW1_CS14,  SW3_CS14},

    {0, SW2_CS1,   SW1_CS1,   SW3_CS1},
    {0, SW2_CS2,   SW1_CS2,   SW3_CS2},
    {0, SW2_CS3,   SW1_CS3,   SW3_CS3},
    {0, SW2_CS4,   SW1_CS4,   SW3_CS4},
    {0, SW2_CS5,   SW1_CS5,   SW3_CS5},
    {0, SW2_CS6,   SW1_CS6,   SW3_CS6},
    {0, SW2_CS7,   SW1_CS7,   SW3_CS7},
    {0, SW2_CS8,   SW1_CS8,   SW3_CS8},
    {0, SW2_CS9,   SW1_CS9,   SW3_CS9},
    {0, SW2_CS10,  SW1_CS10,  SW3_CS10},
    {0, SW2_CS11,  SW1_CS11,  SW3_CS11},
    {0, SW2_CS12,  SW1_CS12,  SW3_CS12},
    {0, SW2_CS13,  SW1_CS13,  SW3_CS13},
    {0, SW2_CS14,  SW1_CS14,  SW3_CS14},
    {1, SW5_CS1,   SW4_CS1,   SW6_CS1},
    {1, SW5_CS2,   SW4_CS2,   SW6_CS2},
    {1, SW5_CS3,   SW4_CS3,   SW6_CS3},

    {0, SW5_CS1,   SW4_CS1,   SW6_CS1},
    {0, SW5_CS2,   SW4_CS2,   SW6_CS2},
    {0, SW5_CS3,   SW4_CS3,   SW6_CS3},
    {0, SW5_CS4,   SW4_CS4,   SW6_CS4},
    {0, SW5_CS5,   SW4_CS5,   SW6_CS5},
    {0, SW5_CS6,   SW4_CS6,   SW6_CS6},
    {0, SW5_CS7,   SW4_CS7,   SW6_CS7},
    {0, SW5_CS8,   SW4_CS8,   SW6_CS8},
    {0, SW5_CS9,   SW4_CS9,   SW6_CS9},
    {0, SW5_CS10,  SW4_CS10,  SW6_CS10},
    {0, SW5_CS11,  SW4_CS11,  SW6_CS11},
    {0, SW5_CS12,  SW4_CS12,  SW6_CS12},
    {0, SW5_CS13,  SW4_CS13,  SW6_CS13},
    {0, SW5_CS14,  SW4_CS14,  SW6_CS14},
    {1, SW5_CS8,   SW4_CS8,   SW6_CS8},
    {1, SW5_CS9,   SW4_CS9,   SW6_CS9},
    {1, SW5_CS10,  SW4_CS10,  SW6_CS10},

    {0, SW8_CS1,   SW7_CS1,   SW9_CS1},
    {0, SW8_CS2,   SW7_CS2,   SW9_CS2},
    {0, SW8_CS3,   SW7_CS3,   SW9_CS3},
    {0, SW8_CS4,   SW7_CS4,   SW9_CS4},
    {0, SW8_CS5,   SW7_CS5,   SW9_CS5},
    {0, SW8_CS6,   SW7_CS6,   SW9_CS6},
    {0, SW8_CS7,   SW7_CS7,   SW9_CS7},
    {0, SW8_CS8,   SW7_CS8,   SW9_CS8},
    {0, SW8_CS9,   SW7_CS9,   SW9_CS9},
    {0, SW8_CS10,  SW7_CS10,  SW9_CS10},
    {0, SW8_CS11,  SW7_CS11,  SW9_CS11},
    {0, SW8_CS12,  SW7_CS12,  SW9_CS12},
    {0, SW8_CS13,  SW7_CS13,  SW9_CS13},

    {0, SW11_CS1,  SW10_CS1,  SW12_CS1},
    {0, SW11_CS2,  SW10_CS2,  SW12_CS2},
    {0, SW11_CS3,  SW10_CS3,  SW12_CS3},
    {0, SW11_CS4,  SW10_CS4,  SW12_CS4},
    {0, SW11_CS5,  SW10_CS5,  SW12_CS5},
    {0, SW11_CS6,  SW10_CS6,  SW12_CS6},
    {0, SW11_CS7,  SW10_CS7,  SW12_CS7},
    {0, SW11_CS8,  SW10_CS8,  SW12_CS8},
    {0, SW11_CS9,  SW10_CS9,  SW12_CS9},
    {0, SW11_CS10, SW10_CS10, SW12_CS10},
    {0, SW11_CS11, SW10_CS11, SW12_CS11},
    {0, SW11_CS12, SW10_CS12, SW12_CS12},
    {1, SW11_CS4,  SW10_CS4,  SW12_CS4},

    {0, SW11_CS13, SW10_CS13, SW12_CS13},
    {0, SW11_CS14, SW10_CS14, SW12_CS14},
    {0, SW11_CS15, SW10_CS15, SW12_CS15},
    {0, SW11_CS16, SW10_CS16, SW12_CS16},
    {0, SW8_CS15,  SW7_CS15,  SW9_CS15},
    {0, SW8_CS16,  SW7_CS16,  SW9_CS16},
    {1, SW11_CS1,  SW10_CS1,  SW12_CS1},
    {1, SW11_CS2,  SW10_CS2,  SW12_CS2},
    {1, SW11_CS3,  SW10_CS3,  SW12_CS3},
    {1, SW11_CS5,  SW10_CS5,  SW12_CS5},
    {1, SW11_CS6,  SW10_CS6,  SW12_CS6},

    {1, SW5_CS11,  SW4_CS11,  SW6_CS11},
    {1, SW5_CS14,  SW4_CS14,  SW6_CS14},
    {1, SW5_CS15,  SW4_CS15,  SW6_CS15},
};
#endif
// clang-format on

bool led_inited = false;

void led_config_all(void) {
    if (!led_inited) {
        // Set our LED pins as output
        // setPinOutput(A14);
        // if (dev_info.devs == DEVS_USB) {
        //     writePinLow(A14);
        // } else {
        //     writePinHigh(A14);
        // }
        led_inited = true;
    }
}

void led_deconfig_all(void) {
    if (led_inited) {
        // Set our LED pins as input
        // setPinInput(A14);
        // writePinLow(A14);
        // writePinHigh(A14);
        led_inited = false;
    }
}
// void suspend_power_down_user(void) {
//     // code will run multiple times while keyboard is suspended
//     led_deconfig_all();
// }

// void suspend_wakeup_init_user(void) {
//     // code will run on keyboard wakeup
//     // led_config_all();
//     led_config_all();
// }
bool via_command_kb(uint8_t *data, uint8_t length) {
    uint8_t *command_id   = &(data[0]);
    switch (*command_id) {
        case 0x07: {
            if(data[1]==0x03&&data[2]==0x02&&data[3]==0x00){
                switch (rgb_matrix_get_flags()) {
                    case LED_FLAG_ALL: {
                        rgb_matrix_set_flags(LED_FLAG_NONE);
                        rgb_matrix_set_color_all(0, 0, 0);
                        // rgb_status_save = 0;
                    } break;
                    default: {
                        rgb_matrix_set_flags(LED_FLAG_ALL);
                        // rgb_status_save = 1;
                    } break;
                }
                // return false;
            } else if(data[1]==0x03&&data[2]==0x02&&data[3]!=0x00){
                if(rgb_matrix_get_flags() != LED_FLAG_ALL)
                    rgb_matrix_set_flags(LED_FLAG_ALL);
            }
            return false;
        }
        default:
            break;
    }
    return false;
}

void set_led_state(void) {
    if (led_inited) {
        // writePin(D2, keymap_config.no_gui);
        // writePin(C11, get_highest_layer(default_layer_state) == 2);
    }
}

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    if (process_record_user(keycode, record) != true) {
        return false;
    }
    switch (keycode) {
        case RGB_TOG:
            if (record->event.pressed) {
                // extern bool rgb_status_save;
                switch (rgb_matrix_get_flags()) {
                    case LED_FLAG_ALL: {
                        rgb_matrix_set_flags(LED_FLAG_NONE);
                        rgb_matrix_set_color_all(0, 0, 0);
                        // rgb_status_save = 0;
                    } break;
                    default: {
                        rgb_matrix_set_flags(LED_FLAG_ALL);
                        // rgb_status_save = 1;
                    } break;
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

uint8_t rgb_test_en    = false;
uint8_t rgb_test_index = 0;

static const uint8_t rgb_test_color_table[][3] = {
    {200, 200, 200},
    {200, 0, 0},
    {0, 200, 0},
    {0, 0, 200},
};

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
    if (rgb_matrix_indicators_advanced_user(led_min, led_max) != true) {
        return false;
    }
#ifdef BT_MODE_ENABLE

#endif
    // clang-format on
    if(!rgb_matrix_get_flags()) {
        rgb_matrix_set_color_all(0, 0, 0);
    }
    // caps lock red
    if (rgb_test_en) {
        // clang-format off
        for (uint8_t i = led_min; i < led_max; i++) {
            rgb_matrix_set_color(i, rgb_test_color_table[rgb_test_index - 1][0],
            rgb_test_color_table[rgb_test_index - 1][1],
            rgb_test_color_table[rgb_test_index - 1][2]);
        }
        return false;
    }

#ifdef BT_MODE_ENABLE
    if (bt_indicator_rgb(led_min, led_max) != true) {
        return false;
    }
#endif
    if ((host_keyboard_led_state().caps_lock) && ((bts_info.bt_info.paired) || (dev_info.devs == DEVS_USB))) {
        rgb_matrix_set_color(58, 50, 50, 50);
    }

    // GUI lock red
    if (keymap_config.no_gui) {
        rgb_matrix_set_color(79, 160, 160, 160);
    }
    return true;
}
