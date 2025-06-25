// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include "common/bt_task.h"

bool led_inited = false;

// ====================
// 调试信息控制宏
// ====================
#ifdef CONSOLE_ENABLE
#    define DEBUG_PRINTF(fmt, ...) uprintf(fmt, ##__VA_ARGS__)
#else
#    define DEBUG_PRINTF(fmt, ...) \
        do {                       \
        } while (0)
#endif

#ifdef CONSOLE_ENABLE
#endif

void led_config_all(void) {
    if (!led_inited) {
        setPinOutputPushPull(WS2812_EN_PIN);
        writePinHigh(WS2812_EN_PIN);
        led_inited = true;
    }
}

void led_deconfig_all(void) {
    if (led_inited) {
        setPinOutputPushPull(WS2812_EN_PIN);
        writePinLow(WS2812_EN_PIN);
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
    keyboard_post_init_user();
}

void eeconfig_init_kb(void) {
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
    if (rgb_matrix_indicators_advanced_user(led_min, led_max) != true) {
        return false;
    }

#ifdef BT_MODE_ENABLE
    if (bt_indicator_rgb(led_min, led_max) != true) {
        return false;
    }
#endif
    return true;
}
