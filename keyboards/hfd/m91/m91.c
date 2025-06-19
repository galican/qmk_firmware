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
    {1, SW4_CS11,  SW5_CS11,  SW6_CS11},
    {1, SW4_CS12,  SW5_CS12,  SW6_CS12},
    {1, SW4_CS13,  SW5_CS13,  SW6_CS13},

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
    {1, SW4_CS4,   SW5_CS4,   SW6_CS4},
    {1, SW4_CS5,   SW5_CS5,   SW6_CS5},
    {1, SW4_CS6,   SW5_CS6,   SW6_CS6},
    {1, SW4_CS7,   SW5_CS7,   SW6_CS7},

    {0, SW4_CS1,   SW5_CS1,   SW6_CS1},
    {0, SW4_CS2,   SW5_CS2,   SW6_CS2},
    {0, SW4_CS3,   SW5_CS3,   SW3_CS3},
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
    {1, SW7_CS1,   SW8_CS1,   SW9_CS1},
    {1, SW7_CS2,   SW8_CS2,   SW9_CS2},
    {1, SW7_CS3,   SW8_CS3,   SW9_CS3},
    {1, SW7_CS4,   SW8_CS4,   SW9_CS4},

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
    {1, SW7_CS5,   SW8_CS5,   SW9_CS5},
    {1, SW7_CS6,   SW8_CS6,   SW9_CS6},
    {1, SW7_CS7,   SW8_CS7,   SW9_CS7},

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
    {1, SW7_CS8,   SW8_CS8,   SW9_CS8},
    {1, SW7_CS9,   SW8_CS9,   SW9_CS9},
    {1, SW7_CS10,  SW8_CS10,  SW9_CS10},
    {1, SW7_CS11,  SW8_CS11,  SW9_CS11},

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
    {1, SW7_CS12,  SW8_CS12,  SW9_CS12},
    {1, SW7_CS13,  SW8_CS13,  SW9_CS13},

    {1, SW4_CS14,  SW5_CS14,  SW6_CS14},
    {1, SW4_CS15,  SW5_CS15,  SW6_CS15},
    {1, SW4_CS16,  SW5_CS16,  SW6_CS16},
};
#endif
// clang-format on

bool led_inited = false;

// ====================
// 调试信息控制宏
// ====================
#ifdef CONSOLE_ENABLE
    #define DEBUG_PRINTF(fmt, ...) uprintf(fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINTF(fmt, ...) do {} while(0)
#endif

// ====================
// 工具函数
// ====================

static inline void validate_per_info_ranges(void) {
    bool changed = false;

    if (per_info.sleep_mode > 3) {
        per_info.sleep_mode = 1;
        changed             = true;
    }
    if (per_info.smd_color_index > 8) {
        per_info.smd_color_index = 0;
        changed                  = true;
    }
    if (per_info.ind_color_index > 8) {
        per_info.ind_color_index = 0;
        changed                  = true;
    }
    if (per_info.saved_rgb_mode > 21) {
        per_info.saved_rgb_mode = RGB_MATRIX_DEFAULT_MODE;
        changed                 = true;
    }

    if (changed) {
        DEBUG_PRINTF("validate_per_info_ranges: corrected invalid values\n");
    }
}

#ifdef CONSOLE_ENABLE
void debug_per_info_layout(void) {
    uprintf("=== per_info_t Debug Layout ===\n");
    uprintf("Structure size: %d bytes\n", sizeof(per_info_t));
    uprintf("Field ranges: sleep_mode(0-3), ind_brightness(0-255), colors(0-8), rgb_mode(0-21)\n");

    per_info_t test = {0};
    uprintf("Initial raw: 0x%08lX\n", test.raw);

    // 测试各字段的最大值
    test.sleep_mode = 3;
    uprintf("sleep_mode=3: 0x%08lX\n", test.raw);

    test.ind_brightness = 255;
    uprintf("ind_brightness=255: 0x%08lX\n", test.raw);

    test.smd_color_index = 8;
    uprintf("smd_color_index=8: 0x%08lX\n", test.raw);

    test.ind_color_index = 8;
    uprintf("ind_color_index=8: 0x%08lX\n", test.raw);

    test.saved_rgb_mode = 21;
    uprintf("saved_rgb_mode=21: 0x%08lX\n", test.raw);

    test.backlight_off = true;
    uprintf("backlight_off=1: 0x%08lX\n", test.raw);

    test.eco_tog_flag = true;
    uprintf("eco_tog_flag=1: 0x%08lX\n", test.raw);

    // 验证读取
    uprintf("=== Field Verification ===\n");
    uprintf("  sleep_mode=%d (expect 3)\n", test.sleep_mode);
    uprintf("  ind_brightness=%d (expect 255)\n", test.ind_brightness);
    uprintf("  smd_color_index=%d (expect 8)\n", test.smd_color_index);
    uprintf("  ind_color_index=%d (expect 8)\n", test.ind_color_index);
    uprintf("  saved_rgb_mode=%d (expect 21)\n", test.saved_rgb_mode);
    uprintf("  backlight_off=%d (expect 1)\n", test.backlight_off);
    uprintf("  eco_tog_flag=%d (expect 1)\n", test.eco_tog_flag);
    uprintf("=== Debug Layout End ===\n");
}

void debug_per_info_current(void) {
    uprintf("=== Current per_info State ===\n");
    uprintf("per_info.raw: 0x%08lX\n", per_info.raw);
    uprintf("eco_tog_flag = %d\n", per_info.eco_tog_flag);
    uprintf("sleep_mode = %d\n", per_info.sleep_mode);
    uprintf("ind_brightness = %d\n", per_info.ind_brightness);
    uprintf("ind_color_index = %d\n", per_info.ind_color_index);
    uprintf("smd_color_index = %d\n", per_info.smd_color_index);
    uprintf("backlight_off = %d\n", per_info.backlight_off);
    uprintf("saved_rgb_mode = %d\n", per_info.saved_rgb_mode);

    // 检查EEPROM中的实际值
    per_info_t eeprom_data;
    eeprom_data.raw = eeconfig_read_kb();
    uprintf("EEPROM backlight_off = %d\n", eeprom_data.backlight_off);
    uprintf("EEPROM raw: 0x%08lX\n", eeprom_data.raw);
    uprintf("=== Current State End ===\n");
}

// 新增：dev_info_t 调试函数
void debug_dev_info_layout(void) {
    uprintf("=== dev_info_t Debug Layout ===\n");
    uprintf("Structure size: %d bytes\n", sizeof(dev_info_t));
    uprintf("Field ranges: devs(0-255), last_devs(0-255)\n");

    dev_info_t test = {0};
    uprintf("Initial raw: 0x%08lX\n", test.raw);

    test.devs = 1;
    uprintf("devs=1: 0x%08lX\n", test.raw);

    test.last_devs = 2;
    uprintf("last_devs=2: 0x%08lX\n", test.raw);

    test.devs = 255;
    test.last_devs = 255;
    uprintf("devs=255, last_devs=255: 0x%08lX\n", test.raw);

    // 验证读取
    uprintf("=== Field Verification ===\n");
    uprintf("  devs=%d (expect 255)\n", test.devs);
    uprintf("  last_devs=%d (expect 255)\n", test.last_devs);
    uprintf("=== dev_info Layout End ===\n");
}

void debug_dev_info_current(void) {
    uprintf("=== Current dev_info State ===\n");
    uprintf("dev_info.raw: 0x%08lX\n", dev_info.raw);
    uprintf("devs = %d\n", dev_info.devs);
    uprintf("last_devs = %d\n", dev_info.last_devs);

    // 显示设备类型含义
    const char* dev_names[] = {"USB", "HOST1", "HOST2", "HOST3", "2.4G"};
    if (dev_info.devs < 5) {
        uprintf("devs = %s\n", dev_names[dev_info.devs]);
    } else {
        uprintf("devs = UNKNOWN(%d)\n", dev_info.devs);
    }

    if (dev_info.last_devs < 5) {
        uprintf("last_devs = %s\n", dev_names[dev_info.last_devs]);
    } else {
        uprintf("last_devs = UNKNOWN(%d)\n", dev_info.last_devs);
    }

    uprintf("=== dev_info State End ===\n");
}

// 新增：综合调试函数
void debug_all_structures(void) {
    uprintf("\n");
    uprintf("============================================\n");
    uprintf("=== COMPREHENSIVE STRUCTURE DEBUG ===\n");
    uprintf("============================================\n");

    debug_per_info_current();
    uprintf("\n");
    debug_dev_info_current();

    uprintf("============================================\n");
    uprintf("=== DEBUG SESSION END ===\n");
    uprintf("============================================\n");
    uprintf("\n");
}
#endif

// ====================
// LED 控制函数
// ====================

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

// ====================
// 按键处理
// ====================

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

                DEBUG_PRINTF("RGB_TOG: current backlight_off=%d\n", per_info.backlight_off);

                if (per_info.backlight_off) {
                    // 开启背光
                    per_info.backlight_off = false;

                    uint8_t target_mode = per_info.saved_rgb_mode;
                    if (target_mode > 21 || target_mode == RGB_MATRIX_CUSTOM_EFFECT_OFF) {
                        target_mode = rgb_matrix_config.mode;
                        if (target_mode == RGB_MATRIX_CUSTOM_EFFECT_OFF || target_mode > 21) {
                            target_mode = RGB_MATRIX_DEFAULT_MODE;
                        }
                    }
                    rgb_matrix_mode(target_mode);
                    DEBUG_PRINTF("RGB_TOG: backlight ON, mode=%d\n", target_mode);

                } else {
                    // 关闭背光
                    per_info.backlight_off = true;

                    uint8_t current_mode = rgb_matrix_get_mode();
                    if (current_mode <= 21 && current_mode != RGB_MATRIX_CUSTOM_EFFECT_OFF) {
                        per_info.saved_rgb_mode = current_mode;
                        DEBUG_PRINTF("RGB_TOG: saved current mode %d\n", current_mode);
                    } else {
                        per_info.saved_rgb_mode = RGB_MATRIX_DEFAULT_MODE;
                        DEBUG_PRINTF("RGB_TOG: used default mode instead of %d\n", current_mode);
                    }
                    rgb_matrix_mode(RGB_MATRIX_CUSTOM_EFFECT_OFF);
                    DEBUG_PRINTF("RGB_TOG: backlight OFF\n");
                }

                validate_per_info_ranges();
                eeconfig_update_kb(per_info.raw);
                DEBUG_PRINTF("RGB_TOG: saved per_info.raw=0x%08lX\n", per_info.raw);
            }
            return false;

        case RGB_MOD:
            if (record->event.pressed) {
                if (bts_info.bt_info.low_vol) {
                    return false;
                }

                // 如果背光关闭，先开启
                if (per_info.backlight_off) {
                    per_info.backlight_off = false;
                    eeconfig_update_kb(per_info.raw);
                    DEBUG_PRINTF("RGB_MOD: auto-enabled backlight\n");
                }

                rgb_matrix_step();
                // 跳过关闭模式
                if (rgb_matrix_get_mode() == RGB_MATRIX_CUSTOM_EFFECT_OFF) {
                    rgb_matrix_step();
                }

                // 保存新模式
                uint8_t new_mode = rgb_matrix_get_mode();
                if (new_mode <= 21) {
                    per_info.saved_rgb_mode = new_mode;
                    eeconfig_update_kb(per_info.raw);
                    DEBUG_PRINTF("RGB_MOD: saved new mode %d\n", new_mode);
                }
            }
            return false;

        case RGB_RMOD:
            if (record->event.pressed) {
                if (bts_info.bt_info.low_vol) {
                    return false;
                }

                // 如果背光关闭，先开启
                if (per_info.backlight_off) {
                    per_info.backlight_off = false;
                    eeconfig_update_kb(per_info.raw);
                    DEBUG_PRINTF("RGB_RMOD: auto-enabled backlight\n");
                }

                rgb_matrix_step_reverse();
                // 跳过关闭模式
                if (rgb_matrix_get_mode() == RGB_MATRIX_CUSTOM_EFFECT_OFF) {
                    rgb_matrix_step_reverse();
                }

                // 保存新模式
                uint8_t new_mode = rgb_matrix_get_mode();
                if (new_mode <= 21) {
                    per_info.saved_rgb_mode = new_mode;
                    eeconfig_update_kb(per_info.raw);
                    DEBUG_PRINTF("RGB_RMOD: saved new mode %d\n", new_mode);
                }
            }
            return false;

#ifdef CONSOLE_ENABLE
        case KC_A: {
            if (record->event.pressed) {
                validate_per_info_ranges();
                debug_per_info_current();
            }
            return false;
        }

        case KC_B: {
            if (record->event.pressed) {
                debug_per_info_layout();
            }
            return false;
        }

        // 新增：dev_info 调试按键
        case KC_C: {
            if (record->event.pressed) {
                debug_dev_info_current();
            }
            return false;
        }

        case KC_D: {
            if (record->event.pressed) {
                debug_dev_info_layout();
            }
            return false;
        }

        // 新增：综合调试按键
        case KC_F: {
            if (record->event.pressed) {
                debug_all_structures();
            }
            return false;
        }
#endif

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

// ====================
// 初始化函数
// ====================

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

void keyboard_post_init_kb(void) {
    per_info.raw = eeconfig_read_kb();

    // 验证和修复可能的无效数据
    validate_per_info_ranges();

    DEBUG_PRINTF("Boot: per_info.raw=0x%08lX, backlight_off=%d, saved_rgb_mode=%d\n",
                 per_info.raw, per_info.backlight_off, per_info.saved_rgb_mode);

    // 新增：开机时显示 dev_info 状态
    DEBUG_PRINTF("Boot: dev_info.raw=0x%08lX, devs=%d, last_devs=%d\n",
                 dev_info.raw, dev_info.devs, dev_info.last_devs);

    if (per_info.backlight_off) {
        // 用户关闭了背光，设置为关闭模式
        rgb_matrix_mode_noeeprom(RGB_MATRIX_CUSTOM_EFFECT_OFF);
        DEBUG_PRINTF("Boot: applied backlight OFF\n");
    } else {
        // 用户开启了背光，恢复保存的模式
        uint8_t target_mode = per_info.saved_rgb_mode;
        if (target_mode > 21 || target_mode == RGB_MATRIX_CUSTOM_EFFECT_OFF) {
            target_mode = RGB_MATRIX_DEFAULT_MODE;
        }
        rgb_matrix_mode_noeeprom(target_mode);
        DEBUG_PRINTF("Boot: applied backlight ON, mode=%d\n", target_mode);
    }

    keyboard_post_init_user();
}

void eeconfig_init_kb(void) {
    // 设置默认值
    per_info.sleep_mode      = 1;                       // 默认睡眠模式
    per_info.ind_brightness  = RGB_MATRIX_VAL_STEP * 3; // 默认亮度
    per_info.smd_color_index = 0;                       // 默认颜色
    per_info.ind_color_index = 0;                       // 默认颜色
    per_info.backlight_off   = true;                    // 默认关闭背光
    per_info.eco_tog_flag    = false;                   // 默认关闭省电
    per_info.saved_rgb_mode  = RGB_MATRIX_DEFAULT_MODE; // 默认RGB模式
    per_info.reserved        = 0;                       // 预留位清零

    // RGB配置
    rgb_matrix_config.hsv.h = 170;
    rgb_matrix_config.mode  = RGB_MATRIX_CUSTOM_EFFECT_OFF;

    validate_per_info_ranges(); // 验证范围

    eeconfig_update_kb(per_info.raw);
    eeconfig_update_rgb_matrix(&rgb_matrix_config);

    DEBUG_PRINTF("eeconfig_init_kb: initialized with backlight_off=true, saved_mode=%d\n",
                 per_info.saved_rgb_mode);

    eeconfig_init_user();
}

// ====================
// 任务和指示器
// ====================

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

    // 可选：每10秒检查一次结构体是否被篡改
    static uint32_t last_check = 0;
    static per_info_t last_per_info = {0};
    static dev_info_t last_dev_info = {0};
    static bool first_run = true;

    if (first_run) {
        last_per_info = per_info;
        last_dev_info = dev_info;
        first_run = false;
    }

    // 每10秒检查一次
    if (timer_elapsed32(last_check) > 10000) {
        last_check = timer_read32();

        // 检查 per_info 是否被篡改
        if (last_per_info.raw != per_info.raw) {
            uprintf("WARNING: per_info changed unexpectedly!\n");
            uprintf("  Old: 0x%08lX, New: 0x%08lX\n", last_per_info.raw, per_info.raw);
            uprintf("  backlight_off: %d -> %d\n", last_per_info.backlight_off, per_info.backlight_off);
            uprintf("  saved_rgb_mode: %d -> %d\n", last_per_info.saved_rgb_mode, per_info.saved_rgb_mode);
        }

        // 检查 dev_info 是否被篡改
        if (last_dev_info.raw != dev_info.raw) {
            uprintf("WARNING: dev_info changed unexpectedly!\n");
            uprintf("  Old: 0x%08lX, New: 0x%08lX\n", last_dev_info.raw, dev_info.raw);
            uprintf("  devs: %d -> %d\n", last_dev_info.devs, dev_info.devs);
            uprintf("  last_devs: %d -> %d\n", last_dev_info.last_devs, dev_info.last_devs);
        }

        last_per_info = per_info;
        last_dev_info = dev_info;
    }
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

    // GUI lock red
    if (keymap_config.no_gui) {
        rgb_matrix_set_color(92, 100, 100, 100);
    }

    return true;
}

// ====================
// DIP 开关处理
// ====================

#ifdef DIP_SWITCH_ENABLE
bool dip_switch_update_kb(uint8_t index, bool active) {
    if (!dip_switch_update_user(index, active)) {
        return false;
    }

    // if (index == 0) {
    //     if (active) {
    //         if (dev_info.devs == DEVS_USB) {
    //             bt_switch_mode(DEVS_USB, dev_info.devs, false); // wireless mode
    //         }
    //     } else {
    //         if (dev_info.devs != DEVS_USB) {
    //             bt_switch_mode(dev_info.devs, DEVS_USB, false); // usb mode
    //         }
    //     }
    // }

    return true;
}
#endif // DIP_SWITCH_ENABLE
