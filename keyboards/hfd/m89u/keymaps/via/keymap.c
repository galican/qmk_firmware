/* Copyright (C) 2023 jonylee@hfd
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include "common/bt_task.h"
#include <lib/lib8tion/lib8tion.h>

enum __layers {
    PAD_B,
    PAD_RESERVED1,
    PAD_RESERVED2,
    PAD_FN,
};

#define IND_HUE INDICATOR_HUE
#define IND_VAL INDICATOR_BRIGHTNESS
#define KEY_SLP KEYBOARD_SLEEP
#define KEY_ECO ECO
#define FACTORY FACTORY_RESET
#define KEY_RES KEYBOARD_RESET
#define BLE_RES BLE_RESET
#define SW_SLEP SLEEP_TOGGLE

static uint8_t  VAL_OUT_LEDINDEX;
static uint8_t  VAL_OUT_blink_cnt;
static RGB      VAL_OUT_blink_color;
static uint32_t VAL_OUT_blink_time;

static uint8_t indicator_color_tab[][3] = {
    {HSV_BLUE},    // BLUE
    {HSV_PURPLE},  // PURPLE
    {HSV_WHITE},   // WHITE
    {HSV_MAGENTA}, // MAGENTA
    {HSV_RED},     // RED
    {HSV_ORANGE},  // ORANGE
    {HSV_YELLOW},  // YELLOW
    {HSV_GREEN},   // GREEN
    {HSV_CYAN},    // CYAN
};

// clang-format off

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    [PAD_B] = LAYOUT_numpad_6x4(
        KC_ESC,  KC_TAB,  KC_BSPC, MO(3),
        KC_NUM,  KC_EQL,  KC_PSLS, KC_PAST,
        KC_P7,   KC_P8,   KC_P9,   KC_PMNS,
        KC_P4,   KC_P5,   KC_P6,   KC_PPLS,
        KC_P1,   KC_P2,   KC_P3,   KC_PENT,
                 KC_P0,   KC_PDOT
    ),

    [1] = LAYOUT_numpad_6x4(
        KC_ESC,  KC_TAB,  KC_BSPC, MO(3),
        TO(2),   KC_EQL,  KC_PSLS, KC_PAST,
        KC_P7,   KC_P8,   KC_P9,   KC_PMNS,
        KC_P4,   KC_P5,   KC_P6,   KC_PPLS,
        KC_P1,   KC_P2,   KC_P3,   KC_PENT,
                 KC_P0,   KC_PDOT
    ),

    [2] = LAYOUT_numpad_6x4(
        KC_ESC,  KC_TAB,  KC_BSPC, MO(3),
        TO(1),   KC_EQL,  KC_PSLS, KC_PAST,
        KC_HOME, KC_UP,   KC_PGUP, KC_PMNS,
        KC_LEFT, _______, KC_RGHT, KC_PPLS,
        KC_END,  KC_DOWN, KC_PGDN, KC_PENT,
                 KC_INS,  KC_DEL
    ),

    [PAD_FN] = LAYOUT_numpad_6x4(
        NK_TOGG, SW_SLEP, KEY_ECO, _______,
        RGB_TOG, BLE_RES, KEY_RES, FACTORY,
        RGB_HUI, RGB_VAI, RGB_MOD, RGB_SAI,
        BT_2_4G, BT_USB,  RGB_SPI, _______,
        BT_HOST1,BT_HOST2,BT_HOST3,_______,
                 SW_OS,   RGB_TEST
    )
};

// clang-format on

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case RGB_VAI: {
            if (record->event.pressed) {
                if (rgb_matrix_get_val() == RGB_MATRIX_MAXIMUM_BRIGHTNESS) {
                    rgb_matrix_config.hsv.v = 50;
                } else {
                    rgb_matrix_increase_val();
                }
            }
        }
            return false;
        case RGB_VAD: {
            if (record->event.pressed) {
                // rgb_matrix_decrease_val();
                rgb_matrix_config.hsv.v = rgb_matrix_get_val() - RGB_MATRIX_VAL_STEP;
                if (rgb_matrix_get_val() <= 50) rgb_matrix_config.hsv.v = 50;
            }
        }
            return false;
        case RGB_SPI: {
            if (record->event.pressed) {
                if (rgb_matrix_get_speed() == UINT8_MAX) {
                    rgb_matrix_config.speed = 0x00; // 设置为最小速度
                } else {
                    rgb_matrix_increase_speed();
                }
            }
            return false;
        }
        case IND_VAL: {
            if (record->event.pressed) {
                per_info.ind_brightness = qadd8(per_info.ind_brightness, RGB_MATRIX_VAL_STEP);
                per_info.ind_brightness = (per_info.ind_brightness > RGB_MATRIX_MAXIMUM_BRIGHTNESS) ? RGB_MATRIX_MAXIMUM_BRIGHTNESS : per_info.ind_brightness;
                eeconfig_update_kb(per_info.raw);
                if (per_info.ind_brightness == RGB_MATRIX_MAXIMUM_BRIGHTNESS) {
                    per_info.ind_brightness = 0x00;
                }
            }
        }
            return false;
        case IND_HUE: {
            if (record->event.pressed) {
                per_info.ind_color_index++;
                if (per_info.ind_color_index >= sizeof(indicator_color_tab) / sizeof(indicator_color_tab[0])) {
                    per_info.ind_color_index = 0;
                }
                eeconfig_update_kb(per_info.raw);
            }
        }
            return false;
        case RGB_HUI: {
            if (record->event.pressed) {
                per_info.smd_color_index++;
                if (per_info.smd_color_index >= sizeof(indicator_color_tab) / sizeof(indicator_color_tab[0])) {
                    per_info.smd_color_index = 0;
                }
                eeconfig_update_kb(per_info.raw);
                rgb_matrix_config.hsv.h = indicator_color_tab[per_info.smd_color_index][0];
                rgb_matrix_config.hsv.s = indicator_color_tab[per_info.smd_color_index][1];
                rgb_matrix_config.hsv.v = rgb_matrix_config.hsv.v;
            }
        }
            return false;
        case RGB_HUD: {
            if (record->event.pressed) {
                if (per_info.smd_color_index == 0) {
                    per_info.smd_color_index = sizeof(indicator_color_tab) / sizeof(indicator_color_tab[0]) - 1;
                } else {
                    per_info.smd_color_index--;
                }
                eeconfig_update_kb(per_info.raw);
                rgb_matrix_config.hsv.h = indicator_color_tab[per_info.smd_color_index][0];
                rgb_matrix_config.hsv.s = indicator_color_tab[per_info.smd_color_index][1];
                rgb_matrix_config.hsv.v = rgb_matrix_config.hsv.v;
            }
        }
            return false;
        case RGB_SAI: {
            if (record->event.pressed) {
                if (rgb_matrix_get_sat() >= UINT8_MAX) {
                    rgb_matrix_config.hsv.s = 0; // 设置为最小饱和度
                } else if (rgb_matrix_get_sat() == 0) {
                    rgb_matrix_config.hsv.s = RGB_MATRIX_SAT_STEP; // 设置为最小饱和度
                    rgb_matrix_increase_sat();
                } else {
                    rgb_matrix_increase_sat();
                }
            }
            return false;
        }
        case KEY_ECO: {
            if (record->event.pressed) {
                per_info.eco_tog_flag = !per_info.eco_tog_flag;
                eeconfig_update_user(dev_info.raw);
            }
            return false;
        }
        case SW_SLEP: {
            if (record->event.pressed) {
                per_info.sleep_mode += 1;
                if (per_info.sleep_mode > 3) {
                    per_info.sleep_mode = 0;
                }
                switch (per_info.sleep_mode) {
                    case 0: // 关闭睡眠
                        bts_send_vendor(v_dis_sleep_bt);
                        bts_send_vendor(v_dis_sleep_wl);
                        VAL_OUT_blink_cnt   = 8;
                        VAL_OUT_LEDINDEX    = 1;
                        VAL_OUT_blink_color = (RGB){0, 0, 200};
                        VAL_OUT_blink_time  = timer_read32();
                        break;
                    case 1: // 开启睡眠1
                        bts_send_vendor(v_en_sleep_bt);
                        bts_send_vendor(v_en_sleep_wl);
                        VAL_OUT_blink_cnt   = 2;
                        VAL_OUT_LEDINDEX    = 1;
                        VAL_OUT_blink_color = (RGB){0, 0, 200};
                        VAL_OUT_blink_time  = timer_read32();
                        break;
                    case 2: // 开启睡眠2
                        bts_send_vendor(v_en_sleep_bt);
                        bts_send_vendor(v_en_sleep_wl);
                        VAL_OUT_blink_cnt   = 4;
                        VAL_OUT_LEDINDEX    = 1;
                        VAL_OUT_blink_color = (RGB){0, 0, 200};
                        VAL_OUT_blink_time  = timer_read32();
                        break;
                    case 3: // 开启睡眠3
                        bts_send_vendor(v_en_sleep_bt);
                        bts_send_vendor(v_en_sleep_wl);
                        VAL_OUT_blink_cnt   = 6;
                        VAL_OUT_LEDINDEX    = 1;
                        VAL_OUT_blink_color = (RGB){0, 0, 200};
                        VAL_OUT_blink_time  = timer_read32();
                        break;
                    default:
                        break;
                }
                eeconfig_update_kb(per_info.raw);
            }
        }
            return false;

        default: {
            // 处理其他按键
            return true; // 允许默认处理
        }
    }
}

static HSV hsv;
static RGB rgb;

bool rgb_matrix_indicators_advanced_user(uint8_t led_min, uint8_t led_max) {
    if (VAL_OUT_blink_cnt) {
        if (timer_elapsed32(VAL_OUT_blink_time) > 500) {
            VAL_OUT_blink_time = timer_read32();
            VAL_OUT_blink_cnt--;
        }
        if (VAL_OUT_blink_cnt % 2) {
            rgb_matrix_set_color(VAL_OUT_LEDINDEX, VAL_OUT_blink_color.r, VAL_OUT_blink_color.g, VAL_OUT_blink_color.b);
        } else {
            rgb_matrix_set_color(VAL_OUT_LEDINDEX, 0, 0, 0);
        }
    }

    hsv.h = indicator_color_tab[per_info.ind_color_index][0];
    hsv.s = indicator_color_tab[per_info.ind_color_index][1];
    hsv.v = per_info.ind_brightness;
    rgb   = hsv_to_rgb(hsv);
    // clang-format off
    if (!per_info.eco_tog_flag && \
        ( \
            ( \
                (host_keyboard_led_state().num_lock && get_highest_layer(default_layer_state) == 0) && \
                ((bts_info.bt_info.paired) || (dev_info.devs == DEVS_USB)) \
            ) || \
                get_highest_layer(default_layer_state | layer_state) == 1 \
        ) \
    ) {
        // clang-format on
        rgb_matrix_set_color(22, rgb.r, rgb.g, rgb.b);
    } else {
        rgb_matrix_set_color(22, 0, 0, 0);
    }

    // if (per_info.eco_tog_flag) {
    //     rgb_matrix_set_color(22, 0, 0, 0);
    // }

    return true;
}

void keyboard_post_init_user() {
    rgb_matrix_config.hsv.h = indicator_color_tab[per_info.smd_color_index][0];
    rgb_matrix_config.hsv.s = indicator_color_tab[per_info.smd_color_index][1];
    rgb_matrix_config.hsv.v = rgb_matrix_config.hsv.v;
}
