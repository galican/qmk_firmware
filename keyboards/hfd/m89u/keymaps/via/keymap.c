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

enum __layers {
    PAD_B,
    // PAD_NUM,
    // PAD_DIR,
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

bool eco_switch_flag = true; // 绿色节能模式开关

// clang-format off

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    [PAD_B] = LAYOUT_numpad_6x4( /* Base */
        KC_ESC,  KC_TAB,  KC_BSPC, MO(1),
        KC_NUM,  KC_EQL,  KC_PSLS, KC_PAST,
        KC_P7,   KC_P8,   KC_P9,   KC_PMNS,
        KC_P4,   KC_P5,   KC_P6,   KC_PPLS,
        KC_P1,   KC_P2,   KC_P3,   KC_PENT,
                 KC_P0,   KC_PDOT
    ),
    // [PAD_NUM] = LAYOUT_ansi(
    //     KC_ESC,  KC_TAB,  KC_BSPC, MO(3),
    //     _______, KC_EQL,  KC_PSLS, KC_PAST,
    //     KC_7,    KC_8,    KC_9,    KC_PMNS,
    //     KC_4,    KC_5,    KC_6,    KC_PPLS,
    //     KC_1,    KC_2,    KC_3,    KC_PENT,
    //              KC_0,    KC_PDOT
    // ),
    // [PAD_DIR] = LAYOUT_ansi(
    //     KC_ESC,  KC_TAB,  KC_BSPC, MO(3),
    //     _______, KC_EQL,  KC_PSLS, KC_PAST,
    //     KC_HOME, KC_UP,   KC_PGUP, KC_PMNS,
    //     KC_LEFT, _______, KC_RGHT, KC_PPLS,
    //     KC_END,  KC_DOWN, KC_PGDN, KC_PENT,
    //              KC_INS,  KC_DEL
    // ),

    [PAD_FN] = LAYOUT_numpad_6x4(
        NK_TOGG, SW_SLEP, KEY_ECO, _______,
        RGB_TOG, BLE_RES, KEY_RES, FACTORY,
        RGB_HUI, RGB_VAI, RGB_MOD, RGB_SAI,
        BT_2_4G, BT_USB,  RGB_SPI, _______,
        BT_HOST1,BT_HOST2,BT_HOST3,_______,
                 _______, _______
    )

    // [3] = LAYOUT_ansi(
    //     BT_HOST, BT_2_4G, BT_USB,  XXXXXXX,
    //     RGB_MOD, EE_CLR3, EE_CLR2, EE_CLR1,
    //     RGB_HUI, RGB_VAD, RGB_TOG, WL_SLEEP,
    //     XXXXXXX, XXXXXXX, XXXXXXX, NL_OFF,
    //     XXXXXXX, XXXXXXX, XXXXXXX, BT_PAIR,
    //              XXXXXXX, EX_PAIR
    // )
};

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case RGB_VAI: {
            if (record->event.pressed) {
                if (rgb_matrix_get_val() == RGB_MATRIX_MAXIMUM_BRIGHTNESS) {
                    rgb_matrix_config.hsv.v = RGB_MATRIX_VAL_STEP;
                } else {
                    rgb_matrix_increase_val();
                }
            }
            return false;
        }
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
        case RGB_HUI: {
            if (record->event.pressed) {
                dev_info.config.smd_color_index++;
                if (dev_info.config.smd_color_index >= sizeof(indicator_color_tab) / sizeof(indicator_color_tab[0])) {
                    dev_info.config.smd_color_index = 0;
                }
                eeconfig_update_user(dev_info.raw);
                rgb_matrix_config.hsv.h = indicator_color_tab[dev_info.config.smd_color_index][0];
                rgb_matrix_config.hsv.s = indicator_color_tab[dev_info.config.smd_color_index][1];
                rgb_matrix_config.hsv.v = rgb_matrix_config.hsv.v;
            }
            return false;
        }
        case RGB_SAI: {
            if (record->event.pressed) {
                if (rgb_matrix_get_sat() >= UINT8_MAX) {
                    rgb_matrix_config.hsv.s = RGB_MATRIX_SAT_STEP; // 设置为最小饱和度
                } else {
                    rgb_matrix_increase_sat();
                }
            }
            return false;
        }
        case KEY_ECO: {
            if (record->event.pressed) {
                if (eco_switch_flag) {
                    dev_info.config.eco_off_flag = !dev_info.config.eco_off_flag;
                    eeconfig_update_user(dev_info.raw);
                }
            }
            return false;
        }
        case SW_SLEP: {
            if (record->event.pressed) {
                dev_info.config.sleep_mode += 1;
                if (dev_info.config.sleep_mode > 3) {
                    dev_info.config.sleep_mode = 0;
                }
                switch (dev_info.config.sleep_mode) {
                    case 0: // 关闭睡眠
                        bts_send_vendor(v_dis_sleep_bt);
                        bts_send_vendor(v_dis_sleep_wl);
                        VAL_OUT_blink_cnt   = 8;
                        VAL_OUT_LEDINDEX    = 1;
                        VAL_OUT_blink_color = (RGB){0, 0, 100};
                        VAL_OUT_blink_time  = timer_read32();
                        break;
                    case 1: // 开启睡眠1
                        bts_send_vendor(v_en_sleep_bt);
                        bts_send_vendor(v_en_sleep_bt);
                        VAL_OUT_blink_cnt   = 2;
                        VAL_OUT_LEDINDEX    = 1;
                        VAL_OUT_blink_color = (RGB){0, 0, 100};
                        VAL_OUT_blink_time  = timer_read32();
                        break;
                    case 2: // 开启睡眠2
                        bts_send_vendor(v_en_sleep_bt);
                        bts_send_vendor(v_en_sleep_bt);
                        VAL_OUT_blink_cnt   = 4;
                        VAL_OUT_LEDINDEX    = 1;
                        VAL_OUT_blink_color = (RGB){0, 0, 100};
                        VAL_OUT_blink_time  = timer_read32();
                        break;
                    case 3: // 开启睡眠3
                        bts_send_vendor(v_en_sleep_bt);
                        bts_send_vendor(v_en_sleep_bt);
                        VAL_OUT_blink_cnt   = 6;
                        VAL_OUT_LEDINDEX    = 1;
                        VAL_OUT_blink_color = (RGB){0, 0, 100};
                        VAL_OUT_blink_time  = timer_read32();
                        break;
                    default:
                        break;
                }
                eeconfig_update_user(dev_info.raw);
            }
            return false;
        }
        default: {
            // 处理其他按键
            return true; // 允许默认处理
        }
    }
}

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

    if (!dev_info.config.eco_off_flag && host_keyboard_led_state().num_lock && ((bts_info.bt_info.paired) || (dev_info.devs == DEVS_USB))) {
        rgb_matrix_set_color(22, 0, 0, 100);
    } else {
        rgb_matrix_set_color(22, 0, 0, 0);
    }

    if (dev_info.config.eco_off_flag) {
        rgb_matrix_set_color(22, 0, 0, 0);
    }

    return true;
}

void keyboard_post_init_user() {
    rgb_matrix_config.hsv.h = indicator_color_tab[dev_info.config.smd_color_index][0];
    rgb_matrix_config.hsv.s = indicator_color_tab[dev_info.config.smd_color_index][1];
    rgb_matrix_config.hsv.v = rgb_matrix_config.hsv.v;
}
