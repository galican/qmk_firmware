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

// clang-format off

#define IND_COL INDICATOR_COLOR
#define IND_VAI INDICATOR_BRIGHTNESS_UP
#define IND_VAD INDICATOR_BRIGHTNESS_DOWN
#define KEY_CLR KEYMAP_CLEAR
#define KEY_SLP KEYBOARD_SLEEP
#define KEY_ECO ECO
#define FACTORY FACTORY_RESET
// #define PAIRING PAIRING_SETTING
#define KEY_RES KEYBOARD_RESET
#define BLE_RES BLE_RESET

enum __layers {
    WIN_B,
    WIN_C,
    WIN_D,
    WIN_FN,
    MAC_B,
    MAC_C,
    MAC_D,
    MAC_FN
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    [WIN_B] = LAYOUT_87_ansi( /* Base */
        KC_ESC,           KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,   KC_F12,
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,   KC_BSPC, KC_INS,  KC_HOME, KC_PGUP,
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC,  KC_BSLS, KC_DEL,  KC_END,  KC_PGDN,
        KC_CAPS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,           KC_ENT,
        KC_LSFT,          KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,           KC_RSFT,          KC_UP,
        KC_LCTL, KC_LWIN, KC_LALT,                            KC_SPC,                             KC_RALT, KC_APP,MO(WIN_FN), KC_RCTL, KC_LEFT, KC_DOWN, KC_RGHT),

    [WIN_C] = LAYOUT_87_ansi( /* windows multimedia */
        _______,          KC_BRID, KC_BRIU, _______, _______, _______, _______, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD,  KC_VOLU,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,          _______,
        _______, _______, _______,                            _______,                            _______, _______,  _______,  _______, _______, _______, _______),

    [WIN_D] = LAYOUT_87_ansi( /* windows app to win */
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,          _______,
        _______, _______, _______,                            _______,                            _______, KC_RWIN, _______,  _______, _______, _______, _______),

    [WIN_FN] = LAYOUT_87_ansi( /* FN */
        _______,          KC_BRID, KC_BRIU, _______, _______, _______, _______, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD,  KC_VOLU,
      TG(WIN_C), BT_HOST1,BT_HOST2,BT_HOST3,BT_2_4G, BT_USB,  _______, _______, _______, _______, IND_COL, IND_VAD, IND_VAI,  KEY_RES, _______, _______, _______,
        FACTORY, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KEY_SLP, KEY_ECO,  RGB_HUI, BLE_RES, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           RGB_SPI,
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, SW_OS1,  TG(WIN_D),         RGB_SPD,          RGB_VAI,
        _______, GU_TOGG, _______,                            _______,                            _______, KC_RWIN,MO(WIN_FN),_______, RGB_MOD, RGB_VAD, RGB_RMOD),

    [MAC_B] = LAYOUT_87_ansi( /* Base */
        KC_ESC,           KC_BRID, KC_BRIU, KC_MCTL, G(KC_SPC), _______, _______, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD,  KC_VOLU,
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,   KC_BSPC, KC_INS,  KC_HOME, KC_PGUP,
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC,  KC_BSLS, KC_DEL,  KC_END,  KC_PGDN,
        KC_CAPS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,           KC_ENT,
        KC_LSFT,          KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,           KC_RSFT, KC_UP,
        KC_LCTL, KC_LOPT, KC_LCMD,                            KC_SPC,                             KC_ROPT, KC_APP,MO(MAC_FN),KC_RCTL, KC_LEFT, KC_DOWN, KC_RGHT),

    [MAC_C] = LAYOUT_87_ansi( /* FN */
        _______,          KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,   KC_F12,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,          _______,
        _______, _______, _______,                            _______,                            _______, _______, _______,  _______, _______, _______, _______),

    [MAC_D] = LAYOUT_87_ansi( /* mac opt */
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,          _______,
        _______, _______, _______,                            _______,                            _______, KC_ROPT, _______,  _______, _______, _______, _______),

    [MAC_FN] = LAYOUT_87_ansi( /* mac fn */
        _______,          KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,   KC_F12,
      TG(MAC_C), _______, _______, _______, _______, _______, _______, _______, _______, _______, IND_COL, IND_VAD, IND_VAI,  KEY_RES, _______, _______, _______,
        FACTORY, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KEY_SLP, KEY_ECO,  RGB_HUI, BLE_RES, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           RGB_SPI,
        _______,          _______, _______, _______, _______, _______, _______, _______, SW_OS1,  _______, TG(MAC_D),         RGB_SPD,          RGB_VAI,
        _______, _______, _______,                            _______,                            _______, KC_ROPT, _______,  _______, RGB_MOD, RGB_VAD, RGB_RMOD),

};
// clang-format on
static uint8_t  VAL_OUT_LEDINDEX;
static uint8_t  VAL_OUT_blink_cnt;
static uint32_t VAL_OUT_blink_time;

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case RGB_VAI: {
            if (record->event.pressed && (rgb_matrix_get_val() == RGB_MATRIX_MAXIMUM_BRIGHTNESS)) {
                if (timer_elapsed32(VAL_OUT_blink_time) > 200) {
                    VAL_OUT_blink_time = timer_read32();
                }
                VAL_OUT_blink_cnt = 6;
                VAL_OUT_LEDINDEX  = 72;
            }
        } break;
        case RGB_VAD: {
            if (record->event.pressed && (rgb_matrix_get_val() == 0x00)) {
                if (timer_elapsed32(VAL_OUT_blink_time) > 200) {
                    VAL_OUT_blink_time = timer_read32();
                }
                VAL_OUT_blink_cnt = 6;
                VAL_OUT_LEDINDEX  = 82;
            }
        } break;
        case RGB_SPI: {
            if (record->event.pressed && (rgb_matrix_get_speed() == 0xff)) {
                if (timer_elapsed32(VAL_OUT_blink_time) > 200) {
                    VAL_OUT_blink_time = timer_read32();
                }
                VAL_OUT_blink_cnt = 6;
                VAL_OUT_LEDINDEX  = 59;
            }
        } break;
        case RGB_SPD: {
            if (record->event.pressed && (rgb_matrix_get_speed() == 0x00)) {
                if (timer_elapsed32(VAL_OUT_blink_time) > 200) {
                    VAL_OUT_blink_time = timer_read32();
                }
                VAL_OUT_blink_cnt = 6;
                VAL_OUT_LEDINDEX  = 71;
            }
        } break;
        case IND_VAI: {
            if (record->event.pressed) {
                dev_info.ind_brightness = qadd8(dev_info.ind_brightness, RGB_MATRIX_VAL_STEP);
                dev_info.ind_brightness = (dev_info.ind_brightness > RGB_MATRIX_MAXIMUM_BRIGHTNESS) ? RGB_MATRIX_MAXIMUM_BRIGHTNESS : dev_info.ind_brightness;
                eeconfig_update_user(dev_info.raw);
                if (dev_info.ind_brightness == RGB_MATRIX_MAXIMUM_BRIGHTNESS) {
                    VAL_OUT_blink_cnt = 6;
                    VAL_OUT_LEDINDEX  = 25;
                }
            }
        } break;
        case IND_VAD: {
            if (record->event.pressed) {
                dev_info.ind_brightness = qsub8(dev_info.ind_brightness, RGB_MATRIX_VAL_STEP);
                dev_info.ind_brightness = (dev_info.ind_brightness > RGB_MATRIX_MAXIMUM_BRIGHTNESS) ? RGB_MATRIX_MAXIMUM_BRIGHTNESS : dev_info.ind_brightness;
                eeconfig_update_user(dev_info.raw);
                if (dev_info.ind_brightness == 0x00) {
                    VAL_OUT_blink_cnt = 6;
                    VAL_OUT_LEDINDEX  = 24;
                }
            }
        } break;
        case IND_COL: {
            if (record->event.pressed) {
                dev_info.ind_color += RGB_MATRIX_HUE_STEP;
                if (dev_info.ind_color > 0xFF) {
                    dev_info.ind_color = 0x00;
                }
                eeconfig_update_user(dev_info.raw);
            }
            eeconfig_update_kb(dev_info.raw);
        } break;
        case KEY_ECO: {
            if (record->event.pressed) {
                dev_info.eco_tog_flag = !dev_info.eco_tog_flag;
                eeconfig_update_user(dev_info.raw);
            }
        } break;
        case KEY_SLP: {
            if (record->event.pressed) {
                dev_info.en_sleep_flag = !dev_info.en_sleep_flag;
                eeconfig_update_user(dev_info.raw);
                if (dev_info.en_sleep_flag) {
                    bts_send_vendor(v_en_sleep_bt);
                    bts_send_vendor(v_en_sleep_wl);
                } else {
                    bts_send_vendor(v_dis_sleep_bt);
                    bts_send_vendor(v_dis_sleep_wl);
                }
            }
        } break;
        default:
            break;
    }
    return true;
}

bool rgb_matrix_indicators_advanced_user(uint8_t led_min, uint8_t led_max) {
    if (VAL_OUT_blink_cnt) {
        if (timer_elapsed32(VAL_OUT_blink_time) > 200) {
            VAL_OUT_blink_time = timer_read32();
            VAL_OUT_blink_cnt--;
        }
        if (VAL_OUT_blink_cnt % 2) {
            rgb_matrix_set_color(VAL_OUT_LEDINDEX, 100, 100, 100);
        } else {
            rgb_matrix_set_color(VAL_OUT_LEDINDEX, 0, 0, 0);
        }
    }

    HSV hsv = rgb_matrix_get_hsv();
    hsv.h   = dev_info.ind_color;
    hsv.v   = dev_info.ind_brightness;
    RGB rgb = hsv_to_rgb(hsv);

    if (host_keyboard_led_state().num_lock && ((bts_info.bt_info.paired) || (dev_info.devs == DEVS_USB))) {
        rgb_matrix_set_color(84, rgb.r, rgb.g, rgb.b);
    }
    if (host_keyboard_led_state().caps_lock && ((bts_info.bt_info.paired) || (dev_info.devs == DEVS_USB))) {
        rgb_matrix_set_color(85, rgb.r, rgb.g, rgb.b);
    }
    if (host_keyboard_led_state().scroll_lock && ((bts_info.bt_info.paired) || (dev_info.devs == DEVS_USB))) {
        rgb_matrix_set_color(86, rgb.r, rgb.g, rgb.b);
    }

    if (dev_info.eco_tog_flag) {
        for (uint8_t i = 84; i < RGB_MATRIX_LED_COUNT; i++) {
            rgb_matrix_set_color(i, 0, 0, 0);
        }
    }

    return true;
}
