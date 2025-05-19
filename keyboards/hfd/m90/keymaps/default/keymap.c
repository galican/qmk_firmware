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
// clang-format off

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

    [WIN_C] = LAYOUT_87_ansi( /* FN */
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,          _______,
        _______, _______, _______,                            _______,                            _______, KC_RWIN, _______,  _______, _______, _______, _______),

    [WIN_D] = LAYOUT_87_ansi( /* FN */
        _______,          KC_BRID, KC_BRIU, _______, _______, _______, _______, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD,  KC_VOLU,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,          _______,
        _______, _______, _______,                            _______,                            _______, KC_APP,  _______,  _______, _______, _______, _______),

    [WIN_FN] = LAYOUT_87_ansi( /* FN */
        TG(WIN_D),        KC_BRID, KC_BRIU, _______, _______, _______, _______, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD,  KC_VOLU,
        _______, BT_HOST1,BT_HOST2,BT_HOST3,BT_2_4G, BT_USB,  _______, _______, _______, _______, IND_COL, IND_VAD, IND_VAI,  KEY_CLR, _______, _______, _______,
        EE_CLR,  _______, _______, _______, _______, _______, _______, _______, _______, _______, BT_PAIR, KC_SLEP, _______,  RGB_HUI, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           RGB_SPD,
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, SW_OS1,  TG(WIN_C),         _______,          RGB_VAI,
        _______, GU_TOGG, _______,                            _______,                            _______, KC_RWIN,MO(WIN_FN),_______, RGB_MOD, RGB_VAD, RGB_RMOD),

    [MAC_B] = LAYOUT_87_ansi( /* Base */
        KC_ESC,           KC_BRID, KC_BRIU, KC_MCTL, KC_SEARH,KC_VOICE,KC_FOCUS,KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD,  KC_VOLU,
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,   KC_BSPC, KC_INS,  KC_HOME, KC_PGUP,
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC,  KC_BSLS, KC_DEL,  KC_END,  KC_PGDN,
        KC_CAPS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,           KC_ENT,
        KC_LSFT,          KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,           KC_RSFT, KC_UP,
        KC_LCTL, KC_LOPT, KC_LCMD,                            KC_SPC,                             KC_ROPT, _______,MO(WIN_FN),KC_RCTL, KC_LEFT, KC_DOWN, KC_RGHT),

    [MAC_C] = LAYOUT_87_ansi( /* FN */
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,          _______,
        _______, _______, _______,                            _______,                            _______, KC_ROPT, _______,  _______, _______, _______, _______),

    [MAC_D] = LAYOUT_87_ansi( /* FN */
        _______,          KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,   KC_F12,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,
        _______,          _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______,          _______,
        _______, _______, _______,                            _______,                            _______, _______, _______,  _______, _______, _______, _______),

    [MAC_FN] = LAYOUT_87_ansi( /* FN */
        TG(WIN_D),        KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,   KC_F12,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, IND_COL, IND_VAD, IND_VAI,  KEY_CLR, _______, _______, _______,
        EE_CLR,  _______, _______, _______, _______, _______, _______, _______, _______, _______, BT_PAIR, KC_SLEP, _______,  RGB_HUI, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           RGB_SPD,
        _______,          _______, _______, _______, _______, _______, _______, _______, SW_OS1,  _______, TG(WIN_C),         _______,          RGB_VAI,
        _______, _______, _______,                            _______,                            _______, KC_ROPT, _______,  _______, RGB_MOD, RGB_VAD, RGB_RMOD),

};
// clang-format on
uint8_t  VAL_OUT_LEDINDEX;
uint8_t  VAL_OUT_blink_cnt;
uint32_t VAL_OUT_blink_time;

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case RGB_VAI: {
            if (record->event.pressed && (rgb_matrix_get_val() == RGB_MATRIX_MAXIMUM_BRIGHTNESS)) {
                if (timer_elapsed32(VAL_OUT_blink_time) > 200) {
                    VAL_OUT_blink_time = timer_read32();
                }
                VAL_OUT_blink_cnt = 6;
                VAL_OUT_LEDINDEX  = 71;
            }
        } break;
        case RGB_VAD: {
            if (record->event.pressed && (rgb_matrix_get_val() == 0x00)) {
                if (timer_elapsed32(VAL_OUT_blink_time) > 200) {
                    VAL_OUT_blink_time = timer_read32();
                }
                VAL_OUT_blink_cnt = 6;
                VAL_OUT_LEDINDEX  = 73;
            }
        } break;
        case RGB_SPI: {
            if (record->event.pressed && (rgb_matrix_get_speed() == 0xff)) {
                if (timer_elapsed32(VAL_OUT_blink_time) > 200) {
                    VAL_OUT_blink_time = timer_read32();
                }
                VAL_OUT_blink_cnt = 6;
                VAL_OUT_LEDINDEX  = 72;
            }
        } break;
        case RGB_SPD: {
            if (record->event.pressed && (rgb_matrix_get_speed() == 0x00)) {
                if (timer_elapsed32(VAL_OUT_blink_time) > 200) {
                    VAL_OUT_blink_time = timer_read32();
                }
                VAL_OUT_blink_cnt = 6;
                VAL_OUT_LEDINDEX  = 74;
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

    return true;
}
