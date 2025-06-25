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

#define FACTORY FACTORY_RESET
#define SW_SLEP SLEEP_TOGGLE
#define KC_SPOT MAC_Spotlight
#define KC_DICT MAC_Dictation
#define KC_DND MAC_Do_Not_Disturb

#define KC_FLEX LGUI(KC_TAB)
#define KC_DESK LGUI(KC_D)
#define KC_EXAK LCTL(KC_DOWN)

enum __layers {
    WIN_B,
    WIN_FN,
    MAC_B,
    MAC_FN,
};
// clang-format off

 const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    [WIN_B] = LAYOUT_82_ansi( /* Base */
        KC_ESC,  KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,     KC_F12,  KC_INS,   KC_MUTE,
        KC_GRV,  KC_1,     KC_2,     KC_3,     KC_4,     KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS,    KC_EQL,  KC_BSPC,
        KC_TAB,  KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC,    KC_RBRC, KC_BSLS,  KC_DEL,  KC_PGUP,
        KC_CAPS, KC_A,     KC_S,     KC_D,     KC_F,     KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,             KC_ENT,   KC_END,  KC_PGDN,
        KC_LSFT,           KC_Z,     KC_X,     KC_C,     KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,             KC_RSFT,  KC_UP,
        KC_LCTL, KC_LWIN,  KC_LALT,                               KC_SPC,                             KC_RALT, MO(WIN_FN), KC_RCTL, KC_LEFT,  KC_DOWN, KC_RGHT),

    [WIN_FN] = LAYOUT_82_ansi( /* FN */
        _______, KC_BRID,  KC_BRIU,  KC_FLEX,  KC_DESK,  KC_WBAK, KC_WSCH, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD,    KC_VOLU, _______, _______,
        _______, BT_HOST1, BT_HOST2, BT_HOST3, BT_2_4G,  BT_USB,  _______, _______, _______, _______, _______, _______,    _______, _______,
        _______, _______,  _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______,
        _______, _______,  _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______,             _______, _______, _______,
        _______,           _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______,             _______, RM_VALU,
        _______, WIN_LOCK, _______,                               _______,                            _______, _______,    _______, RM_SPDD, RM_VALD, RM_SPDU),

    [MAC_B] = LAYOUT_82_ansi( /* Base */
        KC_ESC,  KC_BRID,  KC_BRIU,  KC_MCTL,  KC_LPAD,  KC_EXAK, KC_NO,   KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_VOLD,    KC_VOLU, KC_INS,   KC_MUTE,
        KC_GRV,  KC_1,     KC_2,     KC_3,     KC_4,     KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS,    KC_EQL,  KC_BSPC,  KC_DEL,  KC_PGUP,
        KC_TAB,  KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC,    KC_RBRC, KC_BSLS,  KC_END,  KC_PGDN,
        KC_CAPS, KC_A,     KC_S,     KC_D,     KC_F,     KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,             KC_ENT,
        KC_LSFT,           KC_Z,     KC_X,     KC_C,     KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,             KC_RSFT,  KC_UP,
        KC_LCTL, KC_LOPT,  KC_LCMD,                               KC_SPC,                             KC_RCMD, MO(MAC_FN), KC_RCTL, KC_LEFT,  KC_DOWN, KC_RGHT),

    [MAC_FN] = LAYOUT_82_ansi( /* mac fn */
        _______, KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,  KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,     KC_F12,   _______, _______,
        _______, BT_HOST1, BT_HOST2, BT_HOST3, BT_2_4G,  BT_USB,  _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______,
        _______, _______,  _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______,
        _______, _______,  _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______,             _______,
        _______,           _______,  _______,  _______,  _______, _______, _______, _______, _______, _______, _______,             _______, RM_VALU,
        _______, _______,  _______,                               _______,                            _______, _______,    _______, RM_SPDD, RM_VALD, RM_SPDU),

 };

 #if defined(ENCODER_MAP_ENABLE)
const uint16_t PROGMEM encoder_map[][NUM_ENCODERS][NUM_DIRECTIONS] = {
    [MAC_BASE] = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU) },
    [MAC_FN]   = {ENCODER_CCW_CW(_______, _______) },
    [WIN_BASE] = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU) },
    [WIN_FN]   = {ENCODER_CCW_CW(_______, _______) }
};
#endif // ENCODER_MAP_ENABLE

 // clang-format on

 bool process_record_user(uint16_t keycode, keyrecord_t *record) {
     return true;
 }

bool rgb_matrix_indicators_advanced_user(uint8_t led_min, uint8_t led_max) {
    return true;
}

void keyboard_post_init_user() {}
