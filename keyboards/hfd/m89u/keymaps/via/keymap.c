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
    // WIN_B,
    // WIN_FN,
    // MAC_B,
    // MAC_FN
    PAD_B,
    PAD_NUM,
    PAD_DIR,
    PAD_FN,
};

#define KC_TASK A(KC_TAB)
#define KC_FLXP LGUI(KC_E)
#define KC_SIRI LGUI(KC_SPC)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    [0] = LAYOUT_ansi( /* Base */
        KC_ESC,  KC_TAB,  KC_BSPC, MO(3),
        KC_NUM,  KC_EQL,  KC_PSLS, KC_PAST,
        KC_P7,   KC_P8,   KC_P9,   KC_PMNS,
        KC_P4,   KC_P5,   KC_P6,   KC_PPLS,
        KC_P1,   KC_P2,   KC_P3,   KC_PENT,
                 KC_P0,   KC_PDOT
    ),
    [1] = LAYOUT_ansi(
        KC_ESC,  KC_TAB,  KC_BSPC, MO(3),
        XXXXXXX, KC_EQL,  KC_PSLS, KC_PAST,
        KC_7,    KC_8,    KC_9,    KC_PMNS,
        KC_4,    KC_5,    KC_6,    KC_PPLS,
        KC_1,    KC_2,    KC_3,    KC_PENT,
                 KC_0,    KC_PDOT
    ),
    [2] = LAYOUT_ansi(                              
        KC_ESC,  KC_TAB,  KC_BSPC, MO(3),
        XXXXXXX, KC_EQL,  KC_PSLS, KC_PAST,
        KC_HOME, KC_UP,   KC_PGUP, KC_PMNS,
        KC_LEFT, XXXXXXX, KC_RGHT, KC_PPLS,
        KC_END,  KC_DOWN, KC_PGDN, KC_PENT,
                 KC_INS,  KC_DEL
    ),

    [3] = LAYOUT_ansi(
        NKRO_EN, SLEEP_EN,BT_USB,  XXXXXXX,
        RGB_TOG, RST_BLE, RST_LYR, RST_ALL,
        RGB_HUI, RGB_VAI, RGB_MOD, RGB_SAI,
        BT_2_4G, BT_USB,  RGB_SPI, NL_OFF,
        BT_HOST1,BT_HOST2,BT_HOST3,XXXXXXX,
                 SW_OS, XXXXXXX
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

uint16_t ALT_pressed_time = 0;
uint16_t GUI_pressed_time = 0;
uint16_t CTRL_pressed_time = 0;
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case A(KC_TAB):{
            if (!record->event.pressed) {
                if (dev_info.devs) {
                    bts_process_keys(KC_TAB, 0, dev_info.devs, keymap_config.no_gui);
                } else {
                    unregister_code(KC_TAB);
                }
                ALT_pressed_time = timer_read();
                return false;
            } else {
                if (dev_info.devs) {
                    bts_process_keys(KC_LALT, 1, dev_info.devs, keymap_config.no_gui);
                    bts_process_keys(KC_TAB, 1, dev_info.devs, keymap_config.no_gui);
                } else {
                    register_code(KC_LALT);
                    register_code(KC_TAB);
                }
                return false;
           }
            break;
        }
        case G(KC_TAB):{
            if (!record->event.pressed) {
                if (dev_info.devs) {
                    bts_process_keys(KC_TAB, 0, dev_info.devs, keymap_config.no_gui);
                } else {
                    unregister_code(KC_TAB);
                }
                GUI_pressed_time = timer_read();
                return false;
            } else {
                if (dev_info.devs) {
                    bts_process_keys(KC_LGUI, 1, dev_info.devs, keymap_config.no_gui);
                    bts_process_keys(KC_TAB, 1, dev_info.devs, keymap_config.no_gui);
                } else {
                    register_code(KC_LGUI);
                    register_code(KC_TAB);
                }
                return false;
           }
            break;
        }
        case G(KC_SPC):{
            if (!record->event.pressed) {
                if (dev_info.devs) {
                    bts_process_keys(KC_SPC, 0, dev_info.devs, keymap_config.no_gui);
                } else {
                    unregister_code(KC_SPC);
                }
                GUI_pressed_time = timer_read();
                return false;
            } else {
                if (dev_info.devs) {
                    bts_process_keys(KC_LGUI, 1, dev_info.devs, keymap_config.no_gui);
                    bts_process_keys(KC_SPC, 1, dev_info.devs, keymap_config.no_gui);
                } else {
                    register_code(KC_LGUI);
                    register_code(KC_SPC);
                }
                return false;
           }
        }
        break;
        case C(KC_SPC):{
            if (!record->event.pressed) {
                if (dev_info.devs) {
                    bts_process_keys(KC_SPC, 0, dev_info.devs, keymap_config.no_gui);
                } else {
                    unregister_code(KC_SPC);
                }
                CTRL_pressed_time = timer_read();
                return false;
            } else {
                if (dev_info.devs) {
                    bts_process_keys(KC_LCTL, 1, dev_info.devs, keymap_config.no_gui);
                    bts_process_keys(KC_SPC, 1, dev_info.devs, keymap_config.no_gui);
                } else {
                    register_code(KC_LCTL);
                    register_code(KC_SPC);
                }
                return false;
           }
        }
        break;
        default:
            return true;
    }
    return false;
}
void housekeeping_task_user(void) {

    if (ALT_pressed_time && (timer_elapsed(ALT_pressed_time) >= 500)) {
        ALT_pressed_time = 0;
            if (dev_info.devs) {
                bts_process_keys(KC_LALT, 0, dev_info.devs, keymap_config.no_gui);
            }else{
                unregister_code(KC_LALT);
            }
    }
    if (GUI_pressed_time && (timer_elapsed(GUI_pressed_time) >= 500)) {
        GUI_pressed_time = 0;
            if (dev_info.devs) {
                bts_process_keys(KC_LGUI, 0, dev_info.devs, keymap_config.no_gui);
            }else{
                unregister_code(KC_LGUI);
            }
    }
    if (CTRL_pressed_time && (timer_elapsed(CTRL_pressed_time) >= 500)) {
        CTRL_pressed_time = 0;
            if (dev_info.devs) {
                bts_process_keys(KC_LCTL, 0, dev_info.devs, keymap_config.no_gui);
            }else{
                unregister_code(KC_LCTL);
            }
    }
}
