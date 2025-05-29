// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "quantum.h"

enum bt_keycodes {
    BT_HOST1 = QK_KB_0,
    BT_HOST2,
    BT_HOST3,
    BT_2_4G,
    BT_USB,
    BT_VOL,
    SW_OS,
    INDICATOR_COLOR,
    INDICATOR_BRIGHTNESS_UP,
    INDICATOR_BRIGHTNESS_DOWN,
    ECO,
    FACTORY_RESET,
    KEYBOARD_RESET,
    BLE_RESET,
    FN_FUN,
    FN_MENU,
    SLEEP_TOGGLE,
    KC_SIRI,
};

typedef union {
    uint32_t raw;
    struct {
        uint8_t rgb_matrix_effect;
        // uint8_t rgb_matrix_color_index;
    };
} rgb_info_t;

rgb_info_t rgb_info;
