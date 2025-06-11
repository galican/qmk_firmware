// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "quantum.h"

enum bt_keycodes {
  BT_HOST1 = QK_KB_0,
  BT_HOST2,
  BT_HOST3,
  // BT_PAIR,
  // EX_PAIR,
  BT_2_4G,
  BT_USB,
  BT_VOL,

  SW_OS,

  RST_ALL,
  RST_LYR,
  RST_BLE,

  NL_OFF,
  RGB_TEST,
  SLEEP_EN,
  NKRO_EN,
};
