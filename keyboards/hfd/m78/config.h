// Copyright 2023 JoyLee (@itarze)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/*
 * Feature disable options
 *  These options are also useful to firmware size reduction.
 */

#define BT_MODE_ENABLE
#ifdef BT_MODE_ENABLE
#    define NO_USB_STARTUP_CHECK
#    define ENTRY_STOP_MODE  // 超时进入STOP Mode
#    define BT_CABLE_PIN B9  //
#    define BT_CHARGE_PIN B8 // 充电时为低，充满时为高
#    define BT_HOST1_INDEX 26
#    define BT_HOST2_INDEX 25
#    define BT_HOST3_INDEX 24
#    define BT_2_4G_INDEX 23
#    define BT_USB_INDEX 22
#    define BT_MODE_SW_PIN B11
#endif

/* RGB Matrix */
#define WS2812_EN_PIN D2

/* SPI Config for spi flash*/
#define SPI_DRIVER SPIDQ
#define SPI_SCK_PIN B3
#define SPI_MOSI_PIN B5
#define SPI_MISO_PIN B4
#define SPI_MOSI_PAL_MODE 5

#define EXTERNAL_FLASH_SPI_SLAVE_SELECT_PIN C12
#define WEAR_LEVELING_BACKING_SIZE (4 * 1024)

#define RGB_MATRIX_FRAMEBUFFER_EFFECTS
#define RGB_MATRIX_KEYPRESSES
