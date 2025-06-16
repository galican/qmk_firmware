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
#    define BT_CABLE_PIN B8  // 充电接入时为高
#    define BT_CHARGE_PIN B9 // 充电时为低，充满时为高
#    define RGB_DRIVER_SDB_PIN A15
#    define BT_HOST1_INDEX 14
#    define BT_HOST2_INDEX 15
#    define BT_HOST3_INDEX 16
#    define BT_2_4G_INDEX 17
#    define BT_USB_INDEX 18
#    define BT_MODE_SW_PIN B10
#endif

/* RGB Matrix */
// #define RGB_MATRIX_LED_COUNT 81

/* SPI Config for spi flash*/
#define SPI_DRIVER SPIDQ
#define SPI_SCK_PIN B3
#define SPI_MOSI_PIN B5
#define SPI_MISO_PIN B4
#define SPI_MOSI_PAL_MODE 5

/* I2C Config for LED Driver */
#define IS31FL3733_DRIVER_COUNT 2
#define IS31FL3733_I2C_ADDRESS_1 0b1110100
#define IS31FL3733_I2C_ADDRESS_2 0b1110111
#define I2C1_SDA_PIN B7
#define I2C1_SCL_PIN B6
#define I2C1_SCL_PAL_MODE 4
#define I2C1_OPMODE OPMODE_I2C
#define I2C1_CLOCK_SPEED 400000

#define EXTERNAL_FLASH_SPI_SLAVE_SELECT_PIN C12
#define WEAR_LEVELING_BACKING_SIZE (4 * 1024)

#define RGB_MATRIX_FRAMEBUFFER_EFFECTS
#define RGB_MATRIX_KEYPRESSES

#define RGB_MATRIX_DEFAULT_VAL 80
#define RGB_MATRIX_DEFAULT_SPD 192

#ifdef DIP_SWITCH_ENABLE
#    define DIP_SWITCH_PINS {B10}
#endif
