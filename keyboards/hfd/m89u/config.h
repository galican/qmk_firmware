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
#    define ENTRY_STOP_MODE // 超时进入STOP Mode
#    define BT_CABLE_PIN B8 // 充电接入时为高
#    define BT_CABLE_TRUE 0
#    define BT_CHARGE_PIN B9 // 充电时为低，充满时为高
#    define RGB_DRIVER_SDB_PIN A15

#    define BT_MODE_SW_PIN A1 // 低电平时
#    define BT_MODE_SW_TRUE 1
// #  define RF_MODE_SW_PIN C14 // 低电平时

// #  define INDLED_USB_PIN C11
// #  define INDLED_BT_PIN D2
// #  define INDLED_2_4G_PIN C10
#    define BT_HOST1_INDEX 16
#    define BT_HOST2_INDEX 17
#    define BT_HOST3_INDEX 18
#    define BT_2_4G_INDEX 12
#    define BT_USB_INDEX 13
#endif

#ifdef DIP_SWITCH_ENABLE
#    define DIP_SWITCH_PINS {C11}
#endif

/* I2C Config for LED Driver */
#define IS31FL3733_DRIVER_COUNT 1
#define IS31FL3733_I2C_ADDRESS_1 0b1110100
// #define IS31FL3733_I2C_ADDRESS_2 0b1110111
#define I2C1_SDA_PIN B7
#define I2C1_SCL_PIN B6
#define I2C1_SCL_PAL_MODE 4
#define I2C1_OPMODE OPMODE_I2C
#define I2C1_CLOCK_SPEED 400000

// #define RGB_MATRIX_LED_COUNT 23
// #define RGB_MATRIX_SLEEP
// #define RGB_MATRIX_DEFAULT_VAL 200

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

// #define RGB_MATRIX_VAL_STEP 20
// #define RGB_MATRIX_MINIMUM_BRIGHTNESS RGB_MATRIX_VAL_STEP
#define RGB_MATRIX_DEFAULT_VAL 40
// #define RGB_MATRIX_DEFAULT_SAT 128

// #define RGB_MATRIX_SPD_STEP 64
// #define RGB_MATRIX_MIN_SPD RGB_MATRIX_SPD_STEP
#define RGB_MATRIX_DEFAULT_SPD 128

// #define RGB_MATRIX_DEFAULT_MODE RGB_MATRIX_SOLID_COLOR

// #define BT_VOL_LED 3
// #define LED_NUM 22
// #define LED_2G LED_P4
// #define LED_BT1 LED_P1
// #define LED_BT2 LED_P2
// #define LED_BT3 LED_P3
// #define LED_USB LED_P5
// #define LED_NKRO 0
// #define LED_NKRO_CNT 6
// #define LED_TAB 1
// #define LED_SLEEP LED_TAB
// #define LED_TAB 37
// #define LED_PAST 7
// #define LED_P5 13
// #define LED_RST_ALL LED_P5

// #define LED_PSLS 6
// #define LED_P4 12
// #define LED_RST_LYR LED_P4

// #define LED_EQL 5
// #define LED_P1 16
// #define LED_P2 17
// #define LED_P3 18
// #define LED_P4 12
// #define LED_P5 13
// #define LED_RST_BLE LED_EQL

// #define LED_1 17

// #define LED_BAT LED_NUM
// #define LED_BAT_CNT 16

// #define LED_P0 20
// #define LED_WIN_LYR LED_P0
// #define LED_MAC_LYR LED_P0
// #define LED_MAC_CNT 6
