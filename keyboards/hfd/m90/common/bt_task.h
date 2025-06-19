/**
 * @file bt_task.h
 * @brief
 * @author Joy chang.li@westberrytech.com
 * @version 1.1.0
 * @date 2022-10-03
 *
 * @copyright Copyright (c) 2022 Westberry Technology (ChangZhou) Corp., Ltd
 */

#pragma once

#include "common/bts_lib.h"
#include "host.h"
#include "keycode.h"
#include "keyboard.h"
#include "mousekey.h"
#include "programmable_button.h"
#include "command.h"
#include "led.h"
#include "action_layer.h"
#include "action_tapping.h"
#include "action_util.h"
#include "action.h"
#include "wait.h"
#include "keycode_config.h" // #define BT_DEBUG_MODE

// #define BT_DEBUG_MODE
#define ENTRY_STOP_TIMEOUT 100 // ms
// #define ENTRY_STOP_TIMEOUT (30 * 60000) // ms

typedef union {
    uint32_t raw;
    struct {
        uint8_t devs;
        uint8_t last_devs;
    };
} dev_info_t;

// 在 bt_task.h 中
typedef union PACKED {
    uint32_t raw;
    struct {
        uint8_t sleep_mode : 2;      // 0-3 (4种)
        uint8_t ind_brightness : 8;  // 0-255 (256种)
        uint8_t smd_color_index : 4; // 0-15 (9种，0-8有效)
        uint8_t ind_color_index : 4; // 0-15 (9种，0-8有效)
        uint8_t saved_rgb_mode : 5;  // 0-31 (22种，0-21有效)
        bool    backlight_off : 1;   // 0-1
        bool    eco_tog_flag : 1;    // 0-1
        uint8_t reserved : 7;        // 剩余7位预留
    };
} per_info_t;

extern per_info_t per_info;
extern dev_info_t dev_info;
extern bts_info_t bts_info;

/**
 * @brief bluetooth 初始化函数
 * @param None
 * @return None
 */
void bt_init(void);

/**
 * @brief bluetooth交互任务
 * @param None
 * @return None
 */
void bt_task(void);

/**
 * @brief 处理和BT相关的按键
 * @param keycode: 键值
 * @param record: 记录值
 * @return None
 */
bool process_record_bt(uint16_t keycode, keyrecord_t *record);

/**
 * @brief rgb指示灯任务
 * @param None
 * @return None
 */
uint8_t bt_indicator_rgb(uint8_t led_min, uint8_t led_max);

/**
 * @brief 切换工作模式
 * @param None
 * @return None
 */
void bt_switch_mode(uint8_t last_mode, uint8_t now_mode, uint8_t reset);

// 在其他结构体定义后面添加
typedef struct {
    bool     triggered;   // 是否已触发低电量警告
    uint8_t  blink_count; // 当前闪烁次数
    uint32_t blink_time;  // 闪烁计时器
    bool     blink_state; // 当前闪烁状态（亮/灭）
    bool     completed;   // 闪烁是否完成
} low_power_warning_t;
