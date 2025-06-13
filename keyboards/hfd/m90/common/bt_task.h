/**
 * @file bt_task.h
 * @brief Bluetooth Task Management Header (基于原有代码整理)
 * @author Joy chang.li@westberrytech.com
 * @version 2.3.0
 * @date 2025-06-13
 *
 * @copyright Copyright (c) 2023 Westberry Technology Corp., Ltd
 */

#pragma once

#include "common/bts_lib.h"
#include "../m90.h"
#include "usb_main.h"
#include "uart.h"
#include "quantum.h"

// =============================================
// 配置宏定义
// =============================================

// #define BT_DEBUG_MODE
#ifdef BT_DEBUG_MODE
#    define BT_DEBUG_INFO(fmt, ...) dprintf(fmt, ##__VA_ARGS__)
#else
#    define BT_DEBUG_INFO(fmt, ...)
#endif

// 常量定义（消除魔法数字）
#define BT_ENTRY_STOP_TIMEOUT 100 // ms
#define BT_NUM_LONG_PRESS_KEYS 7
#define BT_MAX_RGB_INDICATORS 5
#define BT_MAX_BATTERY_INDEX 9
#define BT_DEFAULT_BRIGHTNESS 8
#define BT_BATTERY_LOW_THRESHOLD 20
#define BT_BATTERY_CRITICAL_THRESHOLD 15
#define BT_SLEEP_CHECK_INTERVAL 1000            // ms
#define BT_BATTERY_QUERY_INTERVAL 30000         // ms
#define BT_LED_STANDBY_TIMEOUT (10 * 60 * 1000) // 10 minutes

// 参数验证宏
#define BT_VALIDATE_PTR(ptr)                    \
    do {                                        \
        if (!(ptr)) {                           \
            BT_DEBUG_INFO("Invalid pointer\n"); \
            return;                             \
        }                                       \
    } while (0)

#define BT_VALIDATE_DEVICE_TYPE(type)                         \
    do {                                                      \
        if ((type) >= DEVS_COUNT) {                           \
            BT_DEBUG_INFO("Invalid device type: %d\n", type); \
            return;                                           \
        }                                                     \
    } while (0)

#define BT_VALIDATE_PTR_RET(ptr, ret)           \
    do {                                        \
        if (!(ptr)) {                           \
            BT_DEBUG_INFO("Invalid pointer\n"); \
            return (ret);                       \
        }                                       \
    } while (0)

// =============================================
// 蓝牙专用RGB定义（避免与QMK冲突）
// =============================================

// RGB指示灯索引
#define BT_USB_LED_INDEX 18
#define BT_HOST1_LED_INDEX 14
#define BT_HOST2_LED_INDEX 15
#define BT_HOST3_LED_INDEX 16
#define BT_2_4G_LED_INDEX 17

// 蓝牙专用RGB颜色定义（避免与QMK内置宏冲突）
#define BT_RGB_WHITE {100, 100, 100}
#define BT_RGB_BLUE {0, 0, 100}
#define BT_RGB_GREEN {0, 100, 0}
#define BT_RGB_RED {100, 0, 0}
#define BT_RGB_ORANGE {100, 50, 0}
#define BT_RGB_YELLOW {100, 100, 0}
#define BT_RGB_OFF {0, 0, 0}

// =============================================
// 蓝牙专用枚举定义（避免重复定义）
// =============================================

// 使用bts_lib.h中的devs_t作为device_type_t
typedef devs_t device_type_t;
#define DEVS_COUNT 5 // 只使用前5个设备类型

// 指示器状态
typedef enum {
    BT_INDICATOR_OFF = 0,
    BT_INDICATOR_CONNECTING, // 慢闪 - 回连模式
    BT_INDICATOR_PAIRING,    // 快闪 - 配对模式
    BT_INDICATOR_CONNECTED,  // 常亮 - 已连接
    BT_INDICATOR_SLEEP
} bt_indicator_status_t;

// 重置类型
typedef enum { BT_RESET_NONE = 0, BT_RESET_FACTORY, BT_RESET_KEYBOARD, BT_RESET_BLE } bt_reset_type_t;

// 睡眠模式
typedef enum { BT_SLEEP_NEVER = 0, BT_SLEEP_10MIN, BT_SLEEP_30MIN, BT_SLEEP_60MIN } bt_sleep_mode_t;

// =============================================
// 结构体定义
// =============================================

// 设备配置信息
typedef struct PACKED {
    uint8_t sleep_mode : 2;
    uint8_t eco_tog_flag : 2;
    uint8_t ind_color_index : 4;
    uint8_t smd_color_index : 4;
    uint8_t ind_brightness;
} bt_device_config_t;

// 设备信息（兼容性结构）
typedef union PACKED {
    uint32_t raw;
    struct PACKED {
        uint8_t            devs : 3;
        uint8_t            last_devs : 3;
        bt_device_config_t config;
        uint8_t            reserved : 2;
    };
} bt_dev_info_t;

// 长按键定义
typedef struct {
    uint16_t keycode;
    uint32_t press_time;
    uint32_t threshold_ms;
    void (*callback)(uint16_t);
} bt_longpress_key_t;

// 设备管理状态
typedef struct {
    device_type_t      current;
    device_type_t      last_wireless;
    bt_device_config_t config;
    bool               sleep_flag;
    uint32_t           raw;
} bt_device_state_t;

// LED指示器状态
typedef struct {
    bt_indicator_status_t status;
    bool                  reset_requested; // 是否为配对模式
    bool                  connecting_mode; // 是否为回连模式
    uint32_t              blink_time;
    uint32_t              status_start_time;
    uint32_t              connected_time; // 新增：连接成功时间
    RGB                   current_color;
    bool                  flip_state;
    uint32_t              blink_interval; // 闪烁间隔
    bool                  timeout_sleep;  // 新增：超时休眠标志
} bt_indicator_state_t;

// 电源管理状态
typedef struct {
    uint8_t  battery_level;
    bool     low_battery;
    bool     critical_battery;
    bool     charging;
    bool     charge_complete;
    uint32_t low_power_blink_time;
    uint32_t charge_full_blink_time;
    uint32_t last_query_time;
    bool     first_low_warning;
    bool     first_critical_warning;
} bt_power_state_t;

// 长按键管理状态
typedef struct {
    bt_longpress_key_t keys[BT_NUM_LONG_PRESS_KEYS];
    uint8_t            key_count;
    uint32_t           factory_reset_press_time;
    uint8_t            factory_reset_status;
    uint8_t            factory_reset_press_cnt;
} bt_longpress_state_t;

// RGB效果状态
typedef struct {
    bool     rgb_status_save;
    bool     bak_rgb_toggle;
    bool     sober;
    uint32_t close_rgb_time;
    uint32_t key_press_time;

    // 全键闪烁
    uint8_t  all_blink_cnt;
    uint32_t all_blink_time;
    RGB      all_blink_color;

    // 单键闪烁
    uint8_t  single_blink_cnt;
    uint8_t  single_blink_index;
    RGB      single_blink_color;
    uint32_t single_blink_time;

    // USB状态
    uint32_t usb_switch_time;
    uint8_t  usb_blink_cnt;
} bt_rgb_state_t;

// 主要的蓝牙任务上下文
typedef struct {
    bt_device_state_t    device;
    bt_indicator_state_t indicator;
    bt_power_state_t     power;
    bt_longpress_state_t longpress;
    bt_rgb_state_t       rgb;

    uint32_t init_time;
    uint32_t last_activity;
    uint32_t last_update_time;
    bool     initialized;
    bool     query_vol_flag;
    bool     kb_sleep_flag;
} bt_context_t;

// =============================================
// 常量表声明
// =============================================

extern const uint8_t  bt_rgb_index_table[DEVS_COUNT];
extern const uint8_t  bt_rgb_color_table[DEVS_COUNT][3];
extern const uint32_t bt_sleep_time_table[4];
extern const uint8_t  bt_battery_query_index[10];

// =============================================
// 全局变量声明（重命名避免冲突）
// =============================================

extern bt_dev_info_t bt_dev_info;
extern bts_info_t    bt_bts_info;
// 移除 bt_led_inited 声明，使用 m90.c 中的 led_inited

// =============================================
// 主要接口函数
// =============================================

void    bt_init(void);
void    bt_task(void);
bool    bt_process_record(uint16_t keycode, keyrecord_t *record);
uint8_t bt_indicator_rgb(uint8_t led_min, uint8_t led_max);
void    bt_switch_mode(uint8_t last_mode, uint8_t now_mode, uint8_t reset);

// =============================================
// 模块化内部接口
// =============================================

// 设备管理模块
void          bt_device_init(bt_device_state_t *device);
void          bt_device_switch(bt_device_state_t *device, device_type_t target, bool reset);
device_type_t bt_device_get_current(const bt_device_state_t *device);
bool          bt_device_is_wireless(const bt_device_state_t *device);
void          bt_device_scan_hardware_switch(bt_device_state_t *device);

// 电源管理模块
void bt_power_init(bt_power_state_t *power);
void bt_power_update(bt_power_state_t *power);
void bt_power_set_battery_level(bt_power_state_t *power, uint8_t level);
bool bt_power_should_sleep(const bt_power_state_t *power, uint32_t last_activity, uint8_t sleep_mode);
void bt_power_render_indicators(const bt_power_state_t *power);

// LED指示器模块
void bt_indicator_init(bt_indicator_state_t *indicator);
void bt_indicator_update(bt_indicator_state_t *indicator, const bt_device_state_t *device);
void bt_indicator_set_status(bt_indicator_state_t *indicator, bt_indicator_status_t status);
void bt_indicator_render(const bt_indicator_state_t *indicator, uint8_t led_min, uint8_t led_max);

// 长按键处理模块
void bt_longpress_init(bt_longpress_state_t *longpress);
void bt_longpress_register(bt_longpress_state_t *longpress, uint16_t keycode, uint32_t threshold, void (*callback)(uint16_t));
void bt_longpress_update(bt_longpress_state_t *longpress);
void bt_longpress_on_press(bt_longpress_state_t *longpress, uint16_t keycode);
void bt_longpress_on_release(bt_longpress_state_t *longpress, uint16_t keycode);

// RGB效果管理模块
void bt_rgb_init(bt_rgb_state_t *rgb);
void bt_rgb_open(bt_rgb_state_t *rgb);
void bt_rgb_close(bt_rgb_state_t *rgb, uint32_t current_time);
void bt_rgb_set_all_blink(bt_rgb_state_t *rgb, uint8_t count, RGB color);
void bt_rgb_set_single_blink(bt_rgb_state_t *rgb, uint8_t index, uint8_t count, RGB color);

// =============================================
// 兼容性接口
// =============================================

void bt_bat_low_indicator(void);
void bt_open_rgb(void);
void bt_led_off_standby(void);

// =============================================
// 内联辅助函数
// =============================================

static inline bool bt_is_switch_key(uint16_t keycode) {
    return (keycode >= BT_HOST1 && keycode <= BT_2_4G) || keycode == BT_USB;
}

static inline bool bt_is_reset_key(uint16_t keycode) {
    return keycode == FACTORY_RESET || keycode == KEYBOARD_RESET || keycode == BLE_RESET;
}

static inline uint8_t bt_get_rgb_index(device_type_t device) {
    return (device < DEVS_COUNT) ? bt_rgb_index_table[device] : 0;
}

// 数据同步辅助函数
static inline void bt_sync_device_info(bt_device_state_t *device) {
    bt_dev_info.devs      = device->current;
    bt_dev_info.last_devs = device->last_wireless;
    bt_dev_info.raw       = device->raw;
}

// 兼容性宏定义（保持向后兼容）
#define dev_info bt_dev_info
#define bts_info bt_bts_info
// 注释掉这行避免重复定义：// #define led_inited bt_led_inited
#define process_record_bt bt_process_record
#define open_rgb bt_open_rgb
#define led_off_standby bt_led_off_standby
