/**
 * @file bt_task.c
 * @brief Bluetooth Task Management - Complete Implementation
 * @author JoyLee (Refactored)
 * @version 3.3.0
 * @date 2025-06-13
 */

#include "bt_task.h"
#include "config.h"
#include "rgb_matrix.h"

// =============================================
// 常量表定义
// =============================================

const uint8_t bt_rgb_index_table[DEVS_COUNT] = {
    BT_USB_LED_INDEX,   // USB指示灯索引
    BT_HOST1_LED_INDEX, // HOST1指示灯索引
    BT_HOST2_LED_INDEX, // HOST2指示灯索引
    BT_HOST3_LED_INDEX, // HOST3指示灯索引
    BT_2_4G_LED_INDEX,  // 2.4G指示灯索引
};

const uint8_t bt_rgb_color_table[DEVS_COUNT][3] = {
    {100, 100, 100}, // USB - 白色
    {0, 0, 100},     // HOST1 - 蓝色
    {0, 0, 100},     // HOST2 - 蓝色
    {0, 0, 100},     // HOST3 - 蓝色
    {0, 100, 0},     // 2.4G - 绿色
};

const uint32_t bt_sleep_time_table[4] = {0, 10 * 60 * 1000, 30 * 60 * 1000, 60 * 60 * 1000};

const uint8_t bt_battery_query_index[10] = {14, 15, 16, 17, 18, 19, 20, 21, 22, 23};

// =============================================
// 全局变量定义（重命名避免冲突）
// =============================================

static bt_context_t g_bt_ctx = {0};

// 兼容性全局变量（重命名）
bt_dev_info_t bt_dev_info = {0};

bts_info_t bt_bts_info = {
    .bt_name        = {"Alchemie TKL_$", "Alchemie TKL_$", "Alchemie TKL_$"},
    .uart_init      = uart_init,
    .uart_read      = uart_read,
    .uart_transmit  = uart_transmit,
    .uart_receive   = uart_receive,
    .uart_available = uart_available,
    .timer_read32   = timer_read32,
};

extern keymap_config_t keymap_config;
extern bool            led_inited; // 使用 m90.c 中定义的变量

// =============================================
// 线程相关
// =============================================

static THD_WORKING_AREA(bt_waThread1, 128);
static THD_FUNCTION(bt_Thread1, arg) {
    (void)arg;
    chRegSetThreadName("bt_blinker");
    while (true) {
        bts_task(g_bt_ctx.device.current);
        chThdSleepMilliseconds(1);
    }
}

// =============================================
// 设备管理模块实现
// =============================================

void bt_device_init(bt_device_state_t *device) {
    BT_VALIDATE_PTR(device);

    device->raw = eeconfig_read_user();
    if (!device->raw) {
#ifdef BT_MODE_SW_PIN
        if (readPin(BT_MODE_SW_PIN)) {
            device->current       = DEVS_HOST1;
            device->last_wireless = DEVS_HOST1;
        } else {
            device->current       = DEVS_USB;
            device->last_wireless = DEVS_HOST1;
        }
#else
        device->current       = DEVS_USB;
        device->last_wireless = DEVS_HOST1;
#endif
        device->config.sleep_mode      = BT_SLEEP_10MIN;
        device->config.ind_brightness  = BT_DEFAULT_BRIGHTNESS;
        device->config.ind_color_index = 1;
        eeconfig_update_user(device->raw);
    }

    bt_sync_device_info(device);

    // 设置硬件引脚
    setPinOutput(A14);
    if (device->current == DEVS_USB) {
        writePinLow(A14);
    } else {
        writePinHigh(A14);
    }
}

void bt_device_switch(bt_device_state_t *device, device_type_t target, bool reset) {
    BT_VALIDATE_PTR(device);
    BT_VALIDATE_DEVICE_TYPE(target);

    device_type_t last_mode = device->current;

    // 更新无线设备记录
    if (device->current != DEVS_USB && device->current != DEVS_2_4G) {
        device->last_wireless = device->current;
    }

    device->current = target;

    // 硬件控制逻辑
    if (target == DEVS_USB) {
        writePinLow(A14);
        if (last_mode != DEVS_USB) {
            init_usb_driver(&USB_DRIVER);
        }
    } else {
        writePinHigh(A14);
        if (last_mode == DEVS_USB) {
            usbDisconnectBus(&USB_DRIVER);
            usbStop(&USB_DRIVER);
        }

        // 发送蓝牙命令 - 使用现有的命令，让蓝牙模块自动处理回连/配对
        switch (target) {
            case DEVS_HOST1:
                bts_send_vendor(v_host1);
                break;
            case DEVS_HOST2:
                bts_send_vendor(v_host2);
                break;
            case DEVS_HOST3:
                bts_send_vendor(v_host3);
                break;
            case DEVS_2_4G:
                bts_send_vendor(v_2_4g);
                break;
            default:
                BT_DEBUG_INFO("Unknown device type: %d\n", target);
                return;
        }
    }

    // 更新EEPROM
    eeconfig_update_user(device->raw);
    bt_sync_device_info(device);
}

device_type_t bt_device_get_current(const bt_device_state_t *device) {
    BT_VALIDATE_PTR_RET(device, DEVS_USB);
    return device->current;
}

bool bt_device_is_wireless(const bt_device_state_t *device) {
    BT_VALIDATE_PTR_RET(device, false);
    return device->current != DEVS_USB;
}

void bt_device_scan_hardware_switch(bt_device_state_t *device) {
    BT_VALIDATE_PTR(device);

#ifdef BT_MODE_SW_PIN
    static bool last_pin_status    = false;
    bool        current_pin_status = !readPin(BT_MODE_SW_PIN);

    if (last_pin_status != current_pin_status) {
        last_pin_status = current_pin_status;
        if (current_pin_status) {
            if (device->current != DEVS_USB) {
                bt_device_switch(device, DEVS_USB, false);
            }
        } else {
            bt_device_switch(device, device->last_wireless, false);
        }
    }
#endif
}

// =============================================
// 电源管理模块实现
// =============================================

void bt_power_init(bt_power_state_t *power) {
    BT_VALIDATE_PTR(power);
    memset(power, 0, sizeof(bt_power_state_t));
    power->battery_level = 100;
}

void bt_power_update(bt_power_state_t *power) {
    BT_VALIDATE_PTR(power);

#if defined(BT_CABLE_PIN) && defined(BT_CHARGE_PIN)
    static uint32_t last_query_time     = 0;
    static bool     last_paired_status  = false;
    static bool     low_vol_first_entry = false;
    uint32_t        current_time        = timer_read32();

    // 检测充电状态
    power->charging        = !readPin(BT_CHARGE_PIN);
    power->charge_complete = readPin(BT_CABLE_PIN) && !power->charging;

    // 定期查询电池电量
    if (timer_elapsed32(last_query_time) > BT_BATTERY_QUERY_INTERVAL) {
        last_query_time = current_time;
        bts_send_vendor(v_query_vol);
    }

    // 检测连接状态变化（改进的低电量提醒逻辑）
    if (last_paired_status != bt_bts_info.bt_info.paired) {
        last_paired_status = bt_bts_info.bt_info.paired;
        if (bt_bts_info.bt_info.paired && bt_bts_info.bt_info.pvol <= BT_BATTERY_LOW_THRESHOLD) {
            low_vol_first_entry = false;
        }
    }

    // 低电量处理
    if (bt_bts_info.bt_info.pvol <= BT_BATTERY_LOW_THRESHOLD) {
        if (!low_vol_first_entry) {
            power->low_power_blink_time = current_time;
            low_vol_first_entry         = true;
            power->low_battery          = true;
            BT_DEBUG_INFO("Low battery warning: %d%%\n", bt_bts_info.bt_info.pvol);
        }

        if (timer_elapsed32(power->low_power_blink_time) > 1000) {
            power->low_power_blink_time = current_time;
        }
    } else {
        power->low_battery  = false;
        low_vol_first_entry = false;
    }

    // 充电完成闪烁
    if (power->charge_complete && timer_elapsed32(power->charge_full_blink_time) > 500) {
        power->charge_full_blink_time = current_time;
    }
#endif
}

void bt_power_set_battery_level(bt_power_state_t *power, uint8_t level) {
    BT_VALIDATE_PTR(power);

    power->battery_level = level;

    if (level < BT_BATTERY_CRITICAL_THRESHOLD && !power->first_critical_warning) {
        power->critical_battery       = true;
        power->first_critical_warning = true;
        BT_DEBUG_INFO("Critical battery: %d%%\n", level);
    } else if (level >= BT_BATTERY_LOW_THRESHOLD) {
        power->critical_battery       = false;
        power->low_battery            = false;
        power->first_critical_warning = false;
        power->first_low_warning      = false;
    }
}

bool bt_power_should_sleep(const bt_power_state_t *power, uint32_t last_activity, uint8_t sleep_mode) {
    BT_VALIDATE_PTR_RET(power, false);

    if (sleep_mode == 0 || sleep_mode >= sizeof(bt_sleep_time_table) / sizeof(bt_sleep_time_table[0])) return false;

    uint32_t sleep_timeout = bt_sleep_time_table[sleep_mode];
    return timer_elapsed32(last_activity) > sleep_timeout;
}

void bt_power_render_indicators(const bt_power_state_t *power) {
    BT_VALIDATE_PTR(power);

    // 低电量指示
    if (power->low_battery) {
        static bool     blink_state = false;
        static uint32_t last_blink  = 0;

        if (timer_elapsed32(last_blink) > 500) {
            last_blink  = timer_read32();
            blink_state = !blink_state;

            if (blink_state) {
                for (int i = 0; i < DEVS_COUNT; i++) {
                    rgb_matrix_set_color(bt_rgb_index_table[i], 100, 0, 0);
                }
            }
        }
    }

    // 充电指示
    if (power->charging) {
        static uint32_t charge_blink = 0;
        if (timer_elapsed32(charge_blink) > 1000) {
            charge_blink = timer_read32();
            rgb_matrix_set_color(BT_USB_LED_INDEX, 0, 100, 0);
        }
    }
}

// =============================================
// LED指示器模块实现
// =============================================

void bt_indicator_init(bt_indicator_state_t *indicator) {
    BT_VALIDATE_PTR(indicator);
    memset(indicator, 0, sizeof(bt_indicator_state_t));
    indicator->status = BT_INDICATOR_CONNECTED;
}

void bt_indicator_set_status(bt_indicator_state_t *indicator, bt_indicator_status_t status) {
    BT_VALIDATE_PTR(indicator);

    indicator->status            = status;
    indicator->status_start_time = timer_read32();

    switch (status) {
        case BT_INDICATOR_CONNECTING:
            // 回连模式 - 慢闪
            indicator->connecting_mode = true;
            indicator->reset_requested = false;
            indicator->blink_interval  = 500; // 1秒间隔慢闪
            BT_DEBUG_INFO("Entering connecting mode (slow blink)\n");
            break;

        case BT_INDICATOR_PAIRING:
            // 配对模式 - 快闪
            indicator->reset_requested = true;
            indicator->connecting_mode = false;
            indicator->blink_interval  = 200; // 200ms间隔快闪
            BT_DEBUG_INFO("Entering pairing mode (fast blink)\n");
            break;

        case BT_INDICATOR_CONNECTED:
            // 已连接 - 常亮
            indicator->reset_requested = false;
            indicator->connecting_mode = false;
            indicator->blink_interval  = 0;
            BT_DEBUG_INFO("Connected - solid light\n");
            break;

        default:
            indicator->reset_requested = false;
            indicator->connecting_mode = false;
            indicator->blink_interval  = 0;
            break;
    }
}

void bt_indicator_update(bt_indicator_state_t *indicator, const bt_device_state_t *device) {
    BT_VALIDATE_PTR(indicator);
    BT_VALIDATE_PTR(device);

    uint32_t current_time = timer_read32();

    // 检查连接状态变化（只对无线设备）
    static bool last_paired_status = false;
    bool        current_paired     = bt_bts_info.bt_info.paired;

    // 只有无线设备才检查连接状态
    if (bt_device_is_wireless(device)) {
        if (last_paired_status != current_paired) {
            last_paired_status = current_paired;
            if (current_paired && (indicator->status == BT_INDICATOR_CONNECTING || indicator->status == BT_INDICATOR_PAIRING)) {
                // 无线设备连接成功
                bt_indicator_set_status(indicator, BT_INDICATOR_CONNECTED);
                indicator->connected_time = current_time;
                indicator->timeout_sleep  = false;
                BT_DEBUG_INFO("Wireless device connected successfully\n");
            }
        }
    }

    // 处理各种状态的逻辑
    switch (indicator->status) {
        case BT_INDICATOR_CONNECTING:
        case BT_INDICATOR_PAIRING:
            // 检查60秒超时（只对无线设备）
            if (bt_device_is_wireless(device) && timer_elapsed32(indicator->status_start_time) > 60000) {
                indicator->timeout_sleep = true;
                g_bt_ctx.kb_sleep_flag   = true;
                // 超时后设置为OFF状态
                bt_indicator_set_status(indicator, BT_INDICATOR_OFF);
                BT_DEBUG_INFO("Connection timeout (60s), turning off indicator\n");
            } else if (indicator->blink_interval > 0) {
                // 处理闪烁
                if (timer_elapsed32(indicator->blink_time) > indicator->blink_interval) {
                    indicator->blink_time = current_time;
                    indicator->flip_state = !indicator->flip_state;
                }
            }
            break;

        case BT_INDICATOR_CONNECTED:
            // 连接成功后的超时处理
            if (indicator->connected_time != 0) {
                uint32_t timeout = 3000; // 默认3秒

#ifdef BT_USB_INDICATOR_TIME
                // USB模式使用不同的超时时间
                if (device->current == DEVS_USB) {
                    timeout = BT_USB_INDICATOR_TIME;
                }
#endif

                if (timer_elapsed32(indicator->connected_time) > timeout) {
                    indicator->connected_time = 0;
                    // 3秒后设置为OFF状态
                    bt_indicator_set_status(indicator, BT_INDICATOR_OFF);
                    BT_DEBUG_INFO("Connected indicator timeout, turning off\n");
                }
            }
            break;

        default:
            break;
    }

    // 根据设备类型设置颜色
    if (device->current < DEVS_COUNT) {
        indicator->current_color.r = bt_rgb_color_table[device->current][0];
        indicator->current_color.g = bt_rgb_color_table[device->current][1];
        indicator->current_color.b = bt_rgb_color_table[device->current][2];
    }
}

void bt_indicator_render(const bt_indicator_state_t *indicator, uint8_t led_min, uint8_t led_max) {
    BT_VALIDATE_PTR(indicator);

    if (!rgb_matrix_config.enable) return;

    // 定义系统指示灯索引，避免冲突
    const uint8_t lock_leds[] = {84, 85, 86}; // NUM, CAPS, SCROLL

    // 获取当前设备
    device_type_t current_device = bt_device_get_current(&g_bt_ctx.device);

    // 添加调试信息
    static device_type_t last_debug_device = 0xFF;
    if (last_debug_device != current_device) {
        last_debug_device = current_device;
        BT_DEBUG_INFO("Current device: %d, LED index: %d, Status: %d\n", current_device, (current_device < DEVS_COUNT) ? bt_rgb_index_table[current_device] : 0xFF, indicator->status);
    }

    // 检查设备有效性
    if (current_device >= DEVS_COUNT) {
        BT_DEBUG_INFO("Invalid device type: %d\n", current_device);
        return;
    }

    uint8_t current_led_index = bt_rgb_index_table[current_device];

    // 检查LED索引有效性
    if (current_led_index < led_min || current_led_index >= led_max) {
        BT_DEBUG_INFO("LED index %d out of range [%d, %d)\n", current_led_index, led_min, led_max);
        return;
    }

    // 检查是否是系统指示灯
    bool is_lock_led = false;
    for (uint8_t j = 0; j < 3; j++) {
        if (current_led_index == lock_leds[j]) {
            is_lock_led = true;
            BT_DEBUG_INFO("Device LED conflicts with lock LED %d\n", current_led_index);
            break;
        }
    }

    if (is_lock_led) {
        return; // 不处理与系统指示灯冲突的LED
    }

    // USB设备指示灯开关检查
#ifndef BT_ENABLE_USB_INDICATOR
    if (current_device == DEVS_USB) {
        BT_DEBUG_INFO("USB indicator disabled\n");
        return;
    }
#endif

    // 根据状态处理LED
    switch (indicator->status) {
        case BT_INDICATOR_CONNECTING:
        case BT_INDICATOR_PAIRING:
            // 闪烁模式
            if (!indicator->flip_state) {
                // 亮状态：显示设备指示灯颜色
                rgb_matrix_set_color(current_led_index, indicator->current_color.r, indicator->current_color.g, indicator->current_color.b);
                BT_DEBUG_INFO("Device LED %d ON: R%d G%d B%d\n", current_led_index, indicator->current_color.r, indicator->current_color.g, indicator->current_color.b);
            } else {
                // 灭状态：强制设置为黑色
                rgb_matrix_set_color(current_led_index, 0, 0, 0);
                BT_DEBUG_INFO("Device LED %d OFF\n", current_led_index);
            }
            break;

        case BT_INDICATOR_CONNECTED:
            // 连接成功后常亮
            if (indicator->connected_time != 0) {
                rgb_matrix_set_color(current_led_index, indicator->current_color.r, indicator->current_color.g, indicator->current_color.b);
                BT_DEBUG_INFO("Device LED %d CONNECTED: R%d G%d B%d\n", current_led_index, indicator->current_color.r, indicator->current_color.g, indicator->current_color.b);
            } else {
                BT_DEBUG_INFO("Connected indicator timeout, LED %d back to normal\n", current_led_index);
            }
            break;

        case BT_INDICATOR_OFF:
        default:
            // 不做任何操作，保持正常灯效
            BT_DEBUG_INFO("Indicator OFF, LED %d back to normal\n", current_led_index);
            break;
    }
}
void bt_rgb_init(bt_rgb_state_t *rgb) {
    BT_VALIDATE_PTR(rgb);
    rgb->rgb_status_save = rgb_matrix_config.enable;
    rgb->sober           = true;
    rgb->key_press_time  = timer_read32();
}

void bt_rgb_open(bt_rgb_state_t *rgb) {
    BT_VALIDATE_PTR(rgb);

    if (!rgb->sober) {
#ifdef WS2812_EN_PIN
        writePinLow(WS2812_EN_PIN);
#endif
        rgb->sober = true;
        if (rgb->rgb_status_save) {
            rgb_matrix_enable_noeeprom();
        }
    }
    rgb->key_press_time = timer_read32();
}

void bt_rgb_close(bt_rgb_state_t *rgb, uint32_t current_time) {
    BT_VALIDATE_PTR(rgb);

    bt_context_t *ctx = &g_bt_ctx;

    // 检查超时休眠条件
    bool should_sleep = false;

    // 1. 正常超时休眠
    if (timer_elapsed32(rgb->key_press_time) > BT_LED_STANDBY_TIMEOUT) {
        should_sleep = true;
    }

    // 2. 连接超时休眠
    if (ctx->indicator.timeout_sleep) {
        should_sleep                 = true;
        ctx->indicator.timeout_sleep = false; // 重置标志
    }

    // 3. 系统休眠标志
    if (ctx->kb_sleep_flag) {
        should_sleep       = true;
        ctx->kb_sleep_flag = false; // 重置标志
    }

    if (should_sleep && rgb->sober) {
        rgb->bak_rgb_toggle = rgb_matrix_config.enable;
        rgb->sober          = false;
        rgb_matrix_disable_noeeprom();

#ifdef WS2812_EN_PIN
        writePinHigh(WS2812_EN_PIN);
#endif

#ifdef ENTRY_STOP_MODE
        lp_system_sleep();
#endif

        BT_DEBUG_INFO("System entering sleep mode\n");
    }
}

void bt_rgb_set_all_blink(bt_rgb_state_t *rgb, uint8_t count, RGB color) {
    BT_VALIDATE_PTR(rgb);
    rgb->all_blink_cnt   = count;
    rgb->all_blink_color = color;
    rgb->all_blink_time  = timer_read32();
}

void bt_rgb_set_single_blink(bt_rgb_state_t *rgb, uint8_t index, uint8_t count, RGB color) {
    BT_VALIDATE_PTR(rgb);
    rgb->single_blink_index = index;
    rgb->single_blink_cnt   = count;
    rgb->single_blink_color = color;
    rgb->single_blink_time  = timer_read32();
}

// 在长按回调函数之前添加以下实现：

// =============================================
// 长按键处理模块实现
// =============================================

void bt_longpress_init(bt_longpress_state_t *longpress) {
    BT_VALIDATE_PTR(longpress);
    memset(longpress, 0, sizeof(bt_longpress_state_t));
}

void bt_longpress_register(bt_longpress_state_t *longpress, uint16_t keycode, uint32_t threshold, void (*callback)(uint16_t)) {
    BT_VALIDATE_PTR(longpress);

    if (longpress->key_count >= BT_NUM_LONG_PRESS_KEYS) {
        BT_DEBUG_INFO("Cannot register more longpress keys\n");
        return;
    }

    bt_longpress_key_t *key = &longpress->keys[longpress->key_count++];
    key->keycode            = keycode;
    key->threshold_ms       = threshold;
    key->callback           = callback;
    key->press_time         = 0;

    BT_DEBUG_INFO("Registered longpress key: 0x%x, threshold: %dms\n", keycode, threshold);
}

void bt_longpress_on_press(bt_longpress_state_t *longpress, uint16_t keycode) {
    BT_VALIDATE_PTR(longpress);

    // 普通长按键处理
    for (uint8_t i = 0; i < longpress->key_count; i++) {
        if (longpress->keys[i].keycode == keycode) {
            longpress->keys[i].press_time = timer_read32();
            BT_DEBUG_INFO("Longpress key pressed: 0x%x\n", keycode);
            break;
        }
    }

    // 特殊处理工厂重置
    if (keycode == FACTORY_RESET) {
        if (longpress->factory_reset_status == 0) {
            longpress->factory_reset_press_time = timer_read32();
            longpress->factory_reset_status     = 1;
            BT_DEBUG_INFO("Factory reset key pressed\n");
        }
    }
}

void bt_longpress_on_release(bt_longpress_state_t *longpress, uint16_t keycode) {
    BT_VALIDATE_PTR(longpress);

    // 普通长按键处理
    for (uint8_t i = 0; i < longpress->key_count; i++) {
        if (longpress->keys[i].keycode == keycode) {
            longpress->keys[i].press_time = 0;
            BT_DEBUG_INFO("Longpress key released: 0x%x\n", keycode);
            break;
        }
    }

    // 工厂重置释放处理
    if (keycode == FACTORY_RESET) {
        if (longpress->factory_reset_status == 1) {
            if (timer_elapsed32(longpress->factory_reset_press_time) >= 3000) {
                longpress->factory_reset_press_cnt++;
                BT_DEBUG_INFO("Factory reset long press count: %d\n", longpress->factory_reset_press_cnt);

                if (longpress->factory_reset_press_cnt >= 3) {
                    eeconfig_init();
                    longpress->factory_reset_press_cnt = 0;
                    BT_DEBUG_INFO("Factory reset executed\n");
                }
            }
            longpress->factory_reset_status = 0;
        }
    }
}

void bt_longpress_update(bt_longpress_state_t *longpress) {
    BT_VALIDATE_PTR(longpress);

    // uint32_t current_time = timer_read32();

    for (uint8_t i = 0; i < longpress->key_count; i++) {
        bt_longpress_key_t *key = &longpress->keys[i];

        if (key->press_time && timer_elapsed32(key->press_time) >= key->threshold_ms) {
            BT_DEBUG_INFO("Longpress triggered for key: 0x%x\n", key->keycode);

            if (key->callback) {
                key->callback(key->keycode);
            }

            // 防止重复触发
            key->press_time = 0;
        }
    }

    // 处理工厂重置长按检测
    if (longpress->factory_reset_status == 1) {
        if (timer_elapsed32(longpress->factory_reset_press_time) >= 3000) {
            // 工厂重置长按时间达到，等待释放
            BT_DEBUG_INFO("Factory reset long press detected\n");
        }
    }
}

// =============================================
// 长按回调函数
// =============================================

static void bt_longpress_device_callback(uint16_t keycode) {
    bt_context_t *ctx     = &g_bt_ctx;
    device_type_t current = ctx->device.current;

    device_type_t target = DEVS_USB;
    switch (keycode) {
        case BT_HOST1:
            target = DEVS_HOST1;
            break;
        case BT_HOST2:
            target = DEVS_HOST2;
            break;
        case BT_HOST3:
            target = DEVS_HOST3;
            break;
        case BT_2_4G:
            target = DEVS_2_4G;
            break;
        default:
            BT_DEBUG_INFO("Unknown longpress keycode: 0x%x\n", keycode);
            return;
    }

    if (current == target) {
        // 长按相同设备键 - 快闪表示配对模式
        BT_DEBUG_INFO("Long press: Starting pairing mode for device %d\n", target);
        bt_device_switch(&ctx->device, target, false);                  // 发送同样的命令，让蓝牙模块处理
        bt_indicator_set_status(&ctx->indicator, BT_INDICATOR_PAIRING); // 快闪指示
    }
}

static void bt_longpress_reset_callback(uint16_t keycode) {
    switch (keycode) {
        case KEYBOARD_RESET:
            BT_DEBUG_INFO("Keyboard reset triggered\n");
            break;
        case BLE_RESET:
            if (bt_device_is_wireless(&g_bt_ctx.device)) {
                bts_send_vendor(v_clear);
                BT_DEBUG_INFO("BLE reset triggered\n");
            }
            break;
        default:
            BT_DEBUG_INFO("Unknown reset keycode: 0x%x\n", keycode);
            break;
    }
}

// =============================================
// 主要接口实现
// =============================================

void bt_init(void) {
    bt_context_t *ctx = &g_bt_ctx;

    memset(ctx, 0, sizeof(bt_context_t));

    // 初始化蓝牙库
    bts_init(&bt_bts_info);

    // 初始化硬件引脚
#ifdef BT_MODE_SW_PIN
    setPinInputHigh(BT_MODE_SW_PIN);
#endif
#if defined(BT_CABLE_PIN) && defined(BT_CHARGE_PIN)
    setPinInputHigh(BT_CABLE_PIN);
    setPinInput(BT_CHARGE_PIN);
#endif

    // 初始化各个模块
    bt_device_init(&ctx->device);
    bt_indicator_init(&ctx->indicator);
    bt_power_init(&ctx->power);
    bt_longpress_init(&ctx->longpress);
    bt_rgb_init(&ctx->rgb);

    // 注册长按回调
    bt_longpress_register(&ctx->longpress, BT_HOST1, 3000, bt_longpress_device_callback);
    bt_longpress_register(&ctx->longpress, BT_HOST2, 3000, bt_longpress_device_callback);
    bt_longpress_register(&ctx->longpress, BT_HOST3, 3000, bt_longpress_device_callback);
    bt_longpress_register(&ctx->longpress, BT_2_4G, 3000, bt_longpress_device_callback);
    bt_longpress_register(&ctx->longpress, FACTORY_RESET, 3000, NULL);
    bt_longpress_register(&ctx->longpress, KEYBOARD_RESET, 3000, bt_longpress_reset_callback);
    bt_longpress_register(&ctx->longpress, BLE_RESET, 3000, bt_longpress_reset_callback);

    // 启动蓝牙任务线程
    chThdCreateStatic(bt_waThread1, sizeof(bt_waThread1), HIGHPRIO, bt_Thread1, NULL);

    // USB处理
    if (ctx->device.current != DEVS_USB) {
        usbDisconnectBus(&USB_DRIVER);
        usbStop(&USB_DRIVER);
    }

    ctx->init_time   = timer_read32();
    ctx->initialized = true;

    BT_DEBUG_INFO("BT Task initialized, device: %d\n", ctx->device.current);
}

void bt_task(void) {
    bt_context_t *ctx = &g_bt_ctx;
    if (!ctx->initialized) return;

    static uint32_t last_time    = 0;
    uint32_t        current_time = timer_read32();

    // 初始化延迟处理
    if (ctx->init_time && timer_elapsed32(ctx->init_time) >= 2000) {
        ctx->init_time = 0;
        bts_send_name(DEVS_HOST1);

        // 发送当前设备模式命令
        vbs_t cmd;
        switch (ctx->device.current) {
            case DEVS_HOST1:
                cmd = v_host1;
                break;
            case DEVS_HOST2:
                cmd = v_host2;
                break;
            case DEVS_HOST3:
                cmd = v_host3;
                break;
            case DEVS_2_4G:
                cmd = v_2_4g;
                break;
            case DEVS_USB:
                cmd = v_usb;
                break;
            default:
                BT_DEBUG_INFO("Unknown device type: %d\n", ctx->device.current);
                cmd = v_usb;
                break;
        }
        bts_send_vendor(cmd);
        bts_send_vendor(v_en_sleep_bt);
        bts_send_vendor(v_en_sleep_wl);
    }

    // 每1ms执行一次的逻辑
    if (timer_elapsed32(last_time) >= BT_SLEEP_CHECK_INTERVAL) {
        last_time = current_time;

        // 更新LED指示状态
        if (ctx->device.current != DEVS_USB) {
            uint8_t keyboard_led_state = 0;
            led_t  *kb_leds            = (led_t *)&keyboard_led_state;
            kb_leds->raw               = bt_bts_info.bt_info.indictor_rgb_s;
            usb_device_state_set_leds(keyboard_led_state);

#ifdef RGB_MATRIX_ENABLE
            bt_rgb_close(&ctx->rgb, current_time);
#endif
        }

        // 更新各个模块
        bt_longpress_update(&ctx->longpress);
        bt_power_update(&ctx->power);
        bt_indicator_update(&ctx->indicator, &ctx->device);
        bt_device_scan_hardware_switch(&ctx->device);

        // 检查睡眠条件
        if (bt_power_should_sleep(&ctx->power, ctx->last_activity, ctx->device.config.sleep_mode)) {
            ctx->kb_sleep_flag = true;
        }
    }
}

bool bt_process_record(uint16_t keycode, keyrecord_t *record) {
    bt_context_t *ctx = &g_bt_ctx;
    if (!ctx->initialized) return true;

    // 调试和活动记录
    if (record->event.pressed) {
        BT_DEBUG_INFO("keycode = [0x%x], time: [%d]\n", keycode, record->event.time);
        ctx->last_activity = timer_read32();

        // RGB管理
        if (!rgb_matrix_config.enable && ctx->rgb.rgb_status_save) {
            rgb_matrix_enable_noeeprom();
        }
    }

    // 处理长按
    if (record->event.pressed) {
        bt_longpress_on_press(&ctx->longpress, keycode);
    } else {
        bt_longpress_on_release(&ctx->longpress, keycode);
    }

    // 硬件开关检查
#ifdef BT_MODE_SW_PIN
    if (!readPin(BT_MODE_SW_PIN) && bt_is_switch_key(keycode)) {
        return false;
    }
#endif

    bool retval = true;

    // 处理按键逻辑 - 移除USB切换闪烁
    if (record->event.pressed) {
        switch (keycode) {
            case BT_HOST1:
                if (ctx->device.current != DEVS_HOST1) {
                    bt_device_switch(&ctx->device, DEVS_HOST1, false);
                    bt_indicator_set_status(&ctx->indicator, BT_INDICATOR_CONNECTING);
                    BT_DEBUG_INFO("Short press: Switching to HOST1 (connecting mode - slow blink)\n");
                }
                retval = false;
                break;
            case BT_HOST2:
                if (ctx->device.current != DEVS_HOST2) {
                    bt_device_switch(&ctx->device, DEVS_HOST2, false);
                    bt_indicator_set_status(&ctx->indicator, BT_INDICATOR_CONNECTING);
                    BT_DEBUG_INFO("Short press: Switching to HOST2 (connecting mode - slow blink)\n");
                }
                retval = false;
                break;
            case BT_HOST3:
                if (ctx->device.current != DEVS_HOST3) {
                    bt_device_switch(&ctx->device, DEVS_HOST3, false);
                    bt_indicator_set_status(&ctx->indicator, BT_INDICATOR_CONNECTING);
                    BT_DEBUG_INFO("Short press: Switching to HOST3 (connecting mode - slow blink)\n");
                }
                retval = false;
                break;
            case BT_2_4G:
                if (ctx->device.current != DEVS_2_4G) {
                    bt_device_switch(&ctx->device, DEVS_2_4G, false);
                    bt_indicator_set_status(&ctx->indicator, BT_INDICATOR_CONNECTING);
                    BT_DEBUG_INFO("Short press: Switching to 2.4G (connecting mode - slow blink)\n");
                }
                retval = false;
                break;
            case BT_USB:
                if (ctx->device.current != DEVS_USB) {
                    bt_device_switch(&ctx->device, DEVS_USB, false);

#ifdef BT_ENABLE_USB_INDICATOR
                    // USB也显示连接指示（白色常亮3秒）
                    bt_indicator_set_status(&ctx->indicator, BT_INDICATOR_CONNECTED);
                    BT_DEBUG_INFO("Switching to USB mode (with white indicator)\n");
#else
                    // USB不显示指示
                    bt_indicator_set_status(&ctx->indicator, BT_INDICATOR_OFF);
                    BT_DEBUG_INFO("Switching to USB mode (no indicator)\n");
#endif
                }
            case BT_VOL:
                // 电量查询逻辑
                if (!readPin(BT_CABLE_PIN)) {
                    bts_send_vendor(v_query_vol);
                    ctx->query_vol_flag = true;
                }
                retval = false;
                break;
        }
    } else {
        if (keycode == BT_VOL) {
            ctx->query_vol_flag = false;
        }
    }

    // 传递给蓝牙库处理
    if (bt_device_is_wireless(&ctx->device) && retval) {
        retval = bts_process_keys(keycode, record->event.pressed, ctx->device.current, keymap_config.no_gui);
    }

#ifdef RGB_MATRIX_ENABLE
    bt_rgb_open(&ctx->rgb);
#endif

    return retval;
}

uint8_t bt_indicator_rgb(uint8_t led_min, uint8_t led_max) {
    bt_context_t *ctx = &g_bt_ctx;
    if (!ctx->initialized) return 0;

    // 渲染各种指示器
    bt_indicator_render(&ctx->indicator, led_min, led_max);
    bt_power_render_indicators(&ctx->power);

    // 处理电池电量查询显示
    if (ctx->query_vol_flag) {
        uint8_t bat_level = bt_bts_info.bt_info.pvol;
        if (bat_level <= 100) {
            uint8_t bat_index = bat_level / 10;
            if (bat_index > BT_MAX_BATTERY_INDEX) bat_index = BT_MAX_BATTERY_INDEX;

            for (int i = 0; i <= bat_index; i++) {
                if (bt_battery_query_index[i] < led_max) {
                    if (bat_level > 50) {
                        rgb_matrix_set_color(bt_battery_query_index[i], 0, 100, 0);
                    } else if (bat_level > BT_BATTERY_LOW_THRESHOLD) {
                        rgb_matrix_set_color(bt_battery_query_index[i], 100, 100, 0);
                    } else {
                        rgb_matrix_set_color(bt_battery_query_index[i], 100, 0, 0);
                    }
                }
            }
        }
    }

    // 移除USB模式切换闪烁效果
    // if (ctx->rgb.usb_blink_cnt > 0) {
    //     static uint32_t last_blink = 0;
    //     if (timer_elapsed32(last_blink) > 200) {
    //         last_blink = timer_read32();
    //         ctx->rgb.usb_blink_cnt--;
    //
    //         if (ctx->rgb.usb_blink_cnt % 2) {
    //             for (int i = 0; i < DEVS_COUNT; i++) {
    //                 rgb_matrix_set_color(bt_rgb_index_table[i], 100, 100, 100);
    //             }
    //         }
    //     }
    // }

    return 1;
}

// =============================================
// 兼容性接口实现
// =============================================

void bt_bat_low_indicator(void) {
    bt_power_set_battery_level(&g_bt_ctx.power, 10);
}

void bt_switch_mode(uint8_t last_mode, uint8_t now_mode, uint8_t reset) {
    if (now_mode < DEVS_COUNT) {
        bt_device_switch(&g_bt_ctx.device, (device_type_t)now_mode, !!reset);
    } else {
        BT_DEBUG_INFO("Invalid mode: %d\n", now_mode);
    }
}

void bt_open_rgb(void) {
    bt_rgb_open(&g_bt_ctx.rgb);
}

void bt_led_off_standby(void) {
    bt_rgb_close(&g_bt_ctx.rgb, timer_read32());
}
