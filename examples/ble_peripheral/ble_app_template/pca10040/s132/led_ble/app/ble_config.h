#ifndef BLE_CONFIG_H
#define BLE_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp.h"

// 初始化协议栈、连接参数、广播和配对模块，并注册透传服务。
// 调用前先初始化日志、app_timer和板上按键。
void ble_pc_init(void);

// 开始广播；参数为true时先清除保存的绑定信息。
void ble_pc_start(bool erase_bonds);

// 处理BSP中的休眠、断开连接、关闭白名单事件。
void ble_pc_on_bsp_event(bsp_event_t event);

#endif
