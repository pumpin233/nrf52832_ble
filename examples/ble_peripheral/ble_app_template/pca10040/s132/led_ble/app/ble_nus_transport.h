#ifndef BLE_NUS_TRANSPORT_H
#define BLE_NUS_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

// 在蓝牙配置初始化时注册NUS透传服务。
void ble_pc_data_init(void);

// 蓝牙连接状态由配置模块通知透传模块。
void ble_pc_data_on_connected(uint16_t conn_handle);
void ble_pc_data_on_disconnected(void);

// 向电脑发送length个原始字节，不自动添加回车或结束符。
// 电脑需已连接并开启通知；缓冲区满时返回false。
bool ble_pc_send(uint8_t const * data, uint16_t length);

// 收到完整pum1到pum4命令时，翻转对应LED。
void app_ble_rx_command(uint8_t const * data, uint16_t length);

#endif
