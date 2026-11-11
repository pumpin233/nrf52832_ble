// NUS蓝牙透传：发送缓冲、接收回调和LED命令。
#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "ble_nus.h"
#include "nrf_sdh_ble.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "ble_nus_transport.h"

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

// 电脑通过NUS的RX特征写入，开发板通过TX特征通知电脑。
// 等待发送的数据先保存，蓝牙发送队列满时等TX_RDY事件再继续。
#define BLE_PC_QUEUE_SIZE 1024
static uint8_t m_pc_queue[BLE_PC_QUEUE_SIZE];
static uint16_t m_pc_read = 0;
static uint16_t m_pc_write = 0;
static bool m_pc_notify_enabled = false;

//将缓冲区里排队的数据，分段发送给电脑
static void ble_pc_flush(void)
{
    while (m_pc_notify_enabled && m_pc_read != m_pc_write)
    {
        uint8_t packet[BLE_GATT_ATT_MTU_DEFAULT - 3];
        uint16_t length = 0;
        uint16_t cursor = m_pc_read;
        while (cursor != m_pc_write && length < sizeof(packet))
        {
            packet[length++] = m_pc_queue[cursor];
            cursor = (cursor + 1) % BLE_PC_QUEUE_SIZE;
        }
        uint32_t err = ble_nus_data_send(&m_nus, packet, &length, m_conn_handle);
        if (err == NRF_ERROR_RESOURCES)
            return; // 暂时发不出去，保留数据，不能在这里死循环等待。
        if (err == NRF_ERROR_INVALID_STATE || err == NRF_ERROR_NOT_FOUND)
            return;
        APP_ERROR_CHECK(err);
        if (length == 0)
            return;
        m_pc_read = (m_pc_read + length) % BLE_PC_QUEUE_SIZE;
    }
}

// 在蓝牙事件或按键回调中调用此函数，给电脑发送原始字节。
// 返回false表示尚未开启通知，或者缓冲区已满，本次数据未入队。
bool ble_pc_send(uint8_t const * data, uint16_t length)
{
    if (!m_pc_notify_enabled || m_conn_handle == BLE_CONN_HANDLE_INVALID)
        return false;
    uint16_t free_space = (m_pc_read + BLE_PC_QUEUE_SIZE - m_pc_write - 1)
                         % BLE_PC_QUEUE_SIZE;
    if (length > free_space)
    {
        NRF_LOG_WARNING("BLE PC buffer full: dropped %u bytes", length);
        return false;
    }
    for (uint16_t i = 0; i < length; i++)
    {
        m_pc_queue[m_pc_write] = data[i];
        m_pc_write = (m_pc_write + 1) % BLE_PC_QUEUE_SIZE;
    }
    ble_pc_flush();
    return true;
}

// 电脑先打开TX通知；写入RX特征的数据由下方回调原样回传。
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case BLE_NUS_EVT_COMM_STARTED:
            m_conn_handle = p_evt->conn_handle;
            m_pc_notify_enabled = true;
            NRF_LOG_INFO("PC notifications enabled");
            ble_pc_flush();
            break;
        case BLE_NUS_EVT_COMM_STOPPED:
            m_pc_notify_enabled = false;
            m_pc_read = m_pc_write = 0;
            break;
        case BLE_NUS_EVT_RX_DATA:
            NRF_LOG_INFO("BLE PC received %u bytes", p_evt->params.rx_data.length);
            app_ble_rx_command(p_evt->params.rx_data.p_data,
                               p_evt->params.rx_data.length);
            (void)ble_pc_send(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
            break;
        case BLE_NUS_EVT_TX_RDY:
            ble_pc_flush();
            break;
        default:
            break;
    }
}


// 处理电脑通过蓝牙发来的LED命令。
void app_ble_rx_command(uint8_t const * data, uint16_t length)
{
    if (length == 4 && data[0] == 'l' && data[1] == 'e' && data[2] == 'd')
    {
        switch (data[3])
        {
            case '1':
                //nrf_gpio_pin_toggle(LED_1);
                break;
            case '2':
                nrf_gpio_pin_toggle(LED_2);
                break;
            case '3':
                nrf_gpio_pin_toggle(LED_3);
                break;
            case '4':
                nrf_gpio_pin_toggle(LED_4);
                break;
            default:
                break;
        }
    }
    else if(length == 3 && data[0] == 't' && data[1] == 'o' && data[2] == 'g')
    {
        nrf_gpio_pin_toggle(LED_2);
        nrf_gpio_pin_toggle(LED_3);
        nrf_gpio_pin_toggle(LED_4);
    }
}


void ble_pc_data_init(void)
{
    ble_nus_init_t nus_init = {0};
    nus_init.data_handler = nus_data_handler;
    ret_code_t err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

void ble_pc_data_on_connected(uint16_t conn_handle)
{
    m_conn_handle = conn_handle;
}

void ble_pc_data_on_disconnected(void)
{
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    m_pc_notify_enabled = false;
    m_pc_read = m_pc_write = 0;
}
