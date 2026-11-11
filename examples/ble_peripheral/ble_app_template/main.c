/** @file
 * 主程序：保留定时器、按键LED操作、电源管理和主循环。
 * 蓝牙配置位于app/ble_config.c，NUS收发和LED命令位于app/ble_nus_transport.c。
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_uart.h"
#include "nrf_uart.h"
#include "ble_config.h"
#include "ble_nus_transport.h"
#include "mpu6050.h"

#include "nrf_drv_saadc.h"

//---------------------saadc变量------------------------
//采样电压值
volatile nrf_saadc_value_t adc_value;  
//电压值
volatile float voltage_value; 
static volatile bool saadc_sample_due;
volatile uint32_t uart_error_code;

//---------------------定时器变量------------------------
//创建定时器1
APP_TIMER_DEF(m_app_timer_id1);
uint8_t TIME1=0;
#define TIME_LEVEL_MEAS_INTERVAL1       APP_TIMER_TICKS(100)

//创建定时器2
APP_TIMER_DEF(m_app_timer_id2);
uint8_t TIME2=0;
#define TIME_LEVEL_MEAS_INTERVAL2       APP_TIMER_TICKS(2000)

//---------------------mpu6050变量------------------------
static volatile bool mpu_sample_due;
static bool mpu_present;
// 调试观察变量：原始加速度和角速度读数，尚未换算成姿态角。
volatile int16_t mpu_accel_x, mpu_accel_y, mpu_accel_z;
volatile int16_t mpu_gyro_x, mpu_gyro_y, mpu_gyro_z;
volatile bool mpu_data_valid;

//SAADC事件回调函数，因为是堵塞模式，所以不需要事件，这里定义了一个空的事件回调函数
void saadc_callback(nrf_drv_saadc_evt_t const * p_event){}

//saadc初始化
void saadc_init(void)
{
    ret_code_t err_code;

    // P0.04/AIN2连接电位器，经JP4接通后测量其对地电压。
    nrf_saadc_channel_config_t channel_config = 
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    
		//初始化saadc
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);
    
		//加入通道配置，使用通道0
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
} 

/**@brief 初始化软件定时器，创建100毫秒和2秒两个重复定时器。
 */

// 采样通道0，更新adc_value并换算voltage_value。
static void saadc_measure(void)
{
    nrf_saadc_value_t sample;
    ret_code_t err_code;

    err_code = nrf_drv_saadc_sample_convert(0, &sample);
    APP_ERROR_CHECK(err_code);
    adc_value = sample;
    // 10位单端采样，内部0.6V参考、1/6增益：满量程为3.6V。
    voltage_value = adc_value * 3.6f / 1024.0f;

}

//串口事件回调
static void uart_event_handler(app_uart_evt_t * event)
{
    uint8_t byte;
    if (event->evt_type == APP_UART_DATA_READY)
    {
        while (app_uart_get(&byte) == NRF_SUCCESS) { }
    }
    else if (event->evt_type == APP_UART_FIFO_ERROR)
    {
        uart_error_code = event->data.error_code;
    }
    else if (event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        uart_error_code = event->data.error_communication;
    }
}

//初始化串口
static void uart_init(void)
{
    ret_code_t error;
    app_uart_comm_params_t const params = {
        .rx_pin_no = 8,  // P0.08 RXD，接底板CH340的TXD。
        .tx_pin_no = 6,  // P0.06 TXD，接底板CH340的RXD。
        .rts_pin_no = UART_PIN_DISCONNECTED,
        .cts_pin_no = UART_PIN_DISCONNECTED,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity = false,
        .baud_rate = NRF_UART_BAUDRATE_115200
    };
    APP_UART_FIFO_INIT(&params, 64, 256, uart_event_handler, APP_IRQ_PRIORITY_LOWEST, error);
    APP_ERROR_CHECK(error);
}

// 每100毫秒将最新ADC原始值和电位器电压发送到串口与蓝牙。
static void saadc_send(float voltage)
{
    char send_buffer[48];
    int length;
    int32_t millivolts;
    uint32_t magnitude;

    // 转成整数毫伏再格式化，避免依赖printf浮点支持。
    millivolts = (int32_t)(voltage * 1000.0f +
                          (voltage >= 0 ? 0.5f : -0.5f));
    magnitude = (uint32_t)(millivolts < 0 ? -millivolts : millivolts);
    // 每行一组数据，例如 ADC,256,1.800，电压单位为V。
    length = snprintf(send_buffer, sizeof(send_buffer),
                      "ADC,%d,%s%lu.%03lu\r\n", (int)adc_value,
                      millivolts < 0 ? "-" : "",
                      (unsigned long)(magnitude / 1000),
                      (unsigned long)(magnitude % 1000));
    if (length > 0 && length < (int)sizeof(send_buffer))
    {
        int i;
        for (i = 0; i < length; ++i)
        {
            uart_error_code = app_uart_put((uint8_t)send_buffer[i]);
            if (uart_error_code != NRF_SUCCESS)
            {
                break;
            }
        }
        // 保留现有BLE数据格式，供saadc_map继续使用。
        (void)ble_pc_send((uint8_t const *)send_buffer, (uint16_t)length);
    }
}

// 0.1秒定时器回调
static void timer_timeout_handler1(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    saadc_sample_due = true;
    mpu_sample_due = true;
}

// 2秒定时器回调
static void timer_timeout_handler2(void * p_context)
{
    UNUSED_PARAMETER(p_context);
}

static void saadc_process(void)
{
    if (!saadc_sample_due)
    {
        return;
    }
    saadc_sample_due = false;
    saadc_measure();
    saadc_send(voltage_value);
}

//定时器初始化
static void timers_init(void)
{
    //初始化软件定时器
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // 创建定时器。

    // 创建1秒和2秒的重复定时器。
    //创建两个定时，设置定时器模式
       err_code = app_timer_create(&m_app_timer_id1, APP_TIMER_MODE_REPEATED, timer_timeout_handler1);
       APP_ERROR_CHECK(err_code);
       err_code = app_timer_create(&m_app_timer_id2, APP_TIMER_MODE_REPEATED, timer_timeout_handler2);
       APP_ERROR_CHECK(err_code);
}

/**@brief 启动1秒和2秒两个重复定时器。
 */

//定时开始函数
static void application_timers_start(void)
{
    // 启动上面创建的重复定时器。
	uint32_t err_code;

    // 启动定时器：分别每1秒、2秒触发一次。
    err_code = app_timer_start(m_app_timer_id1, TIME_LEVEL_MEAS_INTERVAL1, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_app_timer_id2, TIME_LEVEL_MEAS_INTERVAL2, NULL);
    APP_ERROR_CHECK(err_code);
}

// 按键业务操作；其他蓝牙控制事件交给蓝牙模块处理。
static void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_3:
            nrf_gpio_pin_toggle(LED_4);
            (void)ble_pc_send((uint8_t const *)"Board button\r\n", 14);
            break;
        case BSP_EVENT_KEY_2:
            nrf_gpio_pin_toggle(LED_3);
            break;
        default:
            ble_pc_on_bsp_event(event);
            break;
    }
}

/**@brief 初始化按键和LED，并判断启动时是否请求清除绑定。
 *
 * @param[out] p_erase_bonds 是否清除绑定信息。
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;
    //初始化按键和LED，同时注册按键回调事件
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
    //配置按键事件触发
    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief 初始化日志模块和日志输出后端。
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief 初始化电源管理模块。
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief 处理待输出日志；无日志时等待下一次中断事件。
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void mpu6050_start(void)
{
    twi_master_init();
    mpu_present = mpu6050_init();
    if (mpu_present)
    {
        NRF_LOG_INFO("MPU sensor detected: ID=0x%02x (SCL P0.22, SDA P0.23)",
                     mpu_who_am_i_value);
    }
    else
    {
        NRF_LOG_WARNING("MPU6050 unavailable: stage=%d I2C error=%d ID=0x%02x",
                        mpu_probe_stage, mpu_twi_last_error, mpu_who_am_i_value);
    }
}

static void mpu6050_process(void)
{
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;

    if (!mpu_sample_due)
    {
        return;
    }
    mpu_sample_due = false;
    if (!mpu_present)
    {
        return;
    }

    if (MPU6050_ReadAcc(&accel_x, &accel_y, &accel_z) &&
        MPU6050_ReadGyro(&gyro_x, &gyro_y, &gyro_z))
    {
        mpu_accel_x = accel_x;
        mpu_accel_y = accel_y;
        mpu_accel_z = accel_z;
        mpu_gyro_x = gyro_x;
        mpu_gyro_y = gyro_y;
        mpu_gyro_z = gyro_z;
        mpu_data_valid = true;
        NRF_LOG_INFO("MPU accel: %d %d %d", accel_x, accel_y, accel_z);
        NRF_LOG_INFO("MPU gyro: %d %d %d", gyro_x, gyro_y, gyro_z);
    }
    else
    {
        mpu_data_valid = false;
        NRF_LOG_WARNING("MPU6050 read failed");
    }
}

/**@brief 程序入口：初始化各模块，启动定时器与广播，然后进入主循环。
 */
int main(void)
{
    bool erase_bonds;

    // 按依赖顺序初始化日志、定时器、按键、蓝牙协议栈和服务。
    log_init();
    timers_init();          //软件定时器初始化
    buttons_leds_init(&erase_bonds);        //协议栈下按键初始化
    power_management_init();
    uart_init();
    ble_pc_init();         // 初始化蓝牙配置和NUS服务
		bsp_board_init(BSP_INIT_LEDS);	//初始化LED

    NRF_LOG_INFO("BLE PC link started."); // 启动定时器和广播，等待电脑连接。
    saadc_init();              // 先初始化SAADC，再启动采样定时器
    mpu6050_start();
    application_timers_start();  //启动软件定时器

    ble_pc_start(erase_bonds); //启动广播
		
		
    // 主循环：处理日志并低功耗等待事件。
    for (;;)
    {
        saadc_process();
        mpu6050_process();
        idle_state_handle();
        
    }
}
