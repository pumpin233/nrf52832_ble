// 蓝牙连接、广播和配对配置；NUS数据收发位于ble_nus_transport.c。
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "ble_nus.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_config.h"
#include "ble_nus_transport.h"

#define DEVICE_NAME                     "BLE_PC_Link"                       /**< 广播中的设备名称。 */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< 厂商名称；当前没有初始化设备信息服务。 */
#define APP_ADV_INTERVAL                300                                     /**< 广播间隔：300×0.625ms，即187.5ms。 */

#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED                                   /**< 持续广播，不会在180秒后超时。 */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< 应用程序蓝牙事件回调的优先级。 */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< SoftDevice蓝牙配置标识。 */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< 最短连接间隔100ms。 */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< 最长连接间隔200ms。 */
#define SLAVE_LATENCY                   0                                       /**< 从机延迟为0。 */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< 连接监督超时4秒。 */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< 连接后首次请求更新连接参数的延迟：5秒。 */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< 后续连接参数更新请求间隔：30秒。 */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< 连接参数更新最多尝试3次。 */

//绑定设备，默认1
#define SEC_PARAM_BOND                  1                                       /**< 启用绑定。 */

#define SEC_PARAM_MITM                  0                                       /**< 不要求中间人保护。 */
#define SEC_PARAM_LESC                  0                                       /**< 不启用LE安全连接。 */
#define SEC_PARAM_KEYPRESS              0                                       /**< 不启用按键通知。 */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< 设备没有配对输入输出能力。 */
#define SEC_PARAM_OOB                   0                                       /**< 不使用带外配对数据。 */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< 最小加密密钥长度。 */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< 最大加密密钥长度。 */

#define DEAD_BEEF                       0xDEADBEEF                              /**< 用于识别严重错误的标记值。 */

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT模块实例。 */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< 排队写入模块实例。*/
BLE_ADVERTISING_DEF(m_advertising);
                                       /**< 广播模块实例。 */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< 当前连接句柄；无连接时为无效值。 */

// 本工程使用NUS服务，实例m_nus已在上方声明。

// 告诉扫描设备：本程序提供NUS蓝牙收发服务。
static ble_uuid_t m_adv_uuids[] =                                               /**< 放入扫描响应的服务UUID列表。 */
{
    {BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}
};


static void advertising_start(bool erase_bonds);



/**@brief SoftDevice发生断言时交给错误处理函数。
 *
 * @param[in] line_num 出错行号。
 * @param[in] file_name 出错文件名。
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief 处理绑定信息删除及重新配对请求。
 *
 * @param[in] p_evt Peer Manager事件。
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        case PM_EVT_CONN_SEC_CONFIG_REQ://重新连接
        {
            // 允许已绑定的电脑重新发起配对。
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};  
            //允许再次配对
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        }

        default:
            break;
    }
}

/**@brief 设置蓝牙设备名、外观和首选连接参数。
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    //安全模式，安全权限设置
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);  //
    //设备名称设置
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    // 设置设备外观类型；仅影响扫描界面的图标。

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_GAMEPAD);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    //连接参数配置
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
    
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief 初始化GATT模块。
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief 排队写入模块出错时，交给统一错误处理函数。
 *
 * @param[in] nrf_error SDK返回的错误码。
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief 初始化排队写入模块和NUS蓝牙收发服务。
 */
static void services_init(void)
{
    uint32_t         err_code;
    nrf_ble_qwr_init_t  qwr_init = {0};

    // 初始化排队写入模块。
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // 注册用于电脑蓝牙收发的NUS服务。
    ble_pc_data_init();


}


/**@brief 连接参数协商失败时断开当前连接。
 *
 * @param[in] p_evt 连接参数协商事件。
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief 处理连接参数模块报告的错误。
 *
 * @param[in] nrf_error SDK返回的错误码。
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief 设置连接参数协商的时间、次数和事件回调。
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));
    //设置连接参数
    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    //导入连接参数
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief 收到休眠按键事件时进入系统关闭模式，按键唤醒后会复位。
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // 配置用于唤醒设备的按键。
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // 进入系统关闭模式；按键唤醒后从头运行main。
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief 处理广播开始和广播空闲事件，更新指示灯状态。
 *
 * @param[in] ble_adv_evt 广播模块事件。
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);        //闪灯
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("Advertising idle; application remains active.");
            break;

        default:
            break;
    }
}


/**@brief 处理连接、断开、PHY更新和GATT超时等蓝牙事件。
 *
 * @param[in] p_ble_evt 蓝牙协议栈事件。
 * @param[in] p_context 未使用。
 */
//蓝牙事件回调函数
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            // 断开后重新广播时，广播事件会更新LED状态。
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            ble_pc_data_on_disconnected();
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);  //广播LED变长亮
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            ble_pc_data_on_connected(m_conn_handle);
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // GATT客户端超时，断开连接。
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);//连接断开
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // GATT服务器超时，断开连接。
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);//连接断开
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // 其他事件无需处理。
            break;
    }
}


/**@brief 启用SoftDevice蓝牙协议栈并注册事件回调。
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;
    //协议栈回复使能应答
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // 按sdk_config.h中的默认参数配置蓝牙协议栈。
    // 计算应用程序可用RAM的起始地址。
    //配置协议栈，配置参数和RAM
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    //使能协议栈
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    //注册协议栈回调函数，观察者函数
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief 初始化Peer Manager并设置绑定和配对参数。
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // 配对和绑定时使用的安全参数。
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
	
		//下面四项为绑定密钥，默认都是1
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief 从Flash中删除已保存的绑定设备信息。
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief 处理按键触发的蓝牙控制事件。
 *
 * @param[in] event BSP按键事件。
 */

 //触发事件回调
void ble_pc_on_bsp_event(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0
        
        default:
            break;
    }
}

/**@brief 配置设备名、扫描响应中的NUS UUID及广播间隔。
 */
//广播初始化
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
    //广播初始化配置参数
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;      //显示全名
    //init.advdata.short_name_len          = 6;           //显示的名称长度
    init.advdata.include_appearance      = true;        //显示图标
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;
    //广播初始化函数
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief 根据是否需要清除绑定，启动快速广播。
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // 删除绑定成功后，Peer Manager回调会启动广播。
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}

// 按照原main.c中的顺序初始化，确保服务先于广播完成注册。
void ble_pc_init(void)
{
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();
}

void ble_pc_start(bool erase_bonds)
{
    advertising_start(erase_bonds);
}
