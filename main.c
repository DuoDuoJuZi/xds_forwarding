/**
 * @file main.c
 * @author DuoDuoJuZi
 * @date 2026-05-18
 * @brief XDS 私有 BLE 功率数据接收并转换为 ANT+ 自行车功率广播
 */

#include <stdint.h>
#include <string.h>

#include "ant_bpwr.h"
#include "ant_key_manager.h"
#include "ant_state_indicator.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "hardfault.h"
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "nrf_sdh_ble.h"
#include "xds_ble_bridge.h"

#define APP_BLE_CONN_CFG_TAG 1
#define XDS_ANT_POWER_MAX    4093
#define XDS_ANT_CADENCE_MAX  254

static void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event);
static void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1);
static void xds_measurement_handler(xds_power_measurement_t const * p_measurement);

BPWR_SENS_CHANNEL_CONFIG_DEF(m_ant_bpwr,
                             BPWR_CHANNEL_NUM,
                             CHAN_ID_TRANS_TYPE,
                             CHAN_ID_DEV_NUM,
                             ANTPLUS_NETWORK_NUM);

BPWR_SENS_PROFILE_CONFIG_DEF(m_ant_bpwr,
                             (ant_bpwr_torque_t)(SENSOR_TYPE),
                             ant_bpwr_calib_handler,
                             ant_bpwr_evt_handler);

static ant_bpwr_profile_t m_ant_bpwr;

NRF_SDH_ANT_OBSERVER(m_ant_observer,
                     ANT_BPWR_ANT_OBSERVER_PRIO,
                     ant_bpwr_sens_evt_handler,
                     &m_ant_bpwr);

/**
 * @brief 处理开发板按键事件
 *
 * @param event BSP 按键事件
 */
static void bsp_evt_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_2:
            ant_bpwr_calib_response(&m_ant_bpwr);
            break;

        default:
            break;
    }
}

/**
 * @brief 将 XDS 原始功率限制到 ANT+ 功率页范围
 *
 * @param power_w XDS 原始总功率瓦特值
 *
 * @return ANT+ 瞬时功率值
 */
static uint16_t ant_power_from_xds(uint16_t power_w)
{
    return (power_w > XDS_ANT_POWER_MAX) ? XDS_ANT_POWER_MAX : power_w;
}

/**
 * @brief 将 XDS 原始踏频转换为 ANT+ 踏频字节
 *
 * @param cadence_rpm XDS 原始踏频 rpm 值
 *
 * @return ANT+ 踏频字节
 */
static uint8_t ant_cadence_from_xds(uint16_t cadence_rpm)
{
    return (cadence_rpm > XDS_ANT_CADENCE_MAX) ? XDS_ANT_CADENCE_MAX : (uint8_t)cadence_rpm;
}

/**
 * @brief 将解码后的 XDS 数据写入 ANT+ BPWR 功率页
 *
 * @param p_measurement 已解码的 XDS 私有 BLE 功率数据
 */
static void xds_measurement_handler(xds_power_measurement_t const * p_measurement)
{
    uint16_t const power_w = ant_power_from_xds(p_measurement->total_power_w);

    m_ant_bpwr.BPWR_PROFILE_power_update_event_count++;
    m_ant_bpwr.page_16.pedal_power.byte = 0xFF;
    m_ant_bpwr.BPWR_PROFILE_instantaneous_cadence = ant_cadence_from_xds(p_measurement->cadence_rpm);
    m_ant_bpwr.BPWR_PROFILE_accumulated_power = (uint16_t)(m_ant_bpwr.BPWR_PROFILE_accumulated_power + power_w);
    m_ant_bpwr.BPWR_PROFILE_instantaneous_power = power_w;

    NRF_LOG_INFO("XDS P=%uW C=%uRPM L=%d R=%d Err=%u",
                 p_measurement->total_power_w,
                 p_measurement->cadence_rpm,
                 p_measurement->left_power_w,
                 p_measurement->right_power_w,
                 p_measurement->error_code);
}

/**
 * @brief 在 ANT+ 配置文件事件后喂电源管理
 *
 * @param p_profile ANT+ BPWR 配置文件
 * @param event ANT+ BPWR 配置文件事件
 */
static void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event)
{
    UNUSED_PARAMETER(p_profile);
    UNUSED_PARAMETER(event);
    nrf_pwr_mgmt_feed();
}

/**
 * @brief 处理 ANT+ 接收端校准请求
 *
 * @param p_profile ANT+ BPWR 配置文件
 * @param p_page1 校准请求页
 */
static void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1)
{
    UNUSED_PARAMETER(p_profile);

    switch (p_page1->calibration_id)
    {
        case ANT_BPWR_CALIB_ID_MANUAL:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
            m_ant_bpwr.BPWR_PROFILE_general_calib_data = CALIBRATION_DATA;
            break;

        case ANT_BPWR_CALIB_ID_AUTO:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
            m_ant_bpwr.BPWR_PROFILE_auto_zero_status = p_page1->auto_zero_status;
            m_ant_bpwr.BPWR_PROFILE_general_calib_data = CALIBRATION_DATA;
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_REQ:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_REQ_SUCCESS;
            memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data,
                   p_page1->data.custom_calib,
                   sizeof(m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_UPDATE:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_UPDATE_SUCCESS;
            memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data,
                   p_page1->data.custom_calib,
                   sizeof(m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
            break;

        default:
            break;
    }
}

/**
 * @brief 初始化日志模块
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**
 * @brief 初始化定时器电源管理 LED 和按键
 */
static void board_utils_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = ant_state_indicator_init(m_ant_bpwr.channel_number, BPWR_SENS_CHANNEL_TYPE);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief 启用 S332 SoftDevice BLE 和 ANT 协议栈
 */
static void softdevice_init(void)
{
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);

    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUM);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief 初始化并打开 ANT+ 自行车功率传感器通道
 */
static void bpwr_profile_init(void)
{
    ret_code_t err_code = ant_bpwr_sens_init(&m_ant_bpwr,
                                             BPWR_SENS_CHANNEL_CONFIG(m_ant_bpwr),
                                             BPWR_SENS_PROFILE_CONFIG(m_ant_bpwr));
    APP_ERROR_CHECK(err_code);

    m_ant_bpwr.page_80 = ANT_COMMON_page80(BPWR_HW_REVISION,
                                           BPWR_MANUFACTURER_ID,
                                           BPWR_MODEL_NUMBER);

    m_ant_bpwr.page_81 = ANT_COMMON_page81(BPWR_SW_REVISION_MAJOR,
                                           BPWR_SW_REVISION_MINOR,
                                           BPWR_SERIAL_NUMBER);

    m_ant_bpwr.BPWR_PROFILE_auto_zero_status = ANT_BPWR_AUTO_ZERO_OFF;
    m_ant_bpwr.BPWR_PROFILE_power_update_event_count = 0;
    m_ant_bpwr.page_16.pedal_power.byte = 0xFF;
    m_ant_bpwr.BPWR_PROFILE_instantaneous_cadence = 0xFF;
    m_ant_bpwr.BPWR_PROFILE_accumulated_power = 0;
    m_ant_bpwr.BPWR_PROFILE_instantaneous_power = 0;

    err_code = ant_bpwr_sens_open(&m_ant_bpwr);
    APP_ERROR_CHECK(err_code);

    err_code = ant_state_indicator_channel_opened();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief 应用程序入口
 *
 * @return 不返回
 */
int main(void)
{
    log_init();
    board_utils_init();
    softdevice_init();
    bpwr_profile_init();

    xds_ble_bridge_measurement_handler_set(xds_measurement_handler);
    APP_ERROR_CHECK(xds_ble_bridge_init());
    APP_ERROR_CHECK(xds_ble_bridge_start());

    NRF_LOG_INFO("XDS BLE to ANT+ Bicycle Power bridge started with S332 7.0.1");

    for (;;)
    {
        xds_ble_bridge_process();
        NRF_LOG_FLUSH();
        nrf_pwr_mgmt_run();
    }
}
