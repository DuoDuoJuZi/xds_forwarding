/**
 * @file xds_ble_bridge.c
 * @author DuoDuoJuZi
 * @date 2026-05-18
 * @brief XDS BLE 接收桥接实现
 */

#include "xds_ble_bridge.h"

#include <string.h>

#include "app_timer.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_gatt.h"
#include "ble_gattc.h"
#include "ble_hci.h"
#include "nrf_error.h"
#include "nrf_log.h"
#include "nrf_sdh_ble.h"

#define XDS_BLE_OBSERVER_PRIO              3
#define XDS_BLE_CONN_CFG_TAG               1
#define XDS_SERVICE_UUID                   0x1828
#define XDS_MEASUREMENT_CHAR_UUID          0x2A63
#define XDS_WRITE_CHAR_UUID                0x2A55
#define XDS_START_COMMAND_DELAY_TICKS      APP_TIMER_TICKS(800)
#define XDS_START_COMMAND_RETRY_TICKS      APP_TIMER_TICKS(2000)
#define XDS_START_COMMAND_MAX_ATTEMPTS     5
#define XDS_DATA_TIMEOUT_TICKS             APP_TIMER_TICKS(30000)
#define XDS_SCAN_INTERVAL                  1349
#define XDS_SCAN_WINDOW                    449
#define XDS_CONN_INTERVAL_MIN              MSEC_TO_UNITS(30, UNIT_1_25_MS)
#define XDS_CONN_INTERVAL_MAX              MSEC_TO_UNITS(60, UNIT_1_25_MS)
#define XDS_CONN_SUP_TIMEOUT               MSEC_TO_UNITS(4000, UNIT_10_MS)
#define XDS_SCAN_BUFFER_SIZE               BLE_GAP_SCAN_BUFFER_MIN

typedef enum
{
    XDS_BLE_STATE_IDLE,
    XDS_BLE_STATE_SCANNING,
    XDS_BLE_STATE_CONNECTING,
    XDS_BLE_STATE_DISCOVERING_SERVICE,
    XDS_BLE_STATE_DISCOVERING_CHARS,
    XDS_BLE_STATE_DISCOVERING_MEASUREMENT_CCCD,
    XDS_BLE_STATE_READY
} xds_ble_state_t;

static xds_ble_bridge_measurement_handler_t m_measurement_handler;
static xds_ble_state_t                      m_state;
static uint16_t                             m_conn_handle = BLE_CONN_HANDLE_INVALID;
static ble_gattc_handle_range_t             m_service_range;
static uint16_t                             m_measurement_handle;
static uint16_t                             m_measurement_cccd_handle;
static uint16_t                             m_write_handle;
static uint8_t                              m_start_command_attempts;
static uint32_t                             m_next_start_command_ticks;
static uint32_t                             m_last_packet_ticks;
static uint8_t                              m_scan_buffer_data[XDS_SCAN_BUFFER_SIZE];
static ble_data_t                           m_scan_buffer =
{
    .p_data = m_scan_buffer_data,
    .len    = sizeof(m_scan_buffer_data)
};

static ble_gap_scan_params_t const m_scan_params =
{
    .extended               = 0,
    .report_incomplete_evts = 0,
    .active                 = 1,
    .filter_policy          = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .scan_phys              = BLE_GAP_PHY_1MBPS,
    .interval               = XDS_SCAN_INTERVAL,
    .window                 = XDS_SCAN_WINDOW,
    .timeout                = 0,
    .channel_mask           = {0}
};

static ble_gap_conn_params_t const m_conn_params =
{
    .min_conn_interval = XDS_CONN_INTERVAL_MIN,
    .max_conn_interval = XDS_CONN_INTERVAL_MAX,
    .slave_latency     = 0,
    .conn_sup_timeout  = XDS_CONN_SUP_TIMEOUT
};

/**
 * @brief 构建 Bluetooth SIG 16 位 UUID 描述符
 *
 * @param uuid 16 位 UUID 数值
 *
 * @return BLE UUID 描述符
 */
static ble_uuid_t xds_uuid16(uint16_t uuid)
{
    ble_uuid_t result;

    result.uuid = uuid;
    result.type = BLE_UUID_TYPE_BLE;

    return result;
}

/**
 * @brief 检查广播数据字段是否包含 16 位服务 UUID
 *
 * @param p_data 广播或扫描响应负载
 * @param length 负载长度
 * @param uuid 要查找的 16 位服务 UUID
 *
 * @retval true UUID 存在于完整或不完整 16 位服务 UUID 列表
 * @retval false UUID 不存在
 */
static bool xds_adv_has_service(uint8_t const * p_data, uint8_t length, uint16_t uuid)
{
    uint8_t index = 0;

    while (index < length)
    {
        uint8_t const field_length = p_data[index];

        if (field_length == 0)
        {
            return false;
        }
        if ((uint16_t)index + field_length >= (uint16_t)length)
        {
            return false;
        }

        uint8_t const type = p_data[index + 1];
        if ((type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE) ||
            (type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE))
        {
            uint8_t value_index = index + 2;
            uint8_t const value_end = index + field_length + 1;

            while ((uint8_t)(value_index + 1u) < value_end)
            {
                uint16_t const found = (uint16_t)p_data[value_index] |
                                       (uint16_t)((uint16_t)p_data[value_index + 1] << 8);
                if (found == uuid)
                {
                    return true;
                }
                value_index += 2;
            }
        }

        index = (uint8_t)(index + field_length + 1);
    }

    return false;
}

/**
 * @brief 开始发现 XDS 私有功率主服务
 */
static void xds_service_discovery_start(void)
{
    ble_uuid_t const uuid = xds_uuid16(XDS_SERVICE_UUID);
    uint32_t const err_code = sd_ble_gattc_primary_services_discover(m_conn_handle,
                                                                     BLE_GATT_HANDLE_START,
                                                                     &uuid);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("XDS service discovery failed to start: 0x%08x", err_code);
    }
}

/**
 * @brief 在 XDS 服务范围内开始发现特征
 */
static void xds_characteristic_discovery_start(void)
{
    uint32_t const err_code = sd_ble_gattc_characteristics_discover(m_conn_handle, &m_service_range);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("XDS characteristic discovery failed to start: 0x%08x", err_code);
    }
}

/**
 * @brief 为一个特征值句柄开始发现描述符
 *
 * @param value_handle 特征值句柄
 * @param next_handle 下一个特征声明句柄为 0 时表示服务结束
 */
static void xds_cccd_discovery_start(uint16_t value_handle, uint16_t next_handle)
{
    ble_gattc_handle_range_t range;

    range.start_handle = (uint16_t)(value_handle + 1u);
    range.end_handle = (next_handle != 0u) ? (uint16_t)(next_handle - 1u) : m_service_range.end_handle;

    if (range.start_handle > range.end_handle)
    {
        return;
    }

    uint32_t const err_code = sd_ble_gattc_descriptors_discover(m_conn_handle, &range);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("XDS CCCD discovery failed to start: 0x%08x", err_code);
    }
}

/**
 * @brief 向已发现的 GATT 描述符写入 16 位数值
 *
 * @param handle 描述符句柄
 * @param value 要写入的小端数值
 */
static void xds_write_u16(uint16_t handle, uint16_t value)
{
    static uint8_t bytes[2];
    ble_gattc_write_params_t params;

    bytes[0] = (uint8_t)(value & 0xFFu);
    bytes[1] = (uint8_t)((value >> 8) & 0xFFu);
    memset(&params, 0, sizeof(params));
    params.write_op = BLE_GATT_OP_WRITE_REQ;
    params.handle = handle;
    params.len = sizeof(bytes);
    params.p_value = bytes;

    uint32_t const err_code = sd_ble_gattc_write(m_conn_handle, &params);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("XDS descriptor write failed: 0x%08x", err_code);
    }
}

/**
 * @brief 发送用于启动功率通知的 XDS 私有命令
 */
static void xds_start_command_send(void)
{
    static uint8_t const command[] = {0x02, 0x16, 0xAA, 0x10};
    ble_gattc_write_params_t params;

    if (m_write_handle == BLE_GATT_HANDLE_INVALID)
    {
        return;
    }

    memset(&params, 0, sizeof(params));
    params.write_op = BLE_GATT_OP_WRITE_REQ;
    params.handle = m_write_handle;
    params.len = sizeof(command);
    params.p_value = command;

    uint32_t const err_code = sd_ble_gattc_write(m_conn_handle, &params);

    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_INFO("XDS start command sent");
        m_start_command_attempts = XDS_START_COMMAND_MAX_ATTEMPTS;
    }
    else if ((err_code != NRF_ERROR_BUSY) && (err_code != NRF_ERROR_INVALID_STATE))
    {
        NRF_LOG_WARNING("XDS start command failed: 0x%08x", err_code);
    }
}

/**
 * @brief 为新的 BLE 连接尝试重置句柄和定时状态
 */
static void xds_link_state_reset(void)
{
    m_service_range.start_handle = BLE_GATT_HANDLE_INVALID;
    m_service_range.end_handle = BLE_GATT_HANDLE_INVALID;
    m_measurement_handle = BLE_GATT_HANDLE_INVALID;
    m_measurement_cccd_handle = BLE_GATT_HANDLE_INVALID;
    m_write_handle = BLE_GATT_HANDLE_INVALID;
    m_start_command_attempts = 0;
    m_next_start_command_ticks = 0;
    m_last_packet_ticks = 0;
}

/**
 * @brief 扫描时处理广播报告
 *
 * @param p_report GAP 广播报告
 */
static void xds_on_adv_report(ble_gap_evt_adv_report_t const * p_report)
{
    if ((m_state != XDS_BLE_STATE_SCANNING) ||
        !xds_adv_has_service(p_report->data.p_data, (uint8_t)p_report->data.len, XDS_SERVICE_UUID))
    {
        (void)sd_ble_gap_scan_start(NULL, &m_scan_buffer);
        return;
    }

    uint32_t err_code = sd_ble_gap_scan_stop();

    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE))
    {
        NRF_LOG_WARNING("XDS scan stop failed: 0x%08x", err_code);
    }

    m_state = XDS_BLE_STATE_CONNECTING;
    err_code = sd_ble_gap_connect(&p_report->peer_addr,
                                  &m_scan_params,
                                  &m_conn_params,
                                  XDS_BLE_CONN_CFG_TAG);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("XDS connect failed: 0x%08x", err_code);
        m_state = XDS_BLE_STATE_IDLE;
        (void)xds_ble_bridge_start();
    }
}

/**
 * @brief 处理 GAP 已连接事件
 *
 * @param p_gap_evt GAP 事件数据
 */
static void xds_on_connected(ble_gap_evt_t const * p_gap_evt)
{
    m_conn_handle = p_gap_evt->conn_handle;
    m_state = XDS_BLE_STATE_DISCOVERING_SERVICE;
    xds_link_state_reset();
    xds_service_discovery_start();
}

/**
 * @brief 处理 GAP 断开连接事件
 */
static void xds_on_disconnected(void)
{
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    m_state = XDS_BLE_STATE_IDLE;
    xds_link_state_reset();
    (void)xds_ble_bridge_start();
}

/**
 * @brief 处理 XDS 服务发现响应
 *
 * @param p_gattc_evt GATT Client 事件
 */
static void xds_on_service_discovery(ble_gattc_evt_t const * p_gattc_evt)
{
    if ((p_gattc_evt->gatt_status != BLE_GATT_STATUS_SUCCESS) ||
        (p_gattc_evt->params.prim_srvc_disc_rsp.count == 0))
    {
        (void)sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        return;
    }

    m_service_range = p_gattc_evt->params.prim_srvc_disc_rsp.services[0].handle_range;
    m_state = XDS_BLE_STATE_DISCOVERING_CHARS;
    xds_characteristic_discovery_start();
}

/**
 * @brief 处理 XDS 特征发现响应
 *
 * @param p_gattc_evt GATT Client 事件
 */
static void xds_on_characteristic_discovery(ble_gattc_evt_t const * p_gattc_evt)
{
    uint16_t measurement_next_handle = 0;

    if (p_gattc_evt->gatt_status != BLE_GATT_STATUS_SUCCESS)
    {
        (void)sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        return;
    }

    for (uint16_t i = 0; i < p_gattc_evt->params.char_disc_rsp.count; ++i)
    {
        ble_gattc_char_t const * p_char = &p_gattc_evt->params.char_disc_rsp.chars[i];

        if ((p_char->uuid.type == BLE_UUID_TYPE_BLE) && (p_char->uuid.uuid == XDS_MEASUREMENT_CHAR_UUID))
        {
            m_measurement_handle = p_char->handle_value;
            if ((i + 1u) < p_gattc_evt->params.char_disc_rsp.count)
            {
                measurement_next_handle = p_gattc_evt->params.char_disc_rsp.chars[i + 1u].handle_decl;
            }
        }
        if ((p_char->uuid.type == BLE_UUID_TYPE_BLE) && (p_char->uuid.uuid == XDS_WRITE_CHAR_UUID))
        {
            m_write_handle = p_char->handle_value;
        }
    }

    if (m_measurement_handle == BLE_GATT_HANDLE_INVALID)
    {
        (void)sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        return;
    }

    m_state = XDS_BLE_STATE_DISCOVERING_MEASUREMENT_CCCD;
    xds_cccd_discovery_start(m_measurement_handle, measurement_next_handle);
}

/**
 * @brief 处理功率测量或写入特征的描述符发现
 *
 * @param p_gattc_evt GATT Client 事件
 */
static void xds_on_descriptor_discovery(ble_gattc_evt_t const * p_gattc_evt)
{
    if (p_gattc_evt->gatt_status != BLE_GATT_STATUS_SUCCESS)
    {
        return;
    }

    for (uint16_t i = 0; i < p_gattc_evt->params.desc_disc_rsp.count; ++i)
    {
        ble_gattc_desc_t const * p_desc = &p_gattc_evt->params.desc_disc_rsp.descs[i];

        if ((p_desc->uuid.type == BLE_UUID_TYPE_BLE) &&
            (p_desc->uuid.uuid == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG))
        {
            if (m_state == XDS_BLE_STATE_DISCOVERING_MEASUREMENT_CCCD)
            {
                m_measurement_cccd_handle = p_desc->handle;
            }
        }
    }

    if ((m_state == XDS_BLE_STATE_DISCOVERING_MEASUREMENT_CCCD) &&
        (m_measurement_cccd_handle != BLE_GATT_HANDLE_INVALID))
    {
        xds_write_u16(m_measurement_cccd_handle, 0x0001);
        m_state = XDS_BLE_STATE_READY;
        m_next_start_command_ticks = app_timer_cnt_get() + XDS_START_COMMAND_DELAY_TICKS;
    }
}

/**
 * @brief 处理来自 XDS 功率计的通知或指示
 *
 * @param p_hvx 句柄值事件负载
 */
static void xds_on_hvx(ble_gattc_evt_hvx_t const * p_hvx)
{
    xds_power_measurement_t measurement;

    if (p_hvx->handle != m_measurement_handle)
    {
        return;
    }

    if (!xds_power_measurement_parse(p_hvx->data, p_hvx->len, &measurement))
    {
        return;
    }

    m_last_packet_ticks = app_timer_cnt_get();

    if (m_measurement_handler != NULL)
    {
        m_measurement_handler(&measurement);
    }
}

/**
 * @brief 分发 XDS 桥接使用的 SoftDevice BLE 事件
 *
 * @param p_ble_evt BLE 事件指针
 * @param p_context Observer 上下文
 */
static void xds_ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
            xds_on_adv_report(&p_ble_evt->evt.gap_evt.params.adv_report);
            break;

        case BLE_GAP_EVT_CONNECTED:
            xds_on_connected(&p_ble_evt->evt.gap_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            xds_on_disconnected();
            break;

        case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
            xds_on_service_discovery(&p_ble_evt->evt.gattc_evt);
            break;

        case BLE_GATTC_EVT_CHAR_DISC_RSP:
            xds_on_characteristic_discovery(&p_ble_evt->evt.gattc_evt);
            break;

        case BLE_GATTC_EVT_DESC_DISC_RSP:
            xds_on_descriptor_discovery(&p_ble_evt->evt.gattc_evt);
            break;

        case BLE_GATTC_EVT_HVX:
            xds_on_hvx(&p_ble_evt->evt.gattc_evt.params.hvx);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            (void)sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        default:
            break;
    }
}

NRF_SDH_BLE_OBSERVER(m_xds_ble_observer,
                     XDS_BLE_OBSERVER_PRIO,
                     xds_ble_evt_handler,
                     NULL);

void xds_ble_bridge_measurement_handler_set(xds_ble_bridge_measurement_handler_t handler)
{
    m_measurement_handler = handler;
}

uint32_t xds_ble_bridge_init(void)
{
    m_state = XDS_BLE_STATE_IDLE;
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    xds_link_state_reset();
    return NRF_SUCCESS;
}

uint32_t xds_ble_bridge_start(void)
{
    if ((m_state == XDS_BLE_STATE_SCANNING) ||
        (m_state == XDS_BLE_STATE_CONNECTING) ||
        (m_conn_handle != BLE_CONN_HANDLE_INVALID))
    {
        return NRF_SUCCESS;
    }

    uint32_t const err_code = sd_ble_gap_scan_start(&m_scan_params, &m_scan_buffer);

    if (err_code == NRF_SUCCESS)
    {
        m_state = XDS_BLE_STATE_SCANNING;
    }

    return err_code;
}

void xds_ble_bridge_process(void)
{
    uint32_t const now = app_timer_cnt_get();

    if ((m_state == XDS_BLE_STATE_READY) &&
        (m_start_command_attempts < XDS_START_COMMAND_MAX_ATTEMPTS) &&
        ((int32_t)(now - m_next_start_command_ticks) >= 0))
    {
        ++m_start_command_attempts;
        m_next_start_command_ticks = now + XDS_START_COMMAND_RETRY_TICKS;
        xds_start_command_send();
    }

    if ((m_state == XDS_BLE_STATE_READY) &&
        (m_last_packet_ticks != 0u) &&
        ((uint32_t)(now - m_last_packet_ticks) > XDS_DATA_TIMEOUT_TICKS))
    {
        (void)sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    }
}
