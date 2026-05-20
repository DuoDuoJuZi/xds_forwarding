/**
 * @file xds_ble_bridge.h
 * @author DuoDuoJuZi
 * @date 2026-05-18
 * @brief XDS BLE 接收桥接接口
 */

#ifndef XDS_BLE_BRIDGE_H__
#define XDS_BLE_BRIDGE_H__

#include <stdint.h>

#include "xds_protocol.h"

typedef void (*xds_ble_bridge_measurement_handler_t)(xds_power_measurement_t const * p_measurement);

/**
 * @brief 注册 XDS BLE 功率包解码后的应用回调
 *
 * @param handler 功率数据回调为空则解绑回调
 */
void xds_ble_bridge_measurement_handler_set(xds_ble_bridge_measurement_handler_t handler);

/**
 * @brief 初始化 BLE Central 桥接状态
 *
 * @retval NRF_SUCCESS BLE 桥接状态已初始化
 */
uint32_t xds_ble_bridge_init(void);

/**
 * @brief 开始扫描 XDS 私有功率计服务
 *
 * @retval NRF_SUCCESS 扫描已开始或正在进行
 * @retval uint32_t BLE GAP API 返回的 SoftDevice 错误码
 */
uint32_t xds_ble_bridge_start(void);

/**
 * @brief 驱动 BLE 桥接重试和超时任务
 */
void xds_ble_bridge_process(void);

#endif
