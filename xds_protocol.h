/**
 * @file xds_protocol.h
 * @author DuoDuoJuZi
 * @date 2026-05-18
 * @brief XDS 私有功率数据结构和解析接口
 */

#ifndef XDS_PROTOCOL_H__
#define XDS_PROTOCOL_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint16_t total_power_w;
    int16_t  left_power_w;
    int16_t  right_power_w;
    int16_t  angle_deg;
    uint16_t cadence_rpm;
    uint8_t  error_code;
} xds_power_measurement_t;

/**
 * @brief 解码一帧 XDS 私有自行车功率通知
 *
 * @param p_payload 来自 XDS 0x2A63 特征的原始 BLE 通知负载
 * @param length 负载字节长度
 * @param p_output 解析后的功率数据
 *
 * @retval true 负载长度足够生成一组功率数据
 * @retval false 输入指针为空或负载长度不足
 */
bool xds_power_measurement_parse(uint8_t const * p_payload,
                                 size_t          length,
                                 xds_power_measurement_t * p_output);

#endif
