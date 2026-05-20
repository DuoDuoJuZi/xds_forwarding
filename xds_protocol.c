/**
 * @file xds_protocol.c
 * @author DuoDuoJuZi
 * @date 2026-05-18
 * @brief XDS 私有功率数据解析实现
 */

#include "xds_protocol.h"

/**
 * @brief 读取无符号小端 16 位数值
 *
 * @param p_buffer 源字节缓冲区
 * @param offset 首字节偏移
 *
 * @return 解码后的 16 位数值
 */
static uint16_t xds_read_le_u16(uint8_t const * p_buffer, size_t offset)
{
    return (uint16_t)p_buffer[offset] | (uint16_t)((uint16_t)p_buffer[offset + 1] << 8);
}

/**
 * @brief 读取有符号小端 16 位数值
 *
 * @param p_buffer 源字节缓冲区
 * @param offset 首字节偏移
 *
 * @return 解码后的有符号 16 位数值
 */
static int16_t xds_read_le_s16(uint8_t const * p_buffer, size_t offset)
{
    return (int16_t)xds_read_le_u16(p_buffer, offset);
}

bool xds_power_measurement_parse(uint8_t const * p_payload,
                                 size_t          length,
                                 xds_power_measurement_t * p_output)
{
    if ((p_payload == NULL) || (p_output == NULL) || (length < 2))
    {
        return false;
    }

    *p_output = (xds_power_measurement_t){0};
    p_output->total_power_w = xds_read_le_u16(p_payload, 0);

    if (length >= 4)
    {
        p_output->left_power_w = xds_read_le_s16(p_payload, 2);
    }
    if (length >= 6)
    {
        p_output->right_power_w = xds_read_le_s16(p_payload, 4);
    }
    if (length >= 8)
    {
        int16_t const cadence = xds_read_le_s16(p_payload, 6);
        p_output->cadence_rpm = (cadence > 0) ? (uint16_t)cadence : 0;
    }
    if (length >= 10)
    {
        p_output->angle_deg = (int16_t)xds_read_le_u16(p_payload, 8);
    }
    if (length >= 11)
    {
        p_output->error_code = p_payload[10];
    }

    return true;
}
