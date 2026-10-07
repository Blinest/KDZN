/**
 * @file proto.c
 * @brief 通信协议层（V2）实现：帧封装
 *
 * @date 2026-10-04
 */

#include "proto.h"
#include <string.h>

uint16_t proto_append_checksum(uint8_t *out, uint16_t len)
{
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum += out[i];
    out[len] = (uint8_t)(sum & 0xFF);
    return (uint16_t)(len + 1);
}

uint16_t proto_pack_uplink(uint8_t *out, uint8_t dev, uint8_t func,
                           const uint8_t *payload, uint16_t n)
{
    if (out == NULL || n > 255u) return 0;

    uint16_t i = 0;
    out[i++] = PROTO_UPLINK_MAGIC;   /* 0x55 */
    out[i++] = dev;
    out[i++] = func;
    out[i++] = (uint8_t)n;
    if (payload != NULL) {
        for (uint16_t k = 0; k < n; k++) out[i++] = payload[k];
    }
    return proto_append_checksum(out, i);
}
