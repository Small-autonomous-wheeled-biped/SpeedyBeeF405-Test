#include "crc.h"

uint16_t crc16_ccitt_update(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8U;
        for (uint8_t bit = 0; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    return crc16_ccitt_update(0xFFFFU, data, len);
}
