#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * CRC-16/CCITT-FALSE (polynomial 0x1021, init 0xFFFF, no reflection).
 * All wire packets use this CRC over all bytes before the CRC field.
 */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/* Incremental variant: call with previous result as seed. */
uint16_t crc16_ccitt_update(uint16_t crc, const uint8_t *data, size_t len);
