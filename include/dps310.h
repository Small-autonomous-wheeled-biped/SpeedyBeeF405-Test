#pragma once

#include <stdbool.h>
#include <stdint.h>

#define DPS310_I2C_ADDR 0x76U
#define DPS310_PRODUCT_ID_REG 0x0DU

typedef struct {
    bool acked;
    bool id_read_ok;
    uint8_t product_id;
} dps310_probe_t;

dps310_probe_t dps310_probe(void);
