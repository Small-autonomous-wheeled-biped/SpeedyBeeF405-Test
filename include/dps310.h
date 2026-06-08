#pragma once

#include <stdbool.h>
#include <stdint.h>

#define DPS310_I2C_ADDR          0x76U
#define DPS310_PRODUCT_ID_REG    0x0DU
#define DPS310_PRODUCT_ID_MASK   0xF0U   /* top nibble = product ID = 0x1x */
#define DPS310_PRODUCT_ID_VALUE  0x10U

/* ---- Probe result (backward compatible) ---- */
typedef struct {
    bool    acked;
    bool    id_read_ok;
    uint8_t product_id;
} dps310_probe_t;

/* ---- Compensated measurement ---- */
typedef struct {
    float    pressure_pa;       /* compensated pressure in Pascal            */
    float    temperature_deg_c; /* compensated temperature in °C             */
    float    altitude_m;        /* approximate altitude relative to 101325 Pa*/
    bool     valid;             /* false if sensor not ready or error        */
} dps310_data_t;

/* ---- Driver API ---- */

/* Probe: check I2C ack and product ID. */
dps310_probe_t dps310_probe(void);

/* Full initialization: read calibration coefficients, start continuous mode.
   Call once after board_i2c1_init().
   Returns false if device not found or calibration read fails.               */
bool dps310_init(void);

/* Read one set of pressure+temperature measurements.
   Non-blocking: returns false (out->valid = false) if no new data yet.
   Call at ≤100 Hz (configured rate).                                         */
bool dps310_read(dps310_data_t *out);
