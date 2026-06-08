#include "dps310.h"
#include "board.h"

#include <math.h>
#include <string.h>

/*
 * DPS310 driver.
 * Pressure/temperature compensation formula and register map from
 * Infineon DPS310 datasheet v1.2 (PN: DPS310-INLNZ).
 */

/* ---- Register map ---- */
#define REG_PSR_B2      0x00U
#define REG_TMP_B2      0x03U
#define REG_PRS_CFG     0x06U
#define REG_TMP_CFG     0x07U
#define REG_MEAS_CFG    0x08U
#define REG_CFG_REG     0x09U
#define REG_RESET       0x0CU
#define REG_PROD_ID     0x0DU
#define REG_COEF_BASE   0x10U   /* 18 bytes: 0x10-0x21 */

/* MEAS_CFG bits */
#define MEAS_COEF_RDY   BIT(7)
#define MEAS_SENSOR_RDY BIT(6)
#define MEAS_TMP_RDY    BIT(5)
#define MEAS_PRS_RDY    BIT(4)
#define MEAS_BOTH_CONT  0x07U   /* continuous background pressure+temperature */

/* PRS_CFG / TMP_CFG rate field (bits [6:4]) */
#define PM_RATE_16HZ    (4U << 4U)
#define PM_RATE_32HZ    (5U << 4U)

/* Oversampling (bits [3:0]): 0=1x, 1=2x, 2=4x, 3=8x */
#define PM_PRC_1X       0U
#define PM_PRC_2X       1U

/* TMP_CFG bit 7: use external MEMS temperature sensor (recommended) */
#define TMP_EXT         BIT(7)

/* Scaling factor for oversampling rate 1x (kP = kT = 524288) */
#define SCALE_FACTOR_1X 524288.0f
#define SCALE_FACTOR_2X 1572864.0f

/* Standard atmosphere at sea level (Pa) */
#define STD_SEA_LEVEL_PA 101325.0f

/* ---- Calibration coefficients ---- */
typedef struct {
    int32_t c0, c1;
    int32_t c00, c10;
    int16_t c01, c11, c20, c21, c30;
} dps310_calib_t;

static dps310_calib_t g_cal;
static float          g_kP = SCALE_FACTOR_1X;
static float          g_kT = SCALE_FACTOR_1X;
static bool           g_initialized = false;

/* ---- I2C helpers ---- */

static bool read_reg(uint8_t reg, uint8_t *val)
{
    return board_i2c1_read_reg(DPS310_I2C_ADDR, reg, val);
}

static bool write_reg(uint8_t reg, uint8_t val)
{
    return board_i2c1_write_reg(DPS310_I2C_ADDR, reg, val);
}

static bool read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    return board_i2c1_read_burst(DPS310_I2C_ADDR, reg, buf, len);
}

/* ---- Calibration extraction ---- */

/* Sign-extend a value with 'bits' significant bits to int32_t. */
static int32_t sign_extend(uint32_t val, uint8_t bits)
{
    if ((val & (1UL << (bits - 1U))) != 0U) {
        val |= ~((1UL << bits) - 1UL);
    }
    return (int32_t)val;
}

/* Read and decode the 18 calibration coefficient bytes (0x10-0x21).
   Bit packing from Infineon DPS310 datasheet §4.9.                          */
static bool read_calibration(void)
{
    uint8_t raw[18];
    if (!read_burst(REG_COEF_BASE, raw, 18U)) {
        return false;
    }

    /* c0: 12-bit signed.  MSB in raw[0], LSB bits [7:4] in raw[1]. */
    g_cal.c0 = sign_extend(((uint32_t)raw[0] << 4U) | ((uint32_t)raw[1] >> 4U), 12U);

    /* c1: 12-bit signed.  MSB bits [3:0] in raw[1], LSB in raw[2]. */
    g_cal.c1 = sign_extend((((uint32_t)raw[1] & 0x0FU) << 8U) | (uint32_t)raw[2], 12U);

    /* c00: 20-bit signed. */
    g_cal.c00 = sign_extend(((uint32_t)raw[3] << 12U)
                           | ((uint32_t)raw[4] << 4U)
                           | ((uint32_t)raw[5] >> 4U), 20U);

    /* c10: 20-bit signed.  Top 4 bits from raw[5] bottom nibble. */
    g_cal.c10 = sign_extend((((uint32_t)raw[5] & 0x0FU) << 16U)
                           | ((uint32_t)raw[6] << 8U)
                           | (uint32_t)raw[7], 20U);

    /* c01-c30: 16-bit signed (big-endian). */
    g_cal.c01 = (int16_t)((uint16_t)raw[8]  << 8U | raw[9]);
    g_cal.c11 = (int16_t)((uint16_t)raw[10] << 8U | raw[11]);
    g_cal.c20 = (int16_t)((uint16_t)raw[12] << 8U | raw[13]);
    g_cal.c21 = (int16_t)((uint16_t)raw[14] << 8U | raw[15]);
    g_cal.c30 = (int16_t)((uint16_t)raw[16] << 8U | raw[17]);

    return true;
}

/* ---- Compensation formula (datasheet §4.9.1) ---- */

static float compensate_temperature(int32_t t_raw)
{
    /* Traw' = Traw / kT */
    const float Tsc = (float)t_raw / g_kT;
    /* Tcomp = c0/2 + c1 * Traw' */
    return 0.5f * (float)g_cal.c0 + (float)g_cal.c1 * Tsc;
}

static float compensate_pressure(int32_t p_raw, float Tsc)
{
    /* Praw' = Praw / kP */
    const float Psc = (float)p_raw / g_kP;
    /*
     * Pcomp = c00 + Praw'*(c10 + Praw'*(c20 + Praw'*c30))
     *       + Traw'*(c01 + Praw'*(c11 + Praw'*c21))
     */
    return (float)g_cal.c00
         + Psc * ((float)g_cal.c10
               + Psc * ((float)g_cal.c20
                      + Psc * (float)g_cal.c30))
         + Tsc * ((float)g_cal.c01
               + Psc * ((float)g_cal.c11
                      + Psc * (float)g_cal.c21));
}

/* ---- Public API ---- */

dps310_probe_t dps310_probe(void)
{
    dps310_probe_t probe = {
        .acked      = board_i2c1_probe(DPS310_I2C_ADDR),
        .id_read_ok = false,
        .product_id = 0,
    };

    if (probe.acked) {
        probe.id_read_ok = board_i2c1_read_reg(DPS310_I2C_ADDR,
                                               DPS310_PRODUCT_ID_REG,
                                               &probe.product_id);
    }
    return probe;
}

bool dps310_init(void)
{
    g_initialized = false;

    /* Soft reset. */
    if (!write_reg(REG_RESET, 0x89U)) {
        return false;
    }
    board_delay_ms(40U);  /* datasheet: sensor initialization takes ~40 ms */

    /* Wait for coefficients and sensor ready. */
    uint8_t meas_cfg;
    uint32_t timeout = 200U;
    do {
        board_delay_ms(5U);
        if (!read_reg(REG_MEAS_CFG, &meas_cfg)) {
            return false;
        }
    } while (((meas_cfg & (MEAS_COEF_RDY | MEAS_SENSOR_RDY)) !=
              (MEAS_COEF_RDY | MEAS_SENSOR_RDY)) && (--timeout > 0U));

    if (timeout == 0U) {
        return false;
    }

    if (!read_calibration()) {
        return false;
    }

    /* Configure: 32 Hz, 1x oversampling for both pressure and temperature.
       (1x → kP = kT = 524288, very low latency for logging)                */
    if (!write_reg(REG_PRS_CFG, PM_RATE_32HZ | PM_PRC_1X)) { return false; }
    if (!write_reg(REG_TMP_CFG, TMP_EXT | PM_RATE_32HZ | PM_PRC_1X)) { return false; }

    g_kP = SCALE_FACTOR_1X;
    g_kT = SCALE_FACTOR_1X;

    /* Start continuous background measurement (both pressure and temperature). */
    if (!write_reg(REG_MEAS_CFG, MEAS_BOTH_CONT)) {
        return false;
    }

    g_initialized = true;
    return true;
}

bool dps310_read(dps310_data_t *out)
{
    if (out == NULL) {
        return false;
    }
    out->valid = false;

    if (!g_initialized) {
        return false;
    }

    /* Check if new data is available. */
    uint8_t meas_cfg;
    if (!read_reg(REG_MEAS_CFG, &meas_cfg)) {
        return false;
    }
    if ((meas_cfg & (MEAS_TMP_RDY | MEAS_PRS_RDY)) !=
        (MEAS_TMP_RDY | MEAS_PRS_RDY)) {
        /* No new data yet — non-blocking return. */
        return false;
    }

    /* Read 6 bytes: PSR_B2/B1/B0 (pressure) + TMP_B2/B1/B0 (temperature). */
    uint8_t raw[6];
    if (!read_burst(REG_PSR_B2, raw, 6U)) {
        return false;
    }

    /* Reconstruct 24-bit signed raw values (big-endian, two's complement). */
    int32_t p_raw = (int32_t)(((uint32_t)raw[0] << 16U)
                             | ((uint32_t)raw[1] << 8U)
                             | (uint32_t)raw[2]);
    if ((p_raw & 0x00800000L) != 0L) { p_raw |= (int32_t)0xFF000000L; }

    int32_t t_raw = (int32_t)(((uint32_t)raw[3] << 16U)
                             | ((uint32_t)raw[4] << 8U)
                             | (uint32_t)raw[5]);
    if ((t_raw & 0x00800000L) != 0L) { t_raw |= (int32_t)0xFF000000L; }

    const float Tsc = (float)t_raw / g_kT;

    out->temperature_deg_c = compensate_temperature(t_raw);
    out->pressure_pa       = compensate_pressure(p_raw, Tsc);

    /* Approximate altitude from the ICAO barometric formula.
       Used only for logging/diagnostics — NOT used in the balance loop.     */
    out->altitude_m = 44330.0f
                    * (1.0f - powf(out->pressure_pa / STD_SEA_LEVEL_PA,
                                   1.0f / 5.255f));
    out->valid = true;
    return true;
}
