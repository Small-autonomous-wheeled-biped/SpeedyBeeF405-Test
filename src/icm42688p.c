#include "icm42688p.h"
#include "board.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * ICM-42688-P register map (Bank 0).
 * Values and bit definitions from TDK ICM-42688-P datasheet Rev 1.4.
 * Open-source ArduPilot and Cleanflight drivers were used as cross-references
 * for FIFO header format; the implementation below is original.
 */

/* -- Bank 0 registers -- */
#define REG_DEVICE_CONFIG       0x11U
#define REG_SIGNAL_PATH_RESET   0x4BU
#define REG_INT_CONFIG          0x14U
#define REG_FIFO_CONFIG         0x16U   /* bits [7:6]: FIFO_MODE */
#define REG_TEMP_DATA1          0x1DU
#define REG_ACCEL_DATA_X1       0x1FU
#define REG_INT_STATUS          0x2DU
#define REG_FIFO_COUNT_H        0x2EU
#define REG_FIFO_DATA           0x30U
#define REG_PWR_MGMT0           0x4EU
#define REG_GYRO_CONFIG0        0x4FU
#define REG_ACCEL_CONFIG0       0x50U
#define REG_GYRO_ACCEL_CONFIG0  0x52U
#define REG_FIFO_CONFIG1        0x5FU
#define REG_WHO_AM_I            0x75U
#define REG_BANK_SEL            0x76U

/* SPI read flag */
#define SPI_READ                0x80U

/* PWR_MGMT0: accel LN + gyro LN, not idle */
#define PWR_ACCEL_LN            (3U << 0U)
#define PWR_GYRO_LN             (3U << 2U)
#define PWR_MGMT0_BOTH_LN       (PWR_ACCEL_LN | PWR_GYRO_LN)

/* FIFO_CONFIG: stream-to-FIFO mode (bits [7:6] = 01) */
#define FIFO_MODE_STREAM        (1U << 6U)

/* FIFO_CONFIG1: gyro + accel enabled, 16-bit (no HiRes, no timestamp) */
#define FIFO_GYRO_EN            BIT(1)
#define FIFO_ACCEL_EN           BIT(0)
#define FIFO_TEMP_EN            BIT(3)
#define FIFO_CONFIG1_GYRO_ACCEL (FIFO_GYRO_EN | FIFO_ACCEL_EN)

/*
 * FIFO packet header (16-bit accel + gyro mode):
 *   bit 7 = 1 → empty/marker packet, skip
 *   bits [5:4]: accel encoding  (0b10 = 16-bit present)
 *   bits [3:2]: gyro  encoding  (0b10 = 16-bit present)
 *   bit 6:      ODR-change marker (packet still contains data)
 *   bits [1:0]: timestamp flags (we do not enable timestamps)
 *
 * Expected header for 16-bit accel+gyro without timestamp:
 *   0b 0x xx 10 10 00 = 0x28 (normal) or 0x68 (with ODR-change mark)
 */
#define FIFO_HDR_MSG_MASK       0x80U   /* non-data marker if set */
#define FIFO_HDR_ACCEL_16BIT    0x20U   /* bits [5:4] = 0b10 */
#define FIFO_HDR_GYRO_16BIT     0x08U   /* bits [3:2] = 0b10 */
#define FIFO_HDR_ACCEL_MASK     0x30U
#define FIFO_HDR_GYRO_MASK      0x0CU
#define FIFO_PACKET_SIZE        13U     /* header(1) + accel(6) + gyro(6) */

/* SIGNAL_PATH_RESET: flush FIFO */
#define SIGNAL_PATH_RESET_FIFO_FLUSH  0x02U

/* Physical constants */
#define GRAVITY_M_S2            9.80665f

/* ---- Module state ---- */
static uint8_t          g_gyro_fs;
static uint8_t          g_accel_fs;
static float            g_gyro_scale;    /* rad/s per LSB */
static float            g_accel_scale;   /* m/s² per LSB  */
static icm42688p_stats_t g_stats;

/* ---- SPI helpers ---- */

static uint8_t read_reg(uint8_t reg)
{
    board_imu_cs_set(true);
    (void)board_spi1_transfer(reg | SPI_READ);
    const uint8_t value = board_spi1_transfer(0x00U);
    board_imu_cs_set(false);
    return value;
}

static void write_reg(uint8_t reg, uint8_t value)
{
    board_imu_cs_set(true);
    (void)board_spi1_transfer(reg & (uint8_t)~SPI_READ);
    (void)board_spi1_transfer(value);
    board_imu_cs_set(false);
}

static void read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    board_imu_cs_set(true);
    (void)board_spi1_transfer(reg | SPI_READ);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = board_spi1_transfer(0x00U);
    }
    board_imu_cs_set(false);
}

static int16_t be_i16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] << 8U | data[1]);
}

/* ---- Scale factor computation ---- */

static float gyro_scale_from_fs(uint8_t fs)
{
    /* Sensitivity (LSB/dps) from datasheet §3.1 table:
       FS 0 = 2000 dps → 16.384 LSB/dps
       FS 1 = 1000 dps → 32.768 LSB/dps
       FS 2 =  500 dps → 65.536 LSB/dps
       FS 3 =  250 dps → 131.072 LSB/dps                                    */
    static const float dps_full_scale[4] = { 2000.0f, 1000.0f, 500.0f, 250.0f };
    const float fs_idx = (fs < 4U) ? fs : 1U;
    /* scale = full_range_dps / 2^15 * (π/180) */
    return dps_full_scale[(uint8_t)fs_idx] / 32768.0f * (float)M_PI / 180.0f;
}

static float accel_scale_from_fs(uint8_t fs)
{
    /* FS 0 = ±16 g, FS 1 = ±8 g, FS 2 = ±4 g, FS 3 = ±2 g */
    static const float g_full_scale[4] = { 16.0f, 8.0f, 4.0f, 2.0f };
    const float fs_idx = (fs < 4U) ? fs : 2U;
    return g_full_scale[(uint8_t)fs_idx] / 32768.0f * GRAVITY_M_S2;
}

/* ---- Public API ---- */

uint8_t icm42688p_read_whoami(void)
{
    return read_reg(REG_WHO_AM_I);
}

bool icm42688p_init(uint8_t gyro_fs, uint8_t accel_fs, uint8_t odr_idx)
{
    memset(&g_stats, 0, sizeof(g_stats));

    /* 1. Verify device identity. */
    board_spi1_set_speed(BOARD_SPI_INIT_BR);
    if (icm42688p_read_whoami() != ICM42688P_WHOAMI_EXPECTED) {
        return false;
    }

    /* 2. Soft reset (DEVICE_CONFIG bit 0). */
    write_reg(REG_DEVICE_CONFIG, 0x01U);
    board_delay_ms(2U);   /* datasheet: 1 ms after reset before register access */

    /* 3. Verify device is responsive after reset. */
    if (icm42688p_read_whoami() != ICM42688P_WHOAMI_EXPECTED) {
        return false;
    }

    /* 4. Flush FIFO and reset signal path. */
    write_reg(REG_SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_FIFO_FLUSH);
    board_delay_ms(1U);

    /* 5. Clamp FS and ODR indices to valid ranges. */
    if (gyro_fs > 3U)  { gyro_fs  = ICM42688P_GYRO_FS_1000DPS; }
    if (accel_fs > 3U) { accel_fs = ICM42688P_ACCEL_FS_4G; }
    if ((odr_idx < 1U) || (odr_idx > 7U)) { odr_idx = ICM42688P_ODR_1000HZ; }

    /* 6. Configure gyro: FS + ODR.
       GYRO_CONFIG0 bits [7:5] = FS_SEL, bits [3:0] = ODR.                  */
    write_reg(REG_GYRO_CONFIG0, (uint8_t)((gyro_fs << 5U) | odr_idx));

    /* 7. Configure accel: FS + ODR. */
    write_reg(REG_ACCEL_CONFIG0, (uint8_t)((accel_fs << 5U) | odr_idx));

    /* 8. UI filter bandwidths: accel BW = ODR/4, gyro BW = ODR/4.
       GYRO_ACCEL_CONFIG0 bits [7:4] = ACCEL_UI_FILT_BW, [3:0] = GYRO_UI_FILT_BW.
       Value 0x01 = ODR/4 for both axes.  Balances latency vs. noise.        */
    write_reg(REG_GYRO_ACCEL_CONFIG0, 0x11U);

    /* 9. Configure FIFO: gyro + accel only (no HiRes, no timestamp). */
    write_reg(REG_FIFO_CONFIG1, FIFO_CONFIG1_GYRO_ACCEL);

    /* 10. Enable FIFO in stream-to-FIFO mode. */
    write_reg(REG_FIFO_CONFIG, FIFO_MODE_STREAM);

    /* 11. Enable sensors in low-noise mode. */
    write_reg(REG_PWR_MGMT0, PWR_MGMT0_BOTH_LN);

    /* 12. Wait for sensors to stabilize before reading.
           Datasheet recommends ≥200 µs after enabling, we give 50 ms.      */
    board_delay_ms(50U);

    /* 13. Switch to high-speed SPI for FIFO burst reads. */
    board_spi1_set_speed(BOARD_SPI_RUN_BR);

    /* 14. Record configuration for scale factor computation. */
    g_gyro_fs    = gyro_fs;
    g_accel_fs   = accel_fs;
    g_gyro_scale = gyro_scale_from_fs(gyro_fs);
    g_accel_scale = accel_scale_from_fs(accel_fs);

    return true;
}

void icm42688p_flush_fifo(void)
{
    write_reg(REG_SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_FIFO_FLUSH);
    board_delay_ms(1U);
}

uint16_t icm42688p_read_fifo(icm42688p_sample_t *out, uint16_t max_samples)
{
    if ((out == NULL) || (max_samples == 0U)) {
        return 0U;
    }

    /* Read FIFO byte count (big-endian 16-bit). */
    uint8_t count_buf[2];
    read_burst(REG_FIFO_COUNT_H, count_buf, 2U);
    const uint16_t fifo_bytes = (uint16_t)((uint16_t)count_buf[0] << 8U | count_buf[1]);

    if (fifo_bytes == 0U) {
        return 0U;
    }

    /* FIFO capacity is 2048 bytes; flag overflow if near full. */
    if (fifo_bytes >= 2040U) {
        g_stats.fifo_overflow++;
    }

    const uint16_t num_packets = fifo_bytes / FIFO_PACKET_SIZE;
    const uint16_t to_read     = (num_packets < max_samples) ? num_packets : max_samples;

    uint16_t valid = 0U;
    uint8_t  pkt[FIFO_PACKET_SIZE];

    for (uint16_t i = 0; i < to_read; i++) {
        read_burst(REG_FIFO_DATA, pkt, FIFO_PACKET_SIZE);
        g_stats.total_reads++;

        const uint8_t hdr = pkt[0];

        /* Skip marker / empty packets (bit 7 set). */
        if ((hdr & FIFO_HDR_MSG_MASK) != 0U) {
            g_stats.fifo_bad_header++;
            continue;
        }

        /* Expect both 16-bit accel and gyro to be present. */
        if (((hdr & FIFO_HDR_ACCEL_MASK) != FIFO_HDR_ACCEL_16BIT) ||
            ((hdr & FIFO_HDR_GYRO_MASK)  != FIFO_HDR_GYRO_16BIT)) {
            g_stats.fifo_bad_header++;
            continue;
        }

        /* Parse: bytes 1-6 = accel (X,Y,Z big-endian), 7-12 = gyro. */
        out[valid].accel_x_raw    = be_i16(&pkt[1]);
        out[valid].accel_y_raw    = be_i16(&pkt[3]);
        out[valid].accel_z_raw    = be_i16(&pkt[5]);
        out[valid].gyro_x_raw     = be_i16(&pkt[7]);
        out[valid].gyro_y_raw     = be_i16(&pkt[9]);
        out[valid].gyro_z_raw     = be_i16(&pkt[11]);
        /* Temperature not in this FIFO config; leave raw at 0. */
        out[valid].temperature_raw = 0;

        valid++;
    }

    if (to_read < num_packets) {
        /* Caller's buffer was too small; unconsumed packets will be stale. */
        g_stats.sample_drops += (uint32_t)(num_packets - to_read);
    }

    return valid;
}

bool icm42688p_read_sample(icm42688p_sample_t *sample)
{
    if (sample == NULL) {
        return false;
    }

    uint8_t raw[14];
    read_burst(REG_TEMP_DATA1, raw, sizeof(raw));

    sample->temperature_raw = be_i16(&raw[0]);
    sample->accel_x_raw     = be_i16(&raw[2]);
    sample->accel_y_raw     = be_i16(&raw[4]);
    sample->accel_z_raw     = be_i16(&raw[6]);
    sample->gyro_x_raw      = be_i16(&raw[8]);
    sample->gyro_y_raw      = be_i16(&raw[10]);
    sample->gyro_z_raw      = be_i16(&raw[12]);
    return true;
}

void icm42688p_to_physical(const icm42688p_sample_t *raw,
                            icm42688p_data_t *out)
{
    if ((raw == NULL) || (out == NULL)) {
        return;
    }

    out->gyro_x_rad_s = (float)raw->gyro_x_raw  * g_gyro_scale;
    out->gyro_y_rad_s = (float)raw->gyro_y_raw  * g_gyro_scale;
    out->gyro_z_rad_s = (float)raw->gyro_z_raw  * g_gyro_scale;

    out->accel_x_m_s2 = (float)raw->accel_x_raw * g_accel_scale;
    out->accel_y_m_s2 = (float)raw->accel_y_raw * g_accel_scale;
    out->accel_z_m_s2 = (float)raw->accel_z_raw * g_accel_scale;

    /* Temperature formula from datasheet §3.3:
       T_deg_c = (TEMP_DATA / 132.48) + 25  (for TEMP_DATA as signed 16-bit) */
    out->temperature_deg_c = (float)raw->temperature_raw / 132.48f + 25.0f;

    out->timestamp_us = 0U;  /* filled by caller who knows the read time */
}

const icm42688p_stats_t *icm42688p_stats(void)
{
    return &g_stats;
}

float icm42688p_gyro_scale_rad_s(void)
{
    return g_gyro_scale;
}

float icm42688p_accel_scale_m_s2(void)
{
    return g_accel_scale;
}
