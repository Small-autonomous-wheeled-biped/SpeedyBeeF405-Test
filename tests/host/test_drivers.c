#include "fake_board.h"
#include "dps310.h"
#include "icm42688p.h"

#include <stdio.h>
#include <stdlib.h>

#define ASSERT_TRUE(expr) assert_true((expr), #expr, __FILE__, __LINE__)
#define ASSERT_FALSE(expr) assert_true(!(expr), "!(" #expr ")", __FILE__, __LINE__)
#define ASSERT_EQ_U8(actual, expected) assert_u32((uint32_t)(actual), (uint32_t)(expected), #actual, __FILE__, __LINE__)
#define ASSERT_EQ_U32(actual, expected) assert_u32((actual), (expected), #actual, __FILE__, __LINE__)
#define ASSERT_EQ_I16(actual, expected) assert_i32((int32_t)(actual), (int32_t)(expected), #actual, __FILE__, __LINE__)

static void assert_true(bool passed, const char *expr, const char *file, int line)
{
    if (!passed) {
        fprintf(stderr, "%s:%d: FAILED: %s\n", file, line, expr);
        exit(1);
    }
}

static void assert_u32(uint32_t actual, uint32_t expected,
                        const char *expr, const char *file, int line)
{
    if (actual != expected) {
        fprintf(stderr, "%s:%d: FAILED: %s expected 0x%lX got 0x%lX\n",
                file, line, expr,
                (unsigned long)expected, (unsigned long)actual);
        exit(1);
    }
}

static void assert_i32(int32_t actual, int32_t expected,
                        const char *expr, const char *file, int line)
{
    if (actual != expected) {
        fprintf(stderr, "%s:%d: FAILED: %s expected %ld got %ld\n",
                file, line, expr, (long)expected, (long)actual);
        exit(1);
    }
}

/* ---- ICM42688-P tests ---- */

static void test_icm42688p_read_whoami(void)
{
    fake_board_reset();
    fake_board_queue_spi_rx(0x00U);
    fake_board_queue_spi_rx(ICM42688P_WHOAMI_EXPECTED);

    const uint8_t whoami = icm42688p_read_whoami();
    const fake_board_state_t *s = fake_board_state();

    ASSERT_EQ_U8(whoami, ICM42688P_WHOAMI_EXPECTED);
    ASSERT_EQ_U32(s->spi_tx_count, 2U);
    ASSERT_EQ_U8(s->spi_tx[0], 0xF5U);   /* WHO_AM_I | READ = 0x75 | 0x80 */
    ASSERT_EQ_U8(s->spi_tx[1], 0x00U);
}

static void test_icm42688p_init_rejects_wrong_whoami(void)
{
    fake_board_reset();
    /* First WHO_AM_I check returns 0x00 (wrong device). */
    fake_board_queue_spi_rx(0x00U);
    fake_board_queue_spi_rx(0x00U);

    ASSERT_FALSE(icm42688p_init(ICM42688P_GYRO_FS_1000DPS,
                                ICM42688P_ACCEL_FS_4G,
                                ICM42688P_ODR_1000HZ));

    const fake_board_state_t *s = fake_board_state();
    ASSERT_EQ_U32(s->spi_tx_count, 2U);    /* only WHO_AM_I attempted */
    ASSERT_EQ_U8(s->spi_tx[0], 0xF5U);
    ASSERT_EQ_U32(s->delay_ms_total, 0U);
}

/*
 * Full init sequence (12 SPI transactions × 2 bytes = 24 bytes):
 *   [0-1]   WHO_AM_I check (read)
 *   [2-3]   DEVICE_CONFIG soft reset (write)
 *   [4-5]   WHO_AM_I verify after reset (read)
 *   [6-7]   SIGNAL_PATH_RESET flush FIFO (write)
 *   [8-9]   GYRO_CONFIG0 (write)
 *   [10-11] ACCEL_CONFIG0 (write)
 *   [12-13] GYRO_ACCEL_CONFIG0 (write)
 *   [14-15] FIFO_CONFIG1 (write)
 *   [16-17] FIFO_CONFIG (write)
 *   [18-19] PWR_MGMT0 (write)
 *   Speed changes: INIT_BR at start, RUN_BR after stabilisation.
 */
static void test_icm42688p_init_full_sequence(void)
{
    fake_board_reset();

    /* Queue RX bytes: WHO_AM_I (2 reads) need a response; writes ignore RX. */
    fake_board_queue_spi_rx(0x00U);              /* [0] addr byte RX: ignored  */
    fake_board_queue_spi_rx(ICM42688P_WHOAMI_EXPECTED); /* [1] WHO_AM_I value  */
    /* write DEVICE_CONFIG: no meaningful RX needed */
    fake_board_queue_spi_rx(0x00U);
    fake_board_queue_spi_rx(0x00U);
    /* WHO_AM_I verify */
    fake_board_queue_spi_rx(0x00U);
    fake_board_queue_spi_rx(ICM42688P_WHOAMI_EXPECTED);

    ASSERT_TRUE(icm42688p_init(ICM42688P_GYRO_FS_1000DPS,
                                ICM42688P_ACCEL_FS_4G,
                                ICM42688P_ODR_1000HZ));

    const fake_board_state_t *s = fake_board_state();

    ASSERT_EQ_U32(s->spi_tx_count, 20U);

    /* [0-1]: WHO_AM_I read */
    ASSERT_EQ_U8(s->spi_tx[0], 0xF5U);  /* 0x75 | 0x80 */

    /* [2-3]: DEVICE_CONFIG write (soft reset) */
    ASSERT_EQ_U8(s->spi_tx[2], 0x11U);  /* DEVICE_CONFIG register */
    ASSERT_EQ_U8(s->spi_tx[3], 0x01U);  /* SOFT_RESET_CONFIG = 1   */

    /* [4-5]: WHO_AM_I verify */
    ASSERT_EQ_U8(s->spi_tx[4], 0xF5U);

    /* [6-7]: SIGNAL_PATH_RESET */
    ASSERT_EQ_U8(s->spi_tx[6], 0x4BU);  /* SIGNAL_PATH_RESET       */
    ASSERT_EQ_U8(s->spi_tx[7], 0x02U);  /* FIFO_FLUSH              */

    /* [8-9]: GYRO_CONFIG0: FS=1000dps(001), ODR=1kHz(0110) = 0x26 */
    ASSERT_EQ_U8(s->spi_tx[8], 0x4FU);  /* GYRO_CONFIG0            */
    ASSERT_EQ_U8(s->spi_tx[9], 0x26U);

    /* [10-11]: ACCEL_CONFIG0: FS=4g(010), ODR=1kHz(0110) = 0x46 */
    ASSERT_EQ_U8(s->spi_tx[10], 0x50U); /* ACCEL_CONFIG0           */
    ASSERT_EQ_U8(s->spi_tx[11], 0x46U);

    /* [12-13]: GYRO_ACCEL_CONFIG0: filter BW = ODR/4 for both = 0x11 */
    ASSERT_EQ_U8(s->spi_tx[12], 0x52U);
    ASSERT_EQ_U8(s->spi_tx[13], 0x11U);

    /* [14-15]: FIFO_CONFIG1: gyro+accel = 0x03 */
    ASSERT_EQ_U8(s->spi_tx[14], 0x5FU);
    ASSERT_EQ_U8(s->spi_tx[15], 0x03U);

    /* [16-17]: FIFO_CONFIG: stream mode = 0x40 */
    ASSERT_EQ_U8(s->spi_tx[16], 0x16U);
    ASSERT_EQ_U8(s->spi_tx[17], 0x40U);

    /* [18-19]: PWR_MGMT0: both sensors LN = 0x0F */
    ASSERT_EQ_U8(s->spi_tx[18], 0x4EU);
    ASSERT_EQ_U8(s->spi_tx[19], 0x0FU);

    /* Delays: 2 ms + 1 ms + 50 ms = 53 ms */
    ASSERT_EQ_U32(s->delay_ms_total, 53U);

    /* Speed changed at least twice: INIT → RUN */
    ASSERT_TRUE(s->spi_speed_changes >= 2U);
}

static void test_icm42688p_read_sample_parses_big_endian_values(void)
{
    /* Seed the WHO_AM_I and init bytes so we can configure scale factors. */
    fake_board_reset();
    for (int i = 0; i < 6; i++) { fake_board_queue_spi_rx(0x00U); }
    fake_board_queue_spi_rx(0x00U);
    fake_board_queue_spi_rx(ICM42688P_WHOAMI_EXPECTED); /* first WHO_AM_I */
    fake_board_queue_spi_rx(0x00U);
    fake_board_queue_spi_rx(0x00U);
    fake_board_queue_spi_rx(0x00U);
    fake_board_queue_spi_rx(ICM42688P_WHOAMI_EXPECTED); /* verify         */
    (void)icm42688p_init(ICM42688P_GYRO_FS_1000DPS,
                         ICM42688P_ACCEL_FS_4G,
                         ICM42688P_ODR_1000HZ);

    /* Now test read_sample. */
    fake_board_reset();

    const uint8_t raw[] = {
        0x12U, 0x34U,   /* temperature_raw */
        0xFFU, 0xFEU,   /* accel_x = -2    */
        0x7FU, 0xFFU,   /* accel_y = 32767 */
        0x80U, 0x00U,   /* accel_z = -32768*/
        0x00U, 0x01U,   /* gyro_x  = 1     */
        0xFEU, 0xDCU,   /* gyro_y  = -292  */
        0x01U, 0x23U,   /* gyro_z  = 291   */
    };

    fake_board_queue_spi_rx(0x00U);  /* addr RX */
    for (size_t i = 0; i < sizeof(raw); i++) {
        fake_board_queue_spi_rx(raw[i]);
    }

    icm42688p_sample_t sample;
    ASSERT_TRUE(icm42688p_read_sample(&sample));

    ASSERT_EQ_I16(sample.temperature_raw, 0x1234);
    ASSERT_EQ_I16(sample.accel_x_raw,    -2);
    ASSERT_EQ_I16(sample.accel_y_raw,     32767);
    ASSERT_EQ_I16(sample.accel_z_raw,    -32768);
    ASSERT_EQ_I16(sample.gyro_x_raw,      1);
    ASSERT_EQ_I16(sample.gyro_y_raw,     -292);
    ASSERT_EQ_I16(sample.gyro_z_raw,      0x0123);
}

static void test_icm42688p_read_sample_rejects_null(void)
{
    fake_board_reset();
    ASSERT_FALSE(icm42688p_read_sample(NULL));
    ASSERT_EQ_U32(fake_board_state()->spi_tx_count, 0U);
}

/* ---- DPS310 tests ---- */

static void test_dps310_probe_no_ack_skips_product_id_read(void)
{
    fake_board_reset();
    fake_board_set_i2c_probe_result(false);

    const dps310_probe_t probe = dps310_probe();
    const fake_board_state_t *s = fake_board_state();

    ASSERT_FALSE(probe.acked);
    ASSERT_FALSE(probe.id_read_ok);
    ASSERT_EQ_U8(probe.product_id, 0U);
    ASSERT_EQ_U32(s->i2c_probe_count, 1U);
    ASSERT_EQ_U8(s->i2c_probe_addr, DPS310_I2C_ADDR);
    ASSERT_EQ_U32(s->i2c_read_count, 0U);
}

static void test_dps310_probe_reads_product_id_after_ack(void)
{
    fake_board_reset();
    fake_board_set_i2c_probe_result(true);
    fake_board_set_i2c_read_result(true, 0x10U);

    const dps310_probe_t probe = dps310_probe();
    const fake_board_state_t *s = fake_board_state();

    ASSERT_TRUE(probe.acked);
    ASSERT_TRUE(probe.id_read_ok);
    ASSERT_EQ_U8(probe.product_id, 0x10U);
    ASSERT_EQ_U32(s->i2c_probe_count, 1U);
    ASSERT_EQ_U8(s->i2c_probe_addr, DPS310_I2C_ADDR);
    ASSERT_EQ_U32(s->i2c_read_count, 1U);
    ASSERT_EQ_U8(s->i2c_read_reg, DPS310_PRODUCT_ID_REG);
}

static void test_dps310_probe_reports_failed_product_id_read(void)
{
    fake_board_reset();
    fake_board_set_i2c_probe_result(true);
    fake_board_set_i2c_read_result(false, 0x10U);

    const dps310_probe_t probe = dps310_probe();
    const fake_board_state_t *s = fake_board_state();

    ASSERT_TRUE(probe.acked);
    ASSERT_FALSE(probe.id_read_ok);
    ASSERT_EQ_U8(probe.product_id, 0U);
    ASSERT_EQ_U32(s->i2c_probe_count, 1U);
    ASSERT_EQ_U32(s->i2c_read_count, 1U);
    ASSERT_EQ_U8(s->i2c_read_reg, DPS310_PRODUCT_ID_REG);
}

int main(void)
{
    test_icm42688p_read_whoami();
    test_icm42688p_init_rejects_wrong_whoami();
    test_icm42688p_init_full_sequence();
    test_icm42688p_read_sample_parses_big_endian_values();
    test_icm42688p_read_sample_rejects_null();
    test_dps310_probe_no_ack_skips_product_id_read();
    test_dps310_probe_reads_product_id_after_ack();
    test_dps310_probe_reports_failed_product_id_read();

    puts("test_drivers: all tests passed");
    return 0;
}
