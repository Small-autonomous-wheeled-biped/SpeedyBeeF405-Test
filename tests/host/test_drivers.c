#include "fake_board.h"

#include "dps310.h"
#include "icm42688p.h"

#include <stdio.h>
#include <stdlib.h>

#define ASSERT_TRUE(expr) assert_true((expr), #expr, __FILE__, __LINE__)
#define ASSERT_FALSE(expr) assert_true(!(expr), "!(" #expr ")", __FILE__, __LINE__)
#define ASSERT_EQ_U8(actual, expected) assert_u32((actual), (expected), #actual, __FILE__, __LINE__)
#define ASSERT_EQ_U32(actual, expected) assert_u32((actual), (expected), #actual, __FILE__, __LINE__)
#define ASSERT_EQ_I16(actual, expected) assert_i32((actual), (expected), #actual, __FILE__, __LINE__)

static void assert_true(bool passed, const char *expr, const char *file, int line)
{
    if (!passed) {
        fprintf(stderr, "%s:%d: assertion failed: %s\n", file, line, expr);
        exit(1);
    }
}

static void assert_u32(uint32_t actual, uint32_t expected, const char *expr, const char *file, int line)
{
    if (actual != expected) {
        fprintf(stderr,
                "%s:%d: assertion failed: %s expected 0x%lX got 0x%lX\n",
                file,
                line,
                expr,
                (unsigned long)expected,
                (unsigned long)actual);
        exit(1);
    }
}

static void assert_i32(int32_t actual, int32_t expected, const char *expr, const char *file, int line)
{
    if (actual != expected) {
        fprintf(stderr,
                "%s:%d: assertion failed: %s expected %ld got %ld\n",
                file,
                line,
                expr,
                (long)expected,
                (long)actual);
        exit(1);
    }
}

static void test_icm42688p_read_whoami(void)
{
    fake_board_reset();
    fake_board_queue_spi_rx(0x00U);
    fake_board_queue_spi_rx(ICM42688P_WHOAMI_EXPECTED);

    const uint8_t whoami = icm42688p_read_whoami();
    const fake_board_state_t *state = fake_board_state();

    ASSERT_EQ_U8(whoami, ICM42688P_WHOAMI_EXPECTED);
    ASSERT_EQ_U32(state->spi_tx_count, 2U);
    ASSERT_EQ_U8(state->spi_tx[0], 0xF5U);
    ASSERT_EQ_U8(state->spi_tx[1], 0x00U);
    ASSERT_EQ_U32(state->cs_event_count, 2U);
    ASSERT_TRUE(state->cs_events[0]);
    ASSERT_FALSE(state->cs_events[1]);
}

static void test_icm42688p_init_rejects_wrong_whoami(void)
{
    fake_board_reset();
    fake_board_queue_spi_rx(0x00U);
    fake_board_queue_spi_rx(0x00U);

    ASSERT_FALSE(icm42688p_init());

    const fake_board_state_t *state = fake_board_state();
    ASSERT_EQ_U32(state->spi_tx_count, 2U);
    ASSERT_EQ_U8(state->spi_tx[0], 0xF5U);
    ASSERT_EQ_U32(state->delay_ms_total, 0U);
}

static void test_icm42688p_init_writes_power_management(void)
{
    fake_board_reset();
    fake_board_queue_spi_rx(0x00U);
    fake_board_queue_spi_rx(ICM42688P_WHOAMI_EXPECTED);
    fake_board_queue_spi_rx(0x00U);
    fake_board_queue_spi_rx(0x00U);

    ASSERT_TRUE(icm42688p_init());

    const fake_board_state_t *state = fake_board_state();
    ASSERT_EQ_U32(state->spi_tx_count, 4U);
    ASSERT_EQ_U8(state->spi_tx[0], 0xF5U);
    ASSERT_EQ_U8(state->spi_tx[1], 0x00U);
    ASSERT_EQ_U8(state->spi_tx[2], 0x4EU);
    ASSERT_EQ_U8(state->spi_tx[3], 0x0FU);
    ASSERT_EQ_U32(state->cs_event_count, 4U);
    ASSERT_TRUE(state->cs_events[0]);
    ASSERT_FALSE(state->cs_events[1]);
    ASSERT_TRUE(state->cs_events[2]);
    ASSERT_FALSE(state->cs_events[3]);
    ASSERT_EQ_U32(state->delay_ms_total, 50U);
}

static void test_icm42688p_read_sample_parses_big_endian_values(void)
{
    const uint8_t raw[] = {
        0x12U, 0x34U,
        0xFFU, 0xFEU,
        0x7FU, 0xFFU,
        0x80U, 0x00U,
        0x00U, 0x01U,
        0xFEU, 0xDCU,
        0x01U, 0x23U,
    };

    fake_board_reset();
    fake_board_queue_spi_rx(0x00U);
    for (size_t i = 0; i < sizeof(raw); i++) {
        fake_board_queue_spi_rx(raw[i]);
    }

    icm42688p_sample_t sample;
    ASSERT_TRUE(icm42688p_read_sample(&sample));

    const fake_board_state_t *state = fake_board_state();
    ASSERT_EQ_U32(state->spi_tx_count, 15U);
    ASSERT_EQ_U8(state->spi_tx[0], 0x9DU);
    for (size_t i = 1; i < state->spi_tx_count; i++) {
        ASSERT_EQ_U8(state->spi_tx[i], 0x00U);
    }

    ASSERT_EQ_I16(sample.temperature_raw, 0x1234);
    ASSERT_EQ_I16(sample.accel_x_raw, -2);
    ASSERT_EQ_I16(sample.accel_y_raw, 32767);
    ASSERT_EQ_I16(sample.accel_z_raw, -32768);
    ASSERT_EQ_I16(sample.gyro_x_raw, 1);
    ASSERT_EQ_I16(sample.gyro_y_raw, -292);
    ASSERT_EQ_I16(sample.gyro_z_raw, 0x0123);
}

static void test_icm42688p_read_sample_rejects_null(void)
{
    fake_board_reset();

    ASSERT_FALSE(icm42688p_read_sample(0));
    ASSERT_EQ_U32(fake_board_state()->spi_tx_count, 0U);
}

static void test_dps310_probe_no_ack_skips_product_id_read(void)
{
    fake_board_reset();
    fake_board_set_i2c_probe_result(false);

    const dps310_probe_t probe = dps310_probe();
    const fake_board_state_t *state = fake_board_state();

    ASSERT_FALSE(probe.acked);
    ASSERT_FALSE(probe.id_read_ok);
    ASSERT_EQ_U8(probe.product_id, 0U);
    ASSERT_EQ_U32(state->i2c_probe_count, 1U);
    ASSERT_EQ_U8(state->i2c_probe_addr, DPS310_I2C_ADDR);
    ASSERT_EQ_U32(state->i2c_read_count, 0U);
}

static void test_dps310_probe_reads_product_id_after_ack(void)
{
    fake_board_reset();
    fake_board_set_i2c_probe_result(true);
    fake_board_set_i2c_read_result(true, 0x10U);

    const dps310_probe_t probe = dps310_probe();
    const fake_board_state_t *state = fake_board_state();

    ASSERT_TRUE(probe.acked);
    ASSERT_TRUE(probe.id_read_ok);
    ASSERT_EQ_U8(probe.product_id, 0x10U);
    ASSERT_EQ_U32(state->i2c_probe_count, 1U);
    ASSERT_EQ_U8(state->i2c_probe_addr, DPS310_I2C_ADDR);
    ASSERT_EQ_U32(state->i2c_read_count, 1U);
    ASSERT_EQ_U8(state->i2c_read_addr, DPS310_I2C_ADDR);
    ASSERT_EQ_U8(state->i2c_read_reg, DPS310_PRODUCT_ID_REG);
}

static void test_dps310_probe_reports_failed_product_id_read(void)
{
    fake_board_reset();
    fake_board_set_i2c_probe_result(true);
    fake_board_set_i2c_read_result(false, 0x10U);

    const dps310_probe_t probe = dps310_probe();
    const fake_board_state_t *state = fake_board_state();

    ASSERT_TRUE(probe.acked);
    ASSERT_FALSE(probe.id_read_ok);
    ASSERT_EQ_U8(probe.product_id, 0U);
    ASSERT_EQ_U32(state->i2c_probe_count, 1U);
    ASSERT_EQ_U32(state->i2c_read_count, 1U);
    ASSERT_EQ_U8(state->i2c_read_addr, DPS310_I2C_ADDR);
    ASSERT_EQ_U8(state->i2c_read_reg, DPS310_PRODUCT_ID_REG);
}

int main(void)
{
    test_icm42688p_read_whoami();
    test_icm42688p_init_rejects_wrong_whoami();
    test_icm42688p_init_writes_power_management();
    test_icm42688p_read_sample_parses_big_endian_values();
    test_icm42688p_read_sample_rejects_null();
    test_dps310_probe_no_ack_skips_product_id_read();
    test_dps310_probe_reads_product_id_after_ack();
    test_dps310_probe_reports_failed_product_id_read();

    puts("driver_tests: all tests passed");
    return 0;
}
