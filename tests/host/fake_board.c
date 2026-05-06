#include "fake_board.h"

#include "board.h"

#include <string.h>

uint32_t SystemCoreClock = 16000000UL;

static fake_board_state_t g_state;
static uint8_t g_spi_rx[FAKE_BOARD_MAX_SPI_BYTES];
static size_t g_spi_rx_count;
static size_t g_spi_rx_index;
static bool g_i2c_probe_result;
static bool g_i2c_read_result;
static uint8_t g_i2c_read_value;

void fake_board_reset(void)
{
    memset(&g_state, 0, sizeof(g_state));
    memset(g_spi_rx, 0, sizeof(g_spi_rx));
    g_spi_rx_count = 0;
    g_spi_rx_index = 0;
    g_i2c_probe_result = false;
    g_i2c_read_result = false;
    g_i2c_read_value = 0;
}

void fake_board_queue_spi_rx(uint8_t value)
{
    if (g_spi_rx_count < FAKE_BOARD_MAX_SPI_BYTES) {
        g_spi_rx[g_spi_rx_count++] = value;
    }
}

void fake_board_set_i2c_probe_result(bool result)
{
    g_i2c_probe_result = result;
}

void fake_board_set_i2c_read_result(bool result, uint8_t value)
{
    g_i2c_read_result = result;
    g_i2c_read_value = value;
}

const fake_board_state_t *fake_board_state(void)
{
    return &g_state;
}

void board_delay_ms(uint32_t ms)
{
    g_state.delay_ms_total += ms;
}

uint8_t board_spi1_transfer(uint8_t value)
{
    if (g_state.spi_tx_count < FAKE_BOARD_MAX_SPI_BYTES) {
        g_state.spi_tx[g_state.spi_tx_count++] = value;
    }

    if (g_spi_rx_index < g_spi_rx_count) {
        return g_spi_rx[g_spi_rx_index++];
    }

    return 0;
}

void board_imu_cs_set(bool selected)
{
    if (g_state.cs_event_count < FAKE_BOARD_MAX_CS_EVENTS) {
        g_state.cs_events[g_state.cs_event_count++] = selected;
    }
}

bool board_i2c1_probe(uint8_t addr7)
{
    g_state.i2c_probe_addr = addr7;
    g_state.i2c_probe_count++;
    return g_i2c_probe_result;
}

bool board_i2c1_read_reg(uint8_t addr7, uint8_t reg, uint8_t *value)
{
    g_state.i2c_read_addr = addr7;
    g_state.i2c_read_reg = reg;
    g_state.i2c_read_count++;

    if ((value == 0) || !g_i2c_read_result) {
        return false;
    }

    *value = g_i2c_read_value;
    return true;
}
