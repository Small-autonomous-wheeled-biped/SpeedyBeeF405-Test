#include "fake_board.h"
#include "board.h"

#include <string.h>

uint32_t SystemCoreClock = 168000000UL;

static fake_board_state_t g_state;
static uint8_t  g_spi_rx[FAKE_BOARD_MAX_SPI_BYTES];
static size_t   g_spi_rx_count;
static size_t   g_spi_rx_index;
static bool     g_i2c_probe_result;
static bool     g_i2c_read_result;
static uint8_t  g_i2c_read_value;
static bool     g_i2c_burst_result;
static uint8_t  g_i2c_burst_data[FAKE_BOARD_MAX_I2C_BURST];
static uint8_t  g_i2c_burst_len;

void fake_board_reset(void)
{
    memset(&g_state, 0, sizeof(g_state));
    memset(g_spi_rx, 0, sizeof(g_spi_rx));
    g_spi_rx_count       = 0;
    g_spi_rx_index       = 0;
    g_i2c_probe_result   = false;
    g_i2c_read_result    = false;
    g_i2c_read_value     = 0;
    g_i2c_burst_result   = false;
    g_i2c_burst_len      = 0;
    memset(g_i2c_burst_data, 0, sizeof(g_i2c_burst_data));
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
    g_i2c_read_value  = value;
}

void fake_board_set_i2c_burst_result(bool result,
                                      const uint8_t *data, uint8_t len)
{
    g_i2c_burst_result = result;
    g_i2c_burst_len    = (len < FAKE_BOARD_MAX_I2C_BURST) ? len : FAKE_BOARD_MAX_I2C_BURST;
    if (data != NULL) {
        memcpy(g_i2c_burst_data, data, g_i2c_burst_len);
    }
}

void fake_board_set_micros(uint32_t us)
{
    g_state.micros_value = us;
}

const fake_board_state_t *fake_board_state(void)
{
    return &g_state;
}

/* ---- board.h stubs ---- */

void board_delay_ms(uint32_t ms)
{
    g_state.delay_ms_total += ms;
}

void board_delay_us(uint32_t us) { (void)us; }

uint32_t board_millis(void) { return g_state.delay_ms_total; }

uint32_t board_micros(void) { return g_state.micros_value; }

void board_spi1_set_speed(uint32_t br)
{
    g_state.spi_speed_changes++;
    g_state.last_spi_speed_br = br;
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

bool board_i2c1_write_reg(uint8_t addr7, uint8_t reg, uint8_t value)
{
    (void)addr7; (void)reg; (void)value;
    return g_state.i2c_write_ok;
}

bool board_i2c1_read_reg(uint8_t addr7, uint8_t reg, uint8_t *value)
{
    g_state.i2c_read_addr = addr7;
    g_state.i2c_read_reg  = reg;
    g_state.i2c_read_count++;
    if ((value == NULL) || !g_i2c_read_result) {
        return false;
    }
    *value = g_i2c_read_value;
    return true;
}

bool board_i2c1_read_burst(uint8_t addr7, uint8_t reg,
                             uint8_t *buf, uint8_t len)
{
    g_state.i2c_read_addr = addr7;
    g_state.i2c_read_reg  = reg;
    g_state.i2c_burst_count++;
    if ((buf == NULL) || !g_i2c_burst_result) {
        return false;
    }
    const uint8_t copy_len = (len < g_i2c_burst_len) ? len : g_i2c_burst_len;
    memcpy(buf, g_i2c_burst_data, copy_len);
    g_state.i2c_burst_len = copy_len;
    memcpy(g_state.i2c_burst_buf, g_i2c_burst_data, copy_len);
    return true;
}

/* Stubs for functions not exercised by unit tests. */
void board_led_set(bool on) { (void)on; }
void board_led_toggle(void) {}
void board_buzzer_set(bool on) { (void)on; }
void board_log_char(char c) { (void)c; }
void board_log(const char *t) { (void)t; }
void board_log_line(const char *t) { (void)t; }
void board_log_u32(uint32_t v) { (void)v; }
void board_log_i32(int32_t v) { (void)v; }
void board_log_hex8(uint8_t v) { (void)v; }
void board_log_hex16(uint16_t v) { (void)v; }
void board_log_hex32(uint32_t v) { (void)v; }
bool board_uart1_write(const uint8_t *d, size_t l) { (void)d; (void)l; return true; }
bool board_uart1_tx_busy(void) { return false; }
void board_uart1_irq(void) {}
/* USB CDC stubs (usb_cdc.h not included in host tests) */
void usb_cdc_putchar(char c) { (void)c; }
