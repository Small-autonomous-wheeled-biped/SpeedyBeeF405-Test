#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FAKE_BOARD_MAX_SPI_BYTES  256U
#define FAKE_BOARD_MAX_CS_EVENTS   64U
#define FAKE_BOARD_MAX_I2C_BURST   32U

typedef struct {
    uint8_t  spi_tx[FAKE_BOARD_MAX_SPI_BYTES];
    size_t   spi_tx_count;
    bool     cs_events[FAKE_BOARD_MAX_CS_EVENTS];
    size_t   cs_event_count;
    uint32_t delay_ms_total;
    uint32_t spi_speed_changes;   /* times board_spi1_set_speed was called  */
    uint32_t last_spi_speed_br;

    uint8_t  i2c_probe_addr;
    size_t   i2c_probe_count;
    uint8_t  i2c_read_addr;
    uint8_t  i2c_read_reg;
    size_t   i2c_read_count;
    uint8_t  i2c_burst_buf[FAKE_BOARD_MAX_I2C_BURST];
    uint8_t  i2c_burst_len;
    size_t   i2c_burst_count;
    bool     i2c_write_ok;

    uint32_t micros_value;
} fake_board_state_t;

void fake_board_reset(void);
void fake_board_queue_spi_rx(uint8_t value);
void fake_board_set_i2c_probe_result(bool result);
void fake_board_set_i2c_read_result(bool result, uint8_t value);
/* Queue len bytes for the next burst I2C read response. */
void fake_board_set_i2c_burst_result(bool result,
                                     const uint8_t *data, uint8_t len);
void fake_board_set_micros(uint32_t us);

const fake_board_state_t *fake_board_state(void);
