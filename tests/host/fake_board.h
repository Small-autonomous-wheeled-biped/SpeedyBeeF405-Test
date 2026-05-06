#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FAKE_BOARD_MAX_SPI_BYTES 64U
#define FAKE_BOARD_MAX_CS_EVENTS 16U

typedef struct {
    uint8_t spi_tx[FAKE_BOARD_MAX_SPI_BYTES];
    size_t spi_tx_count;
    bool cs_events[FAKE_BOARD_MAX_CS_EVENTS];
    size_t cs_event_count;
    uint32_t delay_ms_total;
    uint8_t i2c_probe_addr;
    size_t i2c_probe_count;
    uint8_t i2c_read_addr;
    uint8_t i2c_read_reg;
    size_t i2c_read_count;
} fake_board_state_t;

void fake_board_reset(void);
void fake_board_queue_spi_rx(uint8_t value);
void fake_board_set_i2c_probe_result(bool result);
void fake_board_set_i2c_read_result(bool result, uint8_t value);
const fake_board_state_t *fake_board_state(void);
