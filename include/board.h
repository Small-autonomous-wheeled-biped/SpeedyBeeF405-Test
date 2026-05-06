#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stm32f405xx_min.h"

#define BOARD_PWM_OUTPUTS 5U
#define BOARD_UART_LOG_BAUD 115200UL

extern uint32_t SystemCoreClock;

void board_clock_init(void);
void board_gpio_init(void);
void board_systick_init(void);
void board_usart3_init(uint32_t baud);
void board_spi1_init(void);
void board_i2c1_init(void);
void board_adc1_init(void);
void board_pwm_init(void);

uint32_t board_millis(void);
void board_delay_ms(uint32_t ms);

void board_led_set(bool on);
void board_led_toggle(void);
void board_buzzer_set(bool on);

void board_log_char(char c);
void board_log(const char *text);
void board_log_line(const char *text);
void board_log_u32(uint32_t value);
void board_log_i32(int32_t value);
void board_log_hex8(uint8_t value);
void board_log_hex16(uint16_t value);
void board_log_hex32(uint32_t value);

uint8_t board_spi1_transfer(uint8_t value);
void board_imu_cs_set(bool selected);
bool board_i2c1_probe(uint8_t addr7);
bool board_i2c1_read_reg(uint8_t addr7, uint8_t reg, uint8_t *value);
uint16_t board_adc1_read(uint8_t channel);
void board_pwm_set_us(uint8_t output, uint16_t pulse_us);
