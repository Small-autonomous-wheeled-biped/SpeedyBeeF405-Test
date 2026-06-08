#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f405xx_min.h"

#define BOARD_PWM_OUTPUTS 5U

/* USART3 (PC10/PC11, AF7): ASCII debug log, shared with bringup. */
#define BOARD_UART_LOG_BAUD   115200UL

/* USART1 (PA9/PA10, AF7): binary fast-packet output to balance controller.
   2 Mbaud gives headroom for 96-byte packets at up to ~2 kHz.              */
#define BOARD_UART_FAST_BAUD  2000000UL

/* SPI1 prescalers for ICM42688-P (APB2 = 84 MHz):
   INIT  ≈ 1.3 MHz  (DIV64)  — safe for all register transactions
   RUN   ≈ 10.5 MHz (DIV8)   — fast FIFO burst reads at runtime             */
#define BOARD_SPI_INIT_BR     SPI_CR1_BR_DIV64
#define BOARD_SPI_RUN_BR      SPI_CR1_BR_DIV8

extern uint32_t SystemCoreClock;

/* ---- Clock / GPIO / peripheral init ---- */
void board_clock_init(void);
void board_gpio_init(void);
void board_systick_init(void);
void board_tim2_init(void);          /* 32-bit 1 MHz free-running counter    */
void board_usart3_init(uint32_t baud);
void board_usart1_init(uint32_t baud);
void board_spi1_init(void);
void board_i2c1_init(void);
void board_adc1_init(void);
void board_pwm_init(void);

/* ---- Timing ---- */
uint32_t board_millis(void);
uint32_t board_micros(void);         /* wraps after ~71 min                  */
void     board_delay_ms(uint32_t ms);
void     board_delay_us(uint32_t us);

/* ---- LEDs / buzzer ---- */
void board_led_set(bool on);
void board_led_toggle(void);
void board_buzzer_set(bool on);

/* ---- ASCII log (USART3, blocking TX) ---- */
void board_log_char(char c);
void board_log(const char *text);
void board_log_line(const char *text);
void board_log_u32(uint32_t value);
void board_log_i32(int32_t value);
void board_log_hex8(uint8_t value);
void board_log_hex16(uint16_t value);
void board_log_hex32(uint32_t value);

/* ---- Binary fast packet output (USART1, interrupt-driven ring buffer) ---- */
/* Non-blocking ring-buffer write.  Returns false if buffer full. */
bool board_uart1_write(const uint8_t *data, size_t len);

/* Blocking write — waits for each byte to be accepted.
   Use only for diagnostics outside the real-time path.                     */
void board_uart1_write_blocking(const uint8_t *data, size_t len);

/* Returns true if there is still data being transmitted. */
bool board_uart1_tx_busy(void);

/* Internal: called from USART1_IRQHandler.  Do not call directly. */
void board_uart1_irq(void);

/* ---- SPI1 (ICM42688-P) ---- */
/* Set SPI clock prescaler. Use BOARD_SPI_INIT_BR during init,
   BOARD_SPI_RUN_BR for high-rate FIFO reads.                              */
void    board_spi1_set_speed(uint32_t br);
uint8_t board_spi1_transfer(uint8_t value);
void    board_imu_cs_set(bool selected);

/* ---- I2C1 (DPS310 barometer) ---- */
bool board_i2c1_probe(uint8_t addr7);
bool board_i2c1_read_reg(uint8_t addr7, uint8_t reg, uint8_t *value);
bool board_i2c1_read_burst(uint8_t addr7, uint8_t reg,
                            uint8_t *buf, uint8_t len);
bool board_i2c1_write_reg(uint8_t addr7, uint8_t reg, uint8_t value);

/* ---- ADC / PWM (unchanged from bringup) ---- */
uint16_t board_adc1_read(uint8_t channel);
void     board_pwm_set_us(uint8_t output, uint16_t pulse_us);
