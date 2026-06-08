#include "board.h"
#include "usb_cdc.h"

#include <stddef.h>
#include <string.h>

uint32_t SystemCoreClock = 16000000UL;

/* ---- SysTick millisecond counter ---- */
static volatile uint32_t g_ms;

/* ---- USART1 TX ring buffer (interrupt-driven) ---- */
#define UART1_TX_BUF_SIZE 512U  /* must be power of two */
static volatile uint8_t  g_uart1_tx_buf[UART1_TX_BUF_SIZE];
static volatile uint32_t g_uart1_tx_head;  /* write index (producer) */
static volatile uint32_t g_uart1_tx_tail;  /* read  index (consumer, ISR) */

/* ---- GPIO helpers ---- */
static void gpio_set_mode(GPIO_TypeDef *gpio, uint8_t pin, uint32_t mode)
{
    gpio->MODER = (gpio->MODER & ~(3UL << (pin * 2U))) | (mode << (pin * 2U));
}

static void gpio_set_pupd(GPIO_TypeDef *gpio, uint8_t pin, uint32_t pupd)
{
    gpio->PUPDR = (gpio->PUPDR & ~(3UL << (pin * 2U))) | (pupd << (pin * 2U));
}

static void gpio_set_af(GPIO_TypeDef *gpio, uint8_t pin, uint8_t af)
{
    const uint8_t index = pin >> 3U;
    const uint8_t shift = (pin & 7U) * 4U;
    gpio->AFR[index] = (gpio->AFR[index] & ~(15UL << shift)) | ((uint32_t)af << shift);
}

static void gpio_write(GPIO_TypeDef *gpio, uint8_t pin, bool high)
{
    gpio->BSRR = high ? BIT(pin) : BIT(pin + 16U);
}

static void log_nibble(uint8_t nibble)
{
    const uint8_t digit = (nibble < 10U) ? (uint8_t)((uint8_t)'0' + nibble)
                                         : (uint8_t)((uint8_t)'A' + nibble - 10U);
    board_log_char((char)digit);
}

/* ---- ISR handlers ---- */

void SysTick_Handler(void)
{
    g_ms++;
}

void USART1_IRQHandler(void)
{
    board_uart1_irq();
}

/* ---- Clock ---- */

void board_clock_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;

    PWR->CR |= PWR_CR_VOS_SCALE1;
    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

    RCC->CR |= RCC_CR_HSEON;
    uint32_t timeout = 1000000UL;
    while (((RCC->CR & RCC_CR_HSERDY) == 0U) && (timeout-- > 0U)) {}

    if ((RCC->CR & RCC_CR_HSERDY) == 0U) {
        SystemCoreClock = 16000000UL;
        return;
    }

    /* PLL: HSE=8 MHz, M=8, N=336, P=2 → VCO=336 MHz, SYSCLK=168 MHz
       Q=7 → USB FS clock = 48 MHz.                                          */
    const uint32_t pll_m = 8UL;
    const uint32_t pll_n = 336UL;
    const uint32_t pll_q = 7UL;

    RCC->PLLCFGR = pll_m
                 | (pll_n << 6U)
                 | RCC_PLLCFGR_PLLSRC_HSE
                 | (pll_q << 24U);

    RCC->CFGR = RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CR |= RCC_CR_PLLON;

    while ((RCC->CR & RCC_CR_PLLRDY) == 0U) {}

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_MASK) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL) {}

    SystemCoreClock = 168000000UL;
}

/* ---- GPIO ---- */

void board_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN
                 | RCC_AHB1ENR_GPIOBEN
                 | RCC_AHB1ENR_GPIOCEN
                 | RCC_AHB1ENR_GPIODEN;
    (void)RCC->AHB1ENR;

    /* PC13 = LED (active-low), PC15 = buzzer (active-low) */
    gpio_set_mode(GPIOC, 13U, 1U);
    gpio_set_mode(GPIOC, 15U, 1U);
    GPIOC->OSPEEDR |= (3UL << (13U * 2U)) | (3UL << (15U * 2U));
    board_led_set(false);
    board_buzzer_set(false);

    /* PA4 = ICM42688-P CS (GPIO output, active-low) */
    gpio_set_mode(GPIOA, 4U, 1U);
    GPIOA->OSPEEDR |= (3UL << (4U * 2U));
    board_imu_cs_set(false);

    /* PA5/PA6/PA7 = SPI1 SCK/MISO/MOSI (AF5) */
    gpio_set_mode(GPIOA, 5U, 2U);
    gpio_set_mode(GPIOA, 6U, 2U);
    gpio_set_mode(GPIOA, 7U, 2U);
    gpio_set_af(GPIOA, 5U, 5U);
    gpio_set_af(GPIOA, 6U, 5U);
    gpio_set_af(GPIOA, 7U, 5U);
    GPIOA->OSPEEDR |= (3UL << (5U * 2U)) | (3UL << (6U * 2U)) | (3UL << (7U * 2U));

    /* PC4 = ICM42688-P INT (input, pull-up; used for FIFO watermark in future) */
    gpio_set_mode(GPIOC, 4U, 0U);
    gpio_set_pupd(GPIOC, 4U, 1U);

    /* PB8/PB9 = I2C1 SCL/SDA (AF4, open-drain, pull-up) */
    gpio_set_mode(GPIOB, 8U, 2U);
    gpio_set_mode(GPIOB, 9U, 2U);
    gpio_set_af(GPIOB, 8U, 4U);
    gpio_set_af(GPIOB, 9U, 4U);
    GPIOB->OTYPER |= BIT(8) | BIT(9);
    GPIOB->OSPEEDR |= (3UL << (8U * 2U)) | (3UL << (9U * 2U));
    gpio_set_pupd(GPIOB, 8U, 1U);
    gpio_set_pupd(GPIOB, 9U, 1U);

    /* PA9/PA10 = USART1 TX/RX (AF7) — binary fast-packet output */
    gpio_set_mode(GPIOA, 9U, 2U);
    gpio_set_mode(GPIOA, 10U, 2U);
    gpio_set_af(GPIOA, 9U, 7U);
    gpio_set_af(GPIOA, 10U, 7U);
    GPIOA->OSPEEDR |= (3UL << (9U * 2U)) | (3UL << (10U * 2U));

    /* PC10/PC11 = USART3 TX/RX (AF7) — ASCII debug log */
    gpio_set_mode(GPIOC, 10U, 2U);
    gpio_set_mode(GPIOC, 11U, 2U);
    gpio_set_af(GPIOC, 10U, 7U);
    gpio_set_af(GPIOC, 11U, 7U);
    GPIOC->OSPEEDR |= (3UL << (10U * 2U)) | (3UL << (11U * 2U));

    /* PWM outputs: TIM4 CH1/CH2 on PB6/PB7, TIM3 CH3/CH4 on PB0/PB1,
       TIM1 CH1 on PA8.                                                     */
    gpio_set_mode(GPIOB, 6U, 2U);
    gpio_set_mode(GPIOB, 7U, 2U);
    gpio_set_mode(GPIOB, 1U, 2U);
    gpio_set_mode(GPIOB, 0U, 2U);
    gpio_set_mode(GPIOA, 8U, 2U);
    gpio_set_af(GPIOB, 6U, 2U);
    gpio_set_af(GPIOB, 7U, 2U);
    gpio_set_af(GPIOB, 1U, 2U);
    gpio_set_af(GPIOB, 0U, 2U);
    gpio_set_af(GPIOA, 8U, 1U);
    GPIOB->OSPEEDR |= (3UL << (6U * 2U)) | (3UL << (7U * 2U))
                    | (3UL << (1U * 2U)) | (3UL << (0U * 2U));
    GPIOA->OSPEEDR |= (3UL << (8U * 2U));

    /* PC0/PC1 = ADC1 IN10/IN11 (analog, no pull) */
    gpio_set_mode(GPIOC, 0U, 3U);
    gpio_set_mode(GPIOC, 1U, 3U);
    gpio_set_pupd(GPIOC, 0U, 0U);
    gpio_set_pupd(GPIOC, 1U, 0U);
}

/* ---- Systick (1 kHz) ---- */

void board_systick_init(void)
{
    SysTick->LOAD = (SystemCoreClock / 1000UL) - 1UL;
    SysTick->VAL  = 0;
    SysTick->CTRL = SYSTICK_CTRL_CLKSOURCE | SYSTICK_CTRL_TICKINT | SYSTICK_CTRL_ENABLE;
}

/* ---- TIM2: 32-bit 1 MHz free-running microsecond counter ---- */
/* APB1 timer clock = SystemCoreClock/2 × 2 = SystemCoreClock = 168 MHz
   when PPRE1 divider ≠ 1.  PSC = 168-1 → 1 tick = 1 µs.
   ARR = 0xFFFFFFFF → wraps after ~71 min.                                  */
void board_tim2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;

    TIM2->CR1  = 0;
    TIM2->PSC  = 84U - 1U;        /* TIM2 clock = APB1×2 = 84 MHz; /84 = 1 MHz */
    TIM2->ARR  = 0xFFFFFFFFUL;    /* full 32-bit range      */
    TIM2->CNT  = 0;
    TIM2->EGR  = TIM_EGR_UG;      /* load prescaler         */
    TIM2->SR   = 0;
    TIM2->CR1  = TIM_CR1_CEN;
}

/* ---- USART3 (debug log, blocking) ---- */

void board_usart3_init(uint32_t baud)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    (void)RCC->APB1ENR;

    /* USART3 is on APB1 at 42 MHz. */
    const uint32_t pclk1 = (SystemCoreClock == 168000000UL) ? 42000000UL : SystemCoreClock;
    USART3->CR1 = 0;
    USART3->CR2 = 0;
    USART3->CR3 = 0;
    USART3->BRR = (pclk1 + (baud / 2UL)) / baud;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/* ---- USART1 (binary fast packets, interrupt-driven ring buffer) ---- */

void board_usart1_init(uint32_t baud)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;

    /* USART1 is on APB2 at 84 MHz (168/2). */
    const uint32_t pclk2 = (SystemCoreClock == 168000000UL) ? 84000000UL : SystemCoreClock;

    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->BRR = (pclk2 + (baud / 2UL)) / baud;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;  /* TX only; TXEIE enabled later */

    g_uart1_tx_head = 0;
    g_uart1_tx_tail = 0;

    nvic_enable_irq(USART1_IRQn);
}

/* Called from USART1_IRQHandler — feed next byte or disable TXEIE. */
void board_uart1_irq(void)
{
    if ((USART1->SR & USART_SR_TXE) == 0U) {
        return;
    }

    if (g_uart1_tx_tail != g_uart1_tx_head) {
        USART1->DR = g_uart1_tx_buf[g_uart1_tx_tail & (UART1_TX_BUF_SIZE - 1U)];
        g_uart1_tx_tail++;
    } else {
        /* Buffer empty: disable TXEIE to stop further interrupts. */
        USART1->CR1 &= ~USART_CR1_TXEIE;
    }
}

bool board_uart1_write(const uint8_t *data, size_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return true;
    }

    /* Check if there is room in the ring buffer. */
    const uint32_t space = UART1_TX_BUF_SIZE
                         - (uint32_t)(g_uart1_tx_head - g_uart1_tx_tail);
    if ((uint32_t)len > space) {
        return false;   /* overrun: caller sets HEALTH_FLAG_PACKET_OVERRUN */
    }

    for (size_t i = 0; i < len; i++) {
        g_uart1_tx_buf[g_uart1_tx_head & (UART1_TX_BUF_SIZE - 1U)] = data[i];
        g_uart1_tx_head++;
    }

    /* Enable TXEIE to start / continue transmission. */
    USART1->CR1 |= USART_CR1_TXEIE;
    return true;
}

bool board_uart1_tx_busy(void)
{
    return g_uart1_tx_tail != g_uart1_tx_head;
}

/* ---- SPI1 (ICM42688-P) ---- */

void board_spi1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR;

    /* Mode 3: CPOL=1, CPHA=1.  Master.  Software CS.  Start slow. */
    SPI1->CR1 = SPI_CR1_CPOL
              | SPI_CR1_CPHA
              | SPI_CR1_MSTR
              | BOARD_SPI_INIT_BR
              | SPI_CR1_SSM
              | SPI_CR1_SSI;
    SPI1->CR2 = 0;
    SPI1->CR1 |= SPI_CR1_SPE;
}

void board_spi1_set_speed(uint32_t br)
{
    /* Disable SPI, change prescaler, re-enable.
       Changing BR while SPE=1 is not recommended (RM0090 §28.5.3). */
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 = (SPI1->CR1 & ~SPI_CR1_BR_MASK) | (br & SPI_CR1_BR_MASK);
    SPI1->CR1 |= SPI_CR1_SPE;
}

uint8_t board_spi1_transfer(uint8_t value)
{
    while ((SPI1->SR & SPI_SR_TXE) == 0U) {}
    *(__IO uint8_t *)&SPI1->DR = value;
    while ((SPI1->SR & SPI_SR_RXNE) == 0U) {}
    return (uint8_t)SPI1->DR;
}

void board_imu_cs_set(bool selected)
{
    gpio_write(GPIOA, 4U, !selected);
}

/* ---- I2C1 helpers ---- */

static bool i2c1_wait_set(uint32_t mask, uint32_t timeout)
{
    while (((I2C1->SR1 & mask) == 0U) && (timeout-- > 0U)) {
        if ((I2C1->SR1 & I2C_SR1_AF) != 0U) {
            return false;
        }
    }
    return (I2C1->SR1 & mask) != 0U;
}

static bool i2c1_wait_clear_busy(uint32_t timeout)
{
    while (((I2C1->SR2 & I2C_SR2_BUSY) != 0U) && (timeout-- > 0U)) {}
    return (I2C1->SR2 & I2C_SR2_BUSY) == 0U;
}

static void i2c1_clear_addr(void)
{
    volatile uint32_t sr1 = I2C1->SR1;
    volatile uint32_t sr2 = I2C1->SR2;
    (void)sr1;
    (void)sr2;
}

void board_i2c1_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC->APB1ENR;

    /* 100 kHz standard-mode on 42 MHz APB1.
       CCR = 210, TRISE = 43  (from ref. manual §27.4).                     */
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;
    I2C1->CR2 = 42U;
    I2C1->CCR = 210U;
    I2C1->TRISE = 43U;
    I2C1->CR1 = I2C_CR1_PE;
}

bool board_i2c1_probe(uint8_t addr7)
{
    I2C1->SR1 &= ~I2C_SR1_AF;

    if (!i2c1_wait_clear_busy(100000UL)) {
        return false;
    }

    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait_set(I2C_SR1_SB, 100000UL)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    I2C1->DR = (uint32_t)(addr7 << 1U);
    if (!i2c1_wait_set(I2C_SR1_ADDR, 100000UL)) {
        I2C1->SR1 &= ~I2C_SR1_AF;
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    i2c1_clear_addr();
    I2C1->CR1 |= I2C_CR1_STOP;
    return true;
}

bool board_i2c1_write_reg(uint8_t addr7, uint8_t reg, uint8_t value)
{
    I2C1->SR1 &= ~I2C_SR1_AF;
    if (!i2c1_wait_clear_busy(100000UL)) { return false; }

    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait_set(I2C_SR1_SB, 100000UL)) { goto stop; }

    I2C1->DR = (uint32_t)(addr7 << 1U);
    if (!i2c1_wait_set(I2C_SR1_ADDR, 100000UL)) { I2C1->SR1 &= ~I2C_SR1_AF; goto stop; }
    i2c1_clear_addr();

    if (!i2c1_wait_set(I2C_SR1_TXE, 100000UL)) { goto stop; }
    I2C1->DR = reg;
    if (!i2c1_wait_set(I2C_SR1_TXE, 100000UL)) { goto stop; }
    I2C1->DR = value;
    if (!i2c1_wait_set(I2C_SR1_BTF, 100000UL)) { goto stop; }

    I2C1->CR1 |= I2C_CR1_STOP;
    return true;

stop:
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
}

bool board_i2c1_read_reg(uint8_t addr7, uint8_t reg, uint8_t *value)
{
    if (value == NULL) { return false; }
    return board_i2c1_read_burst(addr7, reg, value, 1U);
}

bool board_i2c1_read_burst(uint8_t addr7, uint8_t reg,
                             uint8_t *buf, uint8_t len)
{
    if ((buf == NULL) || (len == 0U)) { return false; }

    I2C1->SR1 &= ~I2C_SR1_AF;
    if (!i2c1_wait_clear_busy(100000UL)) { return false; }

    /* Write phase: send register address. */
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait_set(I2C_SR1_SB, 100000UL)) { goto stop_fail; }
    I2C1->DR = (uint32_t)(addr7 << 1U);
    if (!i2c1_wait_set(I2C_SR1_ADDR, 100000UL)) { I2C1->SR1 &= ~I2C_SR1_AF; goto stop_fail; }
    i2c1_clear_addr();
    if (!i2c1_wait_set(I2C_SR1_TXE, 100000UL)) { goto stop_fail; }
    I2C1->DR = reg;
    if (!i2c1_wait_set(I2C_SR1_BTF, 100000UL)) { goto stop_fail; }

    /* Repeated start and switch to read. */
    I2C1->CR1 |= I2C_CR1_ACK;  /* pre-arm ACK for multi-byte read */
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait_set(I2C_SR1_SB, 100000UL)) { goto stop_fail; }
    I2C1->DR = (uint32_t)((addr7 << 1U) | 1U);
    if (!i2c1_wait_set(I2C_SR1_ADDR, 100000UL)) { I2C1->SR1 &= ~I2C_SR1_AF; goto stop_fail; }

    /* For a single byte: disable ACK before clearing ADDR so NACK is sent. */
    if (len == 1U) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        i2c1_clear_addr();
        I2C1->CR1 |= I2C_CR1_STOP;
        if (!i2c1_wait_set(I2C_SR1_RXNE, 100000UL)) { return false; }
        buf[0] = (uint8_t)I2C1->DR;
        return true;
    }

    i2c1_clear_addr();

    for (uint8_t i = 0; i < len; i++) {
        if (i == (len - 1U)) {
            /* Before reading last byte: NACK + STOP. */
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |= I2C_CR1_STOP;
        }
        if (!i2c1_wait_set(I2C_SR1_RXNE, 100000UL)) { return false; }
        buf[i] = (uint8_t)I2C1->DR;
    }
    return true;

stop_fail:
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
}

/* ---- Timing ---- */

void board_systick_init(void);  /* defined above via SysTick handler */

uint32_t board_millis(void)
{
    return g_ms;
}

uint32_t board_micros(void)
{
    return TIM2->CNT;
}

void board_delay_ms(uint32_t ms)
{
    const uint32_t start = board_millis();
    while ((board_millis() - start) < ms) {}
}

void board_delay_us(uint32_t us)
{
    const uint32_t start = board_micros();
    while ((board_micros() - start) < us) {}
}

/* ---- LED / buzzer ---- */

void board_led_set(bool on)
{
    gpio_write(GPIOC, 13U, !on);   /* active-low */
}

void board_led_toggle(void)
{
    GPIOC->ODR ^= BIT(13);
}

void board_buzzer_set(bool on)
{
    gpio_write(GPIOC, 15U, !on);   /* active-low */
}

/* ---- ASCII log (USART3, blocking) ---- */

void board_log_char(char c)
{
    if (c == '\n') {
        board_log_char('\r');
    }
    /* Send to USART3 (always) and USB CDC (when connected). */
    while ((USART3->SR & USART_SR_TXE) == 0U) {}
    USART3->DR = (uint32_t)(uint8_t)c;
    usb_cdc_putchar(c);
}

void board_log(const char *text)
{
    while ((text != NULL) && (*text != '\0')) {
        board_log_char(*text++);
    }
}

void board_log_line(const char *text)
{
    board_log(text);
    board_log_char('\n');
}

void board_log_u32(uint32_t value)
{
    char buf[10];
    size_t count = 0;
    if (value == 0U) { board_log_char('0'); return; }
    while (value > 0U) {
        buf[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (count > 0U) { board_log_char(buf[--count]); }
}

void board_log_i32(int32_t value)
{
    if (value < 0) {
        board_log_char('-');
        const uint32_t magnitude = (uint32_t)(-(value + 1)) + 1U;
        board_log_u32(magnitude);
    } else {
        board_log_u32((uint32_t)value);
    }
}

void board_log_hex8(uint8_t value)
{
    board_log("0x");
    log_nibble((uint8_t)(value >> 4U));
    log_nibble((uint8_t)(value & 0x0FU));
}

void board_log_hex16(uint16_t value)
{
    board_log("0x");
    log_nibble((uint8_t)((value >> 12U) & 0x0FU));
    log_nibble((uint8_t)((value >> 8U)  & 0x0FU));
    log_nibble((uint8_t)((value >> 4U)  & 0x0FU));
    log_nibble((uint8_t)(value & 0x0FU));
}

void board_log_hex32(uint32_t value)
{
    board_log("0x");
    for (int shift = 28; shift >= 0; shift -= 4) {
        log_nibble((uint8_t)((value >> (uint32_t)shift) & 0x0FU));
    }
}

/* ---- ADC ---- */

void board_adc1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;
    ADC_COMMON->CCR = 1UL << 16U;
    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_ADON;
    ADC1->SMPR1 |= (7UL << 0U) | (7UL << 3U);
}

uint16_t board_adc1_read(uint8_t channel)
{
    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = channel;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while ((ADC1->SR & ADC_SR_EOC) == 0U) {}
    return (uint16_t)(ADC1->DR & 0x0FFFU);
}

/* ---- PWM ---- */

void board_pwm_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB1ENR;
    (void)RCC->APB2ENR;

    TIM4->PSC = 84U - 1U;
    TIM4->ARR = 20000U - 1U;
    TIM4->CCR1 = 1000U;
    TIM4->CCR2 = 1000U;
    TIM4->CCMR1 = (6UL << 4U) | BIT(3) | (6UL << 12U) | BIT(11);
    TIM4->CCER = BIT(0) | BIT(4);
    TIM4->EGR = TIM_EGR_UG;
    TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    TIM3->PSC = 84U - 1U;
    TIM3->ARR = 20000U - 1U;
    TIM3->CCR3 = 1000U;
    TIM3->CCR4 = 1000U;
    TIM3->CCMR2 = (6UL << 4U) | BIT(3) | (6UL << 12U) | BIT(11);
    TIM3->CCER = BIT(8) | BIT(12);
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    TIM1->PSC = 168U - 1U;
    TIM1->ARR = 20000U - 1U;
    TIM1->CCR1 = 1000U;
    TIM1->CCMR1 = (6UL << 4U) | BIT(3);
    TIM1->CCER = BIT(0);
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void board_pwm_set_us(uint8_t output, uint16_t pulse_us)
{
    if (pulse_us < 900U)  { pulse_us = 900U;  }
    if (pulse_us > 2100U) { pulse_us = 2100U; }
    switch (output) {
    case 1: TIM4->CCR1 = pulse_us; break;
    case 2: TIM4->CCR2 = pulse_us; break;
    case 3: TIM3->CCR4 = pulse_us; break;
    case 4: TIM3->CCR3 = pulse_us; break;
    case 5: TIM1->CCR1 = pulse_us; break;
    default: break;
    }
}
