#pragma once

#include <stdint.h>

#define __IO volatile

#define PERIPH_BASE             0x40000000UL
#define APB1PERIPH_BASE         PERIPH_BASE
#define APB2PERIPH_BASE         (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE         (PERIPH_BASE + 0x00020000UL)

#define GPIOA_BASE              (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE              (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE              (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE              (AHB1PERIPH_BASE + 0x0C00UL)
#define RCC_BASE                (AHB1PERIPH_BASE + 0x3800UL)

#define TIM3_BASE               (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE               (APB1PERIPH_BASE + 0x0800UL)
#define I2C1_BASE               (APB1PERIPH_BASE + 0x5400UL)
#define USART3_BASE             (APB1PERIPH_BASE + 0x4800UL)
#define PWR_BASE                (APB1PERIPH_BASE + 0x7000UL)

#define TIM1_BASE               (APB2PERIPH_BASE + 0x0000UL)
#define USART1_BASE             (APB2PERIPH_BASE + 0x1000UL)
#define SPI1_BASE               (APB2PERIPH_BASE + 0x3000UL)
#define ADC1_BASE               (APB2PERIPH_BASE + 0x2000UL)
#define ADC_COMMON_BASE         (APB2PERIPH_BASE + 0x2300UL)
#define FLASH_R_BASE            0x40023C00UL

#define SCS_BASE                0xE000E000UL
#define SYSTICK_BASE            (SCS_BASE + 0x0010UL)
#define SCB_CPACR_ADDR          0xE000ED88UL

typedef struct {
    __IO uint32_t MODER;
    __IO uint32_t OTYPER;
    __IO uint32_t OSPEEDR;
    __IO uint32_t PUPDR;
    __IO uint32_t IDR;
    __IO uint32_t ODR;
    __IO uint32_t BSRR;
    __IO uint32_t LCKR;
    __IO uint32_t AFR[2];
} GPIO_TypeDef;

typedef struct {
    __IO uint32_t CR;
    __IO uint32_t PLLCFGR;
    __IO uint32_t CFGR;
    __IO uint32_t CIR;
    __IO uint32_t AHB1RSTR;
    __IO uint32_t AHB2RSTR;
    __IO uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    __IO uint32_t APB1RSTR;
    __IO uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    __IO uint32_t AHB1ENR;
    __IO uint32_t AHB2ENR;
    __IO uint32_t AHB3ENR;
    uint32_t RESERVED2;
    __IO uint32_t APB1ENR;
    __IO uint32_t APB2ENR;
} RCC_TypeDef;

typedef struct {
    __IO uint32_t ACR;
    __IO uint32_t KEYR;
    __IO uint32_t OPTKEYR;
    __IO uint32_t SR;
    __IO uint32_t CR;
    __IO uint32_t OPTCR;
} FLASH_TypeDef;

typedef struct {
    __IO uint32_t CR;
    __IO uint32_t CSR;
} PWR_TypeDef;

typedef struct {
    __IO uint32_t CTRL;
    __IO uint32_t LOAD;
    __IO uint32_t VAL;
    __IO uint32_t CALIB;
} SysTick_TypeDef;

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMCR;
    __IO uint32_t DIER;
    __IO uint32_t SR;
    __IO uint32_t EGR;
    __IO uint32_t CCMR1;
    __IO uint32_t CCMR2;
    __IO uint32_t CCER;
    __IO uint32_t CNT;
    __IO uint32_t PSC;
    __IO uint32_t ARR;
    __IO uint32_t RCR;
    __IO uint32_t CCR1;
    __IO uint32_t CCR2;
    __IO uint32_t CCR3;
    __IO uint32_t CCR4;
    __IO uint32_t BDTR;
    __IO uint32_t DCR;
    __IO uint32_t DMAR;
    __IO uint32_t OR;
} TIM_TypeDef;

typedef struct {
    __IO uint32_t SR;
    __IO uint32_t DR;
    __IO uint32_t BRR;
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t CR3;
    __IO uint32_t GTPR;
} USART_TypeDef;

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t OAR1;
    __IO uint32_t OAR2;
    __IO uint32_t DR;
    __IO uint32_t SR1;
    __IO uint32_t SR2;
    __IO uint32_t CCR;
    __IO uint32_t TRISE;
    __IO uint32_t FLTR;
} I2C_TypeDef;

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SR;
    __IO uint32_t DR;
    __IO uint32_t CRCPR;
    __IO uint32_t RXCRCR;
    __IO uint32_t TXCRCR;
    __IO uint32_t I2SCFGR;
    __IO uint32_t I2SPR;
} SPI_TypeDef;

typedef struct {
    __IO uint32_t SR;
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMPR1;
    __IO uint32_t SMPR2;
    __IO uint32_t JOFR1;
    __IO uint32_t JOFR2;
    __IO uint32_t JOFR3;
    __IO uint32_t JOFR4;
    __IO uint32_t HTR;
    __IO uint32_t LTR;
    __IO uint32_t SQR1;
    __IO uint32_t SQR2;
    __IO uint32_t SQR3;
    __IO uint32_t JSQR;
    __IO uint32_t JDR1;
    __IO uint32_t JDR2;
    __IO uint32_t JDR3;
    __IO uint32_t JDR4;
    __IO uint32_t DR;
} ADC_TypeDef;

typedef struct {
    __IO uint32_t CSR;
    __IO uint32_t CCR;
    __IO uint32_t CDR;
} ADC_Common_TypeDef;

#define GPIOA                  ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                  ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC                  ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD                  ((GPIO_TypeDef *)GPIOD_BASE)
#define RCC                    ((RCC_TypeDef *)RCC_BASE)
#define FLASH                  ((FLASH_TypeDef *)FLASH_R_BASE)
#define PWR                    ((PWR_TypeDef *)PWR_BASE)
#define SysTick                ((SysTick_TypeDef *)SYSTICK_BASE)
#define TIM1                   ((TIM_TypeDef *)TIM1_BASE)
#define TIM3                   ((TIM_TypeDef *)TIM3_BASE)
#define TIM4                   ((TIM_TypeDef *)TIM4_BASE)
#define USART3                 ((USART_TypeDef *)USART3_BASE)
#define I2C1                   ((I2C_TypeDef *)I2C1_BASE)
#define SPI1                   ((SPI_TypeDef *)SPI1_BASE)
#define ADC1                   ((ADC_TypeDef *)ADC1_BASE)
#define ADC_COMMON             ((ADC_Common_TypeDef *)ADC_COMMON_BASE)
#define SCB_CPACR              (*((__IO uint32_t *)SCB_CPACR_ADDR))

#define BIT(n)                 (1UL << (n))

#define RCC_CR_HSION           BIT(0)
#define RCC_CR_HSIRDY          BIT(1)
#define RCC_CR_HSEON           BIT(16)
#define RCC_CR_HSERDY          BIT(17)
#define RCC_CR_PLLON           BIT(24)
#define RCC_CR_PLLRDY          BIT(25)

#define RCC_PLLCFGR_PLLSRC_HSE BIT(22)

#define RCC_CFGR_SW_MASK       (3UL << 0)
#define RCC_CFGR_SW_PLL        (2UL << 0)
#define RCC_CFGR_SWS_MASK      (3UL << 2)
#define RCC_CFGR_SWS_PLL       (2UL << 2)
#define RCC_CFGR_PPRE1_DIV4    (5UL << 10)
#define RCC_CFGR_PPRE2_DIV2    (4UL << 13)

#define RCC_AHB1ENR_GPIOAEN    BIT(0)
#define RCC_AHB1ENR_GPIOBEN    BIT(1)
#define RCC_AHB1ENR_GPIOCEN    BIT(2)
#define RCC_AHB1ENR_GPIODEN    BIT(3)

#define RCC_APB1ENR_TIM3EN     BIT(1)
#define RCC_APB1ENR_TIM4EN     BIT(2)
#define RCC_APB1ENR_USART3EN   BIT(18)
#define RCC_APB1ENR_I2C1EN     BIT(21)
#define RCC_APB1ENR_PWREN      BIT(28)

#define RCC_APB2ENR_TIM1EN     BIT(0)
#define RCC_APB2ENR_ADC1EN     BIT(8)
#define RCC_APB2ENR_SPI1EN     BIT(12)

#define FLASH_ACR_LATENCY_5WS  5UL
#define FLASH_ACR_PRFTEN       BIT(8)
#define FLASH_ACR_ICEN         BIT(9)
#define FLASH_ACR_DCEN         BIT(10)

#define PWR_CR_VOS_SCALE1      BIT(14)

#define SYSTICK_CTRL_ENABLE    BIT(0)
#define SYSTICK_CTRL_TICKINT   BIT(1)
#define SYSTICK_CTRL_CLKSOURCE BIT(2)

#define USART_SR_TXE           BIT(7)
#define USART_SR_TC            BIT(6)
#define USART_CR1_UE           BIT(13)
#define USART_CR1_TE           BIT(3)
#define USART_CR1_RE           BIT(2)

#define SPI_CR1_CPHA           BIT(0)
#define SPI_CR1_CPOL           BIT(1)
#define SPI_CR1_MSTR           BIT(2)
#define SPI_CR1_BR_DIV64       (5UL << 3)
#define SPI_CR1_SPE            BIT(6)
#define SPI_CR1_SSI            BIT(8)
#define SPI_CR1_SSM            BIT(9)
#define SPI_SR_RXNE            BIT(0)
#define SPI_SR_TXE             BIT(1)
#define SPI_SR_BSY             BIT(7)

#define I2C_CR1_PE             BIT(0)
#define I2C_CR1_START          BIT(8)
#define I2C_CR1_STOP           BIT(9)
#define I2C_CR1_ACK            BIT(10)
#define I2C_CR1_SWRST          BIT(15)
#define I2C_SR1_SB             BIT(0)
#define I2C_SR1_ADDR           BIT(1)
#define I2C_SR1_BTF            BIT(2)
#define I2C_SR1_RXNE           BIT(6)
#define I2C_SR1_TXE            BIT(7)
#define I2C_SR1_AF             BIT(10)
#define I2C_SR2_BUSY           BIT(1)

#define ADC_SR_EOC             BIT(1)
#define ADC_CR2_ADON           BIT(0)
#define ADC_CR2_SWSTART        BIT(30)

#define TIM_CR1_CEN            BIT(0)
#define TIM_CR1_ARPE           BIT(7)
#define TIM_EGR_UG             BIT(0)
#define TIM_BDTR_MOE           BIT(15)
