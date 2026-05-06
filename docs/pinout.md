# SpeedyBee F405 Mini Working Pin Map

This is the subset used by the firmware scaffold. It is derived from matching
ArduPilot `SpeedyBeeF405Mini/hwdef.dat` and Betaflight
`SPBE-SPEEDYBEEF405MINI.config` resources.

## Control And Status

| Function | MCU pin | Notes |
| --- | --- | --- |
| Board LED | `PC13` | Active-low |
| Buzzer | `PC15` | Treat as active-low; leave off during normal bring-up |
| Boot/DFU | BOOT button | STM32 ROM DFU over USB-C |
| USB FS | `PA11/PA12` | DFU now; CDC later |

## Sensors

| Function | MCU pin | Notes |
| --- | --- | --- |
| IMU SPI1 SCK | `PA5` | AF5 |
| IMU SPI1 MISO | `PA6` | AF5 |
| IMU SPI1 MOSI | `PA7` | AF5 |
| ICM42688P CS | `PA4` | GPIO output, active-low |
| ICM42688P INT | `PC4` | Input/EXTI-capable |
| I2C1 SCL | `PB8` | AF4 open-drain |
| I2C1 SDA | `PB9` | AF4 open-drain |
| DPS310 barometer | I2C `0x76` | Probe only in this scaffold |

## UARTs

| UART | TX | RX | Suggested use |
| --- | --- | --- | --- |
| USART1 | `PA9` | `PA10` | DJI/VTX or spare |
| USART2 | `PA2` | `PA3` | RC/SBUS path |
| USART3 | `PC10` | `PC11` | Bring-up logging |
| UART4 | `PA0` | `PA1` | Onboard Bluetooth module; avoid for project wireless |
| UART5 | `PC12` | `PD2` | ESC telemetry path; RX exposed on ESC connector |
| USART6 | `PC6` | `PC7` | GPS or external wireless |

## Outputs And ADC

| Function | MCU pin | Notes |
| --- | --- | --- |
| M1 / PWM1 | `PB6` | TIM4 CH1 |
| M2 / PWM2 | `PB7` | TIM4 CH2 |
| M3 / PWM3 | `PB1` | TIM3 CH4 |
| M4 / PWM4 | `PB0` | TIM3 CH3 |
| M5 / LED pad | `PA8` | TIM1 CH1 |
| Battery voltage | `PC0` | ADC1 IN10 |
| Current sensor | `PC1` | ADC1 IN11 |

SpeedyBee's official specs say analog RSSI is not supported on this board even
though firmware targets mention `PC5` as an ADC resource. Do not depend on it
without probing the physical board.
