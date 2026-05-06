# SpeedyBee F405 Mini Robotics Bring-Up

Bare-metal STM32F405 firmware scaffold for using a SpeedyBee F405 Mini flight
controller as a robotics prototype controller.

This project intentionally treats the board as a known STM32F405 design with a
public firmware-derived pin map, not as a fully documented schematic. The first
firmware target brings up the hardware that matters for robotics:

- UART3 logging at 115200 baud on `T3/R3`
- active-low status LED on `PC13`
- optional beeper output on `PC15`
- ICM42688P IMU on `SPI1`
- DPS310 barometer probe on `I2C1`
- servo-style PWM on `M1-M5`
- battery/current ADC reads on `PC0/PC1`

USB is currently used for STM32 ROM DFU flashing/recovery. USB CDC logging is a
later milestone; it needs a real USB device stack and should not be improvised
before the basic board bring-up is proven.

## Hardware Assumptions

- Board: SpeedyBee F405 Mini flight controller
- MCU: STM32F405RG-class part, 1 MiB flash, 128 KiB SRAM + 64 KiB CCM
- HSE crystal: 8 MHz
- IMU: ICM42688P, SPI1, chip select `PA4`, interrupt `PC4`
- Barometer: DPS310, I2C1 address `0x76`
- External wireless: connect a separate radio MCU/module to an exposed UART

Do not rely on the onboard Bluetooth module as the project wireless MCU. It is
wired to UART4 and intended for SpeedyBee/Betaflight-style configuration.

## Build

Prerequisites already found on this machine:

- Arm GNU Toolchain `arm-none-eabi-gcc`
- CMake
- Ninja

Configure and build:

```powershell
cmake -S . -B build -G Ninja '-DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake'
cmake --build build
```

Outputs:

- `build/speedybee_f405_mini_bringup.elf`
- `build/speedybee_f405_mini_bringup.bin`
- `build/speedybee_f405_mini_bringup.hex`
- `build/speedybee_f405_mini_bringup.map`

## Flashing

This firmware is linked at `0x08000000` and will replace the existing firmware
image. Before flashing, save any Betaflight configuration you care about.

1. Hold the BOOT button.
2. Plug in USB-C.
3. Release BOOT once the board is powered.
4. Flash with a DFU tool, for example:

```powershell
dfu-util -a 0 -s 0x08000000:leave -D build/speedybee_f405_mini_bringup.bin
```

If `dfu-util` is not installed, install it or use STM32CubeProgrammer in USB DFU
mode and write the `.bin` file to address `0x08000000`.

This repo also includes helper scripts for the tooling installed on this
machine. See `docs/tooling.md`.

## Bring-Up Checklist

1. Confirm DFU entry before flashing anything.
2. Connect a 3.3 V USB-UART adapter to `T3/R3/GND`.
3. Open serial at 115200 8N1.
4. Flash the firmware.
5. Verify logs show:
   - clock source and boot banner
   - `ICM42688P WHOAMI=0x47`
   - DPS310 I2C ACK at `0x76`
   - changing IMU raw accel/gyro values when the board moves
   - raw battery/current ADC values
6. Scope or logic-analyze PWM outputs before connecting actuators.
7. Add the external wireless MCU only after the control-loop hardware is stable.

## Pin Map Reference

The board support code follows the SpeedyBee F405 Mini mappings corroborated by
ArduPilot and Betaflight target files. See `docs/pinout.md` for the working
pin map used by this firmware.
