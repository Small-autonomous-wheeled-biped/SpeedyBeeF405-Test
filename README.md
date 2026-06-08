# SpeedyBee F405 Mini — IMU/Attitude Sensor Module

Bare-metal STM32F405 firmware that turns the SpeedyBee F405 Mini flight
controller into a high-rate, deterministic IMU/attitude module for a small
balancing ground robot.

The board belongs to the **fast deterministic layer** of a two-layer robot
architecture.  It runs a Mahony complementary filter at 1 kHz and streams a
compact 96-byte binary attitude packet to the main balance controller at 500 Hz
over USART1.  A richer diagnostic stream is available for the Raspberry Pi over
USART3.

See [`docs/architecture.md`](docs/architecture.md) for the full design rationale.

---

## Hardware

| Component | Device | Connection |
|---|---|---|
| MCU | STM32F405RGT6 | — |
| IMU | ICM-42688-P | SPI1 (PA5/PA6/PA7), CS=PA4 |
| Barometer | DPS310 | I2C1 (PB8/PB9), addr=0x76 |
| Fast packet TX | USART1 | PA9 (TX), 2 Mbaud |
| Debug log | USART3 | PC10 (TX), 115200 baud |

---

## Quick start

### Requirements

- `arm-none-eabi-gcc` ≥ 12 on PATH
- CMake ≥ 3.20
- OpenOCD or DFU-util for flashing
- Python ≥ 3.9 + `pyserial` for the host decoder

### Build

```bash
# STM32 firmware
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
cmake --build build -j4
# → build/speedybee_f405_imu.elf / .bin / .hex

# Host unit tests (Linux/macOS/Windows with GCC)
cmake -B build-host tests/host
cmake --build build-host -j4
ctest --test-dir build-host -V
```

### Flash

```bash
# Via DFU (USB-C, hold BOOT button during power-on):
powershell scripts/flash-dfu.ps1

# Via ST-Link SWD (OpenOCD):
powershell scripts/openocd-stlink.ps1
```

### Decode live packets

```bash
python tools/host_decoder.py COM3 --baud 2000000
python tools/host_decoder.py /dev/ttyUSB0 --baud 2000000 --csv out.csv
python tools/host_decoder.py --selftest   # verify CRC and packet logic
```

---

## Module overview

```
include/           Header files (all module APIs)
src/               Source files
  board.c          Register-level HAL (SPI, I2C, UART, TIM2 µs counter)
  icm42688p.c      ICM-42688-P driver: full init, FIFO read, unit conversion
  dps310.c         DPS310 driver: calibration, pressure/temperature/altitude
  imu_calibration.c  Gyro/accel bias removal + still-stand bias estimation
  imu_filters.c    IIR LPF + biquad notch for gyro/accel
  mahony_filter.c  Mahony nonlinear complementary attitude filter
  health_monitor.c Health and status flag aggregation
  fast_packet.c    96-byte binary attitude packet (encode/decode + CRC)
  log_packet.c     Larger diagnostic packet for Pi-side logging
  crc.c            CRC-16/CCITT-FALSE
  config_store.c   Config struct with flash storage and safe defaults
  main.c           Main loop: 1 kHz IMU, 500 Hz fast packet, 25 Hz baro
docs/              Design documentation
tests/host/        Pure-C unit tests (run natively on host)
tools/             Python host decoder with CSV logging
```

---

## Mahony filter

The filter estimates attitude as a unit quaternion (body-to-Earth, NED frame)
using gyro integration corrected by accelerometer gravity direction.

**Key parameters** (configurable in `config_t` and persistent in flash):

| Parameter | Default | Description |
|---|---|---|
| `mahony_kp` | 2.0 | Proportional gain — bandwidth of accel correction |
| `mahony_ki` | 0.005 | Integral gain — rate of gyro bias estimation |
| `accel_gate_lo_g` | 0.85 g | Lower gate for accel correction weight |
| `accel_gate_hi_g` | 1.15 g | Upper gate for accel correction weight |

**Tuning for balancing:**
- Raise `Kp` (up to ~10) for faster pitch correction at the cost of noise sensitivity.
- Lower `Kp` (down to ~0.5) to smooth noisy accel but slower initial convergence.
- Set `Ki = 0` if the gyro bias is pre-calibrated and stable.
- Narrow the accel gate (`[0.92, 1.08]`) to reject more motion events.

**Yaw limitation:**
Yaw is integrated from gyro only and will drift.  There is no magnetometer.
The `yaw_rad` field in the packet is relative.  Absolute heading must come
from the Pi-side estimator using wheel odometry or compass data.

---

## Packet format

See [`docs/protocol.md`](docs/protocol.md) for the complete field table.

| Field | Value |
|---|---|
| Magic | `0x42F4` (LE) |
| Total size | 96 bytes |
| CRC | CRC-16/CCITT-FALSE over bytes 0–91 |
| Default rate | 500 Hz |
| Transport | USART1, 2 Mbaud |

---

## Calibration

See [`docs/calibration.md`](docs/calibration.md).

Short version: place the board flat and still for 5 seconds; the firmware
estimates and applies the gyro bias automatically.

---

## Bring-up checklist

See [`docs/bringup_checklist.md`](docs/bringup_checklist.md) for the complete
11-step verification procedure.

---

## Known limitations and next steps

| Limitation | Next step |
|---|---|
| Yaw drifts without external reference | Add wheel odometry or compass fusion on Pi side |
| Accel scale errors not corrected | Implement 6-point accel calibration |
| No temperature-compensated gyro bias | Log temperature; fit linear model on Pi |
| Log packet uses blocking USART3 TX | Replace with non-blocking ring buffer |
| Config CLI not yet implemented | Add ASCII command parser on USART3 |
| No SPI slave / CAN output | Add as alternative fast-packet transport |
| No watchdog timer | Add IWDG to detect firmware hang |

---

## License

Original firmware — MIT License.
Drivers reference TDK ICM-42688-P and Infineon DPS310 datasheets for register
values; no GPL-licensed source was copied verbatim.

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

## No-Board Testing

The host test project compiles the IMU and barometer drivers for the PC against
fake `board_*` functions. These tests validate driver transactions and parsing
logic without flashing or connecting the flight controller.

Configure, build, and run the host tests:

```powershell
cmake -S tests/host -B build-host -G Ninja
cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

Run static analysis locally:

```powershell
cppcheck --enable=warning,style,performance,portability --error-exitcode=1 --std=c11 --inline-suppr -I include src include
```

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
