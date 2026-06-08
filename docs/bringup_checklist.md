# Bring-up Checklist

## Prerequisites

- [ ] `arm-none-eabi-gcc` toolchain installed and on PATH
- [ ] CMake ≥ 3.20 installed
- [ ] OpenOCD or DFU-util available for flashing
- [ ] USB-C cable connected to board (DFU mode) or ST-Link for SWD debug
- [ ] Serial terminal at 115200 baud on USART3 (PC10=TX, PC11=RX)
- [ ] Python ≥ 3.9 for the host decoder (optional: `pyserial` for live decode)

---

## Step 1 — Build

```bash
# Build firmware for STM32F405
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
cmake --build build -j4
# Artifacts: build/speedybee_f405_imu.elf / .bin / .hex

# Build and run host unit tests
cmake -B build-host tests/host
cmake --build build-host -j4
ctest --test-dir build-host -V
```

Expected host test output:
```
test_crc: all tests passed
test_drivers: all tests passed
test_mahony: all tests passed
test_packets: all tests passed
```

---

## Step 2 — Flash

```bash
# DFU (via USB-C boot button):
powershell scripts/flash-dfu.ps1

# ST-Link SWD (via OpenOCD):
powershell scripts/openocd-stlink.ps1
```

---

## Step 3 — Verify clock and serial output

Open USART3 (PC10/PC11) at 115200 baud.

Expected startup output:
```
SpeedyBee F405 Mini — IMU attitude module v1
SystemCoreClock=168000000 Hz
config: using defaults
ICM42688-P WHO_AM_I=0x47 ok
ICM42688-P: init ok, FIFO running
DPS310: ack id=0x10
DPS310: init ok
Mahony filter ready — entering main loop
Fast packet rate: 500 Hz on USART1 (binary)
```

- [ ] `SystemCoreClock=168000000` — PLL locked on HSE
- [ ] No garbage characters — baud rate correct

---

## Step 4 — Verify ICM-42688-P WHO_AM_I

- [ ] `ICM42688-P WHO_AM_I=0x47 ok`  
  - If `0xFF`: SPI wiring issue or CS stuck high  
  - If `0x00`: MISO line not connected  
  - If wrong value: wrong chip or SPI mode

---

## Step 5 — Verify ICM raw accel/gyro at rest

Connect USART1 (PA9/PA10) at 2 Mbaud and run the Python decoder:

```bash
python tools/host_decoder.py COM3 --baud 2000000
```

At rest (flat surface) expect:
- `roll=  0.0°  pitch=  0.0°` (within ±2° of level)
- `accel_z ≈ 9.8 m/s²` (gravity on z-axis)
- `gyro_x/y/z ≈ 0` rad/s (within ±0.05 rad/s)
- `[IMU_OK|ESTIMATOR_READY]` in health flags
- [ ] CRC failures = 0 in decoder statistics
- [ ] Sequence numbers incrementing without gaps

---

## Step 6 — Verify DPS310 detection

- [ ] `DPS310: ack id=0x10` in startup log  
  - If `no-ack`: I2C wiring (PB8/PB9), pull-ups, or wrong address  
  - If wrong product ID: different barometer revision (still OK)
- [ ] Barometer fields in log packets show non-zero pressure (~101325 Pa at sea level)

---

## Step 7 — Verify Mahony startup convergence

1. Hold board flat and level.
2. Wait 2 seconds for `ESTIMATOR_READY` flag to appear.
3. Observe in the decoder:
   - [ ] `roll ≈ 0°, pitch ≈ 0°` within 2°
   - [ ] `[IMU_OK|ESTIMATOR_READY]` health flags
   - [ ] `ESTIMATOR_STARTUP` flag cleared after ~200 ms

---

## Step 8 — Verify pitch changes with tilt

1. Tilt the board 30° forward (positive pitch direction).
2. Observe `pitch ≈ 30°` in the decoder within 500 ms.
3. Return to level; confirm `pitch ≈ 0°` within 1 second.

- [ ] Pitch direction correct (forward tilt = positive pitch)
- [ ] No oscillation or runaway after settling

If pitch direction is wrong: update `axis_map[]` and `axis_sign[]` in config.

---

## Step 9 — Verify accel rejection during shaking

1. Shake the board vigorously for 2 seconds.
2. Observe in decoder:
   - [ ] `ACCEL_REJECTED` flag appears during shaking
   - [ ] `HIGH_LINEAR_ACCEL` flag may appear during impacts
   - [ ] Roll and pitch do not diverge wildly (gyro integration continues)
3. Stop shaking; confirm attitude recovers within 2 seconds.

---

## Step 10 — Verify no blocking if Pi is disconnected

1. Disconnect USART3 (Pi / debug log) while the board is running.
2. Observe on USART1:
   - [ ] Fast packets continue at configured rate
   - [ ] Sequence numbers increment without large gaps
3. Reconnect USART3; confirm ASCII log resumes.

---

## Step 11 — Verify balance controller stream stability

Connect the balance controller to USART1 (PA9, 2 Mbaud).

- [ ] Controller receives packets at 500 Hz (2 ms period)
- [ ] `ESTIMATOR_READY = 1`  
- [ ] `sample_age_us < 3000` (< 3 ms latency from IMU read to packet)
- [ ] Sequence gaps = 0 under normal conditions
- [ ] `PACKET_OVERRUN = 0` (TX ring buffer not full)

---

## Common failure modes

| Symptom | Likely cause |
|---|---|
| No serial output | Wrong UART, wrong baud, boot loop |
| `0xFF` WHO_AM_I | CS line stuck high, no SPI clock |
| `0x00` WHO_AM_I | MISO not connected |
| Attitude jitters at rest | Gyro bias not calibrated; let it sit still for 5 s |
| Pitch sign wrong | Board rotation mismatch; update `axis_sign[1]` |
| Fast packets at wrong rate | Check `config.fast_packet_rate_hz` |
| `PACKET_OVERRUN` persistent | USART1 baud too low, or increase ring buffer size |
| DPS310 not found | Check I2C address (0x76 vs 0x77), pull-up resistors |
