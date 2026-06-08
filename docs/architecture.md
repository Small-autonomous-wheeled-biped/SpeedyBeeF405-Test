# Architecture — SpeedyBee F405 Mini IMU/Attitude Module

## Two-layer robot architecture

The robot is divided into two layers with a clean boundary:

```
┌─────────────────────────────────────────────────────────┐
│  HIGH-LEVEL LAYER — Raspberry Pi (ROS 2)                │
│  • State estimation (full EKF/UKF with all sensors)     │
│  • Mapping, localisation, path planning                 │
│  • Logging, dashboarding, diagnostics                   │
│  • Fuses: IMU attitude + encoders + ToF/LiDAR + GNSS   │
│  • Publishes: /cmd_vel, /odom, /map, /tf                │
└──────────────────────────┬──────────────────────────────┘
                           │  binary attitude stream
                           │  (USART3 / USB CDC, 100 Hz)
┌──────────────────────────▼──────────────────────────────┐
│  FAST DETERMINISTIC LAYER — bare-metal MCUs             │
│                                                         │
│  SpeedyBee F405 Mini ← this firmware                   │
│  • ICM-42688-P IMU at 1 kHz                            │
│  • Mahony filter → roll, pitch, pitch_rate              │
│  • Fast binary packet → balance controller (500 Hz)    │
│  • DPS310 barometer at 25 Hz (logging only)            │
│                                                         │
│  Balance controller MCU (separate)                      │
│  • Receives fast attitude packet on USART1              │
│  • Runs PID / LQR balance loop (≥1 kHz)                │
│  • Commands motor drivers                               │
└─────────────────────────────────────────────────────────┘
```

### Why does this MCU only estimate attitude?

| Concern | Reason |
|---|---|
| **Determinism** | A Mahony filter at 1 kHz has a fixed, bounded execution time per update. A full INS or EKF does not. |
| **Latency** | Balance loops typically need < 2 ms from measurement to actuation. Any variable-latency computation (SLAM, EKF with GNSS, wheel encoder fusion) breaks this budget. |
| **Separation of concerns** | Wheel odometry, optical flow, and GNSS data arrive at the Pi with variable latency. Fusing them with IMU data is correct only when done in a single consistent estimator on the Pi, not split across two processors. |
| **Failure isolation** | If the Pi crashes, the balance loop keeps running from the IMU module. The balance controller never blocks on the Pi. |

## Signal chain (per update cycle, 1 kHz)

```
ICM-42688-P FIFO (SPI, burst)
      │
      ▼
Board rotation matrix (axis remap, ROTATION_PITCH_180_YAW_90)
      │
      ▼
Gyro/accel bias subtraction (from stored calibration)
      │
      ▼
Digital low-pass filter (IIR, configurable cutoff)
      │  ┌── raw vs filtered → vibration metric
      ▼
Mahony complementary filter (Kp=2.0, Ki=0.005)
      │  • Gyro integration (quaternion)
      │  • Accel correction (gravity direction)
      │  • Integral feedback (gyro bias estimation)
      │  • Smooth acceleration rejection gate
      ▼
Quaternion → Euler angles
      │
      ▼
fast_packet_encode() → 96-byte binary frame
      │
      ▼
USART1 ring-buffer TX (2 Mbaud, interrupt-driven)
      │
      └──→ balance controller
```

## Why Mahony, not EKF/UKF?

- **Fixed cost per update**: O(1) arithmetic, no matrix inversions.
- **Proven stability**: Lyapunov-stable in the nonlinear sense (Mahony 2008).
- **No process noise tuning**: gains Kp and Ki are physically interpretable.
- **Float32 sufficient**: The F405 FPU gives 0-wait float arithmetic.
- **Yaw limitation is acceptable**: A balancing robot does not need absolute heading; yaw is tracked relatively and corrected by the Pi-side estimator when odometry or compass data arrives.

## Packet flows

| Stream | Transport | Rate | Consumer |
|---|---|---|---|
| Fast attitude | USART1, 2 Mbaud, binary | 500 Hz (config) | Balance controller |
| Diagnostic log | USART3, 115200, binary | 100 Hz (config) | Raspberry Pi |
| ASCII debug | USART3, 115200 | startup only | PC terminal |

## Failure modes and fallbacks

| Failure | Behavior |
|---|---|
| IMU WHO_AM_I fail | Fast packet sent with `IMU_OK=0`; estimator outputs identity attitude |
| IMU FIFO overflow | Counted in stats; `FIFO_OVERFLOW` flag set in packet |
| Barometer fail | Barometer fields = 0 in log packet; `BARO_OK=0`; balance loop unaffected |
| Pi disconnects | Fast packets continue; log stream either drains or blocks (log task is lower priority) |
| Ring buffer overrun | `PACKET_OVERRUN` flag set; packet is dropped; sequence gap visible to receiver |
| Config CRC fail | Safe defaults used; `CONFIG_DIRTY` flag set |
