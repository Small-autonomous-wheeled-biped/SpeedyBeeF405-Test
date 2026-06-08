# Wire Protocol

All packets are binary, little-endian.  A CRC-16/CCITT-FALSE (polynomial
0x1021, init 0xFFFF, no reflection) covers all bytes before the `crc16` field.

## Fast attitude packet (USART1, 2 Mbaud, 500 Hz)

Magic `0x42F4`, type `0x01`, version `1`.  Total size: **96 bytes**.

| Offset | Size | Field | Description |
|---:|---:|---|---|
| 0 | 2 | `magic` | `0x42F4` |
| 2 | 1 | `version` | `1` |
| 3 | 1 | `packet_type` | `0x01` |
| 4 | 2 | `sequence` | Rolling counter (0–65535) |
| 6 | 2 | `_reserved` | Must be 0 |
| 8 | 8 | `timestamp_us` | MCU microseconds at packet generation |
| 16 | 4 | `sample_age_us` | Age of IMU sample at packet time (µs) |
| 20 | 4 | `q_w` | Quaternion w (body-to-Earth, NED) |
| 24 | 4 | `q_x` | Quaternion x |
| 28 | 4 | `q_y` | Quaternion y |
| 32 | 4 | `q_z` | Quaternion z |
| 36 | 4 | `roll_rad` | Roll (rad), gravity-referenced |
| 40 | 4 | `pitch_rad` | Pitch (rad), gravity-referenced |
| 44 | 4 | `yaw_rad` | **Yaw (rad), RELATIVE — drifts** |
| 48 | 4 | `gyro_x_rad_s` | Body angular rate x (rad/s) |
| 52 | 4 | `gyro_y_rad_s` | Body angular rate y (rad/s) — pitch rate for forward-balanced robot |
| 56 | 4 | `gyro_z_rad_s` | Body angular rate z (rad/s) |
| 60 | 4 | `accel_x_m_s2` | Specific force x (m/s²) |
| 64 | 4 | `accel_y_m_s2` | Specific force y (m/s²) |
| 68 | 4 | `accel_z_m_s2` | Specific force z (m/s²) |
| 72 | 4 | `gravity_body_x_m_s2` | Estimated gravity in body frame x (m/s²) |
| 76 | 4 | `gravity_body_y_m_s2` | Estimated gravity in body frame y (m/s²) |
| 80 | 4 | `gravity_body_z_m_s2` | Estimated gravity in body frame z (m/s²) |
| 84 | 4 | `temperature_deg_c` | IMU temperature (°C) |
| 88 | 4 | `health_flags` | Bitmask (see below) |
| 92 | 2 | `crc16` | CRC-16/CCITT-FALSE over bytes 0–91 |
| 94 | 2 | `_end_pad` | Padding (0x0000) |

### Health flags (bit positions)

| Bit | Name | Meaning when SET |
|---:|---|---|
| 0 | `IMU_OK` | ICM-42688-P initialised and responding |
| 1 | `BARO_OK` | DPS310 initialised and reading |
| 2 | `ESTIMATOR_READY` | Mahony filter has converged (startup done) |
| 3 | `ESTIMATOR_STARTUP` | Still in startup phase (< 200 ms) |
| 4 | `ACCEL_REJECTED` | Accelerometer correction partially/fully gated |
| 5 | `HIGH_LINEAR_ACCEL` | Accel norm significantly above 1 g |
| 6 | `HIGH_VIBRATION` | Vibration metric above threshold |
| 7 | `GYRO_CLIPPED` | Gyro reading near FS limit |
| 8 | `ACCEL_CLIPPED` | Accel reading near FS limit |
| 9 | `FIFO_OVERFLOW` | ICM-42688-P FIFO near full |
| 10 | `SAMPLE_DROPPED` | FIFO samples dropped (ring buffer too small) |
| 11 | `SPI_ERROR` | SPI communication error |
| 12 | `I2C_ERROR` | I2C communication error |
| 13 | `TIMESTAMP_JITTER` | dt exceeded sanity bounds |
| 14 | `PACKET_OVERRUN` | USART1 TX ring buffer full; packet dropped |
| 15 | `CONFIG_DIRTY` | Config CRC bad; using defaults |
| 16 | `DT_INVALID` | dt_s outside [0.2 ms, 50 ms]; Mahony update skipped |

**Balance controller safe-mode trigger:**  
Stop / hold safe position when `IMU_OK = 0` OR `ESTIMATOR_READY = 0`.

### Minimum balance-controller fields

The balance controller only needs two fields for PID feedback:
- `pitch_rad` — forward/backward tilt angle (gravity-referenced)
- `gyro_y_rad_s` — pitch rate (body frame, rad/s)

All other fields are available for sanity checking and feed-forward.

## Diagnostic / log packet (USART3, 115200 baud, 100 Hz)

Magic `0x42F5`, type `0x02`, version `1`.  Includes raw calibrated data,
filtered data, full Mahony internals, barometer, and timing diagnostics.

See `log_packet.h` for the complete field list.  CRC covers all bytes before
the `crc16` field using the same CRC-16/CCITT-FALSE algorithm.

## Receiver implementation notes

1. Scan for the magic bytes `0xF4 0x42` (little-endian `0x42F4`).
2. Once found, read the remaining 94 bytes (total 96 bytes from the magic start).
3. Verify magic, version, and type fields.
4. Compute CRC over bytes 0–91 and compare with bytes 92–93.
5. If CRC fails, discard and resync (look for next magic).
6. Check `sequence` for gaps to detect dropped packets.

The Python decoder at `tools/host_decoder.py` implements this with a
byte-by-byte state machine and CRC verification.
