# Calibration Guide

## Gyro bias calibration

The gyro bias is the DC offset of the angular rate sensor when the sensor is
completely stationary.  Even 1–2 °/s of uncorrected bias causes noticeable yaw
drift and degrades pitch/roll accuracy over time.

### Automatic still-stand calibration (firmware)

The firmware continuously checks for stationary conditions and accumulates
samples when stationary:

- **Stationary threshold**: all gyro axes < 0.05 rad/s (~3 °/s) and accel norm
  within ±10% of 1 g.
- **Accumulation window**: 1000 samples at 1 kHz = 1 second stationary.
- **Auto-update**: after 1 second of stillness the gyro bias estimate is updated
  and stored in the running config.  It is also optionally saved to flash (call
  `config_save()` from a CLI command — do not call from a real-time path).

### Manual calibration procedure

1. Place the robot on a flat, vibration-free surface.
2. Do not touch the robot for at least 5 seconds.
3. The firmware will detect the still condition and update the bias estimate
   within 1 second of continuous stillness.
4. Verify in the diagnostic log (`gyro_bias_*_rad_s` fields).
5. Optionally trigger a config save (via a future CLI command or USART3 ASCII
   command) to persist the bias across power cycles.

### Expected gyro bias values

For the ICM-42688-P at room temperature, expect:

| Axis | Typical range |
|---|---|
| X | ±0.01 rad/s (±0.6 °/s) |
| Y | ±0.01 rad/s (±0.6 °/s) |
| Z | ±0.01 rad/s (±0.6 °/s) |

Biases > 0.05 rad/s (3 °/s) suggest a problem (wrong FS range, poor soldering,
or a very warm sensor).

## Attitude startup calibration

On power-up the Mahony filter is warm-started from the accelerometer:

1. `mahony_reset_from_accel()` is called on the first update.
2. Roll and pitch are set from the accelerometer reading (gravity direction).
3. Yaw is set to 0 (unknown — no magnetometer).
4. After 200 ms (200 updates at 1 kHz) the `ESTIMATOR_READY` flag is set.

### Startup in non-level conditions

If the board powers up while tilted (e.g. on a ramp), the filter starts at
the correct tilt angle.  The balance controller should not command motion until
`ESTIMATOR_READY = 1` and `ESTIMATOR_STARTUP = 0`.

## Accelerometer bias calibration (placeholder)

The current firmware applies only a static bias offset to the accelerometer
(`accel_bias_m_s2[3]` in `config_t`).  Full 6-point or 3-point calibration
(scale factors + cross-axis) is not yet implemented.

For a balancing robot the accelerometer is used only to correct gyro drift and
not as a primary measurement, so scale errors have minimal impact.  Zero-level
bias can be estimated by averaging over many stationary periods.

## Board rotation

The firmware applies a fixed rotation matrix from sensor frame to vehicle body
frame.  The default is `ROTATION_PITCH_180_YAW_90` (as used by ArduPilot for
this board target):

```
x_body = -y_sensor
y_body = -x_sensor
z_body = -z_sensor
```

This is configurable in `config_t.axis_map[]` and `config_t.axis_sign[]`.  To
change it:

1. Measure the actual relationship between sensor axes and robot body axes with
   a known rotation (e.g. pitch the robot 90° forward, observe which axis
   changes sign and by how much).
2. Update `axis_map` and `axis_sign` in the config struct.
3. Call `config_save()`.

## Temperature compensation

The ICM-42688-P gyro bias changes slightly with temperature.  The current
firmware does not implement temperature compensation.  For environments with
large temperature swings (> 20 °C), run calibration at operating temperature
or implement a linear temperature model on the Pi side.
