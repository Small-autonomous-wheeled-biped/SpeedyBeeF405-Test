#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Mahony nonlinear complementary attitude filter.
 *
 * Reference:
 *   R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear Complementary Filters on
 *   the Special Orthogonal Group", IEEE TAC, 2008.
 *
 * Internal attitude representation: unit quaternion q = [w, x, y, z]
 *   representing the rotation from body frame to Earth frame (NED).
 *
 * Outputs:
 *   - Quaternion (canonical internal state, always normalized).
 *   - Euler angles (roll, pitch in rad) gravity-referenced via accelerometer.
 *   - Yaw (relative only — drifts without an external yaw reference such as
 *     a magnetometer or vision/wheel odometry fusion).
 *   - Gravity vector in body frame (from quaternion, in m/s²).
 *
 * Accelerometer correction is gated:
 *   If ||accel|| deviates from 1 g by more than the configured thresholds,
 *   the correction is reduced (smooth gate) or disabled (hard gate).
 *   This prevents high-linear-acceleration events from corrupting attitude.
 *
 * Gyro bias estimation (integral term, Ki):
 *   The integral term accumulates the cross-product error between the
 *   estimated and measured gravity directions.  It converges to the gyro
 *   bias in body frame when the sensor is stationary and the accelerometer
 *   correction is trusted.  Anti-windup limits are applied.
 *   Integral is frozen when accelerometer correction is rejected.
 *
 * Sample-time handling:
 *   The filter uses the measured dt_s from hardware timestamps, not a
 *   compile-time constant.  If dt_s is outside a sane range, the update
 *   is skipped and a flag is set.
 */

#define MAHONY_FLAG_STARTUP        (1U << 0U)  /* < STARTUP_SAMPLES collected */
#define MAHONY_FLAG_ESTIMATOR_OK   (1U << 1U)  /* filter converged            */
#define MAHONY_FLAG_ACCEL_REJECTED (1U << 2U)  /* accel norm out of range     */
#define MAHONY_FLAG_HIGH_ACCEL     (1U << 3U)  /* ||a|| significantly > 1 g   */
#define MAHONY_FLAG_DT_INVALID     (1U << 4U)  /* dt_s outside sanity bounds  */

typedef struct {
    /* Quaternion [w, x, y, z] — body to Earth (NED).
       Initialized to identity [1,0,0,0] = level, heading North.            */
    float q[4];

    /* Integral feedback for gyro bias estimation (body frame, rad/s). */
    float integral[3];

    /* Estimated gyro bias from integral term (rad/s).
       Only meaningful after the integral has converged (Ki > 0).           */
    float gyro_bias_estimate[3];

    /* Filter gains. */
    float kp;   /* proportional: determines bandwidth for accel correction  */
    float ki;   /* integral: determines gyro bias estimation rate           */

    /* Accelerometer correction gate (in units of g = 9.80665 m/s²).
       Correction weight is reduced linearly between gate_lo and gate_hi.   */
    float accel_gate_lo_m_s2;  /* e.g. 0.85 * 9.80665 */
    float accel_gate_hi_m_s2;  /* e.g. 1.15 * 9.80665 */
    float integral_max_rad_s;  /* anti-windup limit per axis                */

    /* Startup counter: wait for MAHONY_STARTUP_SAMPLES before setting
       ESTIMATOR_OK to allow attitude to converge from gravity.              */
    uint32_t startup_count;

    /* Status flags — read by health_monitor. */
    uint32_t flags;
} mahony_t;

/* ---- API ---- */

/* Initialise filter state.  Initial attitude is estimated from gravity on the
   first call to mahony_update_imu() (warm start from accelerometer).        */
void mahony_init(mahony_t *m, float kp, float ki);

/* Change gains at runtime. */
void mahony_set_gains(mahony_t *m, float kp, float ki);

/* Set accelerometer correction gate (in g).  0 = always correct.           */
void mahony_set_accel_gate(mahony_t *m, float lo_g, float hi_g);

/* Force-initialize attitude from the current accelerometer reading.
   Roll and pitch will match gravity; yaw is set to zero (unknown).          */
void mahony_reset_from_accel(mahony_t *m,
                              float ax_m_s2, float ay_m_s2, float az_m_s2);

/*
 * Core update — call at the estimator update rate (e.g. 1 kHz).
 *
 * gyro_rad_s[3]: calibrated + filtered angular rate (rad/s, body frame).
 * accel_m_s2[3]: calibrated + filtered specific force (m/s², body frame).
 * dt_s:          elapsed time since previous update (seconds).
 *                Must be > 0 and < MAHONY_DT_MAX_S.
 */
void mahony_update_imu(mahony_t *m,
                       const float gyro_rad_s[3],
                       const float accel_m_s2[3],
                       float dt_s);

/* Get the current unit quaternion [w, x, y, z].  q must point to 4 floats. */
void mahony_get_quaternion(const mahony_t *m, float q[4]);

/* Get Euler angles (rad), ZYX convention.
   roll:  rotation about x-axis, gravity-referenced.
   pitch: rotation about y-axis, gravity-referenced.
   yaw:   rotation about z-axis, RELATIVE — drifts without external reference. */
void mahony_get_euler(const mahony_t *m,
                      float *roll_rad, float *pitch_rad, float *yaw_rad);

/* Get estimated gravity vector in body frame (m/s²).
   Direction is derived from the current quaternion.  Magnitude = 9.80665.   */
void mahony_get_gravity_body(const mahony_t *m, float gravity[3]);

/* Get estimated gyro bias from integral term (rad/s). */
void mahony_get_gyro_bias(const mahony_t *m, float bias[3]);

/* Get status flags (MAHONY_FLAG_*). */
uint32_t mahony_get_status_flags(const mahony_t *m);

/* Number of updates to wait before declaring estimator ready. */
#define MAHONY_STARTUP_SAMPLES  200U  /* 200 ms at 1 kHz */
#define MAHONY_DT_MIN_S         0.0002f   /* 5 kHz max        */
#define MAHONY_DT_MAX_S         0.05f     /* 20 Hz min        */
#define MAHONY_INTEGRAL_MAX     0.1f      /* rad/s anti-windup */
#define GRAVITY_M_S2            9.80665f
