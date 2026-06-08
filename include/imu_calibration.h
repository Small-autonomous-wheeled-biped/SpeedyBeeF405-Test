#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * IMU calibration state and routines.
 *
 * Responsibility: apply stored gyro bias and accel bias/scale to raw
 * sensor data, and accumulate statistics for stationary-detection-based
 * gyro bias estimation.
 *
 * All values are in SI units:  gyro → rad/s,  accel → m/s².
 */

/* Number of samples averaged for still-stand gyro bias calibration. */
#define CALIB_STILL_SAMPLES  1000U

typedef struct {
    /* Bias subtracted from gyro measurements (rad/s). */
    float gyro_bias_rad_s[3];

    /* Bias subtracted from accel measurements (m/s²).
       Scale errors are not corrected in this initial implementation.        */
    float accel_bias_m_s2[3];

    /* Stationary detection state. */
    float  gyro_accum[3];       /* running sum during still-stand sampling  */
    uint32_t still_count;       /* samples collected so far                 */
    bool   calibrated;          /* gyro bias has been estimated at least once*/
} imu_calibration_t;

/*
 * Apply stored biases to raw (physical-unit) sensor data.
 * gyro[3] and accel[3] are modified in place.
 * Units: rad/s and m/s² respectively.
 */
void imu_calibration_apply(const imu_calibration_t *cal,
                            float gyro_rad_s[3],
                            float accel_m_s2[3]);

/*
 * Feed one sample into the stationary gyro bias estimator.
 * Returns true when CALIB_STILL_SAMPLES have been collected and the
 * bias has been updated.  Only call this when the stationarity detector
 * indicates the sensor is stationary.
 */
bool imu_calibration_update_gyro_bias(imu_calibration_t *cal,
                                      const float gyro_rad_s[3]);

/* Reset the accumulator (e.g. motion detected mid-calibration). */
void imu_calibration_reset_accumulator(imu_calibration_t *cal);

/* Load biases from a config struct. */
void imu_calibration_load(imu_calibration_t *cal,
                           const float gyro_bias[3],
                           const float accel_bias[3]);
