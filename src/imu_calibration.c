#include "imu_calibration.h"

#include <string.h>

void imu_calibration_apply(const imu_calibration_t *cal,
                            float gyro_rad_s[3],
                            float accel_m_s2[3])
{
    if ((cal == NULL) || (gyro_rad_s == NULL) || (accel_m_s2 == NULL)) {
        return;
    }

    for (uint8_t i = 0; i < 3U; i++) {
        gyro_rad_s[i]  -= cal->gyro_bias_rad_s[i];
        accel_m_s2[i]  -= cal->accel_bias_m_s2[i];
    }
}

bool imu_calibration_update_gyro_bias(imu_calibration_t *cal,
                                      const float gyro_rad_s[3])
{
    if ((cal == NULL) || (gyro_rad_s == NULL)) {
        return false;
    }

    cal->gyro_accum[0] += gyro_rad_s[0];
    cal->gyro_accum[1] += gyro_rad_s[1];
    cal->gyro_accum[2] += gyro_rad_s[2];
    cal->still_count++;

    if (cal->still_count >= CALIB_STILL_SAMPLES) {
        /* Average over the collection window. */
        const float inv_n = 1.0f / (float)CALIB_STILL_SAMPLES;
        cal->gyro_bias_rad_s[0] = cal->gyro_accum[0] * inv_n;
        cal->gyro_bias_rad_s[1] = cal->gyro_accum[1] * inv_n;
        cal->gyro_bias_rad_s[2] = cal->gyro_accum[2] * inv_n;

        cal->gyro_accum[0] = 0.0f;
        cal->gyro_accum[1] = 0.0f;
        cal->gyro_accum[2] = 0.0f;
        cal->still_count   = 0U;
        cal->calibrated    = true;
        return true;
    }
    return false;
}

void imu_calibration_reset_accumulator(imu_calibration_t *cal)
{
    if (cal == NULL) { return; }
    cal->gyro_accum[0] = 0.0f;
    cal->gyro_accum[1] = 0.0f;
    cal->gyro_accum[2] = 0.0f;
    cal->still_count   = 0U;
}

void imu_calibration_load(imu_calibration_t *cal,
                           const float gyro_bias[3],
                           const float accel_bias[3])
{
    if (cal == NULL) { return; }
    memset(cal, 0, sizeof(*cal));
    if (gyro_bias != NULL) {
        cal->gyro_bias_rad_s[0] = gyro_bias[0];
        cal->gyro_bias_rad_s[1] = gyro_bias[1];
        cal->gyro_bias_rad_s[2] = gyro_bias[2];
        /* Treat non-zero stored bias as already calibrated. */
        if ((gyro_bias[0] != 0.0f) || (gyro_bias[1] != 0.0f) || (gyro_bias[2] != 0.0f)) {
            cal->calibrated = true;
        }
    }
    if (accel_bias != NULL) {
        cal->accel_bias_m_s2[0] = accel_bias[0];
        cal->accel_bias_m_s2[1] = accel_bias[1];
        cal->accel_bias_m_s2[2] = accel_bias[2];
    }
}
