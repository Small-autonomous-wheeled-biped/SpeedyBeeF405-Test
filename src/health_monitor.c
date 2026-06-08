#include "health_monitor.h"
#include "mahony_filter.h"

#include <math.h>
#include <string.h>

void health_update(health_monitor_t *h,
                   bool imu_ok,
                   bool baro_ok,
                   uint32_t mahony_flags,
                   const float gyro_filt[3],
                   const float accel_filt[3],
                   float vibe_gyro_rad2_s2,
                   uint32_t fifo_overflow,
                   uint32_t sample_drops,
                   uint32_t spi_errors,
                   uint32_t i2c_errors,
                   bool packet_overrun)
{
    if (h == NULL) { return; }

    uint32_t flags = 0U;

    if (imu_ok)  { flags |= HEALTH_IMU_OK; }
    if (baro_ok) { flags |= HEALTH_BARO_OK; }

    /* Estimator state from Mahony flags. */
    if ((mahony_flags & MAHONY_FLAG_ESTIMATOR_OK) != 0U) {
        flags |= HEALTH_ESTIMATOR_READY;
    }
    if ((mahony_flags & MAHONY_FLAG_STARTUP) != 0U) {
        flags |= HEALTH_ESTIMATOR_STARTUP;
    }
    if ((mahony_flags & MAHONY_FLAG_ACCEL_REJECTED) != 0U) {
        flags |= HEALTH_ACCEL_REJECTED;
    }
    if ((mahony_flags & MAHONY_FLAG_HIGH_ACCEL) != 0U) {
        flags |= HEALTH_HIGH_LINEAR_ACCEL;
    }
    if ((mahony_flags & MAHONY_FLAG_DT_INVALID) != 0U) {
        flags |= HEALTH_DT_INVALID;
    }

    /* Gyro / accel clipping. */
    if (gyro_filt != NULL) {
        for (uint8_t i = 0; i < 3U; i++) {
            if (fabsf(gyro_filt[i]) >= HEALTH_GYRO_CLIP_THRESHOLD_RAD_S) {
                flags |= HEALTH_GYRO_CLIPPED;
                break;
            }
        }
    }
    if (accel_filt != NULL) {
        for (uint8_t i = 0; i < 3U; i++) {
            if (fabsf(accel_filt[i]) >= HEALTH_ACCEL_CLIP_THRESHOLD_M_S2) {
                flags |= HEALTH_ACCEL_CLIPPED;
                break;
            }
        }
    }

    /* Vibration. */
    if (vibe_gyro_rad2_s2 > HEALTH_VIBE_THRESHOLD_RAD2_S2) {
        flags |= HEALTH_HIGH_VIBRATION;
    }

    /* Counters. */
    if (fifo_overflow > h->fifo_overflow_count) {
        flags |= HEALTH_FIFO_OVERFLOW;
        h->fifo_overflow_count = fifo_overflow;
    }
    if (sample_drops > h->sample_drop_count) {
        flags |= HEALTH_SAMPLE_DROPPED;
        h->sample_drop_count = sample_drops;
    }
    if (spi_errors > h->spi_error_count) {
        flags |= HEALTH_SPI_ERROR;
        h->spi_error_count = spi_errors;
    }
    if (i2c_errors > h->i2c_error_count) {
        flags |= HEALTH_I2C_ERROR;
        h->i2c_error_count = i2c_errors;
    }
    if (packet_overrun) {
        flags |= HEALTH_PACKET_OVERRUN;
        h->packet_overrun_count++;
    }

    h->flags = flags;
}

uint32_t health_get_flags(const health_monitor_t *h)
{
    return (h != NULL) ? h->flags : 0U;
}
