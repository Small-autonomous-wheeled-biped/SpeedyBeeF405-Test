#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Health flags — carried in every fast packet so the balance controller
   can react to sensor/estimator problems.  Never output silence on error.  */
#define HEALTH_IMU_OK           (1U << 0U)
#define HEALTH_BARO_OK          (1U << 1U)
#define HEALTH_ESTIMATOR_READY  (1U << 2U)
#define HEALTH_ESTIMATOR_STARTUP (1U << 3U)
#define HEALTH_ACCEL_REJECTED   (1U << 4U)
#define HEALTH_HIGH_LINEAR_ACCEL (1U << 5U)
#define HEALTH_HIGH_VIBRATION   (1U << 6U)
#define HEALTH_GYRO_CLIPPED     (1U << 7U)
#define HEALTH_ACCEL_CLIPPED    (1U << 8U)
#define HEALTH_FIFO_OVERFLOW    (1U << 9U)
#define HEALTH_SAMPLE_DROPPED   (1U << 10U)
#define HEALTH_SPI_ERROR        (1U << 11U)
#define HEALTH_I2C_ERROR        (1U << 12U)
#define HEALTH_TIMESTAMP_JITTER (1U << 13U)
#define HEALTH_PACKET_OVERRUN   (1U << 14U)
#define HEALTH_CONFIG_DIRTY     (1U << 15U)
#define HEALTH_DT_INVALID       (1U << 16U)

/* Gyro clipping threshold (rad/s): flag if any axis approaches FS limit.
   At ±1000 dps FS: 95% = 950 dps ≈ 16.58 rad/s.                          */
#define HEALTH_GYRO_CLIP_THRESHOLD_RAD_S  16.0f
/* Accel clipping threshold (m/s²): 95% of ±4 g.                            */
#define HEALTH_ACCEL_CLIP_THRESHOLD_M_S2  37.3f
/* Vibration threshold (EMA of squared deviations, rad²/s²).               */
#define HEALTH_VIBE_THRESHOLD_RAD2_S2     0.25f

typedef struct {
    uint32_t flags;
    uint32_t packet_overrun_count;
    uint32_t spi_error_count;
    uint32_t i2c_error_count;
    uint32_t fifo_overflow_count;
    uint32_t sample_drop_count;
} health_monitor_t;

/* Update health flags from current sensor and estimator state.
   All inputs are the most recent values from their respective modules.      */
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
                   bool packet_overrun);

uint32_t health_get_flags(const health_monitor_t *h);
