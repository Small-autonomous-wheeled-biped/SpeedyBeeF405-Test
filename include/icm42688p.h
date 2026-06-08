#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ---- WHO_AM_I ---- */
#define ICM42688P_WHOAMI_EXPECTED  0x47U

/* ---- Gyro full-scale range index (GYRO_CONFIG0 bits [7:5]) ---- */
#define ICM42688P_GYRO_FS_2000DPS  0U
#define ICM42688P_GYRO_FS_1000DPS  1U
#define ICM42688P_GYRO_FS_500DPS   2U
#define ICM42688P_GYRO_FS_250DPS   3U

/* ---- Accel full-scale range index (ACCEL_CONFIG0 bits [7:5]) ---- */
#define ICM42688P_ACCEL_FS_16G     0U
#define ICM42688P_ACCEL_FS_8G      1U
#define ICM42688P_ACCEL_FS_4G      2U
#define ICM42688P_ACCEL_FS_2G      3U

/* ---- ODR index (bits [3:0] in GYRO_CONFIG0 / ACCEL_CONFIG0) ---- */
#define ICM42688P_ODR_32000HZ      1U
#define ICM42688P_ODR_16000HZ      2U
#define ICM42688P_ODR_8000HZ       3U
#define ICM42688P_ODR_4000HZ       4U
#define ICM42688P_ODR_2000HZ       5U
#define ICM42688P_ODR_1000HZ       6U
#define ICM42688P_ODR_200HZ        7U

/* ---- Physical-unit sample (output of calibration + unit conversion) ---- */
typedef struct {
    float    gyro_x_rad_s;
    float    gyro_y_rad_s;
    float    gyro_z_rad_s;
    float    accel_x_m_s2;
    float    accel_y_m_s2;
    float    accel_z_m_s2;
    float    temperature_deg_c;
    uint32_t timestamp_us;   /* board_micros() at time of read */
} icm42688p_data_t;

/* ---- Raw ADC sample (used by calibration and unit tests) ---- */
typedef struct {
    int16_t temperature_raw;
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
} icm42688p_sample_t;

/* ---- Error / diagnostic counters ---- */
typedef struct {
    uint32_t fifo_overflow;
    uint32_t fifo_bad_header;
    uint32_t spi_errors;
    uint32_t sample_drops;
    uint32_t timestamp_jitter;
    uint32_t total_reads;
} icm42688p_stats_t;

/* ---- Driver API ---- */

/* Read WHO_AM_I register.  Returns 0x47 on a good device. */
uint8_t icm42688p_read_whoami(void);

/* Full initialization: reset, configure, enable FIFO in stream mode.
   gyro_fs  : ICM42688P_GYRO_FS_*  (default: ICM42688P_GYRO_FS_1000DPS)
   accel_fs : ICM42688P_ACCEL_FS_* (default: ICM42688P_ACCEL_FS_4G)
   odr_idx  : ICM42688P_ODR_*      (default: ICM42688P_ODR_1000HZ)
   Returns false if WHO_AM_I does not match.                                 */
bool icm42688p_init(uint8_t gyro_fs, uint8_t accel_fs, uint8_t odr_idx);

/* Flush FIFO.  Call after any configuration change. */
void icm42688p_flush_fifo(void);

/* Read a burst of up to max_samples from the FIFO.
   Fills out[], returns number of valid samples read.
   Returns 0 if FIFO is empty or an error occurred.                          */
uint16_t icm42688p_read_fifo(icm42688p_sample_t *out, uint16_t max_samples);

/* Polling (non-FIFO) read — for bring-up and unit tests.
   Returns false if sample is NULL.                                           */
bool icm42688p_read_sample(icm42688p_sample_t *sample);

/* Convert a raw sample to physical units using the configured FS ranges.
   gyro:  rad/s    accel: m/s²    temperature: °C                           */
void icm42688p_to_physical(const icm42688p_sample_t *raw,
                            icm42688p_data_t *out);

/* Return accumulated diagnostic counters (never resets automatically). */
const icm42688p_stats_t *icm42688p_stats(void);

/* Scale factors computed from FS range (exposed for unit tests). */
float icm42688p_gyro_scale_rad_s(void);
float icm42688p_accel_scale_m_s2(void);
