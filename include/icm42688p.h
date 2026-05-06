#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ICM42688P_WHOAMI_EXPECTED 0x47U

typedef struct {
    int16_t temperature_raw;
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
} icm42688p_sample_t;

uint8_t icm42688p_read_whoami(void);
bool icm42688p_init(void);
bool icm42688p_read_sample(icm42688p_sample_t *sample);
