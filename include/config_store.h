#pragma once

#include <stdbool.h>
#include <stdint.h>

#define CONFIG_MAGIC    0x42F40001UL   /* magic | version word */
#define CONFIG_VERSION  1U

/*
 * All runtime-tunable parameters in one flat struct.
 * Stored in flash (Sector 3) with a CRC16 at the end.
 * Safe defaults are applied when flash is blank or CRC fails.
 * Never write to flash inside a real-time path.
 */
typedef struct {
    uint32_t magic;            /* CONFIG_MAGIC when valid                    */
    uint16_t version;
    uint16_t struct_size;      /* sizeof(config_t), guards against version mismatch */

    /* --- Mahony filter gains --- */
    float mahony_kp;           /* proportional gain (default 2.0)            */
    float mahony_ki;           /* integral gain     (default 0.005)          */

    /* --- Accelerometer correction gating --- */
    float accel_gate_lo_g;     /* lower bound on accel norm [g] (default 0.85) */
    float accel_gate_hi_g;     /* upper bound on accel norm [g] (default 1.15) */

    /* --- ICM42688-P sensor config --- */
    uint8_t gyro_fs_dps;       /* 0=2000,1=1000,2=500,3=250 (index, default 1=1000) */
    uint8_t accel_fs_g;        /* 0=16,1=8,2=4,3=2 (index, default 2=4)      */
    uint8_t imu_odr_idx;       /* ODR index per ICM42688-P table (default 6=1kHz) */
    uint8_t _pad0;

    /* --- Axis remap (board rotation) --- */
    /* sign_map[i] ∈ {-1, 0, +1}, axis_map[i] ∈ {0,1,2}
       board_frame[i] = sign_map[i] * sensor_frame[axis_map[i]]
       Default: ROTATION_PITCH_180_YAW_90 as used by ArduPilot for this target.
       That maps: x_b=-y_s, y_b=-x_s, z_b=-z_s                             */
    int8_t  axis_map[3];       /* which sensor axis feeds board axis i        */
    int8_t  axis_sign[3];      /* sign of that axis (+1 or -1)                */
    uint8_t _pad1[2];

    /* --- Soft low-pass filter cutoffs (0 = bypass) --- */
    float   gyro_lpf_hz;       /* default 80 Hz (keep low for balance latency)*/
    float   accel_lpf_hz;      /* default 40 Hz                               */

    /* --- Output rates (Hz) --- */
    uint16_t fast_packet_rate_hz;   /* default 500 Hz  */
    uint16_t log_packet_rate_hz;    /* default 100 Hz  */
    uint16_t baro_sample_rate_hz;   /* default 25 Hz   */
    uint16_t _pad2;

    /* --- Gyro bias calibration (rad/s, subtracted from raw) --- */
    float   gyro_bias_rad_s[3];

    /* --- Accel bias calibration (m/s², subtracted from calibrated) --- */
    float   accel_bias_m_s2[3];

    /* Integrity check — computed over all preceding bytes. */
    uint16_t crc16;
    uint16_t _end_pad;
} config_t;

/* Load config from flash into *cfg.  Returns true if flash data was valid.
   Always populates *cfg — with flash data or with safe defaults.           */
bool config_load(config_t *cfg);

/* Write cfg to flash (Sector 3).  Blocks until complete.
   Do not call from a real-time interrupt or tight loop.                     */
bool config_save(const config_t *cfg);

/* Fill *cfg with compile-time safe defaults. */
void config_set_defaults(config_t *cfg);

/* Return true if the CRC in cfg matches the computed CRC. */
bool config_crc_ok(const config_t *cfg);
