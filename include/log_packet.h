#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Diagnostic / logging packet — sent at 100–500 Hz to the Raspberry Pi
 * over USART3 (ASCII log) or a future high-speed link.
 *
 * This stream MUST NOT block or delay the fast balance packet output.
 * If the log UART is busy, the log packet is silently dropped.
 *
 * Fields include raw sensor data, calibrated data, Mahony internals,
 * barometer measurements, and timing diagnostics.
 */

#define LOG_PACKET_MAGIC    0x42F5U
#define LOG_PACKET_VERSION  1U
#define LOG_PACKET_TYPE     0x02U

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t  version;
    uint8_t  packet_type;
    uint16_t sequence;
    uint16_t _reserved;

    uint64_t timestamp_us;

    /* --- Calibrated (bias-removed, unit-converted) sensor data --- */
    float gyro_cal_x_rad_s;
    float gyro_cal_y_rad_s;
    float gyro_cal_z_rad_s;
    float accel_cal_x_m_s2;
    float accel_cal_y_m_s2;
    float accel_cal_z_m_s2;

    /* --- Filtered sensor data (same as in fast packet) --- */
    float gyro_filt_x_rad_s;
    float gyro_filt_y_rad_s;
    float gyro_filt_z_rad_s;
    float accel_filt_x_m_s2;
    float accel_filt_y_m_s2;
    float accel_filt_z_m_s2;

    /* --- Mahony estimator internals --- */
    float q_w, q_x, q_y, q_z;
    float roll_rad, pitch_rad, yaw_rad;
    float gyro_bias_x_rad_s;
    float gyro_bias_y_rad_s;
    float gyro_bias_z_rad_s;
    float mahony_integral_x;
    float mahony_integral_y;
    float mahony_integral_z;

    /* --- Barometer --- */
    float baro_pressure_pa;
    float baro_temp_deg_c;
    float baro_altitude_m;

    /* --- Timing diagnostics --- */
    uint32_t imu_dt_us;          /* actual dt of last IMU update             */
    uint32_t estimator_dt_us;    /* actual dt used in Mahony update          */
    float    vibe_gyro_rad2_s2;  /* vibration metric (gyro filtered vs raw)  */
    float    vibe_accel_m2_s4;   /* vibration metric (accel filtered vs raw) */

    /* --- Health --- */
    uint32_t health_flags;

    /* --- Debug counters --- */
    uint32_t fifo_overflow_count;
    uint32_t sample_drop_count;
    uint32_t packet_overrun_count;

    uint16_t crc16;
    uint8_t  _end_pad[2];
} log_packet_t;

#define LOG_PACKET_SIZE  ((size_t)(sizeof(log_packet_t)))

void log_packet_encode(log_packet_t   *pkt,
                       uint16_t       *seq,
                       uint64_t        timestamp_us,
                       const float     gyro_cal[3],
                       const float     accel_cal[3],
                       const float     gyro_filt[3],
                       const float     accel_filt[3],
                       const float     q[4],
                       float           roll_rad,
                       float           pitch_rad,
                       float           yaw_rad,
                       const float     gyro_bias[3],
                       const float     mahony_integral[3],
                       float           baro_pressure_pa,
                       float           baro_temp_deg_c,
                       float           baro_altitude_m,
                       uint32_t        imu_dt_us,
                       uint32_t        estimator_dt_us,
                       float           vibe_gyro,
                       float           vibe_accel,
                       uint32_t        health_flags,
                       uint32_t        fifo_overflow,
                       uint32_t        sample_drops,
                       uint32_t        packet_overruns);

bool log_packet_decode(const log_packet_t *pkt);

static inline const uint8_t *log_packet_bytes(const log_packet_t *pkt)
{
    return (const uint8_t *)pkt;
}
