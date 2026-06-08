#include "log_packet.h"
#include "crc.h"

#include <string.h>

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
                       uint32_t        packet_overruns)
{
    if ((pkt == NULL) || (seq == NULL)) { return; }

    pkt->magic       = LOG_PACKET_MAGIC;
    pkt->version     = LOG_PACKET_VERSION;
    pkt->packet_type = LOG_PACKET_TYPE;
    pkt->sequence    = *seq;
    pkt->_reserved   = 0U;
    (*seq)++;

    pkt->timestamp_us = timestamp_us;

    if (gyro_cal != NULL) {
        pkt->gyro_cal_x_rad_s = gyro_cal[0];
        pkt->gyro_cal_y_rad_s = gyro_cal[1];
        pkt->gyro_cal_z_rad_s = gyro_cal[2];
    }
    if (accel_cal != NULL) {
        pkt->accel_cal_x_m_s2 = accel_cal[0];
        pkt->accel_cal_y_m_s2 = accel_cal[1];
        pkt->accel_cal_z_m_s2 = accel_cal[2];
    }
    if (gyro_filt != NULL) {
        pkt->gyro_filt_x_rad_s = gyro_filt[0];
        pkt->gyro_filt_y_rad_s = gyro_filt[1];
        pkt->gyro_filt_z_rad_s = gyro_filt[2];
    }
    if (accel_filt != NULL) {
        pkt->accel_filt_x_m_s2 = accel_filt[0];
        pkt->accel_filt_y_m_s2 = accel_filt[1];
        pkt->accel_filt_z_m_s2 = accel_filt[2];
    }
    if (q != NULL) {
        pkt->q_w = q[0]; pkt->q_x = q[1];
        pkt->q_y = q[2]; pkt->q_z = q[3];
    }
    pkt->roll_rad  = roll_rad;
    pkt->pitch_rad = pitch_rad;
    pkt->yaw_rad   = yaw_rad;

    if (gyro_bias != NULL) {
        pkt->gyro_bias_x_rad_s = gyro_bias[0];
        pkt->gyro_bias_y_rad_s = gyro_bias[1];
        pkt->gyro_bias_z_rad_s = gyro_bias[2];
    }
    if (mahony_integral != NULL) {
        pkt->mahony_integral_x = mahony_integral[0];
        pkt->mahony_integral_y = mahony_integral[1];
        pkt->mahony_integral_z = mahony_integral[2];
    }

    pkt->baro_pressure_pa = baro_pressure_pa;
    pkt->baro_temp_deg_c  = baro_temp_deg_c;
    pkt->baro_altitude_m  = baro_altitude_m;

    pkt->imu_dt_us          = imu_dt_us;
    pkt->estimator_dt_us    = estimator_dt_us;
    pkt->vibe_gyro_rad2_s2  = vibe_gyro;
    pkt->vibe_accel_m2_s4   = vibe_accel;
    pkt->health_flags       = health_flags;
    pkt->fifo_overflow_count = fifo_overflow;
    pkt->sample_drop_count  = sample_drops;
    pkt->packet_overrun_count = packet_overruns;

    pkt->_end_pad[0] = 0U;
    pkt->_end_pad[1] = 0U;

    pkt->crc16 = crc16_ccitt((const uint8_t *)pkt,
                              offsetof(log_packet_t, crc16));
}

bool log_packet_decode(const log_packet_t *pkt)
{
    if (pkt == NULL) { return false; }
    if (pkt->magic   != LOG_PACKET_MAGIC)   { return false; }
    if (pkt->version != LOG_PACKET_VERSION) { return false; }
    const uint16_t computed = crc16_ccitt((const uint8_t *)pkt,
                                          offsetof(log_packet_t, crc16));
    return computed == pkt->crc16;
}
