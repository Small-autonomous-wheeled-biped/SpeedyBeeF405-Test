#include "fast_packet.h"
#include "crc.h"

#include <stddef.h>
#include <string.h>

/* Verify at compile time that the struct is exactly the expected size.
   If this fails, a field was added/removed or alignment differed.           */
_Static_assert(sizeof(fast_packet_t) == FAST_PACKET_SIZE,
               "fast_packet_t size mismatch — check fields and padding");

void fast_packet_encode(fast_packet_t *pkt,
                        uint16_t      *seq,
                        uint64_t       timestamp_us,
                        uint32_t       sample_age_us,
                        const float    q[4],
                        float          roll_rad,
                        float          pitch_rad,
                        float          yaw_rad,
                        const float    gyro_rad_s[3],
                        const float    accel_m_s2[3],
                        const float    gravity_body[3],
                        float          temperature_deg_c,
                        uint32_t       health_flags)
{
    if ((pkt == NULL) || (seq == NULL)) {
        return;
    }

    pkt->magic       = FAST_PACKET_MAGIC;
    pkt->version     = FAST_PACKET_VERSION;
    pkt->packet_type = FAST_PACKET_TYPE;
    pkt->sequence    = *seq;
    pkt->_reserved   = 0U;
    (*seq)++;

    pkt->timestamp_us  = timestamp_us;
    pkt->sample_age_us = sample_age_us;

    if (q != NULL) {
        pkt->q_w = q[0];
        pkt->q_x = q[1];
        pkt->q_y = q[2];
        pkt->q_z = q[3];
    }

    pkt->roll_rad  = roll_rad;
    pkt->pitch_rad = pitch_rad;
    pkt->yaw_rad   = yaw_rad;

    if (gyro_rad_s != NULL) {
        pkt->gyro_x_rad_s = gyro_rad_s[0];
        pkt->gyro_y_rad_s = gyro_rad_s[1];
        pkt->gyro_z_rad_s = gyro_rad_s[2];
    }

    if (accel_m_s2 != NULL) {
        pkt->accel_x_m_s2 = accel_m_s2[0];
        pkt->accel_y_m_s2 = accel_m_s2[1];
        pkt->accel_z_m_s2 = accel_m_s2[2];
    }

    if (gravity_body != NULL) {
        pkt->gravity_body_x_m_s2 = gravity_body[0];
        pkt->gravity_body_y_m_s2 = gravity_body[1];
        pkt->gravity_body_z_m_s2 = gravity_body[2];
    }

    pkt->temperature_deg_c = temperature_deg_c;
    pkt->health_flags      = health_flags;
    pkt->_end_pad[0]       = 0U;
    pkt->_end_pad[1]       = 0U;

    /* CRC over all bytes before the crc16 field. */
    pkt->crc16 = crc16_ccitt((const uint8_t *)pkt,
                              offsetof(fast_packet_t, crc16));
}

bool fast_packet_decode(const fast_packet_t *pkt)
{
    if (pkt == NULL) {
        return false;
    }
    if (pkt->magic != FAST_PACKET_MAGIC) {
        return false;
    }
    if (pkt->version != FAST_PACKET_VERSION) {
        return false;
    }
    if (pkt->packet_type != FAST_PACKET_TYPE) {
        return false;
    }
    const uint16_t computed = crc16_ccitt((const uint8_t *)pkt,
                                          offsetof(fast_packet_t, crc16));
    return computed == pkt->crc16;
}
