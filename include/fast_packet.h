#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Fast balance packet — sent over USART1 at 500–1000 Hz.
 *
 * Designed for the main balance controller (e.g. another STM32 or a
 * tight RTOS loop).  All fields are little-endian.  A CRC-16/CCITT-FALSE
 * covers all bytes preceding the crc16 field.
 *
 * The balance controller must at minimum read pitch_rad and gyro_y_rad_s
 * for PID feedback.  All other fields provide redundancy and diagnostics.
 *
 * NOTE: yaw_rad is relative and drifts without an external yaw reference.
 *       Do not use yaw_rad for absolute heading in the balance controller.
 */

#define FAST_PACKET_MAGIC       0x42F4U   /* 'B' | 0xF400 — arbitrary unique */
#define FAST_PACKET_VERSION     1U
#define FAST_PACKET_TYPE        0x01U

/* Packed struct; fields are explicitly sized to avoid compiler padding.
   Total size is verified by a static assertion in fast_packet.c.           */
typedef struct __attribute__((packed)) {
    uint16_t magic;              /* FAST_PACKET_MAGIC = 0x42F4               */
    uint8_t  version;            /* FAST_PACKET_VERSION                       */
    uint8_t  packet_type;        /* FAST_PACKET_TYPE = 0x01                  */
    uint16_t sequence;           /* rolling counter, wraps at 65535          */
    uint16_t _reserved;          /* must be 0                                */

    uint64_t timestamp_us;       /* MCU board_micros() at packet generation  */
    uint32_t sample_age_us;      /* board_micros() - IMU read timestamp      */

    /* Mahony quaternion [w, x, y, z] — body-to-Earth (NED).
       Canonical internal attitude representation.                            */
    float    q_w;
    float    q_x;
    float    q_y;
    float    q_z;

    /* Euler angles (rad), ZYX convention.
       roll_rad, pitch_rad: gravity-referenced via accelerometer.
       yaw_rad: RELATIVE — drifts without magnetometer / vision / odometry.  */
    float    roll_rad;
    float    pitch_rad;
    float    yaw_rad;

    /* Calibrated + filtered body angular rates (rad/s).
       gyro_y_rad_s ≈ pitch_rate for a forward-balanced robot.               */
    float    gyro_x_rad_s;
    float    gyro_y_rad_s;
    float    gyro_z_rad_s;

    /* Calibrated + filtered specific force in body frame (m/s²). */
    float    accel_x_m_s2;
    float    accel_y_m_s2;
    float    accel_z_m_s2;

    /* Gravity vector in body frame derived from quaternion (m/s²).
       Useful for cross-checking and sensor fusion on the Pi side.           */
    float    gravity_body_x_m_s2;
    float    gravity_body_y_m_s2;
    float    gravity_body_z_m_s2;

    /* IMU temperature (°C). */
    float    temperature_deg_c;

    /* Health and status bitmask (HEALTH_* from health_monitor.h).
       The balance controller should halt or enter safe mode if
       HEALTH_IMU_OK or HEALTH_ESTIMATOR_READY are clear.                    */
    uint32_t health_flags;

    /* CRC-16/CCITT-FALSE over all preceding bytes (0 to offsetof(crc16)-1). */
    uint16_t crc16;
    uint8_t  _end_pad[2];    /* pad to 4-byte boundary                       */
} fast_packet_t;

/* Expected packet size: 4+4+8+4+16+12+12+12+12+4+4+2+2 = 96 bytes. */
#define FAST_PACKET_SIZE  96U

/* ---- Encode / decode ---- */

/* Fill packet fields and compute CRC.  seq is automatically incremented. */
void fast_packet_encode(fast_packet_t *pkt,
                        uint16_t      *seq,          /* in/out */
                        uint64_t       timestamp_us,
                        uint32_t       sample_age_us,
                        const float    q[4],         /* [w,x,y,z] */
                        float          roll_rad,
                        float          pitch_rad,
                        float          yaw_rad,
                        const float    gyro_rad_s[3],
                        const float    accel_m_s2[3],
                        const float    gravity_body[3],
                        float          temperature_deg_c,
                        uint32_t       health_flags);

/* Verify CRC and magic/version.  Returns true if packet is valid. */
bool fast_packet_decode(const fast_packet_t *pkt);

/* Return raw byte pointer and size for DMA / UART writes. */
static inline const uint8_t *fast_packet_bytes(const fast_packet_t *pkt)
{
    return (const uint8_t *)pkt;
}
