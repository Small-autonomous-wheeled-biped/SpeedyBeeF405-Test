#include "mahony_filter.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * Mahony nonlinear complementary filter — implementation.
 *
 * Quaternion convention: q = [w, x, y, z], body-to-Earth (NED).
 *   Identity [1,0,0,0] = sensor aligned with Earth frame.
 *   Gravity in Earth frame points in +z direction (down in NED).
 *
 * Gravity reference vector in body frame derived from current quaternion q:
 *   vx = 2*(q[1]*q[3] - q[0]*q[2])
 *   vy = 2*(q[0]*q[1] + q[2]*q[3])
 *   vz = q[0]^2 - q[1]^2 - q[2]^2 + q[3]^2
 * This is the third column of R^T(q), i.e. the direction of gravity in body
 * frame as seen from the sensor.  Derivation: q* ⊗ [0,0,0,1] ⊗ q.
 *
 * Error vector (cross product between measured and estimated gravity):
 *   e = a_hat × v
 * where a_hat is the normalised accelerometer measurement.
 * When a_hat ≈ v, e → 0 (no correction needed).
 *
 * Quaternion integration:
 *   qdot = 0.5 * q ⊗ ω_corrected
 *   q += qdot * dt
 *   q = q / ||q||
 *
 * Smooth gating:
 *   The accelerometer correction weight is reduced linearly from 1.0 to 0.0
 *   as ||accel|| moves from gate_lo to gate_hi (outside the band).
 *   This avoids sudden jumps that could excite the balance loop.
 */

static float safe_sqrtf(float x)
{
    return (x > 0.0f) ? sqrtf(x) : 0.0f;
}

/* Normalize a 4-element quaternion in place.
   Returns false if the norm is so small the quaternion is degenerate.      */
static bool quat_normalize(float q[4])
{
    const float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (n2 < 1e-10f) {
        /* Degenerate: reset to identity to avoid NaN propagation. */
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
        return false;
    }
    const float inv_n = 1.0f / safe_sqrtf(n2);
    q[0] *= inv_n; q[1] *= inv_n; q[2] *= inv_n; q[3] *= inv_n;
    return true;
}

/* ---- Public API ---- */

void mahony_init(mahony_t *m, float kp, float ki)
{
    if (m == NULL) { return; }
    memset(m, 0, sizeof(*m));

    /* Identity quaternion: no rotation from body to Earth. */
    m->q[0] = 1.0f;

    m->kp = kp;
    m->ki = ki;

    m->accel_gate_lo_m_s2 = 0.85f * GRAVITY_M_S2;
    m->accel_gate_hi_m_s2 = 1.15f * GRAVITY_M_S2;
    m->integral_max_rad_s = MAHONY_INTEGRAL_MAX;

    m->flags = MAHONY_FLAG_STARTUP;
}

void mahony_set_gains(mahony_t *m, float kp, float ki)
{
    if (m == NULL) { return; }
    m->kp = kp;
    m->ki = ki;
}

void mahony_set_accel_gate(mahony_t *m, float lo_g, float hi_g)
{
    if (m == NULL) { return; }
    m->accel_gate_lo_m_s2 = lo_g * GRAVITY_M_S2;
    m->accel_gate_hi_m_s2 = hi_g * GRAVITY_M_S2;
}

void mahony_reset_from_accel(mahony_t *m,
                              float ax_m_s2, float ay_m_s2, float az_m_s2)
{
    if (m == NULL) { return; }

    /* Compute roll and pitch from accelerometer (static tilt sense).
       Assumes sensor body z-axis is nominally aligned with Earth z (down).  */
    const float norm = safe_sqrtf(ax_m_s2*ax_m_s2
                                 + ay_m_s2*ay_m_s2
                                 + az_m_s2*az_m_s2);
    if (norm < 0.5f * GRAVITY_M_S2) {
        /* Accel too weak to trust — keep identity. */
        return;
    }

    const float inv_n = 1.0f / norm;
    const float ax = ax_m_s2 * inv_n;
    const float ay = ay_m_s2 * inv_n;
    const float az = az_m_s2 * inv_n;

    /* Roll angle from accel (rotation about x). */
    const float roll  = atan2f(ay, az);
    /* Pitch angle from accel (rotation about y). */
    const float pitch = atan2f(-ax, safe_sqrtf(ay*ay + az*az));

    /* Build quaternion from roll and pitch (yaw = 0).
       ZYX order: q = Rz(0) * Ry(pitch) * Rx(roll).                         */
    const float cr = cosf(roll  * 0.5f);
    const float sr = sinf(roll  * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);

    m->q[0] = cr * cp;
    m->q[1] = sr * cp;
    m->q[2] = cr * sp;
    m->q[3] = -sr * sp;

    quat_normalize(m->q);

    /* Reset integral to prevent transient from stale bias estimate. */
    m->integral[0] = 0.0f;
    m->integral[1] = 0.0f;
    m->integral[2] = 0.0f;
}

void mahony_update_imu(mahony_t *m,
                       const float gyro_rad_s[3],
                       const float accel_m_s2[3],
                       float dt_s)
{
    if ((m == NULL) || (gyro_rad_s == NULL) || (accel_m_s2 == NULL)) {
        return;
    }

    /* Sanity-check dt. */
    if ((dt_s < MAHONY_DT_MIN_S) || (dt_s > MAHONY_DT_MAX_S)) {
        m->flags |= MAHONY_FLAG_DT_INVALID;
        return;
    }
    m->flags &= ~MAHONY_FLAG_DT_INVALID;

    /* Working copies of gyro rates (may be modified by correction). */
    float gx = gyro_rad_s[0];
    float gy = gyro_rad_s[1];
    float gz = gyro_rad_s[2];

    /* ---- Accelerometer correction ---- */
    const float ax = accel_m_s2[0];
    const float ay = accel_m_s2[1];
    const float az = accel_m_s2[2];

    const float accel_norm = safe_sqrtf(ax*ax + ay*ay + az*az);

    /*
     * Smooth gating: compute correction weight in [0, 1].
     *   weight = 1.0  when norm is within [gate_lo, gate_hi]
     *   weight → 0.0  as norm approaches the gate boundary from outside
     *
     * For norms outside both gates the weight is 0 (no correction).
     */
    float accel_weight = 0.0f;
    const float gate_lo = m->accel_gate_lo_m_s2;
    const float gate_hi = m->accel_gate_hi_m_s2;

    if ((accel_norm >= gate_lo) && (accel_norm <= gate_hi)) {
        accel_weight = 1.0f;
    } else if ((accel_norm < gate_lo) && (gate_lo > 0.0f)) {
        /* Below lower gate: linearly ramp from 0 at 0.5*gate_lo to 1 at gate_lo. */
        const float half_lo = 0.5f * gate_lo;
        if (accel_norm > half_lo) {
            accel_weight = (accel_norm - half_lo) / (gate_lo - half_lo);
        }
    } else if (accel_norm > gate_hi) {
        /* Above upper gate: linearly ramp from 1 at gate_hi to 0 at 2*gate_hi. */
        const float two_hi = 2.0f * gate_hi;
        if (accel_norm < two_hi) {
            accel_weight = (two_hi - accel_norm) / gate_hi;
        }
    }

    if (accel_weight < 0.0f) { accel_weight = 0.0f; }
    if (accel_weight > 1.0f) { accel_weight = 1.0f; }

    /* Update status flags. */
    if (accel_weight < 1.0f) {
        m->flags |= MAHONY_FLAG_ACCEL_REJECTED;
        if (accel_norm > gate_hi) {
            m->flags |= MAHONY_FLAG_HIGH_ACCEL;
        } else {
            m->flags &= ~MAHONY_FLAG_HIGH_ACCEL;
        }
    } else {
        m->flags &= ~(MAHONY_FLAG_ACCEL_REJECTED | MAHONY_FLAG_HIGH_ACCEL);
    }

    /* On the very first call, warm-start from the accelerometer
       so pitch/roll start near the correct value immediately.               */
    if ((m->startup_count == 0U) && (accel_weight > 0.0f)) {
        mahony_reset_from_accel(m, ax, ay, az);
    }

    if (accel_weight > 0.0f) {
        /* Normalise accelerometer to get a unit gravity direction. */
        const float inv_an = 1.0f / accel_norm;
        const float a_nx = ax * inv_an;
        const float a_ny = ay * inv_an;
        const float a_nz = az * inv_an;

        /* Estimated gravity direction in body frame from current quaternion.
           v = R^T(q) * [0,0,1]^T  (third column of rotation matrix from body-to-Earth
           transposed = rotation from Earth-to-body applied to gravity unit vector).    */
        const float q0 = m->q[0], q1 = m->q[1], q2 = m->q[2], q3 = m->q[3];
        const float vx = 2.0f*(q1*q3 - q0*q2);
        const float vy = 2.0f*(q0*q1 + q2*q3);
        const float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        /* Error = measured × estimated (cross product). */
        const float ex = (a_ny*vz - a_nz*vy) * accel_weight;
        const float ey = (a_nz*vx - a_nx*vz) * accel_weight;
        const float ez = (a_nx*vy - a_ny*vx) * accel_weight;

        /* Integral feedback (freeze when correction is fully rejected). */
        if ((m->ki > 0.0f) && (accel_weight > 0.0f)) {
            m->integral[0] += ex * m->ki * dt_s;
            m->integral[1] += ey * m->ki * dt_s;
            m->integral[2] += ez * m->ki * dt_s;

            /* Anti-windup per axis. */
            for (uint8_t i = 0; i < 3U; i++) {
                if (m->integral[i] >  m->integral_max_rad_s) {
                    m->integral[i] =  m->integral_max_rad_s;
                }
                if (m->integral[i] < -m->integral_max_rad_s) {
                    m->integral[i] = -m->integral_max_rad_s;
                }
            }
        }

        /* Expose integral as estimated gyro bias (sign is opposite to bias).
           When e = 0 (gravity estimate matches accel), integral has converged
           to cancel the gyro bias.                                           */
        m->gyro_bias_estimate[0] = -m->integral[0];
        m->gyro_bias_estimate[1] = -m->integral[1];
        m->gyro_bias_estimate[2] = -m->integral[2];

        /* Apply proportional and integral correction to gyro rates. */
        gx += m->kp * ex + m->integral[0];
        gy += m->kp * ey + m->integral[1];
        gz += m->kp * ez + m->integral[2];
    }

    /* ---- Quaternion integration ---- */
    /* qdot = 0.5 * q ⊗ [0, gx, gy, gz]
       Expanded (q = [q0,q1,q2,q3]):
         dq0 = 0.5 * (-q1*gx - q2*gy - q3*gz)
         dq1 = 0.5 * ( q0*gx + q2*gz - q3*gy)
         dq2 = 0.5 * ( q0*gy - q1*gz + q3*gx)
         dq3 = 0.5 * ( q0*gz + q1*gy - q2*gx)             */
    const float q0 = m->q[0], q1 = m->q[1], q2 = m->q[2], q3 = m->q[3];
    const float hdt = 0.5f * dt_s;

    m->q[0] += (-q1*gx - q2*gy - q3*gz) * hdt;
    m->q[1] += ( q0*gx + q2*gz - q3*gy) * hdt;
    m->q[2] += ( q0*gy - q1*gz + q3*gx) * hdt;
    m->q[3] += ( q0*gz + q1*gy - q2*gx) * hdt;

    quat_normalize(m->q);

    /* ---- Startup counter ---- */
    if (m->startup_count < MAHONY_STARTUP_SAMPLES) {
        m->startup_count++;
    }
    if (m->startup_count >= MAHONY_STARTUP_SAMPLES) {
        m->flags &= ~MAHONY_FLAG_STARTUP;
        m->flags |=  MAHONY_FLAG_ESTIMATOR_OK;
    }
}

void mahony_get_quaternion(const mahony_t *m, float q[4])
{
    if ((m == NULL) || (q == NULL)) { return; }
    q[0] = m->q[0];
    q[1] = m->q[1];
    q[2] = m->q[2];
    q[3] = m->q[3];
}

void mahony_get_euler(const mahony_t *m,
                      float *roll_rad, float *pitch_rad, float *yaw_rad)
{
    if (m == NULL) { return; }
    const float q0 = m->q[0], q1 = m->q[1], q2 = m->q[2], q3 = m->q[3];

    /* ZYX Euler: apply Ry(pitch) * Rx(roll) first, then Rz(yaw).
       Roll (rotation about x): atan2(2*(w*x + y*z), 1 - 2*(x² + y²))      */
    if (roll_rad != NULL) {
        *roll_rad = atan2f(2.0f*(q0*q1 + q2*q3),
                           1.0f - 2.0f*(q1*q1 + q2*q2));
    }

    /* Pitch (rotation about y): asin(2*(w*y - z*x))
       Clamp to ±1 before asin to handle numerical edge case at ±90°.        */
    if (pitch_rad != NULL) {
        float sinp = 2.0f*(q0*q2 - q3*q1);
        if (sinp >  1.0f) { sinp =  1.0f; }
        if (sinp < -1.0f) { sinp = -1.0f; }
        *pitch_rad = asinf(sinp);
    }

    /* Yaw (rotation about z): atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
       NOTE: YAW IS RELATIVE — it drifts without a magnetometer or
       external yaw reference.  Do not use for absolute heading.             */
    if (yaw_rad != NULL) {
        *yaw_rad = atan2f(2.0f*(q0*q3 + q1*q2),
                          1.0f - 2.0f*(q2*q2 + q3*q3));
    }
}

void mahony_get_gravity_body(const mahony_t *m, float gravity[3])
{
    if ((m == NULL) || (gravity == NULL)) { return; }
    const float q0 = m->q[0], q1 = m->q[1], q2 = m->q[2], q3 = m->q[3];

    /* Same expression as the internal v vector, scaled to m/s². */
    gravity[0] = 2.0f*(q1*q3 - q0*q2)              * GRAVITY_M_S2;
    gravity[1] = 2.0f*(q0*q1 + q2*q3)              * GRAVITY_M_S2;
    gravity[2] = (q0*q0 - q1*q1 - q2*q2 + q3*q3)  * GRAVITY_M_S2;
}

void mahony_get_gyro_bias(const mahony_t *m, float bias[3])
{
    if ((m == NULL) || (bias == NULL)) { return; }
    bias[0] = m->gyro_bias_estimate[0];
    bias[1] = m->gyro_bias_estimate[1];
    bias[2] = m->gyro_bias_estimate[2];
}

uint32_t mahony_get_status_flags(const mahony_t *m)
{
    return (m != NULL) ? m->flags : 0U;
}
