#include "mahony_filter.h"

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdlib.h>
#include <string.h>

/* Tolerance for float comparisons. */
#define TOL_LOOSE  0.02f
#define TOL_TIGHT  0.001f

static void assert_near(float actual, float expected, float tol,
                         const char *name, const char *file, int line)
{
    const float diff = actual - expected;
    const float abs_diff = (diff < 0.0f) ? -diff : diff;
    if (abs_diff > tol) {
        fprintf(stderr, "%s:%d: FAILED %s: expected %.6f got %.6f (diff=%.6f, tol=%.6f)\n",
                file, line, name, (double)expected, (double)actual,
                (double)diff, (double)tol);
        exit(1);
    }
}

#define ASSERT_NEAR(actual, expected, tol) \
    assert_near((actual), (expected), (tol), #actual, __FILE__, __LINE__)

static void assert_true(bool passed, const char *expr, const char *file, int line)
{
    if (!passed) {
        fprintf(stderr, "%s:%d: FAILED: %s\n", file, line, expr);
        exit(1);
    }
}

#define ASSERT_TRUE(expr) assert_true((expr), #expr, __FILE__, __LINE__)
#define ASSERT_FALSE(expr) assert_true(!(expr), "!(" #expr ")", __FILE__, __LINE__)

/* ---- Tests ---- */

static void test_mahony_init_identity_quaternion(void)
{
    mahony_t m;
    mahony_init(&m, 2.0f, 0.005f);
    float q[4];
    mahony_get_quaternion(&m, q);
    ASSERT_NEAR(q[0], 1.0f, TOL_TIGHT);
    ASSERT_NEAR(q[1], 0.0f, TOL_TIGHT);
    ASSERT_NEAR(q[2], 0.0f, TOL_TIGHT);
    ASSERT_NEAR(q[3], 0.0f, TOL_TIGHT);
    ASSERT_TRUE((mahony_get_status_flags(&m) & MAHONY_FLAG_STARTUP) != 0U);
}

static void test_mahony_stationary_convergence(void)
{
    /* Simulate a stationary sensor: gravity pointing straight down (+z),
       no gyro rotation.  After enough updates the filter should converge
       to near-zero roll and pitch.                                           */
    mahony_t m;
    mahony_init(&m, 2.0f, 0.01f);

    const float gyro[3]  = { 0.0f, 0.0f, 0.0f };
    const float accel[3] = { 0.0f, 0.0f, 9.80665f };  /* gravity down */
    const float dt_s     = 0.001f;

    for (int i = 0; i < 2000; i++) {
        mahony_update_imu(&m, gyro, accel, dt_s);
    }

    float roll_rad, pitch_rad, yaw_rad;
    mahony_get_euler(&m, &roll_rad, &pitch_rad, &yaw_rad);

    /* With gravity aligned with body z, roll and pitch should be ~0. */
    ASSERT_NEAR(roll_rad,  0.0f, TOL_LOOSE);
    ASSERT_NEAR(pitch_rad, 0.0f, TOL_LOOSE);

    /* Estimator should be ready. */
    ASSERT_TRUE((mahony_get_status_flags(&m) & MAHONY_FLAG_ESTIMATOR_OK) != 0U);
}

static void test_mahony_stationary_tilted(void)
{
    /* Simulate 45° roll (gravity in +y and +z equally). */
    mahony_t m;
    mahony_init(&m, 2.0f, 0.0f);

    const float g = 9.80665f;
    const float gyro[3]  = { 0.0f, 0.0f, 0.0f };
    /* 45° roll: ax=0, ay=g*sin(45°), az=g*cos(45°) */
    const float sq2_over2 = 0.70711f;
    const float accel[3] = { 0.0f, g * sq2_over2, g * sq2_over2 };
    const float dt_s     = 0.001f;

    for (int i = 0; i < 3000; i++) {
        mahony_update_imu(&m, gyro, accel, dt_s);
    }

    float roll_rad, pitch_rad, yaw_rad;
    mahony_get_euler(&m, &roll_rad, &pitch_rad, &yaw_rad);

    /* Expected roll ≈ 45° = π/4 ≈ 0.7854 rad, pitch ≈ 0. */
    const float pi_over_4 = 0.7854f;
    ASSERT_NEAR(roll_rad,  pi_over_4, 0.05f);
    ASSERT_NEAR(pitch_rad, 0.0f, TOL_LOOSE);
}

static void test_mahony_accel_rejection_high_accel(void)
{
    /* Simulate a high-linear-acceleration event: accel norm >> 1 g.
       The filter should flag ACCEL_REJECTED and HIGH_ACCEL.                 */
    mahony_t m;
    mahony_init(&m, 2.0f, 0.005f);

    /* First, converge to level. */
    const float gyro[3]   = { 0.0f, 0.0f, 0.0f };
    const float accel_ok[3] = { 0.0f, 0.0f, 9.80665f };
    for (int i = 0; i < 500; i++) {
        mahony_update_imu(&m, gyro, accel_ok, 0.001f);
    }

    /* Now apply a 4g shock. */
    const float accel_high[3] = { 0.0f, 0.0f, 39.2f };
    mahony_update_imu(&m, gyro, accel_high, 0.001f);

    const uint32_t flags = mahony_get_status_flags(&m);
    ASSERT_TRUE((flags & MAHONY_FLAG_ACCEL_REJECTED) != 0U);
    ASSERT_TRUE((flags & MAHONY_FLAG_HIGH_ACCEL) != 0U);
}

static void test_mahony_dt_invalid_skips_update(void)
{
    mahony_t m;
    mahony_init(&m, 2.0f, 0.005f);

    const float gyro[3]  = { 0.1f, 0.0f, 0.0f };
    const float accel[3] = { 0.0f, 0.0f, 9.80665f };

    /* Update with invalid dt (too large). */
    mahony_update_imu(&m, gyro, accel, 1.0f);
    ASSERT_TRUE((mahony_get_status_flags(&m) & MAHONY_FLAG_DT_INVALID) != 0U);

    /* Identity quaternion should be preserved. */
    float q[4];
    mahony_get_quaternion(&m, q);
    ASSERT_NEAR(q[0], 1.0f, TOL_TIGHT);
}

static void test_mahony_quaternion_normalization(void)
{
    /* After many updates the quaternion must remain unit length. */
    mahony_t m;
    mahony_init(&m, 2.0f, 0.005f);

    const float gyro[3]  = { 0.1f, 0.05f, -0.02f };
    const float accel[3] = { 0.0f, 0.0f, 9.80665f };

    for (int i = 0; i < 5000; i++) {
        mahony_update_imu(&m, gyro, accel, 0.001f);
    }

    float q[4];
    mahony_get_quaternion(&m, q);
    const float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    ASSERT_NEAR(n2, 1.0f, TOL_TIGHT);
}

static void test_mahony_gravity_body_level(void)
{
    /* At level attitude, gravity in body frame should ≈ [0, 0, 9.80665]. */
    mahony_t m;
    mahony_init(&m, 5.0f, 0.0f);

    const float gyro[3]  = { 0.0f, 0.0f, 0.0f };
    const float accel[3] = { 0.0f, 0.0f, 9.80665f };

    for (int i = 0; i < 2000; i++) {
        mahony_update_imu(&m, gyro, accel, 0.001f);
    }

    float gravity[3];
    mahony_get_gravity_body(&m, gravity);
    ASSERT_NEAR(gravity[0], 0.0f,     TOL_LOOSE);
    ASSERT_NEAR(gravity[1], 0.0f,     TOL_LOOSE);
    ASSERT_NEAR(gravity[2], 9.80665f, 0.1f);
}

static void test_mahony_reset_from_accel(void)
{
    mahony_t m;
    mahony_init(&m, 2.0f, 0.005f);

    /* Reset from 30° pitch (tilt forward). */
    const float pitch_deg = 30.0f;
    const float pitch_rad = pitch_deg * (float)M_PI / 180.0f;
    const float g = 9.80665f;
    /* accel for 30° pitch: ax = -g*sin(30°), az = g*cos(30°) */
    const float ax = -g * sinf(pitch_rad);
    const float az =  g * cosf(pitch_rad);

    mahony_reset_from_accel(&m, ax, 0.0f, az);

    float roll_rad, pitch_out, yaw_rad;
    mahony_get_euler(&m, &roll_rad, &pitch_out, &yaw_rad);

    ASSERT_NEAR(pitch_out, pitch_rad, 0.05f);
    ASSERT_NEAR(roll_rad,  0.0f,      TOL_LOOSE);
}

static void test_mahony_euler_pitch_to_quaternion_roundtrip(void)
{
    /* Forward pitch: rotate Ry(30°) — check Euler → quaternion roundtrip. */
    mahony_t m;
    mahony_init(&m, 2.0f, 0.005f);

    const float pitch_target = 0.5236f;  /* 30° in rad */
    const float g = 9.80665f;
    const float ax = -g * sinf(pitch_target);
    const float az =  g * cosf(pitch_target);

    mahony_reset_from_accel(&m, ax, 0.0f, az);

    float q[4];
    mahony_get_quaternion(&m, q);

    /* Norm should be 1. */
    const float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    ASSERT_NEAR(n2, 1.0f, TOL_TIGHT);

    /* Convert back to Euler. */
    float roll_rad, pitch_rad, yaw_rad;
    mahony_get_euler(&m, &roll_rad, &pitch_rad, &yaw_rad);
    ASSERT_NEAR(pitch_rad, pitch_target, 0.05f);
}

int main(void)
{
    test_mahony_init_identity_quaternion();
    test_mahony_stationary_convergence();
    test_mahony_stationary_tilted();
    test_mahony_accel_rejection_high_accel();
    test_mahony_dt_invalid_skips_update();
    test_mahony_quaternion_normalization();
    test_mahony_gravity_body_level();
    test_mahony_reset_from_accel();
    test_mahony_euler_pitch_to_quaternion_roundtrip();

    puts("test_mahony: all tests passed");
    return 0;
}
