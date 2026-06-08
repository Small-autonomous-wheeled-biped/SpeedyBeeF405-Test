#include "imu_filters.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_SQRT2
#define M_SQRT2 1.41421356237309504880
#endif

/* ---- First-order IIR low-pass ---- */

float lpf1_alpha(float cutoff_hz, float sample_rate_hz)
{
    if (cutoff_hz <= 0.0f) {
        return 1.0f;  /* pass-through */
    }
    const float tau = 1.0f / (2.0f * (float)M_PI * cutoff_hz);
    const float dt  = 1.0f / sample_rate_hz;
    return dt / (tau + dt);
}

void lpf1_reset(lpf1_t *f, float cutoff_hz, float sample_rate_hz)
{
    if (f == NULL) { return; }
    f->alpha       = lpf1_alpha(cutoff_hz, sample_rate_hz);
    f->state       = 0.0f;
    f->initialized = false;
}

float lpf1_update(lpf1_t *f, float x)
{
    if (!f->initialized) {
        f->state       = x;
        f->initialized = true;
        return x;
    }
    f->state += f->alpha * (x - f->state);
    return f->state;
}

/* ---- Biquad filter (Direct Form I) ---- */

void biquad_lpf_coeffs(biquad_t *f, float fc, float fs)
{
    if (f == NULL) { return; }
    biquad_reset(f);
    if (fc <= 0.0f) {
        /* Pass-through coefficients. */
        f->b0 = 1.0f; f->b1 = 0.0f; f->b2 = 0.0f;
        f->a1 = 0.0f; f->a2 = 0.0f;
        return;
    }
    /* 2nd-order Butterworth low-pass via bilinear transform. */
    const float K = tanf((float)M_PI * fc / fs);
    const float norm = 1.0f / (1.0f + (float)M_SQRT2 * K + K * K);
    f->b0 = K * K * norm;
    f->b1 = 2.0f * f->b0;
    f->b2 = f->b0;
    f->a1 = 2.0f * (K * K - 1.0f) * norm;
    f->a2 = (1.0f - (float)M_SQRT2 * K + K * K) * norm;
}

void biquad_notch_coeffs(biquad_t *f, float freq_hz, float Q, float fs)
{
    if (f == NULL) { return; }
    biquad_reset(f);
    if ((freq_hz <= 0.0f) || (Q <= 0.0f)) {
        f->b0 = 1.0f; f->b1 = 0.0f; f->b2 = 0.0f;
        f->a1 = 0.0f; f->a2 = 0.0f;
        return;
    }
    /* Notch filter via bilinear transform. */
    const float K = tanf((float)M_PI * freq_hz / fs);
    const float norm = 1.0f / (1.0f + K / Q + K * K);
    f->b0 = (1.0f + K * K) * norm;
    f->b1 = 2.0f * (K * K - 1.0f) * norm;
    f->b2 = f->b0;
    f->a1 = f->b1;
    f->a2 = (1.0f - K / Q + K * K) * norm;
}

void biquad_reset(biquad_t *f)
{
    if (f == NULL) { return; }
    f->x1 = 0.0f; f->x2 = 0.0f;
    f->y1 = 0.0f; f->y2 = 0.0f;
}

float biquad_update(biquad_t *f, float x)
{
    const float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2
                              - f->a1 * f->y1 - f->a2 * f->y2;
    f->x2 = f->x1; f->x1 = x;
    f->y2 = f->y1; f->y1 = y;
    return y;
}

/* ---- IMU filter bank ---- */

void imu_filter_init(imu_filter_bank_t *bank,
                     float sample_rate_hz,
                     float gyro_lpf_hz,
                     float accel_lpf_hz,
                     float notch_hz,
                     float notch_Q)
{
    if (bank == NULL) { return; }
    memset(bank, 0, sizeof(*bank));

    for (uint8_t i = 0; i < 3U; i++) {
        lpf1_reset(&bank->gyro_lpf[i],  gyro_lpf_hz,  sample_rate_hz);
        lpf1_reset(&bank->accel_lpf[i], accel_lpf_hz, sample_rate_hz);
    }

    bank->gyro_notch_enabled = (notch_hz > 0.0f) && (notch_Q > 0.0f);
    if (bank->gyro_notch_enabled) {
        for (uint8_t i = 0; i < 3U; i++) {
            biquad_notch_coeffs(&bank->gyro_notch[i], notch_hz, notch_Q,
                                sample_rate_hz);
        }
    }
}

/* Exponential moving average coefficient for vibration metric (~1 s window). */
#define VIBE_ALPHA 0.001f

void imu_filter_update(imu_filter_bank_t *bank,
                       const float raw_gyro[3],
                       const float raw_accel[3],
                       float filt_gyro[3],
                       float filt_accel[3])
{
    if ((bank == NULL) || (raw_gyro == NULL) || (raw_accel == NULL)
        || (filt_gyro == NULL) || (filt_accel == NULL)) {
        return;
    }

    float gyro_vibe  = 0.0f;
    float accel_vibe = 0.0f;

    for (uint8_t i = 0; i < 3U; i++) {
        /* Apply notch before LPF if enabled. */
        float g = raw_gyro[i];
        if (bank->gyro_notch_enabled) {
            g = biquad_update(&bank->gyro_notch[i], g);
        }
        filt_gyro[i]  = lpf1_update(&bank->gyro_lpf[i], g);
        filt_accel[i] = lpf1_update(&bank->accel_lpf[i], raw_accel[i]);

        const float dg = raw_gyro[i]  - filt_gyro[i];
        const float da = raw_accel[i] - filt_accel[i];
        gyro_vibe  += dg * dg;
        accel_vibe += da * da;
    }

    /* EMA vibration metric (sum of squared deviations). */
    bank->vibe_gyro  += VIBE_ALPHA * (gyro_vibe  - bank->vibe_gyro);
    bank->vibe_accel += VIBE_ALPHA * (accel_vibe - bank->vibe_accel);
}
