#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * IMU signal filters.
 *
 * Two filter types are provided:
 *   1. First-order IIR low-pass filter (very cheap, zero-phase lag on
 *      initialisation, suitable for accel and light gyro filtering).
 *   2. Biquad filter (second-order IIR, for notch or stronger low-pass).
 *      Coefficients are set via biquad_lpf_coeffs() or biquad_notch_coeffs().
 *
 * All filters operate on float32 (hardware FPU on Cortex-M4).
 *
 * Design notes:
 *   - Do NOT over-filter pitch_rate (gyro_y for a forward-balanced robot):
 *     the balance loop needs low latency.  Suggest gyro LPF ≥ 80 Hz.
 *   - Accel can be filtered more aggressively (40 Hz) because it is used
 *     only for slow attitude correction (Mahony Kp term), not loop output.
 */

/* ---- First-order IIR low-pass ---- */
typedef struct {
    float alpha;  /* = dt / (tau + dt),  tau = 1 / (2π·fc) */
    float state;
    bool  initialized;
} lpf1_t;

/* Compute alpha for a given cutoff frequency (Hz) and sample rate (Hz).
   Returns 1.0 (pass-through) if fc <= 0.                                    */
float lpf1_alpha(float cutoff_hz, float sample_rate_hz);

void  lpf1_reset(lpf1_t *f, float cutoff_hz, float sample_rate_hz);
float lpf1_update(lpf1_t *f, float x);

/* ---- Biquad filter ---- */
typedef struct {
    /* Direct Form I coefficients (b0,b1,b2 = numerator; a1,a2 = denominator) */
    float b0, b1, b2;
    float a1, a2;
    /* State */
    float x1, x2;   /* previous inputs  */
    float y1, y2;   /* previous outputs */
} biquad_t;

/* Set biquad as a 2nd-order Butterworth low-pass.
   fc = cutoff Hz, fs = sample rate Hz.                                      */
void biquad_lpf_coeffs(biquad_t *f, float fc, float fs);

/* Set biquad as a notch filter at freq_hz with quality factor Q. */
void biquad_notch_coeffs(biquad_t *f, float freq_hz, float Q, float fs);

void  biquad_reset(biquad_t *f);
float biquad_update(biquad_t *f, float x);

/* ---- IMU filter bank (gyro + accel, 3 axes each) ---- */
typedef struct {
    lpf1_t   gyro_lpf[3];
    lpf1_t   accel_lpf[3];
    biquad_t gyro_notch[3];   /* optional; bypass if Q == 0 */
    bool     gyro_notch_enabled;

    /* Vibration metric: exponential moving average of raw vs. filtered diff */
    float vibe_gyro;
    float vibe_accel;
} imu_filter_bank_t;

/* Configure filter bank.
   sample_rate_hz: sensor ODR.
   gyro_lpf_hz / accel_lpf_hz: cutoff; 0 = bypass.
   notch_hz: notch centre frequency; 0 = disabled.  Q: notch quality factor. */
void imu_filter_init(imu_filter_bank_t *bank,
                     float sample_rate_hz,
                     float gyro_lpf_hz,
                     float accel_lpf_hz,
                     float notch_hz,
                     float notch_Q);

/* Apply filters to one IMU sample (in place).
   raw_gyro/raw_accel are the calibrated inputs.
   filt_gyro/filt_accel are the filtered outputs.
   Updates vibration metrics.                                                 */
void imu_filter_update(imu_filter_bank_t *bank,
                       const float raw_gyro[3],
                       const float raw_accel[3],
                       float filt_gyro[3],
                       float filt_accel[3]);
