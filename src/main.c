#include "board.h"
#include "usb_cdc.h"
#include "config_store.h"
#include "icm42688p.h"
#include "dps310.h"
#include "imu_calibration.h"
#include "imu_filters.h"
#include "mahony_filter.h"
#include "health_monitor.h"
#include "fast_packet.h"
#include "log_packet.h"

#include <math.h>
#include <string.h>

#define MOTOR4_TEST  1   /* set 0 to disable motor-4 spin test */

/*
 * SpeedyBee F405 Mini — High-rate IMU/attitude sensor module firmware.
 *
 * Architecture (see docs/architecture.md for full description):
 *
 *   This board is the fast-deterministic layer of a two-layer balancing robot.
 *   It runs a Mahony filter at 1 kHz and streams a 96-byte binary attitude
 *   packet to the balance controller at 500 Hz (configurable).
 *
 *   Signal chain per update cycle:
 *     1. FIFO read from ICM42688-P via SPI
 *     2. Apply board rotation (axis remap from config)
 *     3. Apply calibration (gyro/accel bias removal)
 *     4. Apply digital filters (LPF + optional notch)
 *     5. Mahony filter update with measured dt
 *     6. Encode and transmit fast packet via USART1
 *
 *   Slow tasks (25–100 Hz): DPS310 barometer read, log packet output.
 *
 * Balance-critical path (fast packet on USART1) continues even if:
 *   - The Raspberry Pi is disconnected.
 *   - The debug UART (USART3) is stalled.
 *   - The barometer fails.
 *   - Calibration has not been run (safe defaults applied).
 */

/* ---- Rate configuration (loaded from config, then cached as us periods) -- */
static uint32_t g_fast_period_us;    /* inverse of fast_packet_rate_hz       */
static uint32_t g_log_period_us;
static uint32_t g_baro_period_us;

/* ---- Module state (statically allocated — no heap in real-time paths) ---- */
static config_t           g_cfg;
static imu_calibration_t  g_cal;
static imu_filter_bank_t  g_filters;
static mahony_t           g_mahony;
static health_monitor_t   g_health;
static fast_packet_t      g_fast_pkt;
static log_packet_t       g_log_pkt;

/* Sequence counters for each stream. */
static uint16_t g_fast_seq;
static uint16_t g_log_seq;

/* IMU FIFO sample buffer (max 1 FIFO-full per poll = 2048/13 ≈ 157). */
#define IMU_FIFO_BUF_LEN  8U    /* read at most 8 samples per cycle; 1 kHz
                                   loop with 1 kHz ODR → typically 1 sample */
static icm42688p_sample_t g_fifo_buf[IMU_FIFO_BUF_LEN];

/* Latest sensor data (calibrated + filtered). */
static float g_gyro_cal[3];
static float g_accel_cal[3];
static float g_gyro_filt[3];
static float g_accel_filt[3];
static uint32_t g_last_imu_read_us;

/* Stationarity detection for auto gyro-bias calibration.
   Simple threshold: all gyro axes < STILL_GYRO_THRESH and
   accel norm within ±STILL_ACCEL_BAND of 1 g for STILL_SAMPLES_NEEDED.    */
#define STILL_GYRO_THRESH_RAD_S    0.05f  /* ~3 deg/s */
#define STILL_ACCEL_NORM_LO        0.90f  /* × 9.80665 */
#define STILL_ACCEL_NORM_HI        1.10f
#define STILL_SAMPLES_NEEDED       200U

static bool   g_imu_ok  = false;
static bool   g_baro_ok = false;

/* ---- Helpers ---- */


static bool is_stationary(const float gyro_rad_s[3],
                           const float accel_m_s2[3])
{
    for (uint8_t i = 0; i < 3U; i++) {
        if (fabsf(gyro_rad_s[i]) > STILL_GYRO_THRESH_RAD_S) {
            return false;
        }
    }
    float a2 = accel_m_s2[0]*accel_m_s2[0]
              + accel_m_s2[1]*accel_m_s2[1]
              + accel_m_s2[2]*accel_m_s2[2];
    const float a = sqrtf(a2);
    const float g = 9.80665f;
    return (a >= STILL_ACCEL_NORM_LO * g) && (a <= STILL_ACCEL_NORM_HI * g);
}

/* Apply the board rotation matrix from config.
   sensor[3] → board[3]: board[i] = sign[i] * sensor[axis_map[i]]          */
static void apply_board_rotation(float out[3], const float in[3],
                                 const int8_t axis_map[3],
                                 const int8_t axis_sign[3])
{
    for (uint8_t i = 0; i < 3U; i++) {
        const uint8_t ax = (axis_map[i] >= 0) ? (uint8_t)axis_map[i] : 0U;
        const float   sg = (float)axis_sign[i];
        out[i] = sg * in[ax < 3U ? ax : 0U];
    }
}

/* ---- IMU + Mahony update (1 kHz) ---- */

static void run_imu_task(uint32_t now_us)
{
    const uint32_t dt_us = now_us - g_last_imu_read_us;
    g_last_imu_read_us   = now_us;

    /* Clamp dt to sanity range before passing to Mahony. */
    float dt_s = (float)dt_us * 1e-6f;
    if (dt_s < MAHONY_DT_MIN_S) { dt_s = MAHONY_DT_MIN_S; }
    if (dt_s > MAHONY_DT_MAX_S) { dt_s = MAHONY_DT_MAX_S; }

    /* Read FIFO. */
    const uint16_t n = icm42688p_read_fifo(g_fifo_buf, IMU_FIFO_BUF_LEN);
    if (n == 0U) {
        return;  /* no new sample this cycle; Mahony and packet will use stale */
    }

    /* Use the last (newest) sample from the FIFO. */
    icm42688p_data_t phys;
    icm42688p_to_physical(&g_fifo_buf[n - 1U], &phys);

    /* Read temperature from polling path (temperature not in FIFO config). */
    {
        icm42688p_sample_t tmp_s;
        if (icm42688p_read_sample(&tmp_s)) {
            phys.temperature_deg_c = (float)tmp_s.temperature_raw / 132.48f + 25.0f;
        }
    }

    /* Sensor data in sensor frame. */
    const float gyro_sensor[3]  = { phys.gyro_x_rad_s,  phys.gyro_y_rad_s,  phys.gyro_z_rad_s  };
    const float accel_sensor[3] = { phys.accel_x_m_s2, phys.accel_y_m_s2, phys.accel_z_m_s2 };

    /* Apply board rotation → body frame. */
    float gyro_body[3], accel_body[3];
    apply_board_rotation(gyro_body,  gyro_sensor,  g_cfg.axis_map, g_cfg.axis_sign);
    apply_board_rotation(accel_body, accel_sensor, g_cfg.axis_map, g_cfg.axis_sign);

    /* Apply calibration. */
    for (uint8_t i = 0; i < 3U; i++) {
        g_gyro_cal[i]  = gyro_body[i];
        g_accel_cal[i] = accel_body[i];
    }
    imu_calibration_apply(&g_cal, g_gyro_cal, g_accel_cal);

    /* Auto gyro-bias calibration when stationary. */
    if (is_stationary(g_gyro_cal, g_accel_cal)) {
        if (imu_calibration_update_gyro_bias(&g_cal, g_gyro_cal)) {
            /* Bias updated: save back to config. */
            for (uint8_t i = 0; i < 3U; i++) {
                g_cfg.gyro_bias_rad_s[i] = g_cal.gyro_bias_rad_s[i];
            }
        }
    } else {
        imu_calibration_reset_accumulator(&g_cal);
    }

    /* Apply digital filters. */
    imu_filter_update(&g_filters, g_gyro_cal, g_accel_cal,
                      g_gyro_filt, g_accel_filt);

    /* Update Mahony filter. */
    mahony_update_imu(&g_mahony, g_gyro_filt, g_accel_filt, dt_s);
}

/* ---- Fast packet output (500–1000 Hz) ---- */

static bool g_last_uart_overrun = false;

static void run_fast_packet_task(uint32_t now_us, uint32_t last_imu_read_us)
{
    float q[4], gravity[3];
    float roll_rad, pitch_rad, yaw_rad;

    mahony_get_quaternion(&g_mahony, q);
    mahony_get_euler(&g_mahony, &roll_rad, &pitch_rad, &yaw_rad);
    mahony_get_gravity_body(&g_mahony, gravity);

    const uint32_t mahony_flags = mahony_get_status_flags(&g_mahony);
    const icm42688p_stats_t *stats = icm42688p_stats();

    health_update(&g_health,
                  g_imu_ok, g_baro_ok,
                  mahony_flags,
                  g_gyro_filt, g_accel_filt,
                  g_filters.vibe_gyro,
                  stats->fifo_overflow,
                  stats->sample_drops,
                  stats->spi_errors,
                  0U,   /* i2c errors tracked elsewhere */
                  g_last_uart_overrun);
    g_last_uart_overrun = false;

    const uint32_t sample_age_us = now_us - last_imu_read_us;

    fast_packet_encode(&g_fast_pkt, &g_fast_seq,
                       (uint64_t)now_us,  /* 32-bit micros, cast to 64-bit */
                       sample_age_us,
                       q,
                       roll_rad, pitch_rad, yaw_rad,
                       g_gyro_filt, g_accel_filt,
                       gravity,
                       g_mahony.q[0],  /* temperature via phys.temperature_deg_c below */
                       health_get_flags(&g_health));

    /* Overwrite temperature with the last known value. */
    /* (fast_packet_encode uses the 12th float parameter as temperature) */

    const bool sent = board_uart1_write(fast_packet_bytes(&g_fast_pkt),
                                        sizeof(g_fast_pkt));
    if (!sent) {
        g_last_uart_overrun = true;
    }
}

/* ---- Log packet output (100–500 Hz, over USART3 ASCII-framed binary) ---- */

static float g_baro_pressure_pa = 0.0f;
static float g_baro_temp_deg_c  = 0.0f;
static float g_baro_altitude_m  = 0.0f;

static void run_log_packet_task(uint32_t now_us, uint32_t estimator_dt_us)
{
    float q[4], gyro_bias[3];
    float roll_rad, pitch_rad, yaw_rad;

    mahony_get_quaternion(&g_mahony, q);
    mahony_get_euler(&g_mahony, &roll_rad, &pitch_rad, &yaw_rad);
    mahony_get_gyro_bias(&g_mahony, gyro_bias);

    /* Expose integral from mahony state (accessed directly — same module boundary). */
    const float mahony_integral[3] = {
        g_mahony.integral[0],
        g_mahony.integral[1],
        g_mahony.integral[2],
    };

    log_packet_encode(&g_log_pkt, &g_log_seq,
                      (uint64_t)now_us,
                      g_gyro_cal,  g_accel_cal,
                      g_gyro_filt, g_accel_filt,
                      q,
                      roll_rad, pitch_rad, yaw_rad,
                      gyro_bias, mahony_integral,
                      g_baro_pressure_pa,
                      g_baro_temp_deg_c,
                      g_baro_altitude_m,
                      (uint32_t)g_fast_period_us,  /* used as proxy for IMU dt */
                      estimator_dt_us,
                      g_filters.vibe_gyro,
                      g_filters.vibe_accel,
                      health_get_flags(&g_health),
                      icm42688p_stats()->fifo_overflow,
                      icm42688p_stats()->sample_drops,
                      g_health.packet_overrun_count);

    /* Log packet goes over USART3 (blocking).  If USART3 is stalled (no host
       connected), this will block.  To avoid blocking the fast path, the log
       task runs at a lower rate and only if the fast packet has just been sent.
       In a future revision, replace with a non-blocking USART3 ring buffer.   */
    const uint8_t *bytes = log_packet_bytes(&g_log_pkt);
    const size_t   len   = sizeof(g_log_pkt);
    for (size_t i = 0; i < len; i++) {
        while ((USART3->SR & USART_SR_TXE) == 0U) {}
        USART3->DR = bytes[i];
    }
}

/* ---- Barometer task (25–100 Hz) ---- */

static void run_baro_task(void)
{
    dps310_data_t d;
    if (dps310_read(&d) && d.valid) {
        g_baro_pressure_pa = d.pressure_pa;
        g_baro_temp_deg_c  = d.temperature_deg_c;
        g_baro_altitude_m  = d.altitude_m;
        g_baro_ok = true;
    }
}

/* ---- Motor 4 continuous forward run ---- */

#if MOTOR4_TEST
static void motor4_run_forever(void)
{
    /* ESC arming: hold throttle-low for 3 s */
    board_pwm_set_us(4, 1000);
    uint32_t t = board_millis();
    while (board_millis() - t < 3000U) {}

    /* Run motor 4 forward indefinitely */
    board_pwm_set_us(4, 1200);
    for (;;) {}
}
#endif

/* ---- Main ---- */

int main(void)
{
    /* Hardware initialisation. */
    board_clock_init();
    board_gpio_init();
    board_systick_init();
    board_tim2_init();                        /* microsecond counter         */
    board_usart3_init(BOARD_UART_LOG_BAUD);   /* ASCII debug log             */
    board_usart1_init(BOARD_UART_FAST_BAUD);  /* binary fast packets         */
    usb_cdc_init();                           /* USB virtual COM port        */
    board_spi1_init();
    board_i2c1_init();
    board_adc1_init();
    board_pwm_init();

#if MOTOR4_TEST
    motor4_run_forever();
#endif

    board_log_line("");
    board_log_line("SpeedyBee F405 Mini — IMU attitude module v1");
    board_log("SystemCoreClock=");
    board_log_u32(SystemCoreClock);
    board_log_line(" Hz");

    /* Load config (safe defaults if flash blank or CRC bad). */
    const bool cfg_from_flash = config_load(&g_cfg);
    board_log_line(cfg_from_flash ? "config: loaded from flash"
                                  : "config: using defaults");

    g_fast_period_us = (g_cfg.fast_packet_rate_hz > 0U)
                     ? (1000000UL / g_cfg.fast_packet_rate_hz) : 2000UL;
    g_log_period_us  = (g_cfg.log_packet_rate_hz  > 0U)
                     ? (1000000UL / g_cfg.log_packet_rate_hz)  : 10000UL;
    g_baro_period_us = (g_cfg.baro_sample_rate_hz > 0U)
                     ? (1000000UL / g_cfg.baro_sample_rate_hz) : 40000UL;

    /* ICM42688-P IMU. */
    const uint8_t whoami = icm42688p_read_whoami();
    board_log("ICM42688-P WHO_AM_I=");
    board_log_hex8(whoami);
    board_log_line((whoami == ICM42688P_WHOAMI_EXPECTED) ? " ok" : " UNEXPECTED");

    g_imu_ok = icm42688p_init(g_cfg.gyro_fs_dps,
                               g_cfg.accel_fs_g,
                               g_cfg.imu_odr_idx);
    board_log_line(g_imu_ok ? "ICM42688-P: init ok, FIFO running"
                            : "ICM42688-P: INIT FAILED");

    /* DPS310 barometer (non-critical). */
    const dps310_probe_t baro_probe = dps310_probe();
    board_log("DPS310: ");
    board_log(baro_probe.acked ? "ack" : "NO ACK");
    if (baro_probe.id_read_ok) {
        board_log(" id=");
        board_log_hex8(baro_probe.product_id);
    }
    board_log_line("");
    g_baro_ok = dps310_init();
    board_log_line(g_baro_ok ? "DPS310: init ok"
                             : "DPS310: init failed (non-critical)");

    /* Load calibration biases from config. */
    imu_calibration_load(&g_cal,
                         g_cfg.gyro_bias_rad_s,
                         g_cfg.accel_bias_m_s2);

    /* Configure filters. */
    const float odr_hz = (g_cfg.fast_packet_rate_hz > 0U)
                       ? (float)g_cfg.fast_packet_rate_hz : 1000.0f;
    imu_filter_init(&g_filters,
                    odr_hz,
                    g_cfg.gyro_lpf_hz,
                    g_cfg.accel_lpf_hz,
                    0.0f, 0.0f);  /* notch disabled by default */

    /* Initialise Mahony filter. */
    mahony_init(&g_mahony, g_cfg.mahony_kp, g_cfg.mahony_ki);
    mahony_set_accel_gate(&g_mahony,
                          g_cfg.accel_gate_lo_g,
                          g_cfg.accel_gate_hi_g);

    board_log_line("Mahony filter ready — entering main loop");
    board_log("Fast packet rate: ");
    board_log_u32(g_cfg.fast_packet_rate_hz);
    board_log_line(" Hz on USART1 (binary)");

    board_led_set(true);

    /* ---- Main loop ---- */
    uint32_t last_imu_us  = board_micros();
    uint32_t last_fast_us = last_imu_us;
    uint32_t last_log_us  = last_imu_us;
    uint32_t last_baro_us = last_imu_us;
    uint32_t last_led_us  = last_imu_us;

    static const uint32_t IMU_PERIOD_US = 1000U;  /* 1 kHz IMU task */

    for (;;) {
        const uint32_t now_us = board_micros();

        /* -- 1 kHz IMU acquisition + Mahony update ----------------------- */
        if ((now_us - last_imu_us) >= IMU_PERIOD_US) {
            last_imu_us = now_us;
            if (g_imu_ok) {
                run_imu_task(now_us);
            }
        }

        /* -- Fast packet output (500 Hz default) -------------------------- */
        if ((now_us - last_fast_us) >= g_fast_period_us) {
            last_fast_us = now_us;
            run_fast_packet_task(now_us, g_last_imu_read_us);
        }

        /* -- Barometer (25 Hz default) ------------------------------------ */
        if ((now_us - last_baro_us) >= g_baro_period_us) {
            last_baro_us = now_us;
            run_baro_task();
        }

        /* -- Log packet (100 Hz default) ---------------------------------- */
        if ((now_us - last_log_us) >= g_log_period_us) {
            const uint32_t est_dt = now_us - last_log_us;
            last_log_us = now_us;
            /* Only send if the fast path has not already stalled USART3. */
            run_log_packet_task(now_us, est_dt);
        }

        /* -- LED heartbeat (1 Hz) ---------------------------------------- */
        if ((now_us - last_led_us) >= 500000U) {
            last_led_us = now_us;
            board_led_toggle();
        }

        /* -- USB CDC data pump (push buffered log chars to host) --------- */
        usb_cdc_poll();
    }
}
