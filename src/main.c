#include "board.h"
#include "dps310.h"
#include "icm42688p.h"

static void log_sample(const icm42688p_sample_t *sample)
{
    board_log("imu temp=");
    board_log_i32(sample->temperature_raw);
    board_log(" accel=");
    board_log_i32(sample->accel_x_raw);
    board_log_char(',');
    board_log_i32(sample->accel_y_raw);
    board_log_char(',');
    board_log_i32(sample->accel_z_raw);
    board_log(" gyro=");
    board_log_i32(sample->gyro_x_raw);
    board_log_char(',');
    board_log_i32(sample->gyro_y_raw);
    board_log_char(',');
    board_log_i32(sample->gyro_z_raw);
    board_log_char('\n');
}

static void log_adc(void)
{
    const uint16_t vbat_raw = board_adc1_read(10U);
    const uint16_t curr_raw = board_adc1_read(11U);

    board_log("adc vbat_raw=");
    board_log_u32(vbat_raw);
    board_log(" current_raw=");
    board_log_u32(curr_raw);
    board_log_char('\n');
}

int main(void)
{
    board_clock_init();
    board_gpio_init();
    board_systick_init();
    board_usart3_init(BOARD_UART_LOG_BAUD);
    board_spi1_init();
    board_i2c1_init();
    board_adc1_init();
    board_pwm_init();

    for (uint8_t output = 1; output <= BOARD_PWM_OUTPUTS; output++) {
        board_pwm_set_us(output, 1000U);
    }

    board_log_line("");
    board_log_line("SpeedyBee F405 Mini robotics bring-up");
    board_log("SystemCoreClock=");
    board_log_u32(SystemCoreClock);
    board_log_line(" Hz");

    const uint8_t whoami = icm42688p_read_whoami();
    board_log("ICM42688P WHOAMI=");
    board_log_hex8(whoami);
    if (whoami == ICM42688P_WHOAMI_EXPECTED) {
        board_log_line(" ok");
    } else {
        board_log_line(" unexpected");
    }

    const bool imu_ok = icm42688p_init();
    board_log_line(imu_ok ? "ICM42688P sensors enabled" : "ICM42688P init failed");

    const dps310_probe_t baro = dps310_probe();
    board_log("DPS310 addr ");
    board_log_hex8(DPS310_I2C_ADDR);
    board_log(baro.acked ? " ack" : " no-ack");
    if (baro.id_read_ok) {
        board_log(" product_id=");
        board_log_hex8(baro.product_id);
    }
    board_log_char('\n');

    uint32_t last_sample = board_millis();
    uint32_t last_adc = board_millis();
    uint32_t last_led = board_millis();

    while (1) {
        const uint32_t now = board_millis();

        if ((now - last_led) >= 250U) {
            last_led = now;
            board_led_toggle();
        }

        if ((now - last_sample) >= 100U) {
            last_sample = now;
            if (imu_ok) {
                icm42688p_sample_t sample;
                if (icm42688p_read_sample(&sample)) {
                    log_sample(&sample);
                }
            }
        }

        if ((now - last_adc) >= 1000U) {
            last_adc = now;
            log_adc();
        }
    }
}
