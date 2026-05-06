#include "icm42688p.h"

#include "board.h"

#define ICM42688P_REG_TEMP_DATA1 0x1DU
#define ICM42688P_REG_WHO_AM_I   0x75U
#define ICM42688P_REG_PWR_MGMT0  0x4EU
#define ICM42688P_SPI_READ       0x80U

static uint8_t read_reg(uint8_t reg)
{
    board_imu_cs_set(true);
    (void)board_spi1_transfer(reg | ICM42688P_SPI_READ);
    const uint8_t value = board_spi1_transfer(0x00U);
    board_imu_cs_set(false);
    return value;
}

static void write_reg(uint8_t reg, uint8_t value)
{
    board_imu_cs_set(true);
    (void)board_spi1_transfer(reg & (uint8_t)~ICM42688P_SPI_READ);
    (void)board_spi1_transfer(value);
    board_imu_cs_set(false);
}

static void read_burst(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    board_imu_cs_set(true);
    (void)board_spi1_transfer(reg | ICM42688P_SPI_READ);
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = board_spi1_transfer(0x00U);
    }
    board_imu_cs_set(false);
}

static int16_t be_i16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] << 8U | data[1]);
}

uint8_t icm42688p_read_whoami(void)
{
    return read_reg(ICM42688P_REG_WHO_AM_I);
}

bool icm42688p_init(void)
{
    const uint8_t whoami = icm42688p_read_whoami();
    if (whoami != ICM42688P_WHOAMI_EXPECTED) {
        return false;
    }

    write_reg(ICM42688P_REG_PWR_MGMT0, 0x0FU);
    board_delay_ms(50U);
    return true;
}

bool icm42688p_read_sample(icm42688p_sample_t *sample)
{
    if (sample == 0) {
        return false;
    }

    uint8_t raw[14];
    read_burst(ICM42688P_REG_TEMP_DATA1, raw, sizeof(raw));

    sample->temperature_raw = be_i16(&raw[0]);
    sample->accel_x_raw = be_i16(&raw[2]);
    sample->accel_y_raw = be_i16(&raw[4]);
    sample->accel_z_raw = be_i16(&raw[6]);
    sample->gyro_x_raw = be_i16(&raw[8]);
    sample->gyro_y_raw = be_i16(&raw[10]);
    sample->gyro_z_raw = be_i16(&raw[12]);
    return true;
}
