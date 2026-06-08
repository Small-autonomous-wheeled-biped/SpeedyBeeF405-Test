#include "config_store.h"
#include "crc.h"

#include <string.h>

/* Flash sector used for config storage. */
#ifndef HOST_TEST
#include "stm32f405xx_min.h"
#else
/* Stub for host-side unit tests: no real flash. */
static uint8_t g_fake_flash[sizeof(config_t)];
#endif

/* Rotation: ROTATION_PITCH_180_YAW_90 (ArduPilot convention for this board).
   Sensor → board-body frame:
     x_b = -y_s   (axis_map[0]=1, axis_sign[0]=-1)
     y_b = -x_s   (axis_map[1]=0, axis_sign[1]=-1)
     z_b = -z_s   (axis_map[2]=2, axis_sign[2]=-1)                          */
void config_set_defaults(config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    cfg->magic       = CONFIG_MAGIC;
    cfg->version     = CONFIG_VERSION;
    cfg->struct_size = (uint16_t)sizeof(config_t);

    cfg->mahony_kp = 2.0f;
    cfg->mahony_ki = 0.005f;

    cfg->accel_gate_lo_g = 0.85f;
    cfg->accel_gate_hi_g = 1.15f;

    cfg->gyro_fs_dps = 1U;   /* ±1000 dps */
    cfg->accel_fs_g  = 2U;   /* ±4 g      */
    cfg->imu_odr_idx = 6U;   /* 1 kHz     */

    /* ROTATION_PITCH_180_YAW_90 */
    cfg->axis_map[0] = 1;  cfg->axis_sign[0] = -1;
    cfg->axis_map[1] = 0;  cfg->axis_sign[1] = -1;
    cfg->axis_map[2] = 2;  cfg->axis_sign[2] = -1;

    cfg->gyro_lpf_hz  = 80.0f;
    cfg->accel_lpf_hz = 40.0f;

    cfg->fast_packet_rate_hz = 500U;
    cfg->log_packet_rate_hz  = 100U;
    cfg->baro_sample_rate_hz = 25U;

    /* Zero gyro/accel biases — calibration updates these at runtime. */
    cfg->gyro_bias_rad_s[0] = 0.0f;
    cfg->gyro_bias_rad_s[1] = 0.0f;
    cfg->gyro_bias_rad_s[2] = 0.0f;
    cfg->accel_bias_m_s2[0] = 0.0f;
    cfg->accel_bias_m_s2[1] = 0.0f;
    cfg->accel_bias_m_s2[2] = 0.0f;

    /* Compute CRC over everything before the crc16 field. */
    cfg->crc16 = crc16_ccitt((const uint8_t *)cfg,
                              offsetof(config_t, crc16));
}

bool config_crc_ok(const config_t *cfg)
{
    if (cfg == NULL) {
        return false;
    }
    const uint16_t computed = crc16_ccitt((const uint8_t *)cfg,
                                          offsetof(config_t, crc16));
    return computed == cfg->crc16;
}

#ifndef HOST_TEST
/* ---- Flash helpers (STM32F405 register-level) ---- */

static void flash_unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0U) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static void flash_wait_busy(void)
{
    while ((FLASH->SR & FLASH_SR_BSY) != 0U) {}
}

static void flash_erase_sector(uint32_t sector)
{
    flash_wait_busy();
    FLASH->CR = FLASH_CR_SER
              | FLASH_CR_PSIZE_WORD
              | (sector << FLASH_CR_SNB_SHIFT);
    FLASH->CR |= FLASH_CR_STRT;
    flash_wait_busy();
    FLASH->CR = 0;
}

static void flash_program_word(uint32_t addr, uint32_t data)
{
    flash_wait_busy();
    FLASH->CR = FLASH_CR_PG | FLASH_CR_PSIZE_WORD;
    *((__IO uint32_t *)addr) = data;
    flash_wait_busy();
    FLASH->CR = 0;
}

bool config_save(const config_t *cfg)
{
    if (cfg == NULL) {
        return false;
    }
    if (!config_crc_ok(cfg)) {
        return false;
    }

    flash_unlock();
    flash_erase_sector(CONFIG_FLASH_SECTOR);

    const uint32_t *src  = (const uint32_t *)cfg;
    uint32_t        addr = CONFIG_FLASH_BASE;
    const size_t    words = (sizeof(config_t) + 3U) / 4U;

    for (size_t i = 0; i < words; i++) {
        flash_program_word(addr, src[i]);
        addr += 4U;
    }

    flash_lock();

    /* Verify the write. */
    const config_t *stored = (const config_t *)CONFIG_FLASH_BASE;
    return (stored->magic == cfg->magic) && config_crc_ok(stored);
}

bool config_load(config_t *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    const config_t *stored = (const config_t *)CONFIG_FLASH_BASE;

    if ((stored->magic == CONFIG_MAGIC)
        && (stored->version == CONFIG_VERSION)
        && (stored->struct_size == (uint16_t)sizeof(config_t))
        && config_crc_ok(stored)) {
        memcpy(cfg, stored, sizeof(config_t));
        return true;
    }

    config_set_defaults(cfg);
    return false;
}

#else   /* HOST_TEST */

bool config_save(const config_t *cfg)
{
    if (cfg == NULL || !config_crc_ok(cfg)) {
        return false;
    }
    memcpy(g_fake_flash, cfg, sizeof(config_t));
    return true;
}

bool config_load(config_t *cfg)
{
    if (cfg == NULL) {
        return false;
    }
    const config_t *stored = (const config_t *)g_fake_flash;
    if ((stored->magic == CONFIG_MAGIC) && config_crc_ok(stored)) {
        memcpy(cfg, stored, sizeof(config_t));
        return true;
    }
    config_set_defaults(cfg);
    return false;
}

#endif  /* HOST_TEST */
