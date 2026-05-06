#include "dps310.h"

#include "board.h"

dps310_probe_t dps310_probe(void)
{
    dps310_probe_t probe = {
        .acked = board_i2c1_probe(DPS310_I2C_ADDR),
        .id_read_ok = false,
        .product_id = 0,
    };

    if (probe.acked) {
        probe.id_read_ok = board_i2c1_read_reg(
            DPS310_I2C_ADDR,
            DPS310_PRODUCT_ID_REG,
            &probe.product_id);
    }

    return probe;
}
