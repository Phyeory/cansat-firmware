#include "ltr390.h"
#include "i2c_dev.h"
#include "config.h"

#define LTR390_ADDR       LTR390_I2C_ADDR
#define LTR390_MAIN_CTRL  0x00   /* bit0 enable, bit1 mode (1 = UVS) */
#define LTR390_MEAS_RATE  0x04   /* resolution (bits 4..6) | rate (bits 0..2) */
#define LTR390_GAIN       0x05
#define LTR390_PART_ID    0x06
#define LTR390_UVS_DATA   0x10   /* 3 bytes LSB first */

/* 18-bit resolution, rate 100 ms */
#define LTR390_RES_18BIT_RATE_100MS 0x22
#define LTR390_GAIN_3               0x01

int ltr390_init(I2C_HandleTypeDef *bus)
{
    uint8_t id = 0;
    if (i2c_read_reg(bus, LTR390_ADDR, LTR390_PART_ID, &id) != HAL_OK)
        return -1;
    if ((id >> 4) != 0xB)          /* part number B = LTR390 */
        return -2;

    i2c_write_reg(bus, LTR390_ADDR, LTR390_GAIN, LTR390_GAIN_3);
    i2c_write_reg(bus, LTR390_ADDR, LTR390_MEAS_RATE, LTR390_RES_18BIT_RATE_100MS);
    i2c_write_reg(bus, LTR390_ADDR, LTR390_MAIN_CTRL, 0x02);  /* UVS mode, disabled */
    i2c_write_reg(bus, LTR390_ADDR, LTR390_MAIN_CTRL, 0x03);  /* UVS mode, enable */
    return 0;
}

void ltr390_read(I2C_HandleTypeDef *bus, ltr390_data_t *d)
{
    uint8_t buf[3] = {0};
    if (i2c_read_regs(bus, LTR390_ADDR, LTR390_UVS_DATA, buf, 3) != HAL_OK) {
        d->uv_raw = 0.0f; d->uv_index = 0.0f;
        return;
    }
    d->uv_raw = (float)((uint32_t)buf[0] | (uint32_t)buf[1] << 8 |
                        (uint32_t)buf[2] << 16);
    /* 18-bit / gain-3 scaling: ~2300 counts per UV index unit */
    d->uv_index = d->uv_raw / 2300.0f;
}
