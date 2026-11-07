#include "tsl2591.h"
#include "i2c_dev.h"
#include "config.h"

#define TSL2591_ADDR       TSL2591_I2C_ADDR
#define TSL2591_ENABLE     0x00
#define TSL2591_COMMAND_BIT 0xA0
#define TSL2591_CONFIG     0x01
#define TSL2591_C0DATAL    0x14
#define TSL2591_C1DATAL    0x18

#define ENABLE_POWERON  0x01
#define ENABLE_AEN      0x02
#define TIMING_300MS    0x02
#define GAIN_MEDIUM     0x10

int tsl2591_init(I2C_HandleTypeDef *bus)
{
    uint8_t id = 0;
    if (i2c_read_reg(bus, TSL2591_ADDR, TSL2591_COMMAND_BIT | 0x12, &id) != HAL_OK)
        return -1;
    if (id != 0x50)
        return -2;

    i2c_write_reg(bus, TSL2591_ADDR, TSL2591_COMMAND_BIT | TSL2591_ENABLE,
                  ENABLE_POWERON | ENABLE_AEN);
    i2c_write_reg(bus, TSL2591_ADDR, TSL2591_COMMAND_BIT | TSL2591_CONFIG,
                  GAIN_MEDIUM | TIMING_300MS);
    return 0;
}

void tsl2591_read(I2C_HandleTypeDef *bus, tsl2591_data_t *d)
{
    uint8_t buf[4] = {0};
    if (i2c_read_regs(bus, TSL2591_ADDR, TSL2591_COMMAND_BIT | TSL2591_C0DATAL,
                      buf, 4) != HAL_OK) {
        d->ir_counts = d->full_counts = d->lux = 0.0f;
        return;
    }
    uint16_t ch0 = (uint16_t)(buf[0] | buf[1] << 8);
    uint16_t ch1 = (uint16_t)(buf[2] | buf[3] << 8);

    d->full_counts = ch0;
    d->ir_counts   = ch1;

    const float again = 25.0f, atime = 300.0f, LUX_DF = 408.0f;
    float cpl = (again * atime) / LUX_DF;
    if (ch0 > ch1 && ch0 > 0)
        d->lux = ((float)ch0 - (float)ch1) * (1.0f - (float)ch1 / (float)ch0) / cpl;
    else
        d->lux = 0.0f;
}
