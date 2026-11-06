#include "sht45.h"
#include "config.h"
#include "i2c_dev.h"

/* SHT4x commands are single raw bytes written to the bus (no register) */
#define SHT45_CMD_SOFT_RESET    0x94    /* then 1 ms */
#define SHT45_CMD_TRIG_HIGH     0xFD    /* high-repeatability, max 8.3 ms */

/* raw = -45 + 175 * raw/65535 for T, -6 + 125 * raw/65535 for RH (SHT4x) */
#define SHT45_T_SCALE   (175.0f / 65535.0f)
#define SHT45_T_OFFSET  (-45.0f)
#define SHT45_RH_SCALE  (125.0f / 65535.0f)
#define SHT45_RH_OFFSET (-6.0f)

static int sht45_write_cmd(I2C_HandleTypeDef *bus, uint8_t cmd)
{
    return HAL_I2C_Master_Transmit(bus, SHT45_I2C_ADDR, &cmd, 1, 100);
}

int sht45_init(I2C_HandleTypeDef *bus)
{
    if (sht45_write_cmd(bus, SHT45_CMD_SOFT_RESET) != HAL_OK)
        return -1;
    HAL_Delay(2);
    return 0;
}

void sht45_trigger(I2C_HandleTypeDef *bus)
{
    sht45_write_cmd(bus, SHT45_CMD_TRIG_HIGH);
}

int sht45_read(I2C_HandleTypeDef *bus, sht45_data_t *d)
{
    uint8_t b[6];
    if (HAL_I2C_Master_Receive(bus, SHT45_I2C_ADDR, b, 6, 100) != HAL_OK)
        return -1;
    if (i2c_crc8(b, 2) != b[2] || i2c_crc8(b + 3, 2) != b[5])
        return 0;                       /* CRC bad: keep last value */

    uint16_t t_raw = (uint16_t)((b[0] << 8) | b[1]);
    uint16_t h_raw = (uint16_t)((b[3] << 8) | b[4]);

    d->temp_c = SHT45_T_OFFSET + SHT45_T_SCALE * t_raw;
    float rh  = SHT45_RH_OFFSET + SHT45_RH_SCALE * h_raw;
    if (rh < 0.0f)   rh = 0.0f;
    if (rh > 100.0f) rh = 100.0f;
    d->rh_pct = rh;
    return 1;
}
