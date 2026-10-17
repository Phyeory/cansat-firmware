#include "scd30.h"
#include "i2c_dev.h"
#include "config.h"

#define SCD30_ADDR       SCD30_I2C_ADDR
#define CMD_START_CONT   0x0010   /* start continuous measurement */
#define CMD_STOP         0x0104
#define CMD_READ_MEAS    0x0300   /* 18 bytes: 3 floats, each with CRC */
#define CMD_DATA_READY   0x0202   /* 1 word (3 bytes with CRC) */
#define CMD_ASC          0x5306   /* automatic self-calibration */

static HAL_StatusTypeDef cmd16(I2C_HandleTypeDef *bus, uint16_t cmd,
                               const uint16_t *args, int nargs)
{
    uint8_t b[2 + 3 * 2];
    int n = 0;
    b[n++] = cmd >> 8; b[n++] = cmd & 0xFF;
    for (int i = 0; i < nargs; i++) {
        uint8_t w[2] = { (uint8_t)(args[i] >> 8), (uint8_t)(args[i] & 0xFF) };
        b[n++] = w[0]; b[n++] = w[1]; b[n++] = i2c_crc8(w, 2);
    }
    return HAL_I2C_Master_Transmit(bus, SCD30_ADDR, b, n, 100);
}

static HAL_StatusTypeDef read_words(I2C_HandleTypeDef *bus, uint16_t cmd,
                                    uint8_t *out, int nwords)
{
    uint8_t c[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    if (HAL_I2C_Master_Transmit(bus, SCD30_ADDR, c, 2, 100) != HAL_OK)
        return HAL_ERROR;
    return HAL_I2C_Master_Receive(bus, SCD30_ADDR, out, nwords * 3, 100);
}

int scd30_init(I2C_HandleTypeDef *bus)
{
    /* chip-id check: read firmware version (cmd 0xD100) — some units NACK it;
       treat a NACK on any command below as "sensor absent" */
    uint16_t asc = 1;   /* enable ASC — good for outdoor flights */
    if (cmd16(bus, CMD_ASC, &asc, 1) != HAL_OK) return -1;

    uint16_t start = 0; /* 0 => default 2 s interval */
    if (cmd16(bus, CMD_START_CONT, &start, 1) != HAL_OK) return -2;
    return 0;
}

int scd30_read(I2C_HandleTypeDef *bus, scd30_data_t *d)
{
    uint8_t rdy[3] = {0};
    if (read_words(bus, CMD_DATA_READY, rdy, 1) != HAL_OK) return -1;
    if (!rdy[1]) return 0;                      /* word MSB low => not ready */

    uint8_t buf[18] = {0};
    if (read_words(bus, CMD_READ_MEAS, buf, 6) != HAL_OK) return -2;

    /* verify CRC of each 3-byte word */
    for (int i = 0; i < 6; i++) {
        if (i2c_crc8(&buf[i * 3], 2) != buf[i * 3 + 2]) return -3;
    }

    uint32_t raw;
    raw = ((uint32_t)buf[0] << 24) | (buf[1] << 16) | (buf[3] << 8) | buf[4];
    uint32_t co2_bits = raw;
    raw = ((uint32_t)buf[6] << 24) | (buf[7] << 16) | (buf[9] << 8) | buf[10];
    uint32_t t_bits = raw;
    raw = ((uint32_t)buf[12] << 24) | (buf[13] << 16) | (buf[15] << 8) | buf[16];
    uint32_t rh_bits = raw;

    float co2, t, rh;
    memcpy(&co2, &co2_bits, 4);
    memcpy(&t,   &t_bits,   4);
    memcpy(&rh,  &rh_bits,  4);

    d->co2_ppm = co2;
    d->temp_c  = t;
    d->rh_pct  = rh;
    return 1;
}
