#include "bmp280.h"
#include "i2c_dev.h"
#include "config.h"
#include <math.h>

/* BMP280 registers & datasheet compensation */
#define BMP280_REG_ID      0xD0
#define BMP280_REG_RESET   0xE0
#define BMP280_REG_STATUS  0xF3
#define BMP280_REG_CTRL    0xF4
#define BMP280_REG_CONFIG  0xF5
#define BMP280_REG_DATA    0xF7

/* ctrl_meas: temperature oversampling x2 (010), pressure x16 (101),
   normal mode (11) -> 0b010_101_11 = 0x57 */
#define BMP280_CTRL_NORMAL 0x57
#define BMP280_CONFIG_F16  0x10        /* filter coefficient 16 */

static struct {
    uint16_t T1; int16_t T2, T3;
    uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
} cal;

int bmp280_init(I2C_HandleTypeDef *bus)
{
    uint8_t id = 0;
    if (i2c_read_reg(bus, BMP280_I2C_ADDR, BMP280_REG_ID, &id) != HAL_OK)
        return -1;
    if (id != 0x58 && id != 0x60)   /* 0x58 = BMP280, 0x60 = BME280 (also works) */
        return -2;

    uint8_t buf[24];
    if (i2c_read_regs(bus, BMP280_I2C_ADDR, 0x88, buf, 24) != HAL_OK)
        return -3;
    cal.T1 = (uint16_t)(buf[0]  | buf[1]  << 8);
    cal.T2 = (int16_t)( buf[2]  | buf[3]  << 8);
    cal.T3 = (int16_t)( buf[4]  | buf[5]  << 8);
    cal.P1 = (uint16_t)(buf[6]  | buf[7]  << 8);
    cal.P2 = (int16_t)( buf[8]  | buf[9]  << 8);
    cal.P3 = (int16_t)(buf[10] | buf[11] << 8);
    cal.P4 = (int16_t)(buf[12] | buf[13] << 8);
    cal.P5 = (int16_t)(buf[14] | buf[15] << 8);
    cal.P6 = (int16_t)(buf[16] | buf[17] << 8);
    cal.P7 = (int16_t)(buf[18] | buf[19] << 8);
    cal.P8 = (int16_t)(buf[20] | buf[21] << 8);
    cal.P9 = (int16_t)(buf[22] | buf[23] << 8);

    i2c_write_reg(bus, BMP280_I2C_ADDR, BMP280_REG_RESET, 0xB6);
    HAL_Delay(10);
    i2c_write_reg(bus, BMP280_I2C_ADDR, BMP280_REG_CONFIG, BMP280_CONFIG_F16);
    i2c_write_reg(bus, BMP280_I2C_ADDR, BMP280_REG_CTRL, BMP280_CTRL_NORMAL);
    return 0;
}

static float compensate_T(int32_t adc, int32_t *t_fine)
{
    int32_t v1 = (((adc >> 3) - ((int32_t)cal.T1 << 1)) * (int32_t)cal.T2) >> 11;
    int32_t v2 = (((((adc >> 4) - (int32_t)cal.T1) * ((adc >> 4) - (int32_t)cal.T1)) >> 12)
                  * (int32_t)cal.T3) >> 14;
    *t_fine = v1 + v2;
    return (*t_fine * 5 + 128) / 256.0f;
}

static float compensate_P(int32_t adc, int32_t t_fine)
{
    int64_t v1 = (int64_t)t_fine - 128000;
    int64_t v2 = v1 * v1 * (int64_t)cal.P6;
    v2 = v2 + ((v1 * (int64_t)cal.P5) << 17);
    v2 = v2 + (((int64_t)cal.P4) << 35);
    v1 = ((v1 * v1 * (int64_t)cal.P3) >> 8) + ((v1 * (int64_t)cal.P2) << 12);
    v1 = (((((int64_t)1) << 47) + v1) * (int64_t)cal.P1) >> 33;
    if (v1 == 0) return 0.0f;                     /* avoid div by zero */
    int64_t p = 1048576 - adc;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = (((int64_t)cal.P9) * (p >> 13) * (p >> 13)) >> 25;
    v2 = (((int64_t)cal.P8) * p) >> 19;
    p = ((p + v1 + v2) >> 8) + (((int64_t)cal.P7) << 4);
    return (float)p / 25600.0f;                   /* Pa -> hPa */
}

void bmp280_read(I2C_HandleTypeDef *bus, bmp280_data_t *d)
{
    uint8_t buf[6] = {0};
    int32_t t_fine = 0;

    if (i2c_read_regs(bus, BMP280_I2C_ADDR, BMP280_REG_DATA, buf, 6) != HAL_OK) {
        d->temp_c = 0.0f; d->pressure_hpa = 0.0f; d->altitude_m = 0.0f;
        return;
    }
    int32_t adcP = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
    int32_t adcT = (int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));

    d->temp_c       = compensate_T(adcT, &t_fine);
    d->pressure_hpa = compensate_P(adcP, t_fine);
    d->altitude_m   = 44330.0f * (1.0f - powf(d->pressure_hpa / SEA_LEVEL_PRESSURE_HPA, 0.190294957f));
}
