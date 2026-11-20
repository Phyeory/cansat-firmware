#include "bmp581.h"
#include "spi_dev.h"
#include "config.h"
#include <math.h>

#define BMP581_REG_CHIP_ID    0x01
#define BMP581_CHIP_ID_VAL    0x50
#define BMP581_REG_DATA       0x1D
#define BMP581_REG_OSR_CONFIG 0x36
#define BMP581_REG_ODR_CONFIG 0x37
#define BMP581_REG_CMD        0x7E
#define BMP581_CMD_SOFT_RESET 0xB6

/* press_en | osr_p x16 | osr_t x2 = 0x61.
   deep_dis | ODR 50 Hz | normal mode = 0xBD. */
#define BMP581_OSR_CONFIG_VAL 0x61
#define BMP581_ODR_CONFIG_VAL 0xBD

static spi_dev_t dev;

static int read_reg(uint8_t reg, uint8_t *val)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx[2] = { 0 };

    if (spi_dev_xfer(&dev, tx, rx, 2) != HAL_OK) return -1;
    *val = rx[1];
    return 0;
}

static int write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    uint8_t rx[2] = { 0 };

    return spi_dev_xfer(&dev, tx, rx, 2) == HAL_OK ? 0 : -1;
}

int bmp581_init(SPI_HandleTypeDef *bus)
{
    uint8_t id = 0;

    spi_dev_init(&dev, bus, BMP581_CS_GPIO, BMP581_CS_PIN);

    /* After power-up the BMP581 starts in I2C mode and switches to SPI on the
       first CS edge.  Datasheet 5.6.1: do one dummy read before using it. */
    HAL_Delay(2);
    (void)read_reg(BMP581_REG_CHIP_ID, &id);
    if (read_reg(BMP581_REG_CHIP_ID, &id) != 0)
        return -1;
    if (id != BMP581_CHIP_ID_VAL)
        return -2;

    if (write_reg(BMP581_REG_CMD, BMP581_CMD_SOFT_RESET) != 0) return -1;
    HAL_Delay(3);

    if (write_reg(BMP581_REG_OSR_CONFIG, BMP581_OSR_CONFIG_VAL) != 0) return -1;
    if (write_reg(BMP581_REG_ODR_CONFIG, BMP581_ODR_CONFIG_VAL) != 0) return -1;
    HAL_Delay(5);
    return 0;
}

int bmp581_read(SPI_HandleTypeDef *bus, bmp581_data_t *d)
{
    (void)bus;
    uint8_t tx[7] = { (uint8_t)(BMP581_REG_DATA | 0x80) };
    uint8_t rx[7] = { 0 };

    if (spi_dev_xfer(&dev, tx, rx, sizeof(tx)) != HAL_OK) {
        d->temp_c = 0.0f; d->pressure_hpa = 0.0f; d->altitude_m = 0.0f;
        return -1;
    }

    /* 0x1D..0x1F = temperature XLSB/LSB/MSB, 0x20..0x22 = pressure. */
    int32_t  t_raw = (int32_t)(((uint32_t)rx[3] << 16) |
                               ((uint32_t)rx[2] << 8) |
                               (uint32_t)rx[1]);
    uint32_t p_raw = ((uint32_t)rx[6] << 16) |
                     ((uint32_t)rx[5] << 8) |
                     (uint32_t)rx[4];

    if (t_raw & 0x800000) t_raw |= (int32_t)0xFF000000;

    d->temp_c       = (float)t_raw / 65536.0f;
    d->pressure_hpa = ((float)p_raw / 64.0f) / 100.0f;
    d->altitude_m   = 44330.0f * (1.0f - powf(d->pressure_hpa / SEA_LEVEL_PRESSURE_HPA,
                                              0.190294957f));
    return 0;
}
