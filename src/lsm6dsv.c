#include "lsm6dsv.h"
#include "config.h"
#include "spi_dev.h"
#include <string.h>

#define LSM6DSV_WHO_AM_I_REG    0x0F
#define LSM6DSV_WHO_AM_I_VAL    0x70
#define LSM6DSV_CTRL1_REG       0x10
#define LSM6DSV_CTRL2_REG       0x11
#define LSM6DSV_CTRL3_REG       0x12
#define LSM6DSV_CTRL6_REG       0x15
#define LSM6DSV_CTRL8_REG       0x17
#define LSM6DSV_OUT_GYRO_REG    0x22   /* 0x22..0x27: gyro x/y/z */
#define LSM6DSV_OUT_ACCEL_REG   0x28   /* 0x28..0x2D: accel x/y/z */

#define LSM6DSV_SW_RESET        0x01
#define LSM6DSV_BDU_IF_INC      0x44   /* BDU | IF_INC */
#define LSM6DSV_ODR_120HZ       0x06
#define LSM6DSV_FS_G_2000DPS    0x04
#define LSM6DSV_FS_XL_8G        0x02

#define LSM6DSV_ACC_LSB_PER_G   4096.0f
#define LSM6DSV_GYR_LSB_PER_DPS 14.285714f   /* 70 mdps/LSB */

static void dev_init(spi_dev_t *dev, SPI_HandleTypeDef *bus)
{
    spi_dev_init(dev, bus, LSM6DSV_CS_GPIO, LSM6DSV_CS_PIN);
}

static int read_block(spi_dev_t *dev, uint8_t reg, uint8_t *out, uint16_t len)
{
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};

    if (len + 1U > sizeof(tx)) return -1;
    tx[0] = (uint8_t)(reg | 0x80);
    if (spi_dev_xfer(dev, tx, rx, (uint16_t)(len + 1U)) != HAL_OK) return -1;
    memcpy(out, &rx[1], len);
    return 0;
}

static int read_reg(spi_dev_t *dev, uint8_t reg, uint8_t *v)
{
    return read_block(dev, reg, v, 1);
}

static int write_block(spi_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};

    if (len + 1U > sizeof(tx)) return -1;
    tx[0] = (uint8_t)(reg & 0x7F);
    memcpy(&tx[1], data, len);
    return spi_dev_xfer(dev, tx, rx, (uint16_t)(len + 1U)) == HAL_OK ? 0 : -1;
}

static int write_reg(spi_dev_t *dev, uint8_t reg, uint8_t v)
{
    return write_block(dev, reg, &v, 1);
}

int lsm6dsv_init(SPI_HandleTypeDef *bus)
{
    if (!bus) return -1;

    spi_dev_t dev;
    dev_init(&dev, bus);

    HAL_Delay(10);                    /* boot time */

    uint8_t id = 0;
    if (read_reg(&dev, LSM6DSV_WHO_AM_I_REG, &id) != 0 ||
        id != LSM6DSV_WHO_AM_I_VAL)
        return -1;

    /* Soft reset, then block-data-update + register auto-increment. */
    if (write_reg(&dev, LSM6DSV_CTRL3_REG, LSM6DSV_SW_RESET) != 0) return -1;
    HAL_Delay(10);
    if (write_reg(&dev, LSM6DSV_CTRL3_REG, LSM6DSV_BDU_IF_INC) != 0) return -1;

    /* 120 Hz, +/-8 g, +/-2000 dps. */
    if (write_reg(&dev, LSM6DSV_CTRL8_REG, LSM6DSV_FS_XL_8G) != 0) return -1;
    if (write_reg(&dev, LSM6DSV_CTRL6_REG, LSM6DSV_FS_G_2000DPS) != 0) return -1;
    if (write_reg(&dev, LSM6DSV_CTRL1_REG, LSM6DSV_ODR_120HZ) != 0) return -1;
    if (write_reg(&dev, LSM6DSV_CTRL2_REG, LSM6DSV_ODR_120HZ) != 0) return -1;

    return 0;
}

int lsm6dsv_read(SPI_HandleTypeDef *bus, lsm6dsv_data_t *d)
{
    if (!bus || !d) return -1;

    spi_dev_t dev;
    dev_init(&dev, bus);

    uint8_t b[12];
    if (read_block(&dev, LSM6DSV_OUT_GYRO_REG, b, sizeof(b)) != 0)
        return -1;

    int16_t gx = (int16_t)((uint16_t)b[0]  | ((uint16_t)b[1]  << 8));
    int16_t gy = (int16_t)((uint16_t)b[2]  | ((uint16_t)b[3]  << 8));
    int16_t gz = (int16_t)((uint16_t)b[4]  | ((uint16_t)b[5]  << 8));
    int16_t ax = (int16_t)((uint16_t)b[6]  | ((uint16_t)b[7]  << 8));
    int16_t ay = (int16_t)((uint16_t)b[8]  | ((uint16_t)b[9]  << 8));
    int16_t az = (int16_t)((uint16_t)b[10] | ((uint16_t)b[11] << 8));

    d->acc_x = ax / LSM6DSV_ACC_LSB_PER_G * 9.80665f;
    d->acc_y = ay / LSM6DSV_ACC_LSB_PER_G * 9.80665f;
    d->acc_z = az / LSM6DSV_ACC_LSB_PER_G * 9.80665f;
    d->gyr_x = gx / LSM6DSV_GYR_LSB_PER_DPS;
    d->gyr_y = gy / LSM6DSV_GYR_LSB_PER_DPS;
    d->gyr_z = gz / LSM6DSV_GYR_LSB_PER_DPS;
    return 1;
}
