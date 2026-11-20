#include "mlx90395.h"
#include "spi_dev.h"
#include "config.h"

/* SPI command bytes (datasheet table 18).  zyxt selects components:
   X = 1, Y = 2, Z = 4, T = 8. */
#define MLX_CMD_EXIT_MODE       0x80
#define MLX_CMD_RESET           0xF0
#define MLX_CMD_SINGLE_XYZT     0x3F
#define MLX_CMD_READ_XYZT       0x4F

#define MLX_STATUS_DRDY         0x01
#define MLX_STATUS_RESET        0x02
#define MLX_STATUS_SMMODE       0x20

#define MLX90395_UT_PER_LSB     0.714f

static spi_dev_t dev;

static uint8_t command(uint8_t cmd)
{
    uint8_t tx[2] = { cmd, 0x00 };
    uint8_t rx[2] = { 0 };

    if (spi_dev_xfer(&dev, tx, rx, 2) != HAL_OK) return 0xFF;
    return rx[1];
}

int mlx90395_init(SPI_HandleTypeDef *bus)
{
    spi_dev_init(&dev, bus, MLX90395_CS_GPIO, MLX90395_CS_PIN);

    (void)command(MLX_CMD_EXIT_MODE);
    if (command(MLX_CMD_EXIT_MODE) == 0xFF) return -1;
    HAL_Delay(10);

    /* RT is a single-byte command and deliberately returns no status. */
    {
        uint8_t tx[1] = { MLX_CMD_RESET };
        uint8_t rx[1] = { 0 };
        if (spi_dev_xfer(&dev, tx, rx, 1) != HAL_OK) return -1;
    }
    HAL_Delay(10);
    /* The first command after RT reports the reset event; consume it so the
       first real SM read is not mistaken for a bad status. */
    (void)command(MLX_CMD_EXIT_MODE);
    HAL_Delay(1);
    return 0;
}

int mlx90395_read(SPI_HandleTypeDef *bus, mlx90395_data_t *d)
{
    (void)bus;
    uint8_t tx[12] = { MLX_CMD_READ_XYZT };
    uint8_t rx[12] = { 0 };

    if (command(MLX_CMD_SINGLE_XYZT) != MLX_STATUS_SMMODE)
        return 0;

    /* Small measurement delay; the status byte is the authority. */
    HAL_Delay(2);

    if (spi_dev_xfer(&dev, tx, rx, sizeof(tx)) != HAL_OK)
        return -1;

    if (!(rx[1] & MLX_STATUS_DRDY))
        return 0;

    /* RM with zyxt != 0 returns Status-T-X-Y-Z-CRC. */
    int16_t x = (int16_t)(((uint16_t)rx[4] << 8) | rx[5]);
    int16_t y = (int16_t)(((uint16_t)rx[6] << 8) | rx[7]);
    int16_t z = (int16_t)(((uint16_t)rx[8] << 8) | rx[9]);

    d->mag_x = x * MLX90395_UT_PER_LSB;
    d->mag_y = y * MLX90395_UT_PER_LSB;
    d->mag_z = z * MLX90395_UT_PER_LSB;
    return 1;
}
