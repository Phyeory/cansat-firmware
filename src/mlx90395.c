#include "mlx90395.h"
#include "config.h"
#include "i2c_dev.h"

/* MLX90395 command set (as used by Adafruit_MLX90395) */
#define MLX_CMD_BURST_READ      0x80    /* raw write alone -> 12 data bytes */
#define MLX_CMD_EXIT_MODE       0x80    /* sent as [0x80, 0x80] */
#define MLX_CMD_RESET           0xF0    /* [0x80, 0xF0] -> status 0x02 */
#define MLX_CMD_SINGLE_XYZT     0x3F    /* [0x80, 0x3F] -> status 0x20 */

#define MLX_STATUS_DRDY         0x01
#define MLX_STATUS_RESET        0x02
#define MLX_STATUS_SMMODE       0x20

/* uT per LSB at the power-on default gain (8, high field):
   gainMultipliers[8] = 0.1, _uTLSB = 7.14. VERIFY at bring-up against a
   known heading; define lives here so it is easy to retune. */
#define MLX90395_UT_PER_LSB     0.714f

/* one raw command: write [0x80, cmd], read 1 status byte */
static uint8_t mlx_command(I2C_HandleTypeDef *bus, uint8_t cmd)
{
    uint8_t w[2] = { MLX_CMD_BURST_READ, cmd };
    uint8_t status = 0xFF;
    if (HAL_I2C_Master_Transmit(bus, MLX90395_I2C_ADDR, w, 2, 100) != HAL_OK)
        return 0xFF;
    if (HAL_I2C_Master_Receive(bus, MLX90395_I2C_ADDR, &status, 1, 100) != HAL_OK)
        return 0xFF;
    return status;
}

int mlx90395_init(I2C_HandleTypeDef *bus)
{
    /* exit any burst mode (twice, first status ignored), then reset */
    mlx_command(bus, MLX_CMD_EXIT_MODE);
    if (mlx_command(bus, MLX_CMD_EXIT_MODE) == 0xFF) return -1;
    HAL_Delay(10);
    if (mlx_command(bus, MLX_CMD_RESET) != MLX_STATUS_RESET) return -2;
    HAL_Delay(10);
    return 0;
}

int mlx90395_read(I2C_HandleTypeDef *bus, mlx90395_data_t *d)
{
    uint8_t b[12];

    if (mlx_command(bus, MLX_CMD_SINGLE_XYZT) != MLX_STATUS_SMMODE)
        return 0;                       /* busy or no ack: keep last value */

    uint8_t w = MLX_CMD_BURST_READ;
    if (HAL_I2C_Master_Transmit(bus, MLX90395_I2C_ADDR, &w, 1, 100) != HAL_OK)
        return -1;
    if (HAL_I2C_Master_Receive(bus, MLX90395_I2C_ADDR, b, 12, 100) != HAL_OK)
        return -1;
    if (b[0] != MLX_STATUS_DRDY)
        return 0;

    d->mag_x = (int16_t)((b[2] << 8) | b[3]) * MLX90395_UT_PER_LSB;
    d->mag_y = (int16_t)((b[4] << 8) | b[5]) * MLX90395_UT_PER_LSB;
    d->mag_z = (int16_t)((b[6] << 8) | b[7]) * MLX90395_UT_PER_LSB;
    return 1;
}
