#pragma once
#include "stm32g4xx_hal.h"

/* Melexis MLX90395 magnetometer on the shared sensor SPI bus (SPI1, CS = PB1).
   SPI is mode 3; commands are 16 bits and the status byte comes back in the
   second byte of the frame (datasheet 17.6). */

typedef struct {
    float mag_x, mag_y, mag_z;
} mlx90395_data_t;

int mlx90395_init(SPI_HandleTypeDef *bus);
int mlx90395_read(SPI_HandleTypeDef *bus, mlx90395_data_t *d);
