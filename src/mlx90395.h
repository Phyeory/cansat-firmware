#pragma once
#include "stm32g4xx_hal.h"

/* Melexis MLX90395 magnetometer on I2C1. Commands are raw writes of
   [0x80, cmd]; data burst is a raw [0x80] write then 12 bytes back.
   Earth's field is ~50 uT, so check heading resolution at bring-up and
   adjust MLX90395_UT_PER_LSB if the default gain is too coarse. */

typedef struct {
    float mag_x, mag_y, mag_z;   /* uT */
} mlx90395_data_t;

int  mlx90395_init(I2C_HandleTypeDef *bus);             /* 0 = ok */
int  mlx90395_read(I2C_HandleTypeDef *bus, mlx90395_data_t *d);   /* 1 = fresh */
