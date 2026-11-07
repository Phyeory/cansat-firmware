#pragma once
#include "stm32g4xx_hal.h"

typedef struct {
    float mag_x, mag_y, mag_z;
} mlx90395_data_t;

int  mlx90395_init(I2C_HandleTypeDef *bus);
int  mlx90395_read(I2C_HandleTypeDef *bus, mlx90395_data_t *d);
