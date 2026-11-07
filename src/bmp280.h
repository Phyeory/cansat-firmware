#pragma once
#include "stm32g4xx_hal.h"


typedef struct {
    float temp_c;
    float pressure_hpa;
    float altitude_m;
} bmp280_data_t;

int  bmp280_init(I2C_HandleTypeDef *bus);
void bmp280_read(I2C_HandleTypeDef *bus, bmp280_data_t *d);
