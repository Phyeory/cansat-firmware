#pragma once
#include "stm32g4xx_hal.h"

/* BMP280 barometric sensor (primary mission: temperature + pressure). */

typedef struct {
    float temp_c;
    float pressure_hpa;
    float altitude_m;      /* from SEA_LEVEL_PRESSURE_HPA */
} bmp280_data_t;

int  bmp280_init(I2C_HandleTypeDef *bus);              /* 0 = ok */
void bmp280_read(I2C_HandleTypeDef *bus, bmp280_data_t *d);
