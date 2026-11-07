#pragma once
#include "stm32g4xx_hal.h"

typedef struct {
    float co2_ppm;
    float temp_c;
    float rh_pct;
} scd30_data_t;

int  scd30_init(I2C_HandleTypeDef *bus);
int  scd30_read(I2C_HandleTypeDef *bus, scd30_data_t *d);
