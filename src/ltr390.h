#pragma once
#include "stm32g4xx_hal.h"

typedef struct {
    float uv_raw;
    float uv_index;
} ltr390_data_t;

int  ltr390_init(I2C_HandleTypeDef *bus);
void ltr390_read(I2C_HandleTypeDef *bus, ltr390_data_t *d);
