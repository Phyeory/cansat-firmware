#pragma once
#include "stm32g4xx_hal.h"

typedef struct {
    float ir_counts;
    float full_counts;
    float lux;
} tsl2591_data_t;

int  tsl2591_init(I2C_HandleTypeDef *bus);
void tsl2591_read(I2C_HandleTypeDef *bus, tsl2591_data_t *d);
