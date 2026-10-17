#pragma once
#include "stm32g4xx_hal.h"

/* TSL2591 light sensor — secondary mission (IR vs altitude). */

typedef struct {
    float ir_counts;
    float full_counts;
    float lux;
} tsl2591_data_t;

int  tsl2591_init(I2C_HandleTypeDef *bus);             /* 0 = ok */
void tsl2591_read(I2C_HandleTypeDef *bus, tsl2591_data_t *d);
