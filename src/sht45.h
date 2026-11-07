#pragma once
#include "stm32g4xx_hal.h"

typedef struct {
    float temp_c;
    float rh_pct;
} sht45_data_t;

int  sht45_init(I2C_HandleTypeDef *bus);
int  sht45_poll(I2C_HandleTypeDef *bus, sht45_data_t *d);
