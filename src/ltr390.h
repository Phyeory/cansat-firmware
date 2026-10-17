#pragma once
#include "stm32g4xx_hal.h"

/* LTR390 UV sensor — secondary mission (UV vs altitude). */

typedef struct {
    float uv_raw;      /* raw UVS counts */
    float uv_index;    /* scaled UVI */
} ltr390_data_t;

int  ltr390_init(I2C_HandleTypeDef *bus);              /* 0 = ok */
void ltr390_read(I2C_HandleTypeDef *bus, ltr390_data_t *d);
