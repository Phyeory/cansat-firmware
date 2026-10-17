#pragma once
#include "stm32g4xx_hal.h"

/* SCD30 CO2 sensor (I2C mode) — secondary mission (CO2 vs altitude). */

typedef struct {
    float co2_ppm;
    float temp_c;
    float rh_pct;
} scd30_data_t;

int  scd30_init(I2C_HandleTypeDef *bus);               /* 0 = ok */
/* returns 1 if a fresh measurement was read, 0 if not ready, <0 on error */
int  scd30_read(I2C_HandleTypeDef *bus, scd30_data_t *d);
