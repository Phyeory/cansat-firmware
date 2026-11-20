#pragma once
#include "stm32g4xx_hal.h"

/* Bosch BMP581 barometer on the shared sensor SPI bus (SPI1, CS = PA8).
   The part outputs factory-calibrated 24-bit temperature and pressure, so the
   driver only scales the words: T = raw / 2^16 degC, p = raw / 2^6 Pa. */

typedef struct {
    float temp_c;
    float pressure_hpa;
    float altitude_m;
} bmp581_data_t;

int bmp581_init(SPI_HandleTypeDef *bus);
int bmp581_read(SPI_HandleTypeDef *bus, bmp581_data_t *d);
