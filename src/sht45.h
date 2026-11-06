#pragma once
#include "stm32g4xx_hal.h"

/* Sensirion SHT45 humidity + independent temperature, I2C1 @ 0x44. */

typedef struct {
    float temp_c;    /* SHT45's own temperature (independent of BMP280) */
    float rh_pct;    /* relative humidity, clamped 0..100 */
} sht45_data_t;

int  sht45_init(I2C_HandleTypeDef *bus);                /* 0 = ok */
/* Polls at 1 Hz internally (trigger on one tick, read on the next, so the
   8.3 ms conversion never blocks the 10 Hz loop). 1 = fresh sample. */
int  sht45_poll(I2C_HandleTypeDef *bus, sht45_data_t *d);
