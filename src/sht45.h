#pragma once
#include "stm32g4xx_hal.h"

/* Sensirion SHT45 (SHT4x family) humidity + independent temperature,
 * I2C1 @ 0x44 (fixed address). Humidity is what the PDR's dew-point and
 * absolute-humidity calculations need; the sensor also gives a second
 * temperature so the BMP280 reading can be sanity-checked.
 *
 * Single-shot high-repeatability (0xFD) needs up to 8.3 ms, so the call
 * pattern is: sht45_trigger() on one 100 ms loop tick, sht45_read() on the
 * next — the measurement finishes inside the 100 ms gap, keeping the loop
 * non-blocking. Words are CRC-8 (0x31/0xFF), same as the SCD30.
 */

typedef struct {
    float temp_c;    /* SHT45's own temperature (independent of BMP280) */
    float rh_pct;    /* relative humidity, clamped 0..100 */
} sht45_data_t;

int  sht45_init(I2C_HandleTypeDef *bus);                /* 0 = ok */
void sht45_trigger(I2C_HandleTypeDef *bus);             /* start measurement */
/* 1 = fresh CRC-valid sample, 0 = not ready / CRC bad (keep last), <0 = bus error */
int  sht45_read(I2C_HandleTypeDef *bus, sht45_data_t *d);
