#pragma once
#include "stm32g4xx_hal.h"

/* Bosch BMI270 6-axis IMU on I2C1. Produces no data until the 8 KB config
   file is uploaded (bmi270_config_file.c). +/-2 g, +/-2000 dps, 100 Hz ODR;
   polled every 100 ms (5 Hz Nyquist, enough for parachute pendulation). */

typedef struct {
    float acc_x, acc_y, acc_z;   /* m/s^2 */
    float gyr_x, gyr_y, gyr_z;   /* deg/s */
} bmi270_data_t;

int  bmi270_init(I2C_HandleTypeDef *bus);               /* 0 = ok */
int  bmi270_read(I2C_HandleTypeDef *bus, bmi270_data_t *d);   /* 1 = fresh */
