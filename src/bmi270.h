#pragma once
#include "stm32g4xx_hal.h"

/* Bosch BMI270 6-axis IMU on I2C1. The 8 KB configuration must be uploaded
   before readings are available. */

typedef struct {
    float acc_x, acc_y, acc_z;
    float gyr_x, gyr_y, gyr_z;
} bmi270_data_t;

int  bmi270_init(I2C_HandleTypeDef *bus);
int  bmi270_read(I2C_HandleTypeDef *bus, bmi270_data_t *d);
