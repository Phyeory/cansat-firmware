#pragma once
#include "stm32g4xx_hal.h"

/* Bosch BMI270 6-axis IMU (accel + gyro) on I2C1 @ 0x68 (SDO -> GND).
 *
 * The chip stays silent until its 8 KB configuration file is streamed in
 * (bmi270_config_file.c, from Bosch's BSD-licensed BMI270_SensorAPI).
 * After init: accel +/-2 g and gyro +/-2000 dps, both 100 Hz ODR; we poll
 * the latest sample every 100 ms (10 Hz logging -> 5 Hz Nyquist, enough
 * for the sub-1 Hz pendulation of a can under a parachute).
 */

typedef struct {
    float acc_x, acc_y, acc_z;   /* m/s^2 */
    float gyr_x, gyr_y, gyr_z;   /* deg/s */
} bmi270_data_t;

int  bmi270_init(I2C_HandleTypeDef *bus);               /* 0 = ok */
/* 1 = fresh sample, 0 = bus hiccup (keep last), <0 = init needed */
int  bmi270_read(I2C_HandleTypeDef *bus, bmi270_data_t *d);
