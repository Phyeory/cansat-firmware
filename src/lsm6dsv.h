#pragma once
#include "stm32g4xx_hal.h"

/* ST LSM6DSV 6-axis IMU on the shared sensor SPI bus (SPI1, CS = PB0).
 *
 * Default configuration used here: 120 Hz ODR, +/-8 g, +/-2000 dps,
 * block-data-update + register auto-increment.  Outputs are converted to
 * m/s^2 and deg/s to match the telemetry contract. */
typedef struct {
    float acc_x, acc_y, acc_z;
    float gyr_x, gyr_y, gyr_z;
} lsm6dsv_data_t;

int lsm6dsv_init(SPI_HandleTypeDef *bus);
int lsm6dsv_read(SPI_HandleTypeDef *bus, lsm6dsv_data_t *d);
