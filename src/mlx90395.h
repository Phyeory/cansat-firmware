#pragma once
#include "stm32g4xx_hal.h"

/* Melexis MLX90395 magnetometer on I2C1 @ 0x0C (A1 -> GND, A0 -> GND).
 * Protocol mirrors the Adafruit_MLX90395 reference implementation:
 * commands are raw writes of [0x80, cmd] (1 status byte back), and the
 * burst data read is a raw write of [0x80] followed by 12 bytes
 * [status, resv, X MSB, X LSB, Y MSB, Y LSB, Z MSB, Z LSB, T MSB, T LSB,
 *  V MSB, V LSB].
 * Uses plain Master_Transmit/Master_Receive (STOP between phases); if a
 * board revision refuses it, switch those two calls to a repeated-start.
 * Earth's field is ~50 uT; at the default (high-field) gain that is only
 * ~70 counts — check heading resolution at bring-up and adjust
 * MLX90395_UT_PER_LSB / the gain setting if it is too coarse.
 */

typedef struct {
    float mag_x, mag_y, mag_z;   /* uT */
} mlx90395_data_t;

int  mlx90395_init(I2C_HandleTypeDef *bus);             /* 0 = ok */
/* 1 = fresh sample, 0 = not ready (keep last), <0 = bus error */
int  mlx90395_read(I2C_HandleTypeDef *bus, mlx90395_data_t *d);
