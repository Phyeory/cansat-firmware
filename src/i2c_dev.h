#pragma once
#include <stdint.h>
#include "stm32g4xx_hal.h"

/* Thin register-level helpers shared by every I2C sensor driver. */

HAL_StatusTypeDef i2c_write_reg(I2C_HandleTypeDef *bus, uint16_t addr,
                                uint8_t reg, uint8_t val);
HAL_StatusTypeDef i2c_read_reg(I2C_HandleTypeDef *bus, uint16_t addr,
                               uint8_t reg, uint8_t *val);
HAL_StatusTypeDef i2c_read_regs(I2C_HandleTypeDef *bus, uint16_t addr,
                                uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t i2c_crc8(const uint8_t *data, int len);   /* SCD30/SCD polynomial 0x31 */
