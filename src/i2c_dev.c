#include "i2c_dev.h"

HAL_StatusTypeDef i2c_write_reg(I2C_HandleTypeDef *bus, uint16_t addr,
                                uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return HAL_I2C_Master_Transmit(bus, addr, b, 2, 100);
}

HAL_StatusTypeDef i2c_read_reg(I2C_HandleTypeDef *bus, uint16_t addr,
                               uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(bus, addr, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 100);
}

HAL_StatusTypeDef i2c_read_regs(I2C_HandleTypeDef *bus, uint16_t addr,
                                uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(bus, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

/* CRC-8, polynomial 0x31, init 0xFF — used by SCD30 and other Sensirion parts */
uint8_t i2c_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}
