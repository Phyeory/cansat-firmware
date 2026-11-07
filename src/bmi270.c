#include "bmi270.h"
#include "config.h"
#include "i2c_dev.h"
#include "bmi270_config_file.h"

#define BMI270_CHIP_ID_REG      0x00
#define BMI270_CHIP_ID_VAL      0x24
#define BMI270_INTERNAL_STATUS  0x21
#define BMI270_ACC_DATA_REG     0x0C
#define BMI270_ACC_CONF_REG     0x40
#define BMI270_GYR_CONF_REG     0x42
#define BMI270_INIT_CTRL_REG    0x59
#define BMI270_INIT_ADDR_REG    0x5B
#define BMI270_INIT_DATA_REG    0x5E
#define BMI270_PWR_CONF_REG     0x7C
#define BMI270_PWR_CTRL_REG     0x7D
#define BMI270_CMD_REG          0x7E
#define BMI270_CMD_SOFT_RESET   0xB6

#define BMI270_ACC_LSB_PER_G    16384.0f
#define BMI270_GYR_LSB_PER_DPS  16.384f
#define BMI270_CONFIG_CHUNK     32U

int bmi270_init(I2C_HandleTypeDef *bus)
{
    uint8_t v;

    if (i2c_read_reg(bus, BMI270_I2C_ADDR, BMI270_CHIP_ID_REG, &v) != HAL_OK ||
        v != BMI270_CHIP_ID_VAL)
        return -1;

    i2c_write_reg(bus, BMI270_I2C_ADDR, BMI270_CMD_REG, BMI270_CMD_SOFT_RESET);
    HAL_Delay(2);

    i2c_write_reg(bus, BMI270_I2C_ADDR, BMI270_PWR_CONF_REG, 0x00);
    HAL_Delay(1);

    i2c_write_reg(bus, BMI270_I2C_ADDR, BMI270_INIT_CTRL_REG, 0x00);
    for (uint16_t i = 0; i < BMI270_CONFIG_FILE_SIZE; i += BMI270_CONFIG_CHUNK) {
        uint8_t addr[2] = { (uint8_t)((i / 2) & 0x0F), (uint8_t)((i / 2) >> 4) };
        HAL_I2C_Mem_Write(bus, BMI270_I2C_ADDR, BMI270_INIT_ADDR_REG,
                          I2C_MEMADD_SIZE_8BIT, addr, 2, 100);
        HAL_I2C_Mem_Write(bus, BMI270_I2C_ADDR, BMI270_INIT_DATA_REG,
                          I2C_MEMADD_SIZE_8BIT,
                          (uint8_t *)&bmi270_config_file[i], BMI270_CONFIG_CHUNK, 100);
    }
    i2c_write_reg(bus, BMI270_I2C_ADDR, BMI270_INIT_CTRL_REG, 0x01);

    for (int tries = 0; tries < 10; tries++) {
        HAL_Delay(2);
        if (i2c_read_reg(bus, BMI270_I2C_ADDR, BMI270_INTERNAL_STATUS, &v) != HAL_OK)
            return -1;
        if ((v & 0x0F) != 0x01)
            continue;

        i2c_write_reg(bus, BMI270_I2C_ADDR, BMI270_PWR_CTRL_REG, 0x14);
        HAL_Delay(30);
        i2c_write_reg(bus, BMI270_I2C_ADDR, BMI270_ACC_CONF_REG, 0x48);
        i2c_write_reg(bus, BMI270_I2C_ADDR, BMI270_GYR_CONF_REG, 0xA8);
        HAL_Delay(3);
        return 0;
    }
    return -1;
}

int bmi270_read(I2C_HandleTypeDef *bus, bmi270_data_t *d)
{
    uint8_t b[12];
    if (i2c_read_regs(bus, BMI270_I2C_ADDR, BMI270_ACC_DATA_REG, b, 12) != HAL_OK)
        return -1;

    int16_t raw[6];
    for (int i = 0; i < 6; i++)
        raw[i] = (int16_t)((b[2 * i + 1] << 8) | b[2 * i]);

    d->acc_x = raw[0] / BMI270_ACC_LSB_PER_G * 9.80665f;
    d->acc_y = raw[1] / BMI270_ACC_LSB_PER_G * 9.80665f;
    d->acc_z = raw[2] / BMI270_ACC_LSB_PER_G * 9.80665f;
    d->gyr_x = raw[3] / BMI270_GYR_LSB_PER_DPS;
    d->gyr_y = raw[4] / BMI270_GYR_LSB_PER_DPS;
    d->gyr_z = raw[5] / BMI270_GYR_LSB_PER_DPS;
    return 1;
}
