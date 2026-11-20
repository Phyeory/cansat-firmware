#include "spi_dev.h"

#define SPI_TIMEOUT_MS 50U

void spi_dev_init(spi_dev_t *dev, SPI_HandleTypeDef *bus,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->bus     = bus;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;
}

HAL_StatusTypeDef spi_dev_xfer(const spi_dev_t *dev, const uint8_t *tx,
                               uint8_t *rx, uint16_t len)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(
        dev->bus, (uint8_t *)tx, rx, len, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    return st;
}
