#pragma once
#include "stm32g4xx_hal.h"

/* One chip-select line on the shared sensor SPI bus (SPI1).
 *
 * The MLX90395 and BMP581 sit on SPI1 with separate CS pins and
 * both support SPI mode 3 (CPOL = CPHA = 1), which is how board.c configures
 * the bus.  Keeping the transfer in one HAL call means a fake or a logic
 * analyser sees exactly one frame per transaction.
 */
typedef struct {
    SPI_HandleTypeDef *bus;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
} spi_dev_t;

void spi_dev_init(spi_dev_t *dev, SPI_HandleTypeDef *bus,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin);

HAL_StatusTypeDef spi_dev_xfer(const spi_dev_t *dev, const uint8_t *tx,
                               uint8_t *rx, uint16_t len);
