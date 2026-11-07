#pragma once
#include "stm32g4xx_hal.h"

int     lora_init(SPI_HandleTypeDef *spi);
int     lora_send(const uint8_t *data, uint8_t len);
uint8_t lora_busy(void);
uint32_t lora_tx_count(void);
uint32_t lora_fail_count(void);
