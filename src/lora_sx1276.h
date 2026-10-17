#pragma once
#include "stm32g4xx_hal.h"

/* Minimal SX1276 (RFM96W) LoRa TX driver for the 433 MHz downlink. */

int  lora_init(SPI_HandleTypeDef *spi);           /* 0 = ok */
int  lora_send(const uint8_t *data, uint8_t len); /* 0 = ok, blocks until TxDone */
