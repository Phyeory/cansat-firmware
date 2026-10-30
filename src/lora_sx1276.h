#pragma once
#include "stm32g4xx_hal.h"

/* Minimal SX1276 (RFM96W) LoRa TX driver for the 433 MHz downlink.
 * Sending is non-blocking: lora_send() loads the FIFO and starts TX, then
 * returns. A 170-byte packet takes ~290 ms on air at SF7, so blocking would
 * stall the 10 Hz sampling loop. */

int     lora_init(SPI_HandleTypeDef *spi);            /* 0 = ok, <0 = no radio */
int     lora_send(const uint8_t *data, uint8_t len);  /* 0 = started, -1 busy/not ready */
uint8_t lora_busy(void);                              /* 1 while a packet is on air */
uint32_t lora_tx_count(void);                         /* packets fully sent */
uint32_t lora_fail_count(void);                       /* TX timeouts */
