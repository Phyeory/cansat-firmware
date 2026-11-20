#pragma once
#include "stm32g4xx_hal.h"

/* EBYTE E22-900T22D telemetry radio on USART3.
 *
 * The E22 is a transparent UART pipe: a packet is a UART write.  The module
 * itself is configured (with EBYTE's tool or its C0 command mode) for
 * 868.125 MHz, 14 dBm, 62.5 kbps and transparent transmission; no SPI, RF
 * layout or vendor library is involved. */

int      e22_init(UART_HandleTypeDef *uart);
int      e22_send(const uint8_t *data, uint16_t len);
uint8_t  e22_busy(void);
uint32_t e22_tx_count(void);
uint32_t e22_fail_count(void);
