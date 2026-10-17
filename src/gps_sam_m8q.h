#pragma once
#include "stm32g4xx_hal.h"
#include <stdbool.h>

/* u-blox SAM-M8Q GPS over USART1 — polls the UART, parses NMEA RMC/GGA. */

typedef struct {
    double lat_deg, lon_deg;
    float  alt_m;          /* MSL altitude from GGA */
    uint8_t sats;
    bool   fix;            /* RMC status A */
} gps_data_t;

void gps_init(UART_HandleTypeDef *uart);   /* configures NMEA RMC+GGA @ 1 Hz */
void gps_poll(gps_data_t *g);              /* drains UART RX, updates fix data */
