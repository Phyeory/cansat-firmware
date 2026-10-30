#pragma once
#include "stm32g4xx_hal.h"
#include <stdbool.h>

/* u-blox SAM-M8Q GPS over USART1.
 * Bytes arrive by interrupt into a ring buffer (polling at 10 Hz loses most
 * of a 9600-baud NMEA stream to UART overrun). gps_poll() parses whatever
 * full sentences have arrived. Uses the module's factory NMEA output. */

typedef struct {
    double  lat_deg, lon_deg;
    float   alt_m;          /* MSL altitude from GGA */
    uint8_t sats;
    bool    fix;            /* RMC status A */
    uint32_t sentences;     /* valid sentences parsed (wiring check) */
} gps_data_t;

void gps_init(UART_HandleTypeDef *uart);
void gps_poll(gps_data_t *g);
