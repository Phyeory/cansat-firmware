#pragma once
#include "stm32g4xx_hal.h"
#include <stdbool.h>

typedef struct {
    double  lat_deg, lon_deg;
    float   alt_m;
    uint8_t sats;
    bool    fix;
    uint32_t sentences;
} gps_data_t;

void gps_init(UART_HandleTypeDef *uart);
void gps_poll(gps_data_t *g);
