#pragma once
#include "stm32g4xx_hal.h"

void rad_counter_start(void);
uint32_t rad_counter_latch(void);
uint8_t rad_counter_active(void);
