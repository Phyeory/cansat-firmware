#pragma once
#include <stdint.h>
#include "stm32g4xx_hal.h"

uint16_t adc_read_ch(uint32_t channel);
float    adc_to_v(uint16_t raw);
