#pragma once
#include <stdint.h>
#include "stm32g4xx_hal.h"

/* Single-shot polled ADC reads (O3-B4 cell, battery monitor). */
uint16_t adc_read_ch(uint32_t channel);   /* returns 0..4095 (12-bit) */
float    adc_to_v(uint16_t raw);          /* -> volts at the pin */
