#pragma once
#include "stm32g4xx_hal.h"


void servos_start(void);
void servos_set_us(int idx, uint32_t pulse_us);
void servos_stow(void);
void servos_deploy(void);
