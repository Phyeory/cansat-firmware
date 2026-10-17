#pragma once
#include "stm32g4xx_hal.h"

/* Payload servos: 4x spoiler blades on TIM8 CH1-4, 50 Hz PWM, pulse width in µs. */

void servos_start(void);
void servos_set_us(int idx, uint32_t pulse_us);   /* idx 0..3 */
void servos_stow(void);
void servos_deploy(void);
