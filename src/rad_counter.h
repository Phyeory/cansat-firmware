#pragma once
#include "stm32g4xx_hal.h"

/* BPW34 S-Z radiation pulse counter (PDR secondary mission).
 *
 * A photodiode cannot drive a GPIO: the PCB needs a transimpedance amp
 * plus a comparator whose open-drain output pulls **PB10** low for every
 * pulse; the internal pull-up holds it high between pulses. The firmware
 * side is just a falling-edge counter, latched once per second so the
 * downlink carries pulses/second (the science quantity is count rate vs
 * altitude).
 */

void rad_counter_start(void);           /* configure PB10 EXTI + NVIC */
uint32_t rad_counter_latch(void);       /* pulses since last latch, resets to 0 */
uint8_t rad_counter_active(void);       /* 1 if a pulse was seen in the last 60 s */
