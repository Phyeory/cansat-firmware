#pragma once
#include "stm32g4xx_hal.h"

/* BPW34 S-Z radiation pulse counter on PB10. The photodiode feeds a
   transimpedance amp + open-drain comparator on the PCB; every pulse pulls
   PB10 low. Latched once per second as pulses/second. */

void rad_counter_start(void);           /* configure PB10 EXTI + NVIC */
uint32_t rad_counter_latch(void);       /* pulses since last latch, resets to 0 */
uint8_t rad_counter_active(void);       /* 1 if a pulse was seen in the last 60 s */
