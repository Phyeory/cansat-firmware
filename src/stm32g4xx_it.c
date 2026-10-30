/* Interrupt handlers. SysTick drives the HAL tick; USART1 (GPS RX) lives in
   gps_sam_m8q.c; everything else polls. */
#include "stm32g4xx_hal.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void HardFault_Handler(void)  { while (1) { } }
void NMI_Handler(void)        { while (1) { } }
void MemManage_Handler(void)  { while (1) { } }
void BusFault_Handler(void)   { while (1) { } }
void UsageFault_Handler(void) { while (1) { } }

void Error_Handler(void);
void Error_Handler(void) { __disable_irq(); while (1) { } }
