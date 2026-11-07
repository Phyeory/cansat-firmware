#include "stm32g4xx_hal.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
}

void HardFault_Handler(void)  { while (1) { } }
void NMI_Handler(void)        { while (1) { } }
void MemManage_Handler(void)  { while (1) { } }
void BusFault_Handler(void)   { while (1) { } }
void UsageFault_Handler(void) { while (1) { } }

void Error_Handler(void);
void Error_Handler(void) { __disable_irq(); while (1) { } }
