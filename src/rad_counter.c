#include "rad_counter.h"
#include "config.h"

static volatile uint32_t pulses;        /* written in ISR, latched in main */
static volatile uint32_t last_pulse_tick;

void rad_counter_start(void)
{
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_IT_FALLING;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = RAD_INPUT_PIN;
    HAL_GPIO_Init(RAD_INPUT_GPIO, &g);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);   /* same level as the GPS IRQ */
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

uint32_t rad_counter_latch(void)
{
    __disable_irq();
    uint32_t c = pulses;
    pulses = 0;
    __enable_irq();
    return c;
}

uint8_t rad_counter_active(void)
{
    uint32_t t = last_pulse_tick;       /* single 32-bit read: atomic */
    return (t != 0 && HAL_GetTick() - t < 60000U) ? 1U : 0U;
}

/* overrides the HAL weak symbol used by EXTI15_10_IRQHandler */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == RAD_INPUT_PIN) {
        pulses++;
        last_pulse_tick = HAL_GetTick();
    }
}
