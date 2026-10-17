#include "servos.h"
#include "board.h"
#include "config.h"

void servos_start(void)
{
    /* TIM8 channels were started in board_init(); just park them stowed */
    servos_stow();
}

void servos_set_us(int idx, uint32_t pulse_us)
{
    static const uint32_t ch[4] = { TIM_CHANNEL_1, TIM_CHANNEL_2,
                                    TIM_CHANNEL_3, TIM_CHANNEL_4 };
    if (idx < 0 || idx > 3 || pulse_us < 500U || pulse_us > 2500U) return;
    __HAL_TIM_SET_COMPARE(&htim8, ch[idx], pulse_us);
}

void servos_stow(void)
{
    for (int i = 0; i < 4; i++) servos_set_us(i, SERVO_STOWED_US);
}

void servos_deploy(void)
{
    for (int i = 0; i < 4; i++) servos_set_us(i, SERVO_DEPLOYED_US);
}
