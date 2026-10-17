#include "adc_read.h"
#include "board.h"
#include "config.h"

uint16_t adc_read_ch(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;

    if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK) return 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) return 0;
    return (uint16_t)HAL_ADC_GetValue(&hadc1);
}

float adc_to_v(uint16_t raw)
{
    return (float)raw * VREF_V / ADC_MAX;
}
