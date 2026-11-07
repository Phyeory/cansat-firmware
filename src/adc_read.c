#include "adc_read.h"
#include "board.h"
#include "config.h"

uint16_t adc_read_ch(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset       = 0;

    if (HAL_ADC_ConfigChannel(&O3_ADC, &cfg) != HAL_OK) return 0;
    if (HAL_ADC_Start(&O3_ADC) != HAL_OK) return 0;
    if (HAL_ADC_PollForConversion(&O3_ADC, 10) != HAL_OK) return 0;
    uint16_t v = (uint16_t)HAL_ADC_GetValue(&O3_ADC);
    HAL_ADC_Stop(&O3_ADC);
    return v;
}

float adc_to_v(uint16_t raw)
{
    return (float)raw * VREF_V / ADC_MAX;
}
