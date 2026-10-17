/**
 * board.c — MCU bring-up: clock tree (HSI16 -> PLL -> 170 MHz), GPIO,
 * and all peripheral inits (I2C1, SPI1, SPI2, USART1/2, ADC1, TIM8 PWM).
 * Hand-written equivalents of CubeMX MX_*_Init(); regenerating code from
 * stm32g474.ioc replaces this file's role — keep the two in sync.
 */
#include "board.h"

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
ADC_HandleTypeDef  hadc1;
TIM_HandleTypeDef  htim8;

static void clock_init(void);
static void gpio_init(void);

void SystemClock_Config(void);
static void Error_Handler(void);

void board_init(void)
{
    HAL_Init();
    clock_init();
    gpio_init();

    /* I2C1: 400 kHz fast mode, PB8=SCL PB9=SDA */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    hi2c1.Instance             = I2C1;
    hi2c1.Init.Timing          = 0x10909CECU;  /* 400 kHz @ 170 MHz HCLK */
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();

    /* SPI1: LoRa, ~10 MHz safe, mode 0 */
    __HAL_RCC_SPI1_CLK_ENABLE();
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /* 170/16 ≈ 10.6 MHz */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();

    /* SPI2: SD card, start slow (400 kHz-ish), FatFs raises speed later */
    __HAL_RCC_SPI2_CLK_ENABLE();
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi2.Init.NSS               = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; /* slow for init */
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();

    /* USART1: SAM-M8Q GPS @ 9600 8N1 */
    __HAL_RCC_USART1_CLK_ENABLE();
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 9600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();

    /* USART2: debug console @ 115200 */
    __HAL_RCC_USART2_CLK_ENABLE();
    huart2 = huart1;
    huart2.Instance      = USART2;
    huart2.Init.BaudRate = 115200;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();

    /* ADC1: 12-bit, single conversion, polled */
    __HAL_RCC_ADC12_CLK_ENABLE();
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation      = 0;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    /* TIM8: 4-channel 50 Hz PWM for servos. 170 MHz / 170 = 1 MHz tick;
       ARR 19999 -> 20 ms period; CCR set in µs from servos.c */
    __HAL_RCC_TIM8_CLK_ENABLE();
    TIM_OC_InitTypeDef oc = {0};
    htim8.Instance               = TIM8;
    htim8.Init.Prescaler         = 169;
    htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim8.Init.Period            = 19999;
    htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) Error_Handler();
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = SERVO_STOWED_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    for (uint32_t ch = TIM_CHANNEL_1; ch <= TIM_CHANNEL_4; ch++) {
        if (HAL_TIM_PWM_ConfigChannel(&htim8, &oc, ch) != HAL_OK) Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
}

static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState       = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState  = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;   /* 16 MHz */
    osc.PLL.PLLM      = 4;                   /* /4  = 4 MHz   */
    osc.PLL.PLLN      = 85;                  /* *85 = 340 MHz */
    osc.PLL.PLLP      = RCC_PLLP_DIV2;
    osc.PLL.PLLQ      = RCC_PLLQ_DIV2;
    osc.PLL.PLLR      = RCC_PLLR_DIV2;       /* /2  = 170 MHz */
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* digital IO: LoRa CS/RST (idle high), SD CS (idle high), DIO0 in */
    HAL_GPIO_WritePin(LORA_CS_GPIO, LORA_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SD_CS_GPIO, SD_CS_PIN, GPIO_PIN_SET);

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;

    g.Pin = LORA_CS_PIN;  HAL_GPIO_Init(LORA_CS_GPIO, &g);
    g.Pin = LORA_RST_PIN; HAL_GPIO_Init(LORA_RST_GPIO, &g);
    g.Pin = SD_CS_PIN;    HAL_GPIO_Init(SD_CS_GPIO, &g);

    g.Pin = LORA_DIO0_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(LORA_DIO0_GPIO, &g);
}

/* printf to debug UART */
int _write(int fd, const char *ptr, int len)
{
    (void)fd;
    HAL_UART_Transmit(&huart2, (const uint8_t *)ptr, (uint16_t)len, 100);
    return len;
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
