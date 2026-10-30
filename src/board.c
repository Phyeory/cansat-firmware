/**
 * board.c: MCU bring-up for the Celestium CanSat.
 *
 * Clock tree (HSI16 -> PLL -> 170 MHz), then every peripheral AND the GPIO
 * alternate-function setup that physically connects each pin to its
 * peripheral. Without the AF setup the pins stay as plain inputs and no
 * sensor, radio or GPS traffic ever leaves the chip.
 *
 * Hand-written equivalent of CubeMX MX_*_Init() + HAL_*_MspInit().
 * Keep in sync with stm32g474.ioc.
 */
#include "board.h"

I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi1;
SPI_HandleTypeDef  hspi2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
ADC_HandleTypeDef  hadc2;
TIM_HandleTypeDef  htim8;

static void clock_init(void);
static void gpio_init(void);
static void i2c1_init(void);
static void spi_init(SPI_HandleTypeDef *h, SPI_TypeDef *inst, uint32_t prescaler);
static void uart_init(UART_HandleTypeDef *h, USART_TypeDef *inst, uint32_t baud);
static void adc2_init(void);
static void tim8_init(void);
static void board_fault(void);

/* one pin -> one peripheral alternate function */
static void af_pin(GPIO_TypeDef *port, uint32_t pin, uint32_t af,
                   uint32_t mode, uint32_t pull)
{
    GPIO_InitTypeDef g = {0};
    g.Pin       = pin;
    g.Mode      = mode;
    g.Pull      = pull;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = af;
    HAL_GPIO_Init(port, &g);
}

void board_init(void)
{
    HAL_Init();
    clock_init();
    gpio_init();
    i2c1_init();

    /* SPI1: LoRa. 170/16 = 10.6 MHz (SX1276 max is 10 MHz, close enough
       in practice; use _32 if you see corrupt register reads). */
    __HAL_RCC_SPI1_CLK_ENABLE();
    af_pin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_AF5_SPI1,
           GPIO_MODE_AF_PP, GPIO_NOPULL);
    spi_init(&hspi1, SPI1, SPI_BAUDRATEPRESCALER_16);

    /* SPI2: SD card, slow for card init; FatFs can raise it later */
    __HAL_RCC_SPI2_CLK_ENABLE();
    af_pin(GPIOB, GPIO_PIN_13 | GPIO_PIN_15, GPIO_AF5_SPI2,
           GPIO_MODE_AF_PP, GPIO_NOPULL);
    af_pin(GPIOB, GPIO_PIN_14, GPIO_AF5_SPI2, GPIO_MODE_AF_PP, GPIO_PULLUP);
    spi_init(&hspi2, SPI2, SPI_BAUDRATEPRESCALER_256);

    /* USART1: GPS. RX pulled up so a missing GPS reads as idle line. */
    __HAL_RCC_USART1_CLK_ENABLE();
    af_pin(GPIOA, GPIO_PIN_9,  GPIO_AF7_USART1, GPIO_MODE_AF_PP, GPIO_NOPULL);
    af_pin(GPIOA, GPIO_PIN_10, GPIO_AF7_USART1, GPIO_MODE_AF_PP, GPIO_PULLUP);
    uart_init(&huart1, USART1, GPS_BAUD);

    /* USART2: debug console */
    __HAL_RCC_USART2_CLK_ENABLE();
    af_pin(GPIOA, GPIO_PIN_2, GPIO_AF7_USART2, GPIO_MODE_AF_PP, GPIO_NOPULL);
    af_pin(GPIOA, GPIO_PIN_3, GPIO_AF7_USART2, GPIO_MODE_AF_PP, GPIO_PULLUP);
    uart_init(&huart2, USART2, 115200U);

    adc2_init();
    tim8_init();
}

static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState  = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;   /* 16 MHz */
    osc.PLL.PLLM      = RCC_PLLM_DIV4;       /* /4  = 4 MHz   */
    osc.PLL.PLLN      = 85;                  /* *85 = 340 MHz */
    osc.PLL.PLLP      = RCC_PLLP_DIV2;
    osc.PLL.PLLQ      = RCC_PLLQ_DIV2;
    osc.PLL.PLLR      = RCC_PLLR_DIV2;       /* /2  = 170 MHz */
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) board_fault();

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) board_fault();

    /* I2C1 from HSI16 so the bus timing is fixed whatever SYSCLK does */
    RCC_PeriphCLKInitTypeDef p = {0};
    p.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    p.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&p) != HAL_OK) board_fault();
}

static void gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* chip selects idle high BEFORE they become outputs (no glitch) */
    HAL_GPIO_WritePin(LORA_CS_GPIO, LORA_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_RST_GPIO, LORA_RST_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SD_CS_GPIO, SD_CS_PIN, GPIO_PIN_SET);

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = LORA_CS_PIN;  HAL_GPIO_Init(LORA_CS_GPIO, &g);
    g.Pin = LORA_RST_PIN; HAL_GPIO_Init(LORA_RST_GPIO, &g);
    g.Pin = SD_CS_PIN;    HAL_GPIO_Init(SD_CS_GPIO, &g);

    g.Pin  = LORA_DIO0_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(LORA_DIO0_GPIO, &g);
}

static void i2c1_init(void)
{
    /* PB8=SCL, PB9=SDA, AF4, open drain. Internal pull-ups are only a weak
       backup; the PCB must have real 4.7k pull-ups on both lines. */
    __HAL_RCC_I2C1_CLK_ENABLE();
    af_pin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_AF4_I2C1,
           GPIO_MODE_AF_OD, GPIO_PULLUP);

    hi2c1.Instance             = I2C1;
    hi2c1.Init.Timing          = I2C_TIMING_100K;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;  /* SCD30 stretches */
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) board_fault();
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

static void spi_init(SPI_HandleTypeDef *h, SPI_TypeDef *inst, uint32_t prescaler)
{
    h->Instance               = inst;
    h->Init.Mode              = SPI_MODE_MASTER;
    h->Init.Direction         = SPI_DIRECTION_2LINES;
    h->Init.DataSize          = SPI_DATASIZE_8BIT;
    h->Init.CLKPolarity       = SPI_POLARITY_LOW;      /* mode 0 */
    h->Init.CLKPhase          = SPI_PHASE_1EDGE;
    h->Init.NSS               = SPI_NSS_SOFT;
    h->Init.BaudRatePrescaler = prescaler;
    h->Init.FirstBit          = SPI_FIRSTBIT_MSB;
    h->Init.TIMode            = SPI_TIMODE_DISABLE;
    h->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    h->Init.CRCPolynomial     = 7;
    h->Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    h->Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(h) != HAL_OK) board_fault();
}

static void uart_init(UART_HandleTypeDef *h, USART_TypeDef *inst, uint32_t baud)
{
    h->Instance                    = inst;
    h->Init.BaudRate               = baud;
    h->Init.WordLength             = UART_WORDLENGTH_8B;
    h->Init.StopBits               = UART_STOPBITS_1;
    h->Init.Parity                 = UART_PARITY_NONE;
    h->Init.Mode                   = UART_MODE_TX_RX;
    h->Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    h->Init.OverSampling           = UART_OVERSAMPLING_16;
    h->Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    h->Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    h->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(h) != HAL_OK) board_fault();
}

static void adc2_init(void)
{
    /* PA0, PA1, PC4 as analog inputs */
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1; HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_4;              HAL_GPIO_Init(GPIOC, &g);

    __HAL_RCC_ADC12_CLK_ENABLE();
    hadc2.Instance                   = ADC2;
    hadc2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;  /* 42.5 MHz */
    hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc2.Init.GainCompensation      = 0;
    hadc2.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc2.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc2.Init.LowPowerAutoWait      = DISABLE;
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.NbrOfConversion       = 1;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc2.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) board_fault();
    /* factory offset calibration, must run once before first conversion */
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) board_fault();
}

static void tim8_init(void)
{
    /* PC6..PC9 = TIM8 CH1..CH4, AF4 */
    __HAL_RCC_TIM8_CLK_ENABLE();
    af_pin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
           GPIO_AF4_TIM8, GPIO_MODE_AF_PP, GPIO_NOPULL);

    /* 170 MHz / 170 = 1 MHz tick, ARR 19999 -> 20 ms (50 Hz); CCR = µs */
    htim8.Instance               = TIM8;
    htim8.Init.Prescaler         = 169;
    htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim8.Init.Period            = 19999;
    htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) board_fault();

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = SERVO_STOWED_US;
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    /* channel IDs are 0x0, 0x4, 0x8, 0xC, so list them, don't ch++ */
    static const uint32_t ch[4] = { TIM_CHANNEL_1, TIM_CHANNEL_2,
                                    TIM_CHANNEL_3, TIM_CHANNEL_4 };
    for (int i = 0; i < 4; i++) {
        if (HAL_TIM_PWM_ConfigChannel(&htim8, &oc, ch[i]) != HAL_OK) board_fault();
    }
    for (int i = 0; i < 4; i++) {
        /* HAL also sets MOE here, which TIM8 (advanced timer) needs */
        if (HAL_TIM_PWM_Start(&htim8, ch[i]) != HAL_OK) board_fault();
    }
}

/* printf -> debug UART */
int _write(int fd, const char *ptr, int len)
{
    (void)fd;
    HAL_UART_Transmit(&huart2, (const uint8_t *)ptr, (uint16_t)len, 100);
    return len;
}

static void board_fault(void)
{
    __disable_irq();
    while (1) { }
}
