/* Host-side stand-in for the STM32G4 HAL.
 *
 * The firmware in src/ is written against exactly this surface: it never
 * includes a HAL module header directly, only "stm32g4xx_hal.h".  Compiling
 * the real sources with -Itest/fakes therefore swaps the whole vendor stack
 * for this file plus fake_hal.c, with no change to src/.
 *
 * Rules for this header:
 *   - only what src/ actually uses (plus a little slack so board.c compiles),
 *   - same names, same signatures, same field names as the vendor headers,
 *   - no inline logic beyond trivial register access; behaviour lives in
 *     fake_hal.c so tests can control it.
 *
 * Keep it in sync with the real HAL if src/ starts using a new call: the
 * target build (pio run -e stm32g474) is unaffected either way.
 */
#ifndef FAKE_STM32G4XX_HAL_H
#define FAKE_STM32G4XX_HAL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define __IO volatile

/* ------------------------------------------------------------------ status */

typedef enum {
    HAL_OK      = 0x00U,
    HAL_ERROR   = 0x01U,
    HAL_BUSY    = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET   = 1
} GPIO_PinState;

typedef enum { DISABLE = 0, ENABLE = 1 } FunctionalState;

/* --------------------------------------------------------------- registers */

typedef struct {
    __IO uint32_t MODER;
    __IO uint32_t OTYPER;
    __IO uint32_t OSPEEDR;
    __IO uint32_t PUPDR;
    __IO uint32_t IDR;
    __IO uint32_t ODR;
    __IO uint32_t BSRR;
    __IO uint32_t LCKR;
    __IO uint32_t AFR[2];
    __IO uint32_t BRR;
} GPIO_TypeDef;

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t CR3;
    __IO uint32_t BRR;
    __IO uint32_t GTPR;
    __IO uint32_t RTOR;
    __IO uint32_t RQR;
    __IO uint32_t ISR;
    __IO uint32_t ICR;
    __IO uint32_t RDR;
    __IO uint32_t TDR;
} USART_TypeDef;

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMCR;
    __IO uint32_t DIER;
    __IO uint32_t SR;
    __IO uint32_t EGR;
    __IO uint32_t CCMR1;
    __IO uint32_t CCMR2;
    __IO uint32_t CCER;
    __IO uint32_t CNT;
    __IO uint32_t PSC;
    __IO uint32_t ARR;
    __IO uint32_t RCR;
    __IO uint32_t CCR1;
    __IO uint32_t CCR2;
    __IO uint32_t CCR3;
    __IO uint32_t CCR4;
    __IO uint32_t BDTR;
    __IO uint32_t DCR;
    __IO uint32_t DMAR;
    __IO uint32_t OR;
} TIM_TypeDef;

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t OAR1;
    __IO uint32_t OAR2;
    __IO uint32_t TIMINGR;
    __IO uint32_t TIMEOUTR;
    __IO uint32_t ISR;
    __IO uint32_t ICR;
    __IO uint32_t PECR;
    __IO uint32_t RXDR;
    __IO uint32_t TXDR;
} I2C_TypeDef;

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SR;
    __IO uint32_t DR;
    __IO uint32_t CRCPR;
    __IO uint32_t RXCRCR;
    __IO uint32_t TXCRCR;
    __IO uint32_t I2SCFGR;
    __IO uint32_t I2SPR;
} SPI_TypeDef;

typedef struct {
    __IO uint32_t ISR;
    __IO uint32_t IER;
    __IO uint32_t CR;
    __IO uint32_t CFGR;
    __IO uint32_t CFGR2;
    __IO uint32_t SMPR1;
    __IO uint32_t SMPR2;
    __IO uint32_t TR1;
    __IO uint32_t TR2;
    __IO uint32_t TR3;
    __IO uint32_t SQR1;
    __IO uint32_t SQR2;
    __IO uint32_t SQR3;
    __IO uint32_t SQR4;
    __IO uint32_t DR;
} ADC_TypeDef;

/* The vendor headers expose the peripherals as (TYPE *)BASE_ADDR macros.
   Here they are pointers to real objects so a test can inspect them. */
extern GPIO_TypeDef  fake_gpio_a, fake_gpio_b, fake_gpio_c, fake_gpio_d,
                     fake_gpio_e, fake_gpio_f, fake_gpio_g, fake_gpio_h,
                     fake_gpio_i;
extern USART_TypeDef fake_usart1, fake_usart2, fake_usart3;
extern TIM_TypeDef   fake_tim8;
extern I2C_TypeDef   fake_i2c1;
extern SPI_TypeDef   fake_spi1, fake_spi2;
extern ADC_TypeDef   fake_adc2;

#define GPIOA (&fake_gpio_a)
#define GPIOB (&fake_gpio_b)
#define GPIOC (&fake_gpio_c)
#define GPIOD (&fake_gpio_d)
#define GPIOE (&fake_gpio_e)
#define GPIOF (&fake_gpio_f)
#define GPIOG (&fake_gpio_g)
#define GPIOH (&fake_gpio_h)
#define GPIOI (&fake_gpio_i)
#define USART1 (&fake_usart1)
#define USART2 (&fake_usart2)
#define USART3 (&fake_usart3)
#define TIM8   (&fake_tim8)
#define I2C1   (&fake_i2c1)
#define SPI1   (&fake_spi1)
#define SPI2   (&fake_spi2)
#define ADC2   (&fake_adc2)

/* ----------------------------------------------------------- init structs */

typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
    uint32_t Alternate;
} GPIO_InitTypeDef;

typedef struct {
    uint32_t Timing;
    uint32_t OwnAddress1;
    uint32_t AddressingMode;
    uint32_t DualAddressMode;
    uint32_t OwnAddress2;
    uint32_t OwnAddress2Masks;
    uint32_t GeneralCallMode;
    uint32_t NoStretchMode;
} I2C_InitTypeDef;

typedef struct {
    uint32_t Mode;
    uint32_t Direction;
    uint32_t DataSize;
    uint32_t CLKPolarity;
    uint32_t CLKPhase;
    uint32_t NSS;
    uint32_t BaudRatePrescaler;
    uint32_t FirstBit;
    uint32_t TIMode;
    uint32_t CRCCalculation;
    uint32_t CRCPolynomial;
    uint32_t CRCLength;
    uint32_t NSSPMode;
} SPI_InitTypeDef;

typedef struct {
    uint32_t BaudRate;
    uint32_t WordLength;
    uint32_t StopBits;
    uint32_t Parity;
    uint32_t Mode;
    uint32_t HwFlowCtl;
    uint32_t OverSampling;
    uint32_t OneBitSampling;
    uint32_t ClockPrescaler;
} UART_InitTypeDef;

typedef struct {
    uint32_t AdvFeatureInit;
} UART_AdvFeatureInitTypeDef;

typedef struct {
    uint32_t ClockPrescaler;
    uint32_t Resolution;
    uint32_t DataAlign;
    uint32_t GainCompensation;
    uint32_t ScanConvMode;
    uint32_t EOCSelection;
    uint32_t LowPowerAutoWait;
    uint32_t ContinuousConvMode;
    uint32_t NbrOfConversion;
    uint32_t DiscontinuousConvMode;
    uint32_t ExternalTrigConv;
    uint32_t ExternalTrigConvEdge;
    uint32_t DMAContinuousRequests;
    uint32_t Overrun;
    uint32_t OversamplingMode;
} ADC_InitTypeDef;

typedef struct {
    uint32_t Prescaler;
    uint32_t CounterMode;
    uint32_t Period;
    uint32_t ClockDivision;
    uint32_t RepetitionCounter;
    uint32_t AutoReloadPreload;
} TIM_Base_InitTypeDef;

typedef struct {
    uint32_t OCMode;
    uint32_t Pulse;
    uint32_t OCPolarity;
    uint32_t OCNPolarity;
    uint32_t OCFastMode;
    uint32_t OCIdleState;
    uint32_t OCNIdleState;
} TIM_OC_InitTypeDef;

typedef struct {
    uint32_t Channel;
    uint32_t Rank;
    uint32_t SamplingTime;
    uint32_t SingleDiff;
    uint32_t OffsetNumber;
    uint32_t Offset;
} ADC_ChannelConfTypeDef;

typedef struct {
    uint32_t PLLState;
    uint32_t PLLSource;
    uint32_t PLLM;
    uint32_t PLLN;
    uint32_t PLLP;
    uint32_t PLLQ;
    uint32_t PLLR;
} RCC_PLLInitTypeDef;

typedef struct {
    uint32_t OscillatorType;
    uint32_t HSIState;
    uint32_t HSICalibrationValue;
    uint32_t LSIState;
    RCC_PLLInitTypeDef PLL;
} RCC_OscInitTypeDef;

typedef struct {
    uint32_t ClockType;
    uint32_t SYSCLKSource;
    uint32_t AHBCLKDivider;
    uint32_t APB1CLKDivider;
    uint32_t APB2CLKDivider;
} RCC_ClkInitTypeDef;

typedef struct {
    uint32_t PeriphClockSelection;
    uint32_t I2c1ClockSelection;
} RCC_PeriphCLKInitTypeDef;

/* --------------------------------------------------------------- handles */

typedef struct {
    I2C_TypeDef *Instance;
    I2C_InitTypeDef Init;
} I2C_HandleTypeDef;

typedef struct {
    SPI_TypeDef *Instance;
    SPI_InitTypeDef Init;
} SPI_HandleTypeDef;

typedef struct {
    USART_TypeDef *Instance;
    UART_InitTypeDef Init;
    UART_AdvFeatureInitTypeDef AdvancedInit;
} UART_HandleTypeDef;

typedef struct {
    ADC_TypeDef *Instance;
    ADC_InitTypeDef Init;
} ADC_HandleTypeDef;

typedef struct {
    TIM_TypeDef *Instance;
    TIM_Base_InitTypeDef Init;
} TIM_HandleTypeDef;

/* ------------------------------------------------------------- constants */

/* GPIO */
#define GPIO_PIN_0   ((uint16_t)0x0001)
#define GPIO_PIN_1   ((uint16_t)0x0002)
#define GPIO_PIN_2   ((uint16_t)0x0004)
#define GPIO_PIN_3   ((uint16_t)0x0008)
#define GPIO_PIN_4   ((uint16_t)0x0010)
#define GPIO_PIN_5   ((uint16_t)0x0020)
#define GPIO_PIN_6   ((uint16_t)0x0040)
#define GPIO_PIN_7   ((uint16_t)0x0080)
#define GPIO_PIN_8   ((uint16_t)0x0100)
#define GPIO_PIN_9   ((uint16_t)0x0200)
#define GPIO_PIN_10  ((uint16_t)0x0400)
#define GPIO_PIN_11  ((uint16_t)0x0800)
#define GPIO_PIN_12  ((uint16_t)0x1000)
#define GPIO_PIN_13  ((uint16_t)0x2000)
#define GPIO_PIN_14  ((uint16_t)0x4000)
#define GPIO_PIN_15  ((uint16_t)0x8000)

#define GPIO_MODE_INPUT      0x00000000U
#define GPIO_MODE_OUTPUT_PP  0x00000001U
#define GPIO_MODE_OUTPUT_OD  0x00000011U
#define GPIO_MODE_AF_PP      0x00000002U
#define GPIO_MODE_AF_OD      0x00000012U
#define GPIO_MODE_ANALOG     0x00000003U
#define GPIO_MODE_IT_RISING  0x10110000U
#define GPIO_MODE_IT_FALLING 0x10210000U

#define GPIO_NOPULL   0x00000000U
#define GPIO_PULLUP   0x00000001U
#define GPIO_PULLDOWN 0x00000002U

#define GPIO_SPEED_FREQ_LOW       0x00000000U
#define GPIO_SPEED_FREQ_MEDIUM    0x00000001U
#define GPIO_SPEED_FREQ_HIGH      0x00000002U
#define GPIO_SPEED_FREQ_VERY_HIGH 0x00000003U

#define GPIO_AF4_I2C1   0x04U
#define GPIO_AF4_TIM8   0x04U
#define GPIO_AF5_SPI1   0x05U
#define GPIO_AF5_SPI2   0x05U
#define GPIO_AF7_USART1 0x07U
#define GPIO_AF7_USART2 0x07U
#define GPIO_AF7_USART3 0x07U

/* I2C */
#define I2C_MEMADD_SIZE_8BIT  0x00000001U
#define I2C_MEMADD_SIZE_16BIT 0x00000010U
#define I2C_ADDRESSINGMODE_7BIT 0x00000001U
#define I2C_DUALADDRESS_DISABLE 0x00000000U
#define I2C_OA2_NOMASK          0x00000000U
#define I2C_GENERALCALL_DISABLE 0x00000000U
#define I2C_NOSTRETCH_DISABLE   0x00000000U
#define I2C_ANALOGFILTER_ENABLE 0x00000000U
#define I2C_ANALOGFILTER_DISABLE 0x00000001U

/* SPI */
#define SPI_MODE_MASTER             0x00000004U
#define SPI_DIRECTION_2LINES        0x00000000U
#define SPI_DATASIZE_8BIT           0x00000007U
#define SPI_POLARITY_LOW            0x00000000U
#define SPI_POLARITY_HIGH           0x00000002U
#define SPI_PHASE_1EDGE             0x00000000U
#define SPI_PHASE_2EDGE             0x00000004U
#define SPI_NSS_SOFT                0x00000200U
#define SPI_BAUDRATEPRESCALER_2     0x00000000U
#define SPI_BAUDRATEPRESCALER_16    0x00000018U
#define SPI_BAUDRATEPRESCALER_32    0x00000020U
#define SPI_BAUDRATEPRESCALER_256   0x000001C0U
#define SPI_FIRSTBIT_MSB            0x00000000U
#define SPI_TIMODE_DISABLE          0x00000000U
#define SPI_CRCCALCULATION_DISABLE  0x00000000U
#define SPI_CRC_LENGTH_DATASIZE     0x00000000U
#define SPI_NSS_PULSE_DISABLE       0x00000000U

/* UART */
#define UART_WORDLENGTH_8B          0x00000000U
#define UART_STOPBITS_1             0x00000000U
#define UART_PARITY_NONE            0x00000000U
#define UART_MODE_TX_RX             0x0000000CU
#define UART_HWCONTROL_NONE         0x00000000U
#define UART_OVERSAMPLING_16        0x00000000U
#define UART_ONE_BIT_SAMPLE_DISABLE 0x00000000U
#define UART_PRESCALER_DIV1         0x00000000U
#define UART_ADVFEATURE_NO_INIT     0x00000000U

#define UART_CLEAR_OREF 0x00000008U
#define UART_CLEAR_FEF  0x00000004U
#define UART_CLEAR_NEF  0x00000002U

#define USART_CR1_RXNEIE_RXFNEIE (1UL << 5)
#define USART_ISR_ORE            (1UL << 3)
#define USART_ISR_FE             (1UL << 1)
#define USART_ISR_NE             (1UL << 2)
#define USART_ISR_RXNE_RXFNE     (1UL << 5)
#define USART_ICR_ORECF          (1UL << 3)
#define USART_ICR_FECF           (1UL << 1)
#define USART_ICR_NECF           (1UL << 2)

/* ADC */
#define ADC_CHANNEL_1            0x00000001U
#define ADC_CHANNEL_2            0x00000002U
#define ADC_CHANNEL_5            0x00000005U
#define ADC_REGULAR_RANK_1       0x00000001U
#define ADC_SAMPLETIME_247CYCLES_5 0x00000006U
#define ADC_SINGLE_ENDED         0x00000000U
#define ADC_OFFSET_NONE          0x00000000U
#define ADC_CLOCK_SYNC_PCLK_DIV4 0x00000003U
#define ADC_RESOLUTION_12B       0x00000000U
#define ADC_DATAALIGN_RIGHT      0x00000000U
#define ADC_SCAN_DISABLE         0x00000000U
#define ADC_EOC_SINGLE_CONV      0x00000000U
#define ADC_SOFTWARE_START       0x00000000U
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0x00000000U
#define ADC_OVR_DATA_OVERWRITTEN 0x00000000U

/* TIM */
#define TIM_CHANNEL_1            0x00000000U
#define TIM_CHANNEL_2            0x00000004U
#define TIM_CHANNEL_3            0x00000008U
#define TIM_CHANNEL_4            0x0000000CU
#define TIM_COUNTERMODE_UP       0x00000000U
#define TIM_CLOCKDIVISION_DIV1   0x00000000U
#define TIM_AUTORELOAD_PRELOAD_ENABLE 0x00000001U
#define TIM_OCMODE_PWM1          0x00000060U
#define TIM_OCPOLARITY_HIGH      0x00000000U
#define TIM_OCNPOLARITY_HIGH     0x00000000U
#define TIM_OCFAST_DISABLE       0x00000000U
#define TIM_OCIDLESTATE_RESET    0x00000000U
#define TIM_OCNIDLESTATE_RESET   0x00000000U

/* RCC / PWR / FLASH */
#define RCC_OSCILLATORTYPE_HSI   0x00000001U
#define RCC_HSI_ON               0x00000001U
#define RCC_HSICALIBRATION_DEFAULT 0x00000010U
#define RCC_PLL_ON               0x00000002U
#define RCC_PLLSOURCE_HSI        0x00000000U
#define RCC_PLLM_DIV4            0x00000003U
#define RCC_PLLP_DIV2            0x00000002U
#define RCC_PLLQ_DIV2            0x00000002U
#define RCC_PLLR_DIV2            0x00000002U

#define RCC_CLOCKTYPE_SYSCLK     0x00000001U
#define RCC_CLOCKTYPE_HCLK       0x00000002U
#define RCC_CLOCKTYPE_PCLK1      0x00000004U
#define RCC_CLOCKTYPE_PCLK2      0x00000008U
#define RCC_SYSCLKSOURCE_PLLCLK  0x00000003U
#define RCC_SYSCLK_DIV1          0x00000000U
#define RCC_HCLK_DIV1            0x00000000U
#define RCC_HCLK_DIV2            0x00000008U
#define FLASH_LATENCY_4          0x00000004U

#define RCC_PERIPHCLK_I2C1       0x00000020U
#define RCC_I2C1CLKSOURCE_HSI    0x00000002U

#define PWR_REGULATOR_VOLTAGE_SCALE1_BOOST 0x00000002U

/* IRQs */
#define USART1_IRQn    37
#define EXTI15_10_IRQn 40

/* ------------------------------------------------------------ functions */

HAL_StatusTypeDef HAL_Init(void);
uint32_t          HAL_GetTick(void);
void              HAL_Delay(uint32_t ms);
void              HAL_IncTick(void);

void          HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void          HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void          HAL_GPIO_EXTI_IRQHandler(uint16_t pin);
void          HAL_GPIO_EXTI_Callback(uint16_t pin);   /* weak, provided by the app */

void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub);
void HAL_NVIC_EnableIRQ(int irq);

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t filter);
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t filter);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t dev,
                                          uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t dev,
                                         uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t dev,
                                   uint16_t mem, uint16_t mem_size,
                                   uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t dev,
                                    uint16_t mem, uint16_t mem_size,
                                    uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t dev,
                                        uint32_t trials, uint32_t timeout);

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *data,
                                   uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *tx,
                                          uint8_t *rx, uint16_t size, uint32_t timeout);

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *data,
                                    uint16_t size, uint32_t timeout);

HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t single);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *cfg);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t timeout);
uint32_t          HAL_ADC_GetValue(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc);

HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *oc,
                                            uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *osc);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *clk, uint32_t latency);
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *p);
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t scale);

/* --------------------------------------------------------------- macros */

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define MODIFY_REG(REG, CLEARMASK, SETMASK) \
    ((REG) = (((REG) & ~(CLEARMASK)) | (SETMASK)))

#define __disable_irq() do { } while (0)
#define __enable_irq()  do { } while (0)

#define __HAL_RCC_GPIOA_CLK_ENABLE() do { } while (0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() do { } while (0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() do { } while (0)
#define __HAL_RCC_I2C1_CLK_ENABLE()  do { } while (0)
#define __HAL_RCC_SPI1_CLK_ENABLE()  do { } while (0)
#define __HAL_RCC_SPI2_CLK_ENABLE()  do { } while (0)
#define __HAL_RCC_USART1_CLK_ENABLE() do { } while (0)
#define __HAL_RCC_USART2_CLK_ENABLE() do { } while (0)
#define __HAL_RCC_USART3_CLK_ENABLE() do { } while (0)
#define __HAL_RCC_TIM8_CLK_ENABLE()  do { } while (0)
#define __HAL_RCC_ADC12_CLK_ENABLE() do { } while (0)
#define __HAL_RCC_PWR_CLK_ENABLE()   do { } while (0)

#define __HAL_UART_CLEAR_FLAG(HANDLE, FLAGS) do { (HANDLE)->Instance->ICR = (FLAGS); } while (0)

/* The real macro is a switch on the channel number. */
static inline void fake_hal_tim_set_compare(TIM_HandleTypeDef *h, uint32_t ch, uint32_t v)
{
    switch (ch) {
    case TIM_CHANNEL_1: h->Instance->CCR1 = v; break;
    case TIM_CHANNEL_2: h->Instance->CCR2 = v; break;
    case TIM_CHANNEL_3: h->Instance->CCR3 = v; break;
    case TIM_CHANNEL_4: h->Instance->CCR4 = v; break;
    default: break;
    }
}
#define __HAL_TIM_SET_COMPARE(HANDLE, CHANNEL, COMPARE) \
    fake_hal_tim_set_compare((HANDLE), (CHANNEL), (COMPARE))

#ifdef __cplusplus
}
#endif
#endif /* FAKE_STM32G4XX_HAL_H */
