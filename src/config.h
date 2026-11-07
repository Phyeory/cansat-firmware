#pragma once
#include "stm32g4xx_hal.h"

#define HCLK_HZ 170000000U

#define I2C_TIMING_100K     0x00303D5BU
#define BMP280_I2C_ADDR     (0x76U << 1)
#define LTR390_I2C_ADDR     (0x53U << 1)
#define TSL2591_I2C_ADDR    (0x29U << 1)
#define SCD30_I2C_ADDR      (0x61U << 1)
#define BMI270_I2C_ADDR     (0x68U << 1)   // SDO -> GND (0x69 if -> 3V3)
#define MLX90395_I2C_ADDR   (0x0CU << 1)   // A1 -> GND, A0 -> GND
#define SHT45_I2C_ADDR      (0x44U << 1)

#define LORA_SPI            hspi1
#define LORA_CS_GPIO        GPIOB
#define LORA_CS_PIN         GPIO_PIN_0     // PB0  (NSS, active low)
#define LORA_RST_GPIO       GPIOB
#define LORA_RST_PIN        GPIO_PIN_1     // PB1
#define LORA_DIO0_GPIO      GPIOA
#define LORA_DIO0_PIN       GPIO_PIN_8     // PA8  (goes high on TxDone)

#define LORA_FREQ_HZ        434000000U
#define LORA_SF             7
#define LORA_BW_CODE        0x08
#define LORA_CR_CODE        0x01
#define LORA_PREAMBLE       8
#define LORA_SYNC_WORD      0x12
#define LORA_TX_POWER_DB    10

#define SD_SPI              hspi2
#define SD_CS_GPIO          GPIOB
#define SD_CS_PIN           GPIO_PIN_12

#define GPS_UART            huart1
#define GPS_BAUD            9600U

#define DEBUG_UART          huart2

#define O3_ADC              hadc2
#define O3_WORK_CHANNEL     ADC_CHANNEL_1
#define O3_AUX_CHANNEL      ADC_CHANNEL_2
#define VBAT_CHANNEL        ADC_CHANNEL_5
#define VBAT_DIVIDER        2.0f
#define VREF_V              3.3f
#define ADC_MAX             4095.0f

#define SERVO_TIM           htim8
#define SERVO_TIM_CHANNEL_1 TIM_CHANNEL_1
#define SERVO_TIM_CHANNEL_2 TIM_CHANNEL_2
#define SERVO_TIM_CHANNEL_3 TIM_CHANNEL_3
#define SERVO_TIM_CHANNEL_4 TIM_CHANNEL_4
#define SERVO_STOWED_US     1000U
#define SERVO_DEPLOYED_US   2000U

#define RAD_INPUT_GPIO      GPIOB
#define RAD_INPUT_PIN       GPIO_PIN_10

#define SAMPLE_PERIOD_MS    100U
#define TELEMETRY_PERIOD_MS 1000U

#define SEA_LEVEL_PRESSURE_HPA  1013.25f

#define O3_MV_PER_PPB   1.0f
#define O3_ZERO_MV      0.0f
