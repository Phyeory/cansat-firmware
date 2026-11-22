#pragma once
#include "stm32g4xx_hal.h"

#define HCLK_HZ 170000000U

#define I2C_TIMING_100K     0x00303D5BU
#define LTR390_I2C_ADDR     (0x53U << 1)
#define TSL2591_I2C_ADDR    (0x29U << 1)
#define SCD30_I2C_ADDR      (0x61U << 1)
#define SHT45_I2C_ADDR      (0x44U << 1)

/* Shared sensor SPI bus.  LSM6DSV, MLX90395 and BMP581 each have their own
   chip select and all use SPI mode 3 (see board.c). */
#define SENSOR_SPI          hspi1
#define LSM6DSV_CS_GPIO     GPIOB
#define LSM6DSV_CS_PIN      GPIO_PIN_0     // PB0
#define MLX90395_CS_GPIO    GPIOB
#define MLX90395_CS_PIN     GPIO_PIN_1     // PB1
#define BMP581_CS_GPIO      GPIOA
#define BMP581_CS_PIN       GPIO_PIN_8     // PA8

/* Telemetry: EBYTE E22-900T22D on USART3, transparent UART.  The module
   itself is configured for 868.125 MHz / 14 dBm / 62.5 kbps (see README). */
#define E22_UART            huart3
#define E22_BAUD            9600U
#define E22_FREQ_HZ         868125000U
#define E22_TX_POWER_DBM    14
#define E22_AIR_RATE_BPS    62500U

#define SD_SPI              hspi2
#define SD_CS_GPIO          GPIOB
#define SD_CS_PIN           GPIO_PIN_12

/* SAM-M8Q: UART (USART1) is the wired interface.  The module has no SPI
   pins (u-blox UBX-16018358: UART + DDC/I2C only), so the requested SPI2
   link needs a different GNSS part or a bridge. */
#define GPS_UART            huart1
#define GPS_BAUD            9600U

#define DEBUG_UART          huart2

#define O3_ADC              hadc2
#define O3_WORK_CHANNEL     ADC_CHANNEL_1
#define O3_AUX_CHANNEL      ADC_CHANNEL_2
#define VBAT_CHANNEL        ADC_CHANNEL_5
#define VBAT_DIVIDER        2.0f
#define VREF_V              3.0f          // 3.0 V LDO rail (Main.md)
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
