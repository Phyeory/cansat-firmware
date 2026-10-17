/**
 * stm32g4xx_hal_conf.h — module selection for the Celestium CanSat build.
 * Only the HAL modules we use are enabled (smaller, faster builds).
 */
#ifndef STM32G4xx_HAL_CONF_H
#define STM32G4xx_HAL_CONF_H

#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
/* #define HAL_FATFS_MODULE_ENABLED */   /* via CubeMX FatFS middleware */

#ifndef HSE_VALUE
#define HSE_VALUE 8000000U
#endif
#define HSE_STARTUP_TIMEOUT 100U
#define HSI_VALUE 16000000U
#define LSI_VALUE 32000U
#define LSE_VALUE 32768U
#define LSE_STARTUP_TIMEOUT 5000U
#define EXTERNAL_SAI1_CLOCK_VALUE 2097000U
#define EXTERNAL_SAI2_CLOCK_VALUE 2097000U

#define VDD_VALUE 3300U
#define TICK_INT_PRIORITY 15U
#define USE_RTOS 0U
#define PREFETCH_ENABLE 0U
#define INSTRUCTION_CACHE_ENABLE 1U
#define DATA_CACHE_ENABLE 1U

#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_cortex.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_i2c.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_uart.h"

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#endif /* STM32G4xx_HAL_CONF_H */
