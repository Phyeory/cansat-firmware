/**
 * stm32g4xx_hal_conf.h: HAL module selection for the Celestium CanSat.
 * Only the modules the firmware uses are enabled. DMA, EXTI, FLASH and PWR
 * are required by the other HAL headers even though we don't use them
 * directly (leaving DMA out is what stopped the original from compiling).
 */
#ifndef STM32G4xx_HAL_CONF_H
#define STM32G4xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- modules ---- */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* ---- oscillators ---- */
#if !defined(HSE_VALUE)
#define HSE_VALUE            (8000000UL)
#endif
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT  (100UL)
#endif
#if !defined(HSI_VALUE)
#define HSI_VALUE            (16000000UL)
#endif
#if !defined(HSI48_VALUE)
#define HSI48_VALUE          (48000000UL)
#endif
#if !defined(LSI_VALUE)
#define LSI_VALUE            (32000UL)
#endif
#if !defined(LSE_VALUE)
#define LSE_VALUE            (32768UL)
#endif
#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT  (5000UL)
#endif
#if !defined(EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE (12288000UL)
#endif

/* ---- system ---- */
#define VDD_VALUE                 (3300UL)
#define TICK_INT_PRIORITY         (0x0FUL)
#define USE_RTOS                  0U
#define PREFETCH_ENABLE           0U
#define INSTRUCTION_CACHE_ENABLE  1U
#define DATA_CACHE_ENABLE         1U

/* no HAL register-callback mode */
#define USE_HAL_ADC_REGISTER_CALLBACKS   0U
#define USE_HAL_I2C_REGISTER_CALLBACKS   0U
#define USE_HAL_SPI_REGISTER_CALLBACKS   0U
#define USE_HAL_TIM_REGISTER_CALLBACKS   0U
#define USE_HAL_UART_REGISTER_CALLBACKS  0U

#define USE_SPI_CRC  0U

/* ---- includes ---- */
#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_dma.h"
#include "stm32g4xx_hal_cortex.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_exti.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_i2c.h"
#include "stm32g4xx_hal_pwr.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_uart.h"

#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32G4xx_HAL_CONF_H */
