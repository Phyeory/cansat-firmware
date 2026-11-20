/* Test-facing control surface for the fake HAL (fake_hal.c).
 *
 * Everything the firmware sees goes through stm32g4xx_hal.h; everything a
 * test (or the flight simulator in sim/) needs to drive or inspect goes
 * through this header.  No firmware file includes it.
 */
#ifndef FAKE_HAL_H
#define FAKE_HAL_H

#include <stdint.h>
#include <stddef.h>
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- one I2C device */

typedef struct fake_i2c_dev fake_i2c_dev_t;

struct fake_i2c_dev {
    uint8_t  addr7;      /* 7-bit address, as in config.h */
    int      present;    /* 0 -> NACK everything */
    uint32_t writes;     /* raw write transactions seen, for assertions */
    uint32_t reads;      /* raw read transactions seen */
    HAL_StatusTypeDef (*on_write)(fake_i2c_dev_t *dev, const uint8_t *buf, uint16_t len);
    HAL_StatusTypeDef (*on_read)(fake_i2c_dev_t *dev, uint8_t *buf, uint16_t len);
    void    *ctx;        /* device model state */
};

/* Returns the slot for a 7-bit address (never NULL, 0x00..0x7F).
   Fill in present/on_write/on_read/ctx to model a chip. */
fake_i2c_dev_t *fake_i2c_device(uint8_t addr7);

/* ---------------------------------------------------------------- SPI bus */

/* One handler sees every SPI byte.  The sensor fake in fake_devices.c
   resolves the device from the (already driven) chip-select levels.
   rx == NULL means "transmit only". */
typedef HAL_StatusTypeDef (*fake_spi_fn_t)(SPI_HandleTypeDef *h,
                                           uint8_t *tx, uint8_t *rx, uint16_t n);
void fake_spi_set_handler(fake_spi_fn_t fn);

/* ---------------------------------------------------------------- clock */

/* Reset tick/GPIO/UART/ADC and forget every registered I2C device. */
void     fake_hal_reset(void);

uint32_t fake_tick(void);
void     fake_tick_set(uint32_t ms);
/* Advance the fake clock; runs the tick hook (see below) if installed. */
void     fake_tick_advance_ms(uint32_t ms);

/* Optional: HAL_GetTick() itself advances the clock by `ms` per call and then
   invokes the hook.  The flight simulator uses this to turn the firmware's
   own polling into simulation time; unit tests leave it at 0 (default). */
void     fake_hal_set_tick_step(uint32_t ms);
void     fake_hal_set_tick_hook(void (*fn)(void));

/* ---------------------------------------------------------------- GPIO */

/* Level seen by HAL_GPIO_ReadPin (external world -> MCU). */
void fake_gpio_set_input(GPIO_TypeDef *port, uint16_t pin, int level);
/* Level last written by HAL_GPIO_WritePin (MCU -> external world). */
int  fake_gpio_get_output(GPIO_TypeDef *port, uint16_t pin);
/* Observe every HAL_GPIO_WritePin (the SPI mux watches chip selects). */
void fake_gpio_set_write_hook(void (*fn)(GPIO_TypeDef *port, uint16_t pin,
                                         GPIO_PinState state));
/* Number of HAL_GPIO_WritePin calls that set this pin, for edge counting. */
uint32_t fake_gpio_write_count(GPIO_TypeDef *port, uint16_t pin);

/* Simulate one external interrupt on `pin`: drive the input low, run
   HAL_GPIO_EXTI_IRQHandler (which calls the app's callback), release it. */
void fake_gpio_exti_pulse(uint16_t pin);

/* ---------------------------------------------------------------- UART */

/* Feed bytes into USART1's RX register and call the installed RX handler once
   per byte.  The GPS suite and the simulator install the real
   USART1_IRQHandler here, so the parser under test is the firmware's own. */
void fake_uart_set_rx_handler(void (*fn)(void));
void fake_uart_feed(const uint8_t *buf, size_t len);
/* Observe every HAL_UART_Transmit, with the handle that sent it.  The
   simulator uses this to capture only the E22 telemetry UART. */
typedef void (*fake_uart_tx_fn_t)(UART_HandleTypeDef *h,
                                  const uint8_t *data, uint16_t len);
void fake_uart_set_tx_hook(fake_uart_tx_fn_t fn);
/* Bytes the firmware transmitted on any UART (debug + E22). */
void   fake_uart_tx_clear(void);
size_t fake_uart_tx_take(uint8_t *buf, size_t max);
size_t fake_uart_tx_len(void);

/* ---------------------------------------------------------------- ADC */

/* Scripted conversion result per channel (ADC_CHANNEL_x). */
void     fake_adc_set(uint32_t channel, uint16_t raw);
uint32_t fake_adc_last_channel(void);
uint32_t fake_adc_conversion_count(void);

#ifdef __cplusplus
}
#endif
#endif /* FAKE_HAL_H */
