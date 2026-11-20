/* Host implementation of the ~25 HAL entry points used by src/.
 *
 * Behaviour is deliberately boring: the clock is a counter, GPIO is a pair of
 * bitfields, UART RX pushes bytes through the real ISR, and every I2C
 * transaction is forwarded to a scriptable device model (fake_devices.c).
 * Nothing here fakes sensor physics - that belongs to the models.
 */
#include "fake_hal.h"

#include <string.h>

/* ------------------------------------------------------------- instances */

GPIO_TypeDef  fake_gpio_a, fake_gpio_b, fake_gpio_c, fake_gpio_d, fake_gpio_e,
              fake_gpio_f, fake_gpio_g, fake_gpio_h, fake_gpio_i;
USART_TypeDef fake_usart1, fake_usart2, fake_usart3;
TIM_TypeDef   fake_tim8;
I2C_TypeDef   fake_i2c1;
SPI_TypeDef   fake_spi1, fake_spi2;
ADC_TypeDef   fake_adc2;

/* ------------------------------------------------------------- internals */

#define FAKE_I2C_MAX 128

static fake_i2c_dev_t  i2c_devs[FAKE_I2C_MAX];
static uint32_t        tick_now;
static uint32_t        tick_step;
static void          (*tick_hook)(void);
static int             in_hook;

static uint32_t        gpio_out[9];      /* indexed by port letter A..I */
static uint32_t        gpio_in[9];
static uint32_t        gpio_writes[9][16];
static void          (*gpio_write_hook)(GPIO_TypeDef *, uint16_t, GPIO_PinState);

static uint8_t         uart_tx[16384];
static size_t          uart_tx_n;

static uint16_t        adc_value[32];
static uint32_t        adc_last_ch;
static uint32_t        adc_conversions;

static fake_spi_fn_t   spi_handler;
static void          (*uart_rx_handler)(void);
static fake_uart_tx_fn_t uart_tx_hook;

/* ------------------------------------------------------------ fake API */

static int port_index(GPIO_TypeDef *port)
{
    if (port == GPIOA) return 0;
    if (port == GPIOB) return 1;
    if (port == GPIOC) return 2;
    if (port == GPIOD) return 3;
    if (port == GPIOE) return 4;
    if (port == GPIOF) return 5;
    if (port == GPIOG) return 6;
    if (port == GPIOH) return 7;
    if (port == GPIOI) return 8;
    return -1;
}

static int pin_index(uint16_t pin)
{
    for (int i = 0; i < 16; i++)
        if (pin == (uint16_t)(1u << i)) return i;
    return -1;
}

fake_i2c_dev_t *fake_i2c_device(uint8_t addr7)
{
    fake_i2c_dev_t *d = &i2c_devs[addr7 & 0x7F];
    d->addr7 = (uint8_t)(addr7 & 0x7F);
    return d;
}

void fake_hal_reset(void)
{
    memset(i2c_devs, 0, sizeof(i2c_devs));
    for (int i = 0; i < FAKE_I2C_MAX; i++) i2c_devs[i].addr7 = (uint8_t)i;

    tick_now = tick_step = 0;
    tick_hook = NULL;
    in_hook = 0;

    memset(gpio_out, 0, sizeof(gpio_out));
    memset(gpio_in, 0, sizeof(gpio_in));
    memset(gpio_writes, 0, sizeof(gpio_writes));

    uart_tx_n = 0;
    memset(adc_value, 0, sizeof(adc_value));
    adc_last_ch = 0;
    adc_conversions = 0;
    spi_handler = NULL;
    gpio_write_hook = NULL;
    uart_rx_handler = NULL;
    uart_tx_hook = NULL;

    memset(&fake_gpio_a, 0, sizeof(fake_gpio_a));
    memset(&fake_gpio_b, 0, sizeof(fake_gpio_b));
    memset(&fake_gpio_c, 0, sizeof(fake_gpio_c));
    memset(&fake_gpio_d, 0, sizeof(fake_gpio_d));
    memset(&fake_gpio_e, 0, sizeof(fake_gpio_e));
    memset(&fake_gpio_f, 0, sizeof(fake_gpio_f));
    memset(&fake_gpio_g, 0, sizeof(fake_gpio_g));
    memset(&fake_gpio_h, 0, sizeof(fake_gpio_h));
    memset(&fake_gpio_i, 0, sizeof(fake_gpio_i));
    memset(&fake_usart1, 0, sizeof(fake_usart1));
    memset(&fake_usart2, 0, sizeof(fake_usart2));
    memset(&fake_usart3, 0, sizeof(fake_usart3));
    memset(&fake_tim8, 0, sizeof(fake_tim8));
    memset(&fake_i2c1, 0, sizeof(fake_i2c1));
    memset(&fake_spi1, 0, sizeof(fake_spi1));
    memset(&fake_spi2, 0, sizeof(fake_spi2));
    memset(&fake_adc2, 0, sizeof(fake_adc2));
}

uint32_t fake_tick(void) { return tick_now; }

void fake_tick_set(uint32_t ms) { tick_now = ms; }

void fake_hal_set_tick_step(uint32_t ms) { tick_step = ms; }

void fake_hal_set_tick_hook(void (*fn)(void)) { tick_hook = fn; }

static void run_tick_hook(void)
{
    if (!tick_hook || in_hook) return;
    in_hook = 1;
    tick_hook();
    in_hook = 0;
}

void fake_tick_advance_ms(uint32_t ms)
{
    tick_now += ms;
    run_tick_hook();
}

/* ----------------------------------------------------------------- clock */

HAL_StatusTypeDef HAL_Init(void) { return HAL_OK; }

uint32_t HAL_GetTick(void)
{
    if (tick_step) {
        tick_now += tick_step;
        run_tick_hook();
    }
    return tick_now;
}

void HAL_IncTick(void) { tick_now++; }

void HAL_Delay(uint32_t ms) { fake_tick_advance_ms(ms); }

void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub)
{
    (void)irq; (void)preempt; (void)sub;
}

void HAL_NVIC_EnableIRQ(int irq) { (void)irq; }

/* ------------------------------------------------------------------ GPIO */

void fake_gpio_set_input(GPIO_TypeDef *port, uint16_t pin, int level)
{
    int p = port_index(port), b = pin_index(pin);
    if (p < 0 || b < 0) return;
    if (level) gpio_in[p] |= (1u << b);
    else       gpio_in[p] &= ~(1u << b);
    port->IDR = gpio_in[p];
}

int fake_gpio_get_output(GPIO_TypeDef *port, uint16_t pin)
{
    int p = port_index(port), b = pin_index(pin);
    if (p < 0 || b < 0) return 0;
    return (gpio_out[p] >> b) & 1u;
}

uint32_t fake_gpio_write_count(GPIO_TypeDef *port, uint16_t pin)
{
    int p = port_index(port), b = pin_index(pin);
    if (p < 0 || b < 0) return 0;
    return gpio_writes[p][b];
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port; (void)init;
}

void fake_gpio_set_write_hook(void (*fn)(GPIO_TypeDef *, uint16_t, GPIO_PinState))
{
    gpio_write_hook = fn;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    int p = port_index(port), b = pin_index(pin);
    if (p < 0 || b < 0) return;
    if (state == GPIO_PIN_SET) gpio_out[p] |= (1u << b);
    else                       gpio_out[p] &= ~(1u << b);
    port->ODR = gpio_out[p];
    gpio_writes[p][b]++;
    if (gpio_write_hook) gpio_write_hook(port, pin, state);
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    int p = port_index(port), b = pin_index(pin);
    if (p < 0 || b < 0) return GPIO_PIN_RESET;
    return (gpio_in[p] >> b) & 1u ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/* Both are weak so the application (rad_counter.c) and fake_devices.c can
   provide the real thing, exactly like the vendor HAL does.  A suite that
   includes rad_counter.c in the same translation unit defines
   FAKE_HAL_NO_DEFAULT_EXTI_CB first: two definitions in one TU would be a
   compile error even though they are weak. */
#ifndef FAKE_HAL_NO_DEFAULT_EXTI_CB
__attribute__((weak)) void HAL_GPIO_EXTI_Callback(uint16_t pin) { (void)pin; }
#endif

__attribute__((weak)) void HAL_GPIO_EXTI_IRQHandler(uint16_t pin)
{
    HAL_GPIO_EXTI_Callback(pin);
}

void fake_gpio_exti_pulse(uint16_t pin)
{
    /* Only the pin mask is visible to the callback, so the port does not
       matter; drive the line low (falling edge), run the ISR, release it. */
    int b = pin_index(pin);
    if (b >= 0) {
        for (int p = 0; p < 9; p++) {
            gpio_in[p] &= ~(1u << b);
            ((GPIO_TypeDef *[]){ GPIOA, GPIOB, GPIOC, GPIOD, GPIOE,
                                 GPIOF, GPIOG, GPIOH, GPIOI })[p]->IDR = gpio_in[p];
        }
    }
    HAL_GPIO_EXTI_IRQHandler(pin);
    if (b >= 0) {
        for (int p = 0; p < 9; p++) {
            gpio_in[p] |= (1u << b);
            ((GPIO_TypeDef *[]){ GPIOA, GPIOB, GPIOC, GPIOD, GPIOE,
                                 GPIOF, GPIOG, GPIOH, GPIOI })[p]->IDR = gpio_in[p];
        }
    }
}

/* ------------------------------------------------------------------- I2C */

static fake_i2c_dev_t *dev_for(uint16_t dev_addr)
{
    fake_i2c_dev_t *d = &i2c_devs[(dev_addr >> 1) & 0x7F];
    return d;
}

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c) { (void)hi2c; return HAL_OK; }
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *h, uint32_t f)
{ (void)h; (void)f; return HAL_OK; }
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *h, uint32_t f)
{ (void)h; (void)f; return HAL_OK; }

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t dev,
                                          uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)hi2c; (void)timeout;
    fake_i2c_dev_t *d = dev_for(dev);
    if (!d->present || !d->on_write) return HAL_ERROR;
    d->writes++;
    return d->on_write(d, data, size);
}

HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t dev,
                                         uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)hi2c; (void)timeout;
    fake_i2c_dev_t *d = dev_for(dev);
    if (!d->present || !d->on_read) return HAL_ERROR;
    d->reads++;
    memset(data, 0xFF, size);
    return d->on_read(d, data, size);
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t dev,
                                    uint16_t mem, uint16_t mem_size,
                                    uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)hi2c; (void)timeout;
    fake_i2c_dev_t *d = dev_for(dev);
    if (!d->present || !d->on_write) return HAL_ERROR;

    uint8_t buf[2 + 256];
    uint16_t n = 0;
    if (mem_size == I2C_MEMADD_SIZE_16BIT) {
        buf[n++] = (uint8_t)(mem >> 8);
        buf[n++] = (uint8_t)mem;
    } else {
        buf[n++] = (uint8_t)mem;
    }
    if ((uint32_t)size + n > sizeof(buf)) return HAL_ERROR;
    memcpy(&buf[n], data, size);
    n = (uint16_t)(n + size);

    d->writes++;
    return d->on_write(d, buf, n);
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t dev,
                                   uint16_t mem, uint16_t mem_size,
                                   uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)hi2c; (void)timeout;
    fake_i2c_dev_t *d = dev_for(dev);
    if (!d->present || !d->on_read || !d->on_write) return HAL_ERROR;

    uint8_t ptr[2];
    uint16_t n = 0;
    if (mem_size == I2C_MEMADD_SIZE_16BIT) {
        ptr[n++] = (uint8_t)(mem >> 8);
        ptr[n++] = (uint8_t)mem;
    } else {
        ptr[n++] = (uint8_t)mem;
    }
    d->writes++;
    if (d->on_write(d, ptr, n) != HAL_OK) return HAL_ERROR;

    d->reads++;
    memset(data, 0xFF, size);
    return d->on_read(d, data, size);
}

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t dev,
                                        uint32_t trials, uint32_t timeout)
{
    (void)hi2c; (void)trials; (void)timeout;
    return dev_for(dev)->present ? HAL_OK : HAL_TIMEOUT;
}

/* ------------------------------------------------------------------- SPI */

void fake_spi_set_handler(fake_spi_fn_t fn) { spi_handler = fn; }

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi) { (void)hspi; return HAL_OK; }

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *data,
                                   uint16_t size, uint32_t timeout)
{
    (void)timeout;
    if (!spi_handler) return HAL_OK;
    return spi_handler(hspi, data, NULL, size);
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *tx,
                                          uint8_t *rx, uint16_t size, uint32_t timeout)
{
    (void)timeout;
    if (!spi_handler) return HAL_OK;
    return spi_handler(hspi, tx, rx, size);
}

/* ------------------------------------------------------------------ UART */

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart) { (void)huart; return HAL_OK; }

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *data,
                                    uint16_t size, uint32_t timeout)
{
    (void)timeout;
    for (uint16_t i = 0; i < size && uart_tx_n < sizeof(uart_tx); i++)
        uart_tx[uart_tx_n++] = data[i];
    if (uart_tx_hook) uart_tx_hook(huart, data, size);
    return HAL_OK;
}

void fake_uart_set_tx_hook(fake_uart_tx_fn_t fn) { uart_tx_hook = fn; }

void fake_uart_set_rx_handler(void (*fn)(void)) { uart_rx_handler = fn; }

void fake_uart_feed(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        fake_usart1.RDR = buf[i];
        fake_usart1.ISR |= USART_ISR_RXNE_RXFNE;
        if (uart_rx_handler) uart_rx_handler();
    }
}

void fake_uart_tx_clear(void) { uart_tx_n = 0; }

size_t fake_uart_tx_len(void) { return uart_tx_n; }

size_t fake_uart_tx_take(uint8_t *buf, size_t max)
{
    size_t n = uart_tx_n < max ? uart_tx_n : max;
    memcpy(buf, uart_tx, n);
    return n;
}

/* ------------------------------------------------------------------- ADC */

void fake_adc_set(uint32_t channel, uint16_t raw)
{
    if (channel < 32) adc_value[channel] = raw;
}

uint32_t fake_adc_last_channel(void) { return adc_last_ch; }
uint32_t fake_adc_conversion_count(void) { return adc_conversions; }

HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *hadc) { (void)hadc; return HAL_OK; }
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t single)
{ (void)hadc; (void)single; return HAL_OK; }

HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *cfg)
{
    (void)hadc;
    adc_last_ch = cfg->Channel;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    adc_conversions++;
    fake_adc2.DR = adc_value[adc_last_ch & 31];
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t timeout)
{
    (void)hadc; (void)timeout;
    return HAL_OK;
}

uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    return adc_value[adc_last_ch & 31];
}

HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc) { (void)hadc; return HAL_OK; }

/* ------------------------------------------------------------------- TIM */

HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim) { (void)htim; return HAL_OK; }

HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *oc,
                                            uint32_t channel)
{
    (void)htim;
    fake_hal_tim_set_compare(htim, channel, oc->Pulse);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)htim; (void)channel;
    return HAL_OK;
}

/* -------------------------------------------------------------- RCC / PWR */

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *osc) { (void)osc; return HAL_OK; }
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *clk, uint32_t latency)
{ (void)clk; (void)latency; return HAL_OK; }
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *p)
{ (void)p; return HAL_OK; }
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t scale)
{ (void)scale; return HAL_OK; }
