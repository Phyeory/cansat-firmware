#include "radio_e22.h"
#include "config.h"

#define E22_TX_TIMEOUT_MS 500U

static UART_HandleTypeDef *s;
static uint8_t  ready = 0;
static uint32_t n_tx = 0, n_fail = 0;

int e22_init(UART_HandleTypeDef *uart)
{
    if (uart == NULL) {
        s = NULL;
        ready = 0;
        return -1;
    }
    s = uart;
    ready = 1;
    n_tx = n_fail = 0;
    /* Give the module time to finish its own power-up before the first
       packet, but do not block the sensor bring-up for long. */
    HAL_Delay(50);
    return 0;
}

uint8_t e22_busy(void)
{
    /* HAL_UART_Transmit() is blocking for the length of one packet, so there
       is never a packet in flight when the superloop asks. */
    return 0;
}

int e22_send(const uint8_t *data, uint16_t len)
{
    if (!ready || data == NULL || len == 0) return -1;

    if (HAL_UART_Transmit(s, (uint8_t *)data, len, E22_TX_TIMEOUT_MS) != HAL_OK) {
        n_fail++;
        return -1;
    }
    n_tx++;
    return 0;
}

uint32_t e22_tx_count(void)   { return n_tx; }
uint32_t e22_fail_count(void) { return n_fail; }
