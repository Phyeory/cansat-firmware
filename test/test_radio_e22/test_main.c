/* E22-900T22D: the radio is a transparent UART pipe.
 *
 * The contract is simply "the packet bytes leave on USART3".  The fake HAL
 * records every transmit and its handle, so the test also proves the driver
 * does not accidentally write telemetry to the debug console (USART2).
 */
#include <unity.h>
#include <string.h>

#include "../fakes/fake_hal.c"
#include "../fakes/fake_devices.c"
#include "../../src/board.c"
#include "../../src/radio_e22.c"

static UART_HandleTypeDef *last_uart;
static uint8_t  last[512];
static uint16_t last_len;

static void tx_hook(UART_HandleTypeDef *h, const uint8_t *data, uint16_t len)
{
    last_uart = h;
    last_len = len;
    if (len > sizeof(last)) len = sizeof(last);
    memcpy(last, data, len);
}

void setUp(void)
{
    fake_hal_reset();
    fdev_reset();
    fake_uart_tx_clear();
    fake_uart_set_tx_hook(tx_hook);
    last_uart = NULL;
    last_len = 0;
    memset(last, 0, sizeof(last));
    TEST_ASSERT_EQUAL_INT(0, e22_init(&huart3));
}

void tearDown(void) { }

static void test_init_selects_usart3(void)
{
    TEST_ASSERT_EQUAL_INT(0, e22_init(&huart3));
}

static void test_send_puts_the_packet_on_the_radio_uart(void)
{
    const uint8_t msg[] = "hello";

    TEST_ASSERT_EQUAL_INT(0, e22_send(msg, 5));
    TEST_ASSERT_EQUAL_PTR(&huart3, last_uart);
    TEST_ASSERT_EQUAL_UINT16(5, last_len);
    TEST_ASSERT_EQUAL_MEMORY(msg, last, 5);
    TEST_ASSERT_EQUAL_UINT32(1, e22_tx_count());
    TEST_ASSERT_EQUAL_UINT32(0, e22_fail_count());
}

static void test_two_packets_keep_their_payloads_distinct(void)
{
    const uint8_t a[] = "first-packet";
    const uint8_t b[] = "second";

    TEST_ASSERT_EQUAL_INT(0, e22_send(a, sizeof(a) - 1));
    TEST_ASSERT_EQUAL_UINT16(sizeof(a) - 1, last_len);
    TEST_ASSERT_EQUAL_MEMORY(a, last, sizeof(a) - 1);

    TEST_ASSERT_EQUAL_INT(0, e22_send(b, sizeof(b) - 1));
    TEST_ASSERT_EQUAL_UINT16(sizeof(b) - 1, last_len);
    TEST_ASSERT_EQUAL_MEMORY(b, last, sizeof(b) - 1);
    TEST_ASSERT_EQUAL_UINT32(2, e22_tx_count());
}

static void test_empty_payload_is_rejected(void)
{
    TEST_ASSERT_EQUAL_INT(-1, e22_send((const uint8_t *)"", 0));
    TEST_ASSERT_EQUAL_UINT32(0, e22_tx_count());
}

static void test_null_payload_is_rejected(void)
{
    TEST_ASSERT_EQUAL_INT(-1, e22_send(NULL, 4));
    TEST_ASSERT_EQUAL_UINT32(0, e22_tx_count());
}

static void test_busy_is_always_false_because_tx_is_blocking(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, e22_busy());
    TEST_ASSERT_EQUAL_INT(0, e22_send((const uint8_t *)"x", 1));
    TEST_ASSERT_EQUAL_UINT8(0, e22_busy());
}

static void test_maximum_payload_roundtrips(void)
{
    uint8_t big[255];
    for (int i = 0; i < 255; i++) big[i] = (uint8_t)(i * 7 + 3);

    TEST_ASSERT_EQUAL_INT(0, e22_send(big, sizeof(big)));
    TEST_ASSERT_EQUAL_UINT16(255, last_len);
    TEST_ASSERT_EQUAL_MEMORY(big, last, 255);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_selects_usart3);
    RUN_TEST(test_send_puts_the_packet_on_the_radio_uart);
    RUN_TEST(test_two_packets_keep_their_payloads_distinct);
    RUN_TEST(test_empty_payload_is_rejected);
    RUN_TEST(test_null_payload_is_rejected);
    RUN_TEST(test_busy_is_always_false_because_tx_is_blocking);
    RUN_TEST(test_maximum_payload_roundtrips);
    return UNITY_END();
}
