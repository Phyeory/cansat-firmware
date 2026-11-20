/* SHT45: a polled, stateful sensor.
 *
 * sht45_poll() is a two-call state machine: one call fires the measurement,
 * a later call collects it.  The chip NACKs the read until the conversion has
 * finished, and the driver drops the sample if the CRCs do not match.  These
 * tests drive that state machine with an explicit clock; the fake enforces the
 * 10 ms conversion time exactly like the datasheet.
 */
#include <unity.h>
#include <string.h>

#include "../fakes/fake_hal.c"
#include "../fakes/fake_devices.c"
#include "../../src/board.c"
#include "../../src/i2c_dev.c"
#include "../../src/sht45.c"

/* sht45_poll() keeps its trigger time in a function-static, which survives
   between tests.  Moving the fake clock forward by a large step per test (and
   never backwards) keeps "has a second elapsed?" unambiguous. */
static uint32_t clock_base;

void setUp(void)
{
    fake_hal_reset();
    fdev_reset();
    fdev_sht45_enable();
    TEST_ASSERT_EQUAL_INT(0, sht45_init(&hi2c1));

    clock_base += 10000U;
    fake_tick_set(clock_base);
}

void tearDown(void) { }

/* Fire a measurement at clock_base and collect it 10 ms later. */
static int measure(sht45_data_t *d)
{
    if (sht45_poll(&hi2c1, d) != 0) return -1;
    fake_tick_set(clock_base + 10U);
    return sht45_poll(&hi2c1, d);
}

static void test_measurement_roundtrip(void)
{
    fdev_sht45_set(25.0f, 50.0f);

    sht45_data_t d;
    TEST_ASSERT_EQUAL_INT(1, measure(&d));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, d.temp_c);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, d.rh_pct);
}

static void test_temperature_sweep(void)
{
    static const float temps[] = { -40.0f, -10.0f, 0.0f, 25.0f, 60.0f, 125.0f };

    for (unsigned i = 0; i < sizeof(temps) / sizeof(temps[0]); i++) {
        setUp();
        fdev_sht45_set(temps[i], 50.0f);

        sht45_data_t d;
        TEST_ASSERT_EQUAL_INT(1, measure(&d));
        TEST_ASSERT_FLOAT_WITHIN(0.05f, temps[i], d.temp_c);
    }
    tearDown();
}

static void test_humidity_sweep(void)
{
    static const float rhs[] = { 0.0f, 10.0f, 50.0f, 90.0f, 100.0f };

    for (unsigned i = 0; i < sizeof(rhs) / sizeof(rhs[0]); i++) {
        setUp();
        fdev_sht45_set(25.0f, rhs[i]);

        sht45_data_t d;
        TEST_ASSERT_EQUAL_INT(1, measure(&d));
        TEST_ASSERT_FLOAT_WITHIN(0.05f, rhs[i], d.rh_pct);
    }
    tearDown();
}

static void test_humidity_is_clamped_to_0_100(void)
{
    fdev_sht45_set(25.0f, 120.0f);
    sht45_data_t d;
    TEST_ASSERT_EQUAL_INT(1, measure(&d));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, d.rh_pct);

    setUp();
    fdev_sht45_set(25.0f, -20.0f);
    TEST_ASSERT_EQUAL_INT(1, measure(&d));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, d.rh_pct);
}

static void test_premature_read_is_rejected_and_drops_the_sample(void)
{
    fdev_sht45_set(25.0f, 50.0f);

    sht45_data_t d = { 7.0f, 7.0f };
    TEST_ASSERT_EQUAL_INT(0, sht45_poll(&hi2c1, &d));   /* trigger */

    fake_tick_set(clock_base + 1U);
    TEST_ASSERT_EQUAL_INT(-1, sht45_poll(&hi2c1, &d));  /* still converting */

    fake_tick_set(clock_base + 2U);
    TEST_ASSERT_EQUAL_INT(0, sht45_poll(&hi2c1, &d));   /* sample was dropped */
    TEST_ASSERT_EQUAL_FLOAT(7.0f, d.temp_c);
}

static void test_trigger_cadence_is_one_hertz(void)
{
    fdev_sht45_set(25.0f, 50.0f);

    sht45_data_t d;
    TEST_ASSERT_EQUAL_INT(1, measure(&d));              /* t = base + 10 ms */

    fake_tick_set(clock_base + 500U);
    TEST_ASSERT_EQUAL_INT(0, sht45_poll(&hi2c1, &d));   /* too early to re-trigger */

    fake_tick_set(clock_base + 999U);
    TEST_ASSERT_EQUAL_INT(0, sht45_poll(&hi2c1, &d));

    fake_tick_set(clock_base + 1000U);
    TEST_ASSERT_EQUAL_INT(0, sht45_poll(&hi2c1, &d));   /* second trigger */

    fake_tick_set(clock_base + 1010U);
    TEST_ASSERT_EQUAL_INT(1, sht45_poll(&hi2c1, &d));   /* and its sample */
}

static void test_crc_error_is_dropped(void)
{
    fdev_sht45_set(25.0f, 50.0f);
    fdev_sht45_corrupt_crc(1);

    sht45_data_t d = { 7.0f, 7.0f };
    TEST_ASSERT_EQUAL_INT(0, measure(&d));
    TEST_ASSERT_EQUAL_FLOAT(7.0f, d.temp_c);
}

static void test_soft_reset_mid_conversion_costs_one_sample(void)
{
    fdev_sht45_set(25.0f, 50.0f);

    sht45_data_t d;
    TEST_ASSERT_EQUAL_INT(0, sht45_poll(&hi2c1, &d));   /* trigger */
    TEST_ASSERT_EQUAL_INT(1, fdev_sht45_pending());

    TEST_ASSERT_EQUAL_INT(0, sht45_init(&hi2c1));       /* soft reset */
    TEST_ASSERT_EQUAL_INT(0, fdev_sht45_pending());     /* the chip forgot it */

    fake_tick_set(clock_base + 10U);
    TEST_ASSERT_EQUAL_INT(-1, sht45_poll(&hi2c1, &d));  /* driver still waiting: NACK */

    fake_tick_set(clock_base + 1000U);
    TEST_ASSERT_EQUAL_INT(0, sht45_poll(&hi2c1, &d));   /* recovers on the next trigger */
    fake_tick_set(clock_base + 1010U);
    TEST_ASSERT_EQUAL_INT(1, sht45_poll(&hi2c1, &d));   /* and the sample after it */
}

static void test_bus_loss_is_reported(void)
{
    fdev_sht45_set(25.0f, 50.0f);
    fdev_sht45_disable();

    sht45_data_t d;
    TEST_ASSERT_EQUAL_INT(0, sht45_poll(&hi2c1, &d));   /* trigger write NACKs */
    fake_tick_set(clock_base + 10U);
    TEST_ASSERT_EQUAL_INT(-1, sht45_poll(&hi2c1, &d));  /* read NACKs */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_measurement_roundtrip);
    RUN_TEST(test_temperature_sweep);
    RUN_TEST(test_humidity_sweep);
    RUN_TEST(test_humidity_is_clamped_to_0_100);
    RUN_TEST(test_premature_read_is_rejected_and_drops_the_sample);
    RUN_TEST(test_trigger_cadence_is_one_hertz);
    RUN_TEST(test_crc_error_is_dropped);
    RUN_TEST(test_soft_reset_mid_conversion_costs_one_sample);
    RUN_TEST(test_bus_loss_is_reported);
    return UNITY_END();
}
