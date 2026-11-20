/* SCD30 CO2 sensor: command framing, CRC checking and float reassembly.
 *
 * The chip speaks big-endian 16-bit commands followed by CRC'd argument words,
 * and answers with three IEEE-754 floats each split across two 16-bit words
 * with their own CRCs.  A driver bug here silently produces plausible but
 * wrong CO2 numbers, so the golden vectors below are exact float values.
 */
#include <unity.h>
#include <string.h>

#include "../fakes/fake_hal.c"
#include "../fakes/fake_devices.c"
#include "../../src/board.c"
#include "../../src/i2c_dev.c"
#include "../../src/scd30.c"

void setUp(void)
{
    fake_hal_reset();
    fdev_reset();
    fdev_scd30_enable();
}

void tearDown(void) { }

static void test_init_enables_asc_then_starts_continuous_mode(void)
{
    TEST_ASSERT_EQUAL_INT(0, scd30_init(&hi2c1));

    /* The driver's last command must be start-continuous, not ASC. */
    TEST_ASSERT_EQUAL_HEX16(0x0010, fdev_scd30_last_cmd());
    TEST_ASSERT_EQUAL_INT(1, fdev_scd30_running());
}

static void test_init_fails_without_device(void)
{
    fdev_scd30_disable();
    TEST_ASSERT_EQUAL_INT(-1, scd30_init(&hi2c1));
}

static void test_init_is_idempotent(void)
{
    TEST_ASSERT_EQUAL_INT(0, scd30_init(&hi2c1));
    TEST_ASSERT_EQUAL_INT(0, scd30_init(&hi2c1));
    TEST_ASSERT_EQUAL_INT(1, fdev_scd30_running());
}

static void test_read_before_measurement_returns_zero(void)
{
    TEST_ASSERT_EQUAL_INT(0, scd30_init(&hi2c1));

    scd30_data_t d = { 1.0f, 1.0f, 1.0f };
    TEST_ASSERT_EQUAL_INT(0, scd30_read(&hi2c1, &d));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, d.co2_ppm);   /* untouched when there is no data */
}

static void test_read_decodes_golden_values(void)
{
    TEST_ASSERT_EQUAL_INT(0, scd30_init(&hi2c1));
    fdev_scd30_set(420.0f, 24.5f, 50.0f);
    fdev_scd30_set_ready(1);

    scd30_data_t d;
    TEST_ASSERT_EQUAL_INT(1, scd30_read(&hi2c1, &d));
    TEST_ASSERT_EQUAL_FLOAT(420.0f, d.co2_ppm);
    TEST_ASSERT_EQUAL_FLOAT(24.5f, d.temp_c);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, d.rh_pct);
}

static void test_read_decodes_negative_temperature(void)
{
    TEST_ASSERT_EQUAL_INT(0, scd30_init(&hi2c1));
    fdev_scd30_set(1000.0f, -12.25f, 95.5f);
    fdev_scd30_set_ready(1);

    scd30_data_t d;
    TEST_ASSERT_EQUAL_INT(1, scd30_read(&hi2c1, &d));
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, d.co2_ppm);
    TEST_ASSERT_EQUAL_FLOAT(-12.25f, d.temp_c);
    TEST_ASSERT_EQUAL_FLOAT(95.5f, d.rh_pct);
}

static void test_reading_consumes_the_measurement(void)
{
    TEST_ASSERT_EQUAL_INT(0, scd30_init(&hi2c1));
    fdev_scd30_set_ready(1);

    scd30_data_t d;
    TEST_ASSERT_EQUAL_INT(1, scd30_read(&hi2c1, &d));
    TEST_ASSERT_EQUAL_INT(0, scd30_read(&hi2c1, &d));
}

static void test_measurement_crc_error_is_rejected(void)
{
    TEST_ASSERT_EQUAL_INT(0, scd30_init(&hi2c1));
    fdev_scd30_set_ready(1);
    fdev_scd30_corrupt_crc(1);

    scd30_data_t d;
    TEST_ASSERT_EQUAL_INT(-3, scd30_read(&hi2c1, &d));
}

static void test_bus_loss_is_reported(void)
{
    TEST_ASSERT_EQUAL_INT(0, scd30_init(&hi2c1));
    fdev_scd30_disable();

    scd30_data_t d;
    TEST_ASSERT_EQUAL_INT(-1, scd30_read(&hi2c1, &d));
}

static void test_co2_sweep_is_exact(void)
{
    static const float ppm[] = { 400.0f, 450.0f, 800.0f, 2000.0f, 5000.0f, 40000.0f };

    TEST_ASSERT_EQUAL_INT(0, scd30_init(&hi2c1));
    for (unsigned i = 0; i < sizeof(ppm) / sizeof(ppm[0]); i++) {
        fdev_scd30_set(ppm[i], 20.0f, 40.0f);
        fdev_scd30_set_ready(1);

        scd30_data_t d;
        TEST_ASSERT_EQUAL_INT(1, scd30_read(&hi2c1, &d));
        TEST_ASSERT_EQUAL_FLOAT(ppm[i], d.co2_ppm);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_enables_asc_then_starts_continuous_mode);
    RUN_TEST(test_init_fails_without_device);
    RUN_TEST(test_init_is_idempotent);
    RUN_TEST(test_read_before_measurement_returns_zero);
    RUN_TEST(test_read_decodes_golden_values);
    RUN_TEST(test_read_decodes_negative_temperature);
    RUN_TEST(test_reading_consumes_the_measurement);
    RUN_TEST(test_measurement_crc_error_is_rejected);
    RUN_TEST(test_bus_loss_is_reported);
    RUN_TEST(test_co2_sweep_is_exact);
    return UNITY_END();
}
