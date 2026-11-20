/* TSL2591 light sensor: command-bit addressing, channel split and the lux
 * formula (which is where the subtle bugs live - luminance is not linear in
 * the raw channels, and IR has to be subtracted with a correction term).
 */
#include <unity.h>
#include <string.h>

#include "../fakes/fake_hal.c"
#include "../fakes/fake_devices.c"
#include "../../src/board.c"
#include "../../src/i2c_dev.c"
#include "../../src/tsl2591.c"

void setUp(void)
{
    fake_hal_reset();
    fdev_reset();
    fdev_tsl2591_enable();
}

void tearDown(void) { }

static void test_init_checks_id_and_enables_sensor(void)
{
    TEST_ASSERT_EQUAL_INT(0, tsl2591_init(&hi2c1));

    TEST_ASSERT_EQUAL_HEX8(0x03, fdev_reg(0x29, 0x00));   /* power on + ALS enable */
    TEST_ASSERT_EQUAL_HEX8(0x12, fdev_reg(0x29, 0x01));   /* medium gain, 300 ms */
}

static void test_init_rejects_wrong_id(void)
{
    fdev_set_reg(0x29, 0x12, 0x51);
    TEST_ASSERT_EQUAL_INT(-2, tsl2591_init(&hi2c1));
}

static void test_init_fails_without_device(void)
{
    fdev_tsl2591_disable();
    TEST_ASSERT_EQUAL_INT(-1, tsl2591_init(&hi2c1));
}

static void test_read_splits_channels(void)
{
    TEST_ASSERT_EQUAL_INT(0, tsl2591_init(&hi2c1));
    fdev_tsl2591_set_counts(1000, 200);

    tsl2591_data_t d;
    tsl2591_read(&hi2c1, &d);

    TEST_ASSERT_EQUAL_FLOAT(1000.0f, d.full_counts);
    TEST_ASSERT_EQUAL_FLOAT(200.0f, d.ir_counts);
}

static void test_read_applies_the_lux_formula(void)
{
    TEST_ASSERT_EQUAL_INT(0, tsl2591_init(&hi2c1));
    fdev_tsl2591_set_counts(1000, 200);

    tsl2591_data_t d;
    tsl2591_read(&hi2c1, &d);

    /* cpl = 25 * 300 / 408; lux = (1000-200) * (1 - 200/1000) / cpl
       = 640 * 408 / 7500 = 34.816 */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 34.816f, d.lux);
}

static void test_read_clamps_negative_lux_to_zero(void)
{
    TEST_ASSERT_EQUAL_INT(0, tsl2591_init(&hi2c1));

    fdev_tsl2591_set_counts(100, 100);   /* equal channels -> zero lux */
    tsl2591_data_t d;
    tsl2591_read(&hi2c1, &d);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.lux);

    fdev_tsl2591_set_counts(100, 400);   /* IR above full -> still zero */
    tsl2591_read(&hi2c1, &d);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.lux);
}

static void test_read_with_zero_full_scale_is_zero_lux(void)
{
    TEST_ASSERT_EQUAL_INT(0, tsl2591_init(&hi2c1));
    fdev_tsl2591_set_counts(0, 0);

    tsl2591_data_t d;
    tsl2591_read(&hi2c1, &d);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.lux);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.full_counts);
}

static void test_full_scale_counts(void)
{
    TEST_ASSERT_EQUAL_INT(0, tsl2591_init(&hi2c1));
    fdev_tsl2591_set_counts(65535, 0);

    tsl2591_data_t d;
    tsl2591_read(&hi2c1, &d);
    TEST_ASSERT_EQUAL_FLOAT(65535.0f, d.full_counts);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 65535.0f * 408.0f / 7500.0f, d.lux);
}

static void test_bus_loss_zeroes_the_sample(void)
{
    TEST_ASSERT_EQUAL_INT(0, tsl2591_init(&hi2c1));
    fdev_tsl2591_set_counts(1000, 200);
    fdev_tsl2591_disable();

    tsl2591_data_t d = { 9.0f, 9.0f, 9.0f };
    tsl2591_read(&hi2c1, &d);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.ir_counts);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.full_counts);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.lux);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_checks_id_and_enables_sensor);
    RUN_TEST(test_init_rejects_wrong_id);
    RUN_TEST(test_init_fails_without_device);
    RUN_TEST(test_read_splits_channels);
    RUN_TEST(test_read_applies_the_lux_formula);
    RUN_TEST(test_read_clamps_negative_lux_to_zero);
    RUN_TEST(test_read_with_zero_full_scale_is_zero_lux);
    RUN_TEST(test_full_scale_counts);
    RUN_TEST(test_bus_loss_zeroes_the_sample);
    return UNITY_END();
}
