/* ADC: channel selection and the raw-to-volt conversion.
 *
 * adc_read_ch() reconfigures ADC2 for every sample (channel, 247.5 cycles,
 * single-ended) and reads one conversion.  The fake records the channel and
 * the conversion count, so both the routing and the "one conversion per call"
 * contract are checked; adc_to_v() is checked at the endpoints and midpoint.
 */
#include <unity.h>
#include <math.h>

#include "../fakes/fake_hal.c"
#include "../fakes/fake_devices.c"
#include "../../src/board.c"
#include "../../src/adc_read.c"

void setUp(void)
{
    fake_hal_reset();
    fdev_reset();
    board_init();
}

void tearDown(void) { }

static void test_read_returns_the_scripted_value(void)
{
    fake_adc_set(O3_WORK_CHANNEL, 1234);
    TEST_ASSERT_EQUAL_UINT16(1234, adc_read_ch(O3_WORK_CHANNEL));
    TEST_ASSERT_EQUAL_UINT32(O3_WORK_CHANNEL, fake_adc_last_channel());
}

static void test_channels_are_routed_independently(void)
{
    fake_adc_set(O3_WORK_CHANNEL, 100);
    fake_adc_set(O3_AUX_CHANNEL, 2000);
    fake_adc_set(VBAT_CHANNEL, 3000);

    TEST_ASSERT_EQUAL_UINT16(100, adc_read_ch(O3_WORK_CHANNEL));
    TEST_ASSERT_EQUAL_UINT16(2000, adc_read_ch(O3_AUX_CHANNEL));
    TEST_ASSERT_EQUAL_UINT16(3000, adc_read_ch(VBAT_CHANNEL));

    TEST_ASSERT_EQUAL_UINT32(VBAT_CHANNEL, fake_adc_last_channel());
}

static void test_one_conversion_per_call(void)
{
    uint32_t before = fake_adc_conversion_count();
    (void)adc_read_ch(O3_WORK_CHANNEL);
    TEST_ASSERT_EQUAL_UINT32(before + 1, fake_adc_conversion_count());
    (void)adc_read_ch(O3_AUX_CHANNEL);
    TEST_ASSERT_EQUAL_UINT32(before + 2, fake_adc_conversion_count());
}

static void test_conversion_endpoints_and_midpoint(void)
{
    fake_adc_set(O3_WORK_CHANNEL, 0);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, adc_to_v(adc_read_ch(O3_WORK_CHANNEL)));

    fake_adc_set(O3_WORK_CHANNEL, 4095);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, VREF_V, adc_to_v(adc_read_ch(O3_WORK_CHANNEL)));

    fake_adc_set(O3_WORK_CHANNEL, 2048);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2048.0f * VREF_V / 4095.0f,
                             adc_to_v(adc_read_ch(O3_WORK_CHANNEL)));
}

static void test_conversion_is_linear(void)
{
    float prev = -1.0f;
    for (uint16_t raw = 0; raw <= 4095; raw += 455) {
        float v = adc_to_v(raw);
        TEST_ASSERT_TRUE(v > prev);
        prev = v;
    }
}

static void test_vbat_divider_scaling(void)
{
    /* The pack voltage is the ADC node times VBAT_DIVIDER. */
    fake_adc_set(VBAT_CHANNEL, 2048);
    float pack = adc_to_v(adc_read_ch(VBAT_CHANNEL)) * VBAT_DIVIDER;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f * 2048.0f * VREF_V / 4095.0f, pack);
}

static void test_reading_is_repeatable(void)
{
    fake_adc_set(O3_WORK_CHANNEL, 777);
    for (int i = 0; i < 5; i++)
        TEST_ASSERT_EQUAL_UINT16(777, adc_read_ch(O3_WORK_CHANNEL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_read_returns_the_scripted_value);
    RUN_TEST(test_channels_are_routed_independently);
    RUN_TEST(test_one_conversion_per_call);
    RUN_TEST(test_conversion_endpoints_and_midpoint);
    RUN_TEST(test_conversion_is_linear);
    RUN_TEST(test_vbat_divider_scaling);
    RUN_TEST(test_reading_is_repeatable);
    return UNITY_END();
}
