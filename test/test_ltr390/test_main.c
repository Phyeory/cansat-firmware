/* LTR390 UV sensor: part-ID gate, register programming, 18-bit UVS and index.
 *
 * Note for the bench: the driver addresses registers without the datasheet's
 * 0x10 command bit, and the fake masks bit 7 so both conventions work.  That
 * is a deliberate liberal fake, not a hardware-proven bus sequence - see
 * TESTING.md ("LTR390 command bit").
 */
#include <unity.h>
#include <string.h>

#include "../fakes/fake_hal.c"
#include "../fakes/fake_devices.c"
#include "../../src/board.c"
#include "../../src/i2c_dev.c"
#include "../../src/ltr390.c"

void setUp(void)
{
    fake_hal_reset();
    fdev_reset();
    fdev_ltr390_enable();
}

void tearDown(void) { }

static void test_init_programs_gain_rate_and_enables_uv(void)
{
    TEST_ASSERT_EQUAL_INT(0, ltr390_init(&hi2c1));

    TEST_ASSERT_EQUAL_HEX8(0x01, fdev_reg(0x53, 0x05));   /* GAIN: 3x */
    TEST_ASSERT_EQUAL_HEX8(0x22, fdev_reg(0x53, 0x04));   /* 18-bit, 100 ms */
    TEST_ASSERT_EQUAL_HEX8(0x03, fdev_reg(0x53, 0x00));   /* UVS + enable */
}

static void test_init_rejects_wrong_part_id(void)
{
    fdev_set_reg(0x53, 0x06, 0xA1);   /* high nibble 0xA, not 0xB */
    TEST_ASSERT_EQUAL_INT(-2, ltr390_init(&hi2c1));
}

static void test_init_fails_without_device(void)
{
    fdev_ltr390_disable();
    TEST_ASSERT_EQUAL_INT(-1, ltr390_init(&hi2c1));
}

static void test_read_decodes_18_bit_uvs(void)
{
    TEST_ASSERT_EQUAL_INT(0, ltr390_init(&hi2c1));
    fdev_ltr390_set_uvs(0x12345);

    ltr390_data_t d;
    ltr390_read(&hi2c1, &d);

    TEST_ASSERT_EQUAL_FLOAT(0x12345, d.uv_raw);
    /* index = raw / (2300 * (3/18) * 0.25) = raw / 95.8333 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0x12345 / (2300.0f * (3.0f / 18.0f) * 0.25f),
                             d.uv_index);
}

static void test_read_uses_all_three_bytes(void)
{
    TEST_ASSERT_EQUAL_INT(0, ltr390_init(&hi2c1));

    fdev_ltr390_set_uvs(0x000001);
    ltr390_data_t d;
    ltr390_read(&hi2c1, &d);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, d.uv_raw);

    fdev_ltr390_set_uvs(0x010000);
    ltr390_read(&hi2c1, &d);
    TEST_ASSERT_EQUAL_FLOAT(65536.0f, d.uv_raw);

    fdev_ltr390_set_uvs(0x3FFFF);
    ltr390_read(&hi2c1, &d);
    TEST_ASSERT_EQUAL_FLOAT(262143.0f, d.uv_raw);
}

static void test_zero_uv_is_zero_index(void)
{
    TEST_ASSERT_EQUAL_INT(0, ltr390_init(&hi2c1));
    fdev_ltr390_set_uvs(0);

    ltr390_data_t d = { 9.0f, 9.0f };
    ltr390_read(&hi2c1, &d);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.uv_raw);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.uv_index);
}

static void test_bus_loss_zeroes_the_sample(void)
{
    TEST_ASSERT_EQUAL_INT(0, ltr390_init(&hi2c1));
    fdev_ltr390_set_uvs(1234);
    fdev_ltr390_disable();

    ltr390_data_t d = { 9.0f, 9.0f };
    ltr390_read(&hi2c1, &d);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.uv_raw);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.uv_index);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_programs_gain_rate_and_enables_uv);
    RUN_TEST(test_init_rejects_wrong_part_id);
    RUN_TEST(test_init_fails_without_device);
    RUN_TEST(test_read_decodes_18_bit_uvs);
    RUN_TEST(test_read_uses_all_three_bytes);
    RUN_TEST(test_zero_uv_is_zero_index);
    RUN_TEST(test_bus_loss_zeroes_the_sample);
    return UNITY_END();
}
