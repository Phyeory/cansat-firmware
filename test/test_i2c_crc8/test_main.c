/* Sensirion CRC-8 and the thin I2C helpers.
 *
 * Golden vectors come from the Sensirion datasheet ("CRC-8 for 123456789
 * gives 0xF7") and from bytes whose CRC is published in SCD30/SHT45 examples.
 */
#include <unity.h>

#include "../fakes/fake_hal.c"
#include "../fakes/fake_devices.c"
#include "../../src/board.c"      /* real hi2c1/hspi1/... handle objects */
#include "../../src/i2c_dev.c"

void setUp(void)
{
    fake_hal_reset();
    fdev_reset();
}

void tearDown(void) { }

static void test_sensirion_check_value(void)
{
    const uint8_t check[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    TEST_ASSERT_EQUAL_HEX8(0xF7, i2c_crc8(check, 9));
}

static void test_crc8_datagram_vector(void)
{
    const uint8_t v[] = { 0xBE, 0xEF };
    TEST_ASSERT_EQUAL_HEX8(0x92, i2c_crc8(v, 2));
}

static void test_crc8_zeros(void)
{
    const uint8_t v[] = { 0x00, 0x00 };
    TEST_ASSERT_EQUAL_HEX8(0x81, i2c_crc8(v, 2));
}

static void test_crc8_one_byte_and_empty(void)
{
    const uint8_t v[] = { 0xFF };
    TEST_ASSERT_EQUAL_HEX8(0x00, i2c_crc8(v, 1));
    TEST_ASSERT_EQUAL_HEX8(0xFF, i2c_crc8(v, 0));   /* seed survives */
}

static void test_write_reg_builds_two_byte_frame(void)
{
    fdev_ltr390_enable();
    TEST_ASSERT_EQUAL(HAL_OK, i2c_write_reg(&hi2c1, 0x53 << 1, 0x05, 0x01));
    TEST_ASSERT_EQUAL_HEX8(0x01, fdev_reg(0x53, 0x05));
}

static void test_read_reg_and_read_regs(void)
{
    fdev_ltr390_enable();
    fdev_ltr390_set_uvs(123456);

    uint8_t one = 0;
    TEST_ASSERT_EQUAL(HAL_OK, i2c_read_reg(&hi2c1, 0x53 << 1, 0x06, &one));
    TEST_ASSERT_EQUAL_HEX8(0xB1, one);

    uint8_t three[3] = { 0 };
    TEST_ASSERT_EQUAL(HAL_OK, i2c_read_regs(&hi2c1, 0x53 << 1, 0x10, three, 3));
    TEST_ASSERT_EQUAL_HEX32(123456u,
        (uint32_t)three[0] | ((uint32_t)three[1] << 8) | ((uint32_t)three[2] << 16));
}

static void test_absent_device_nacks(void)
{
    uint8_t b;
    TEST_ASSERT_EQUAL(HAL_ERROR, i2c_read_reg(&hi2c1, 0x53 << 1, 0x06, &b));
    TEST_ASSERT_EQUAL(HAL_TIMEOUT, HAL_I2C_IsDeviceReady(&hi2c1, 0x53 << 1, 2, 5));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_sensirion_check_value);
    RUN_TEST(test_crc8_datagram_vector);
    RUN_TEST(test_crc8_zeros);
    RUN_TEST(test_crc8_one_byte_and_empty);
    RUN_TEST(test_write_reg_builds_two_byte_frame);
    RUN_TEST(test_read_reg_and_read_regs);
    RUN_TEST(test_absent_device_nacks);
    return UNITY_END();
}
