/* LSM6DSV: 6-axis IMU on SPI1.
 *
 * The register image is the contract: WHO_AM_I = 0x70, BDU + IF_INC,
 * 120 Hz ODR, +/-8 g and +/-2000 dps.  The data path checks the
 * datasheet sensitivities: 0.244 mg/LSB and 70 mdps/LSB.
 */
#include <unity.h>
#include <math.h>

#include "../fakes/fake_hal.c"
#include "../fakes/fake_devices.c"
#include "../../src/board.c"
#include "../../src/spi_dev.c"
#include "../../src/lsm6dsv.c"

void setUp(void)
{
    fake_hal_reset();
    fdev_reset();
    fdev_lsm6dsv_enable();
    fdev_lsm6dsv_set_motion(0.0f, 0.0f, 9.80665f, 0.0f, 0.0f, 0.0f);
}

void tearDown(void) { }

static void test_init_programs_expected_configuration(void)
{
    TEST_ASSERT_EQUAL_INT(0, lsm6dsv_init(&hspi1));
    TEST_ASSERT_EQUAL_HEX8(0x44, fdev_lsm6dsv_reg(0x12));  /* BDU | IF_INC */
    TEST_ASSERT_EQUAL_HEX8(0x02, fdev_lsm6dsv_reg(0x17));  /* +/-8 g */
    TEST_ASSERT_EQUAL_HEX8(0x04, fdev_lsm6dsv_reg(0x15));  /* +/-2000 dps */
    TEST_ASSERT_EQUAL_HEX8(0x06, fdev_lsm6dsv_reg(0x10));  /* 120 Hz accel */
    TEST_ASSERT_EQUAL_HEX8(0x06, fdev_lsm6dsv_reg(0x11));  /* 120 Hz gyro */
}

static void test_init_is_idempotent(void)
{
    TEST_ASSERT_EQUAL_INT(0, lsm6dsv_init(&hspi1));
    TEST_ASSERT_EQUAL_INT(0, lsm6dsv_init(&hspi1));
}

static void test_init_fails_without_device(void)
{
    fdev_lsm6dsv_disable();
    TEST_ASSERT_EQUAL_INT(-1, lsm6dsv_init(&hspi1));
}

static void test_init_rejects_wrong_chip_id(void)
{
    fdev_lsm6dsv_set_reg(0x0F, 0x71);
    TEST_ASSERT_EQUAL_INT(-1, lsm6dsv_init(&hspi1));
}

static void test_read_decodes_static_gravity(void)
{
    TEST_ASSERT_EQUAL_INT(0, lsm6dsv_init(&hspi1));

    lsm6dsv_data_t d;
    TEST_ASSERT_EQUAL_INT(1, lsm6dsv_read(&hspi1, &d));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, d.acc_x);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, d.acc_y);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 9.80665f, d.acc_z);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, d.gyr_x);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, d.gyr_y);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, d.gyr_z);
}

static void test_read_uses_datasheet_scales(void)
{
    TEST_ASSERT_EQUAL_INT(0, lsm6dsv_init(&hspi1));
    fdev_lsm6dsv_set_motion(1.0f, 2.0f, 3.0f, 10.0f, 20.0f, 30.0f);

    lsm6dsv_data_t d;
    TEST_ASSERT_EQUAL_INT(1, lsm6dsv_read(&hspi1, &d));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, d.acc_x);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 2.0f, d.acc_y);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 3.0f, d.acc_z);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, d.gyr_x);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 20.0f, d.gyr_y);
    TEST_ASSERT_FLOAT_WITHIN(0.08f, 30.0f, d.gyr_z);
}

static void test_read_tracks_negative_full_scale(void)
{
    TEST_ASSERT_EQUAL_INT(0, lsm6dsv_init(&hspi1));
    fdev_lsm6dsv_set_motion(-9.80665f, 0.0f, 0.0f, -45.0f, 0.0f, 0.0f);

    lsm6dsv_data_t d;
    TEST_ASSERT_EQUAL_INT(1, lsm6dsv_read(&hspi1, &d));
    TEST_ASSERT_TRUE(d.acc_x < -9.7f && d.acc_x > -9.9f);
    TEST_ASSERT_TRUE(d.gyr_x < -44.9f && d.gyr_x > -45.1f);
}

static void test_read_after_bus_loss_is_rejected(void)
{
    TEST_ASSERT_EQUAL_INT(0, lsm6dsv_init(&hspi1));
    fdev_lsm6dsv_disable();

    lsm6dsv_data_t d;
    TEST_ASSERT_EQUAL_INT(-1, lsm6dsv_read(&hspi1, &d));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_programs_expected_configuration);
    RUN_TEST(test_init_is_idempotent);
    RUN_TEST(test_init_fails_without_device);
    RUN_TEST(test_init_rejects_wrong_chip_id);
    RUN_TEST(test_read_decodes_static_gravity);
    RUN_TEST(test_read_uses_datasheet_scales);
    RUN_TEST(test_read_tracks_negative_full_scale);
    RUN_TEST(test_read_after_bus_loss_is_rejected);
    return UNITY_END();
}
