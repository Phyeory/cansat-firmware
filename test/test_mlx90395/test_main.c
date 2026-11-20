/* MLX90395 magnetometer: 16-bit SPI commands, status byte on the second byte.
 *
 * The driver refuses to decode unless the SM command returns "single
 * measurement mode" and the RM payload says DRDY, so a status mix-up yields
 * zeros instead of a crash.  Both gates are tested, plus the 0.714 uT/LSB
 * scale.
 */
#include <unity.h>
#include <string.h>

#include "../fakes/fake_hal.c"
#include "../fakes/fake_devices.c"
#include "../../src/board.c"
#include "../../src/spi_dev.c"
#include "../../src/mlx90395.c"

void setUp(void)
{
    fake_hal_reset();
    fdev_reset();
    fdev_mlx90395_enable();
}

void tearDown(void) { }

static void test_init_succeeds(void)
{
    TEST_ASSERT_EQUAL_INT(0, mlx90395_init(&hspi1));
}

static void test_init_fails_without_device(void)
{
    fdev_mlx90395_disable();
    TEST_ASSERT_EQUAL_INT(-1, mlx90395_init(&hspi1));
}

static void test_read_decodes_golden_vector(void)
{
    TEST_ASSERT_EQUAL_INT(0, mlx90395_init(&hspi1));
    fdev_mlx90395_set(100.0f, -200.0f, 300.0f);

    mlx90395_data_t d;
    TEST_ASSERT_EQUAL_INT(1, mlx90395_read(&hspi1, &d));

    /* One LSB is 0.714 uT, so the quantisation error is +-0.36 uT. */
    TEST_ASSERT_FLOAT_WITHIN(0.4f, 100.0f, d.mag_x);
    TEST_ASSERT_FLOAT_WITHIN(0.4f, -200.0f, d.mag_y);
    TEST_ASSERT_FLOAT_WITHIN(0.4f, 300.0f, d.mag_z);
}

static void test_read_is_not_axis_swapped(void)
{
    TEST_ASSERT_EQUAL_INT(0, mlx90395_init(&hspi1));
    fdev_mlx90395_set(10.0f, 20.0f, 30.0f);

    mlx90395_data_t d;
    TEST_ASSERT_EQUAL_INT(1, mlx90395_read(&hspi1, &d));

    TEST_ASSERT_FLOAT_WITHIN(0.4f, 10.0f, d.mag_x);
    TEST_ASSERT_FLOAT_WITHIN(0.4f, 20.0f, d.mag_y);
    TEST_ASSERT_FLOAT_WITHIN(0.4f, 30.0f, d.mag_z);
}

static void test_read_without_drdy_returns_no_sample(void)
{
    TEST_ASSERT_EQUAL_INT(0, mlx90395_init(&hspi1));
    fdev_mlx90395_set(100.0f, 100.0f, 100.0f);
    fdev_mlx90395_set_drdy(0);

    mlx90395_data_t d = { 1.0f, 2.0f, 3.0f };
    TEST_ASSERT_EQUAL_INT(0, mlx90395_read(&hspi1, &d));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, d.mag_x);   /* untouched */
}

static void test_read_outside_single_measurement_mode_returns_no_sample(void)
{
    TEST_ASSERT_EQUAL_INT(0, mlx90395_init(&hspi1));
    fdev_mlx90395_set(100.0f, 100.0f, 100.0f);
    fdev_mlx90395_set_status(0x00);

    mlx90395_data_t d;
    TEST_ASSERT_EQUAL_INT(0, mlx90395_read(&hspi1, &d));
}

static void test_read_failure_is_reported(void)
{
    TEST_ASSERT_EQUAL_INT(0, mlx90395_init(&hspi1));
    fdev_mlx90395_set(100.0f, 100.0f, 100.0f);
    fdev_mlx90395_fail_burst(1);

    mlx90395_data_t d;
    TEST_ASSERT_EQUAL_INT(-1, mlx90395_read(&hspi1, &d));
}

static void test_bus_loss_during_command_returns_no_sample(void)
{
    TEST_ASSERT_EQUAL_INT(0, mlx90395_init(&hspi1));
    fdev_mlx90395_disable();

    mlx90395_data_t d;
    TEST_ASSERT_EQUAL_INT(0, mlx90395_read(&hspi1, &d));
}

static void test_field_sweep_tracks_signed_full_scale(void)
{
    static const float ut[] = { -32768.0f * 0.714f, -1000.0f, -1.0f,
                                0.0f, 1.0f, 1000.0f, 32000.0f * 0.714f };

    TEST_ASSERT_EQUAL_INT(0, mlx90395_init(&hspi1));
    for (unsigned i = 0; i < sizeof(ut) / sizeof(ut[0]); i++) {
        fdev_mlx90395_set(ut[i], 0.0f, 0.0f);
        mlx90395_data_t d;
        TEST_ASSERT_EQUAL_INT(1, mlx90395_read(&hspi1, &d));
        TEST_ASSERT_FLOAT_WITHIN(0.4f, ut[i], d.mag_x);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_succeeds);
    RUN_TEST(test_init_fails_without_device);
    RUN_TEST(test_read_decodes_golden_vector);
    RUN_TEST(test_read_is_not_axis_swapped);
    RUN_TEST(test_read_without_drdy_returns_no_sample);
    RUN_TEST(test_read_outside_single_measurement_mode_returns_no_sample);
    RUN_TEST(test_read_failure_is_reported);
    RUN_TEST(test_bus_loss_during_command_returns_no_sample);
    RUN_TEST(test_field_sweep_tracks_signed_full_scale);
    return UNITY_END();
}
