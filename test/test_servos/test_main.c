/* Servo channels: TIM8 compare values.
 *
 * The interesting behaviour is the guard rail: servo_set_us() must ignore
 * pulses outside 500..2500 us and invalid channel indices, because a bad
 * value would drive a horn into its stop.  That is exactly what the tests
 * pin down, along with the stow/deploy presets.
 */
#include <unity.h>

#include "../fakes/fake_hal.c"
#include "../fakes/fake_devices.c"
#include "../../src/board.c"
#include "../../src/servos.c"

void setUp(void)
{
    fake_hal_reset();
    fdev_reset();
    board_init();          /* htim8.Instance must be valid for the CCR macro */
}

void tearDown(void) { }

static uint32_t ccr(int ch)
{
    switch (ch) {
    case 0: return htim8.Instance->CCR1;
    case 1: return htim8.Instance->CCR2;
    case 2: return htim8.Instance->CCR3;
    default: return htim8.Instance->CCR4;
    }
}

static void test_stow_sets_all_four_channels(void)
{
    servos_stow();
    for (int i = 0; i < 4; i++)
        TEST_ASSERT_EQUAL_UINT32(SERVO_STOWED_US, ccr(i));
}

static void test_deploy_sets_all_four_channels(void)
{
    servos_deploy();
    for (int i = 0; i < 4; i++)
        TEST_ASSERT_EQUAL_UINT32(SERVO_DEPLOYED_US, ccr(i));
}

static void test_start_leaves_the_servos_stowed(void)
{
    servos_deploy();
    servos_start();
    for (int i = 0; i < 4; i++)
        TEST_ASSERT_EQUAL_UINT32(SERVO_STOWED_US, ccr(i));
}

static void test_channels_are_independent(void)
{
    servos_stow();
    servos_set_us(1, 1500);
    servos_set_us(3, 1250);

    TEST_ASSERT_EQUAL_UINT32(1000, ccr(0));
    TEST_ASSERT_EQUAL_UINT32(1500, ccr(1));
    TEST_ASSERT_EQUAL_UINT32(1000, ccr(2));
    TEST_ASSERT_EQUAL_UINT32(1250, ccr(3));
}

static void test_pulse_limits_are_inclusive(void)
{
    servos_stow();
    servos_set_us(0, 500);
    TEST_ASSERT_EQUAL_UINT32(500, ccr(0));
    servos_set_us(0, 2500);
    TEST_ASSERT_EQUAL_UINT32(2500, ccr(0));
}

static void test_out_of_range_pulses_are_ignored(void)
{
    servos_stow();

    servos_set_us(0, 499);
    TEST_ASSERT_EQUAL_UINT32(SERVO_STOWED_US, ccr(0));

    servos_set_us(0, 2501);
    TEST_ASSERT_EQUAL_UINT32(SERVO_STOWED_US, ccr(0));

    servos_set_us(0, 0);
    TEST_ASSERT_EQUAL_UINT32(SERVO_STOWED_US, ccr(0));
}

static void test_out_of_range_channels_are_ignored(void)
{
    servos_stow();

    servos_set_us(-1, 1500);
    servos_set_us(4, 1500);
    servos_set_us(99, 1500);

    for (int i = 0; i < 4; i++)
        TEST_ASSERT_EQUAL_UINT32(SERVO_STOWED_US, ccr(i));
}

static void test_stow_and_deploy_are_idempotent(void)
{
    servos_stow();
    servos_stow();
    TEST_ASSERT_EQUAL_UINT32(1000, ccr(0));

    servos_deploy();
    servos_deploy();
    TEST_ASSERT_EQUAL_UINT32(2000, ccr(0));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_stow_sets_all_four_channels);
    RUN_TEST(test_deploy_sets_all_four_channels);
    RUN_TEST(test_start_leaves_the_servos_stowed);
    RUN_TEST(test_channels_are_independent);
    RUN_TEST(test_pulse_limits_are_inclusive);
    RUN_TEST(test_out_of_range_pulses_are_ignored);
    RUN_TEST(test_out_of_range_channels_are_ignored);
    RUN_TEST(test_stow_and_deploy_are_idempotent);
    return UNITY_END();
}
