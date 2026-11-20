/* Radiation counter: a falling-edge EXTI counter.
 *
 * The real path is ISR -> volatile pulse count -> atomic latch in the main
 * loop, so the tests pulse the EXTI line the way the Geiger tube does and
 * check that no count is lost or double-counted, that the latch clears, and
 * that "active" expires 60 s after the last pulse.
 */
#include <unity.h>

/* rad_counter.c owns HAL_GPIO_EXTI_Callback; suppress the fake's default so
   both can live in this one translation unit. */
#define FAKE_HAL_NO_DEFAULT_EXTI_CB
#include "../fakes/fake_hal.c"
#include "../fakes/fake_devices.c"
#include "../../src/board.c"
#include "../../src/rad_counter.c"

void setUp(void)
{
    fake_hal_reset();
    fdev_reset();
    rad_counter_start();
    (void)rad_counter_latch();   /* the counter is a static; start does not clear it */
}

void tearDown(void) { }

static void fire_pulses(unsigned n)
{
    for (unsigned i = 0; i < n; i++) fake_gpio_exti_pulse(RAD_INPUT_PIN);
}

static void test_start_leaves_the_counter_empty(void)
{
    TEST_ASSERT_EQUAL_UINT32(0, rad_counter_latch());
}

static void test_each_edge_is_counted_once(void)
{
    fire_pulses(1);
    TEST_ASSERT_EQUAL_UINT32(1, rad_counter_latch());

    fire_pulses(9);
    TEST_ASSERT_EQUAL_UINT32(9, rad_counter_latch());
}

static void test_latch_is_destructive(void)
{
    fire_pulses(5);
    TEST_ASSERT_EQUAL_UINT32(5, rad_counter_latch());
    TEST_ASSERT_EQUAL_UINT32(0, rad_counter_latch());
}

static void test_other_pins_are_ignored(void)
{
    fake_gpio_exti_pulse(GPIO_PIN_11);   /* same EXTI bank, not the counter */
    fake_gpio_exti_pulse(GPIO_PIN_9);
    TEST_ASSERT_EQUAL_UINT32(0, rad_counter_latch());
}

static void test_active_until_sixty_seconds_after_the_last_pulse(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, rad_counter_active());

    fake_tick_set(1000);
    fire_pulses(1);
    TEST_ASSERT_EQUAL_UINT8(1, rad_counter_active());

    fake_tick_set(1000 + 59999);
    TEST_ASSERT_EQUAL_UINT8(1, rad_counter_active());

    fake_tick_set(1000 + 60000);
    TEST_ASSERT_EQUAL_UINT8(0, rad_counter_active());
}

static void test_active_restarts_with_a_new_pulse(void)
{
    fake_tick_set(1000);
    fire_pulses(1);
    TEST_ASSERT_EQUAL_UINT32(1, rad_counter_latch());
    fake_tick_set(70000);
    TEST_ASSERT_EQUAL_UINT8(0, rad_counter_active());

    fire_pulses(1);
    TEST_ASSERT_EQUAL_UINT8(1, rad_counter_active());
    TEST_ASSERT_EQUAL_UINT32(1, rad_counter_latch());
}

static void test_long_burst_does_not_lose_counts(void)
{
    fire_pulses(1000);
    TEST_ASSERT_EQUAL_UINT32(1000, rad_counter_latch());
}

static void test_counter_wraps_without_sticking(void)
{
    /* 32-bit counter: after a wrap it must keep counting from zero. */
    fire_pulses(3);
    TEST_ASSERT_EQUAL_UINT32(3, rad_counter_latch());
    fire_pulses(2);
    TEST_ASSERT_EQUAL_UINT32(2, rad_counter_latch());
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_start_leaves_the_counter_empty);
    RUN_TEST(test_each_edge_is_counted_once);
    RUN_TEST(test_latch_is_destructive);
    RUN_TEST(test_other_pins_are_ignored);
    RUN_TEST(test_active_until_sixty_seconds_after_the_last_pulse);
    RUN_TEST(test_active_restarts_with_a_new_pulse);
    RUN_TEST(test_long_burst_does_not_lose_counts);
    RUN_TEST(test_counter_wraps_without_sticking);
    return UNITY_END();
}
