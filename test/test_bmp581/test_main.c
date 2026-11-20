/* BMP581: factory-calibrated 24-bit barometer on SPI1.
 *
 * The register image is the contract: CHIP_ID = 0x50, OSR = x16/x2 with
 * pressure enabled, ODR = 50 Hz in normal mode with deep standby disabled.
 * The data path is checked against the datasheet scaling:
 * T = raw / 2^16 degC, p = raw / 2^6 Pa.
 */
#include <unity.h>
#include <math.h>

#include "../fakes/fake_hal.c"
#include "../fakes/fake_devices.c"
#include "../../src/board.c"
#include "../../src/spi_dev.c"
#include "../../src/bmp581.c"

void setUp(void)
{
    fake_hal_reset();
    fdev_reset();
    fdev_bmp581_enable();
}

void tearDown(void) { }

static void test_init_programs_expected_configuration(void)
{
    TEST_ASSERT_EQUAL_INT(0, bmp581_init(&hspi1));
    TEST_ASSERT_EQUAL_HEX8(0x50, fdev_bmp581_reg(0x01));  /* CHIP_ID */
    TEST_ASSERT_EQUAL_HEX8(0x61, fdev_bmp581_reg(0x36));  /* press_en, x16/x2 */
    TEST_ASSERT_EQUAL_HEX8(0xBD, fdev_bmp581_reg(0x37));  /* 50 Hz, normal */
}

static void test_init_is_idempotent(void)
{
    TEST_ASSERT_EQUAL_INT(0, bmp581_init(&hspi1));
    TEST_ASSERT_EQUAL_INT(0, bmp581_init(&hspi1));
}

static void test_init_fails_without_device(void)
{
    fdev_bmp581_disable();
    TEST_ASSERT_EQUAL_INT(-1, bmp581_init(&hspi1));
}

static void test_init_rejects_wrong_chip_id(void)
{
    fdev_bmp581_set_reg(0x01, 0x51);
    TEST_ASSERT_EQUAL_INT(-2, bmp581_init(&hspi1));
}

static void test_read_decodes_calibrated_words(void)
{
    TEST_ASSERT_EQUAL_INT(0, bmp581_init(&hspi1));
    fdev_bmp581_set_env(25.08f, 1006.5327f);

    bmp581_data_t d;
    TEST_ASSERT_EQUAL_INT(0, bmp581_read(&hspi1, &d));

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.08f, d.temp_c);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1006.5327f, d.pressure_hpa);
    /* 44330 * (1 - (1006.5327/1013.25)^0.190294957) */
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 56.08f, d.altitude_m);
}

static void test_sea_level_gives_zero_altitude(void)
{
    TEST_ASSERT_EQUAL_INT(0, bmp581_init(&hspi1));
    fdev_bmp581_set_env(15.0f, 1013.25f);

    bmp581_data_t d;
    TEST_ASSERT_EQUAL_INT(0, bmp581_read(&hspi1, &d));

    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1013.25f, d.pressure_hpa);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, d.altitude_m);
}

static void test_altitude_tracks_isa_up_to_2km(void)
{
    TEST_ASSERT_EQUAL_INT(0, bmp581_init(&hspi1));

    static const struct { float press_hpa, alt_m; } pts[] = {
        { 954.6169f,  500.0f },
        { 926.3485f,  750.0f },
        { 898.7618f, 1000.0f },
        { 794.9809f, 2000.0f },
    };
    for (unsigned i = 0; i < sizeof(pts) / sizeof(pts[0]); i++) {
        bmp581_data_t d;
        fdev_bmp581_set_env(15.0f, pts[i].press_hpa);
        TEST_ASSERT_EQUAL_INT(0, bmp581_read(&hspi1, &d));
        TEST_ASSERT_FLOAT_WITHIN(2.0f, pts[i].alt_m, d.altitude_m);
    }
}

static void test_read_after_bus_loss_zeroes_the_sample(void)
{
    TEST_ASSERT_EQUAL_INT(0, bmp581_init(&hspi1));
    fdev_bmp581_set_env(20.0f, 1000.0f);

    fdev_bmp581_disable();
    bmp581_data_t d = { 1.0f, 1.0f, 1.0f };
    TEST_ASSERT_EQUAL_INT(-1, bmp581_read(&hspi1, &d));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.temp_c);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.pressure_hpa);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.altitude_m);
}

static void test_temperature_sweep_is_monotonic(void)
{
    TEST_ASSERT_EQUAL_INT(0, bmp581_init(&hspi1));

    float prev = -1000.0f;
    for (float t = -20.0f; t <= 60.0f; t += 10.0f) {
        bmp581_data_t d;
        fdev_bmp581_set_env(t, 1000.0f);
        TEST_ASSERT_EQUAL_INT(0, bmp581_read(&hspi1, &d));
        TEST_ASSERT_FLOAT_WITHIN(0.05f, t, d.temp_c);
        TEST_ASSERT_TRUE(d.temp_c > prev);
        prev = d.temp_c;
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_programs_expected_configuration);
    RUN_TEST(test_init_is_idempotent);
    RUN_TEST(test_init_fails_without_device);
    RUN_TEST(test_init_rejects_wrong_chip_id);
    RUN_TEST(test_read_decodes_calibrated_words);
    RUN_TEST(test_sea_level_gives_zero_altitude);
    RUN_TEST(test_altitude_tracks_isa_up_to_2km);
    RUN_TEST(test_read_after_bus_loss_zeroes_the_sample);
    RUN_TEST(test_temperature_sweep_is_monotonic);
    return UNITY_END();
}
