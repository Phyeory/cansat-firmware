/* GPS: real NMEA bytes pushed through the real USART1_IRQHandler.
 *
 * The sentences below are the classic u-blox examples (their published
 * checksums are 0x6A and 0x47), plus a southern/western hemisphere pair so
 * the sign logic is exercised in both directions.
 */
#include <unity.h>
#include <string.h>

#include "../fakes/fake_hal.c"
#include "../../src/board.c"
#include "../../src/gps_sam_m8q.c"

void setUp(void)
{
    fake_hal_reset();
    board_init();        /* gives huart1 its USART1 instance */
    gps_init(&huart1);   /* huart1.Instance is set by board_init */
    fake_uart_set_rx_handler(USART1_IRQHandler);
}

void tearDown(void) { }

static void feed(const char *s) { fake_uart_feed((const uint8_t *)s, strlen(s)); }

static void test_rmc_and_gga_give_position_fix_and_altitude(void)
{
    feed("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    feed("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");

    gps_data_t g;
    gps_poll(&g);

    TEST_ASSERT_TRUE(g.fix);
    TEST_ASSERT_EQUAL_UINT32(2, g.sentences);
    TEST_ASSERT_EQUAL_UINT8(8, g.sats);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 48.1173, g.lat_deg);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 11.516666666666667, g.lon_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 545.4f, g.alt_m);
}

static void test_southern_western_hemisphere_is_negative(void)
{
    feed("$GPRMC,000000,A,3345.678,S,07040.123,W,000.0,000.0,010120,000.0,E*70\r\n");
    feed("$GPGGA,000000,3345.678,S,07040.123,W,1,12,0.8,100.0,M,0.0,M,,*7C\r\n");

    gps_data_t g;
    gps_poll(&g);

    TEST_ASSERT_TRUE(g.fix);
    TEST_ASSERT_EQUAL_UINT8(12, g.sats);
    TEST_ASSERT_TRUE(g.lat_deg < 0.0);
    TEST_ASSERT_TRUE(g.lon_deg < 0.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -33.7613, g.lat_deg);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -70.66871666666667, g.lon_deg);
}

static void test_bad_checksum_is_rejected(void)
{
    feed("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6B\r\n");

    gps_data_t g;
    gps_poll(&g);
    TEST_ASSERT_EQUAL_UINT32(0, g.sentences);   /* nothing parsed */
    TEST_ASSERT_FALSE(g.fix);
}

static void test_truncated_or_non_nmea_lines_are_ignored(void)
{
    feed("$GPRMC,123519,A\r\n");          /* too few fields */
    feed("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    feed("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W\r\n"); /* no checksum */

    gps_data_t g;
    gps_poll(&g);
    TEST_ASSERT_EQUAL_UINT32(0, g.sentences);
}

static void test_void_status_clears_fix_but_still_counts(void)
{
    feed("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    gps_data_t g;
    gps_poll(&g);
    TEST_ASSERT_TRUE(g.fix);

    feed("$GPRMC,123520,V,4807.038,N,01131.000,E,0.0,0.0,230394,003.1,W*7B\r\n");
    gps_poll(&g);
    TEST_ASSERT_FALSE(g.fix);
    TEST_ASSERT_EQUAL_UINT32(2, g.sentences);
}

static void test_gga_without_lock_reports_zero_sats_and_no_altitude(void)
{
    feed("$GPGGA,123519,4807.038,N,01131.000,E,0,00,,,M,,M,,*52\r\n");

    gps_data_t g;
    gps_poll(&g);
    TEST_ASSERT_EQUAL_UINT8(0, g.sats);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, g.alt_m);
    TEST_ASSERT_EQUAL_UINT32(1, g.sentences);
}

static void test_ring_buffer_wraps_and_keeps_parsing(void)
{
    /* Overflow the 1 KiB ring in chunks, with the consumer running between
       them, then make sure a normal sentence still parses. */
    for (int chunk = 0; chunk < 3; chunk++) {
        for (int i = 0; i < 900; i++) feed("X");
        gps_data_t g;
        gps_poll(&g);
    }
    feed("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");

    gps_data_t g;
    gps_poll(&g);
    TEST_ASSERT_EQUAL_UINT32(1, g.sentences);
    TEST_ASSERT_TRUE(g.fix);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 48.1173, g.lat_deg);
}

static void test_overlong_line_is_dropped(void)
{
    for (int i = 0; i < 150; i++) feed("Z");   /* longer than LINE_LEN */
    feed("\r\n");
    feed("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");

    gps_data_t g;
    gps_poll(&g);
    TEST_ASSERT_EQUAL_UINT32(1, g.sentences);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_rmc_and_gga_give_position_fix_and_altitude);
    RUN_TEST(test_southern_western_hemisphere_is_negative);
    RUN_TEST(test_bad_checksum_is_rejected);
    RUN_TEST(test_truncated_or_non_nmea_lines_are_ignored);
    RUN_TEST(test_void_status_clears_fix_but_still_counts);
    RUN_TEST(test_gga_without_lock_reports_zero_sats_and_no_altitude);
    RUN_TEST(test_ring_buffer_wraps_and_keeps_parsing);
    RUN_TEST(test_overlong_line_is_dropped);
    return UNITY_END();
}
