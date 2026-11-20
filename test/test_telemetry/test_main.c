/* Telemetry is the firmware <-> ground-station contract: field names, order,
 * precision and the health bitmask all have to stay put.  These are literal
 * golden strings, not reformatted copies of the implementation. */
#include <unity.h>
#include <string.h>

#include "../../src/telemetry.c"

void setUp(void) { }
void tearDown(void) { }

static flight_frame_t full_frame(void)
{
    flight_frame_t f;
    memset(&f, 0, sizeof(f));
    f.t_ms         = 12345;
    f.seq          = 7;
    f.temp_c       = 21.5f;
    f.pressure_hpa = 1006.5f;
    f.altitude_m   = 56.0f;
    f.uv_raw       = 1200.0f;
    f.uv_index     = 1.25f;
    f.ir_counts    = 100.0f;
    f.full_counts  = 2000.0f;
    f.lux          = 33.5f;
    f.co2_ppm      = 420.0f;
    f.o3_ppb       = 12.3f;
    f.gps_lat      = 52.12345;
    f.gps_lon      = 4.56789;
    f.gps_alt_m    = 12.0f;
    f.gps_sats     = 9;
    f.mag_x        = 24.0f;
    f.mag_y        = -6.0f;
    f.mag_z        = 42.0f;
    f.rh_pct       = 45.5f;
    f.sht_temp_c   = 22.25f;
    f.rad_cps      = 12;
    f.vbat_v       = 4.10f;
    f.ok_bmp = f.ok_ltr = f.ok_tsl = f.ok_scd = 1;
    f.ok_gps = f.gps_fix = f.ok_sd = 1;
    f.ok_mlx = f.ok_sht = f.ok_rad = 1;
    return f;
}

static const char *JSON_GOLDEN =
    "{\"n\":7,\"t\":12345,\"T\":21.5,\"P\":1006.5,\"A\":56.0,"
    "\"uv\":1200,"
    "\"co2\":420,\"o3\":12.3,"
    "\"la\":52.12345,\"lo\":4.56789,\"ga\":12,\"s\":9,"
    "\"rh\":45.5,\"rad\":12,"
    "\"v\":4.10,\"h\":1023}";

static const char *CSV_GOLDEN =
    "12345,21.50,1006.50,56.00,1200.0,1.25,100.0,2000.0,33.5,420,12.3,"
    "52.12345,4.56789,12.0,9,4.10,"
    "24.0,-6.0,42.0,"
    "45.5,22.25,12,1023";

static void test_health_all_ok_is_0x3ff(void)
{
    flight_frame_t f = full_frame();
    TEST_ASSERT_EQUAL_HEX16(0x03FF, telemetry_health(&f));
}

static void test_health_zero_and_single_bits(void)
{
    flight_frame_t f;
    memset(&f, 0, sizeof(f));
    TEST_ASSERT_EQUAL_HEX16(0x0000, telemetry_health(&f));

    f.ok_sd = 1;   TEST_ASSERT_EQUAL_HEX16(1u << 6, telemetry_health(&f));
    f.ok_sd = 0;
    f.gps_fix = 1; TEST_ASSERT_EQUAL_HEX16(1u << 5, telemetry_health(&f));
    f.ok_rad = 1;  TEST_ASSERT_EQUAL_HEX16((1u << 5) | (1u << 9), telemetry_health(&f));
}

static void test_gps_data_and_fix_are_separate_bits(void)
{
    flight_frame_t f;
    memset(&f, 0, sizeof(f));
    f.ok_gps = 1;                       /* sentences seen, no lock */
    TEST_ASSERT_EQUAL_HEX16(1u << 4, telemetry_health(&f));
    f.gps_fix = 1;
    TEST_ASSERT_EQUAL_HEX16((1u << 4) | (1u << 5), telemetry_health(&f));
    f.ok_gps = 0;                       /* lock but no fresh data */
    TEST_ASSERT_EQUAL_HEX16(1u << 5, telemetry_health(&f));
}

static void test_json_golden(void)
{
    flight_frame_t f = full_frame();
    char buf[256];
    int n = telemetry_json(&f, buf, (int)sizeof(buf));
    TEST_ASSERT_EQUAL_STRING(JSON_GOLDEN, buf);
    TEST_ASSERT_EQUAL((int)strlen(JSON_GOLDEN), n);
    TEST_ASSERT_TRUE(n < 256);
}

static void test_csv_header_golden(void)
{
    TEST_ASSERT_EQUAL_STRING(
        "t_ms,temp_c,pressure_hpa,alt_m,uv_raw,uv_index,ir_counts,"
        "full_counts,lux,co2_ppm,o3_ppb,lat,lon,gps_alt_m,sats,vbat_v,"
        "mag_x,mag_y,mag_z,"
        "rh_pct,sht_temp_c,rad_cps,health",
        telemetry_csv_header());
}

static void test_csv_row_golden(void)
{
    flight_frame_t f = full_frame();
    char row[340];
    telemetry_csv_row(&f, row, (int)sizeof(row));
    TEST_ASSERT_EQUAL_STRING(CSV_GOLDEN, row);
}

static void test_json_truncation_is_detectable(void)
{
    flight_frame_t f = full_frame();
    char small[16];
    int n = telemetry_json(&f, small, (int)sizeof(small));
    /* snprintf returns what it *would* have written; main.c drops the packet
       when that value is >= the packet buffer size. */
    TEST_ASSERT_EQUAL((int)strlen(JSON_GOLDEN), n);
    TEST_ASSERT_EQUAL('\0', small[sizeof(small) - 1]);
    TEST_ASSERT_TRUE(strlen(small) == sizeof(small) - 1);
}

static void test_packet_fits_the_e22_buffer(void)
{
    flight_frame_t f = full_frame();
    f.temp_c = -40.0f; f.pressure_hpa = 1100.0f; f.altitude_m = 12345.0f;
    f.gps_lat = -89.99999; f.gps_lon = -179.99999; f.gps_alt_m = 99999.0f;
    f.o3_ppb = -999.9f; f.vbat_v = 9.99f;

    char buf[256];
    int n = telemetry_json(&f, buf, (int)sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_TRUE(n < 256);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_health_all_ok_is_0x3ff);
    RUN_TEST(test_health_zero_and_single_bits);
    RUN_TEST(test_gps_data_and_fix_are_separate_bits);
    RUN_TEST(test_json_golden);
    RUN_TEST(test_csv_header_golden);
    RUN_TEST(test_csv_row_golden);
    RUN_TEST(test_json_truncation_is_detectable);
    RUN_TEST(test_packet_fits_the_e22_buffer);
    return UNITY_END();
}
