#pragma once
#include <stdint.h>

typedef struct {
    uint32_t t_ms;
    uint32_t seq;
    float temp_c;
    float pressure_hpa;
    float altitude_m;
    float uv_raw, uv_index;
    float ir_counts, full_counts, lux;
    float co2_ppm;
    float o3_ppb;
    float acc_x, acc_y, acc_z;
    float gyr_x, gyr_y, gyr_z;
    float mag_x, mag_y, mag_z;
    float rh_pct;
    float sht_temp_c;
    uint16_t rad_cps;
    double gps_lat, gps_lon;
    float  gps_alt_m;
    uint8_t gps_sats;
    uint8_t gps_fix;
    float vbat_v;
    uint8_t ok_bmp, ok_ltr, ok_tsl, ok_scd, ok_gps, ok_sd;
    uint8_t ok_bmi, ok_mlx, ok_sht, ok_rad;
} flight_frame_t;

uint16_t telemetry_health(const flight_frame_t *f);

int telemetry_json(const flight_frame_t *f, char *buf, int n);

const char *telemetry_csv_header(void);
void telemetry_csv_row(const flight_frame_t *f, char *buf, int n);
