#pragma once
#include <stdint.h>

/* Fused flight data: one struct filled by the sensors, shared by
   telemetry (radio JSON), sdlog (CSV) and the flight state machine. */

typedef struct {
    uint32_t t_ms;
    uint32_t seq;          /* packet number, lets the ground count losses */
    /* primary mission */
    float temp_c;
    float pressure_hpa;
    float altitude_m;
    /* secondary mission: atmospheric radiation & gas profiling */
    float uv_raw, uv_index;
    float ir_counts, full_counts, lux;
    float co2_ppm;
    float o3_ppb;
    /* motion: BMI270 + MLX90395 */
    float acc_x, acc_y, acc_z;    /* m/s^2 */
    float gyr_x, gyr_y, gyr_z;    /* deg/s */
    float mag_x, mag_y, mag_z;    /* uT */
    /* humidity: SHT45 */
    float rh_pct;                 /* relative humidity % */
    float sht_temp_c;             /* SHT45's own temperature */
    /* radiation: BPW34 S-Z */
    uint16_t rad_cps;             /* pulses in the last second */
    /* navigation / housekeeping */
    double gps_lat, gps_lon;
    float  gps_alt_m;
    uint8_t gps_sats;
    uint8_t gps_fix;
    float vbat_v;
    /* sensor health flags */
    uint8_t ok_bmp, ok_ltr, ok_tsl, ok_scd, ok_gps, ok_sd;
    uint8_t ok_bmi, ok_mlx, ok_sht, ok_rad;
} flight_frame_t;

/* health bitmask "h": bits 0..10 = BMP, LTR, TSL, SCD, GPSdata, GPSfix,
   SD, BMI, MLX, SHT, RAD */
uint16_t telemetry_health(const flight_frame_t *f);

/* builds the 1 Hz JSON downlink packet into buf; returns length */
int telemetry_json(const flight_frame_t *f, char *buf, int n);

/* CSV for SD logging */
const char *telemetry_csv_header(void);
void telemetry_csv_row(const flight_frame_t *f, char *buf, int n);
