#pragma once
#include <stdint.h>

/* Fused flight data — one struct filled by the sensors, shared by
   telemetry (radio JSON), sdlog (CSV) and the flight state machine. */

typedef struct {
    uint32_t t_ms;
    /* primary mission */
    float temp_c;
    float pressure_hpa;
    float altitude_m;
    /* secondary mission — atmospheric radiation & gas profiling */
    float uv_raw, uv_index;
    float ir_counts, full_counts, lux;
    float co2_ppm;
    float o3_ppb;
    /* navigation / housekeeping */
    double gps_lat, gps_lon;
    float  gps_alt_m;
    uint8_t gps_sats;
    uint8_t gps_fix;
    float vbat_v;
    /* sensor health flags */
    uint8_t ok_bmp, ok_ltr, ok_tsl, ok_scd, ok_gps;
} flight_frame_t;

/* builds the 1 Hz JSON downlink packet into buf; returns length */
int telemetry_json(const flight_frame_t *f, char *buf, int n);

/* CSV for SD logging */
const char *telemetry_csv_header(void);
void telemetry_csv_row(const flight_frame_t *f, char *buf, int n);
