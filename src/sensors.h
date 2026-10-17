#pragma once
#include <Arduino.h>
#include "config.h"

// One fused reading of every sensor on the Can. Populated by SensorSuite::poll().
struct SensorFrame {
    uint32_t t_ms;          // millis() at sample time
    // primary mission
    float temp_c;
    float pressure_hpa;
    float altitude_m;       // derived from pressure
    // secondary mission — atmospheric radiation & gas profiling
    float uv_raw;           // LTR390 raw UV counts
    float uv_index;
    float ir_lux;           // TSL2591 infrared + full-spectrum
    float full_lux;
    float co2_ppm;          // SCD30
    float o3_ppb;           // Alphasense O3-B4
    // navigation / housekeeping
    float gps_lat, gps_lon, gps_alt_m;
    bool  gps_fix;
    uint8_t sats;
    float vbat_v;
    bool  ok_bmp, ok_ltr, ok_tsl, ok_scd, ok_o3, ok_gps;  // per-sensor health
};

class SensorSuite {
public:
    // initialises every sensor; call once from setup(). Returns true if all
    // mandatory sensors (BMP280) came up. Non-fatal failures set ok_* false
    // and that sensor simply reports NaN.
    bool begin();

    // samples everything into `f`. Cheap enough to call at 10 Hz.
    void poll(SensorFrame& f);

private:
    float readO3mV();
};
