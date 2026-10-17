#include "telemetry.h"
#include <stdio.h>

/* Same JSON schema as the previous Arduino build, so the Python ground
   station (ground-station/station.py) needs no changes. */
int telemetry_json(const flight_frame_t *f, char *buf, int n)
{
    return snprintf(buf, n,
        "{\"t\":%lu,\"T\":%.2f,\"P\":%.2f,\"A\":%.2f,"
        "\"uv\":%.1f,\"uvi\":%.2f,\"ir\":%.1f,\"lux\":%.1f,"
        "\"co2\":%.0f,\"o3\":%.1f,"
        "\"la\":%.5f,\"lo\":%.5f,\"ga\":%.1f,\"s\":%u,"
        "\"v\":%.2f}",
        (unsigned long)f->t_ms,
        f->temp_c, f->pressure_hpa, f->altitude_m,
        f->uv_raw, f->uv_index, f->ir_counts, f->lux,
        f->co2_ppm, f->o3_ppb,
        f->gps_lat, f->gps_lon, f->gps_alt_m, f->gps_sats,
        f->vbat_v);
}

const char *telemetry_csv_header(void)
{
    return "t_ms,temp_c,pressure_hpa,alt_m,uv_raw,uv_index,ir_counts,"
           "full_counts,lux,co2_ppm,o3_ppb,lat,lon,gps_alt_m,sats,vbat_v";
}

void telemetry_csv_row(const flight_frame_t *f, char *buf, int n)
{
    snprintf(buf, n,
        "%lu,%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.1f,%.1f,%.0f,%.1f,"
        "%.5f,%.5f,%.1f,%u,%.2f",
        (unsigned long)f->t_ms, f->temp_c, f->pressure_hpa, f->altitude_m,
        f->uv_raw, f->uv_index, f->ir_counts, f->full_counts, f->lux,
        f->co2_ppm, f->o3_ppb,
        f->gps_lat, f->gps_lon, f->gps_alt_m, f->gps_sats, f->vbat_v);
}
