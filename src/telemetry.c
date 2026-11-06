#include "telemetry.h"
#include <stdio.h>

uint16_t telemetry_health(const flight_frame_t *f)
{
    return (uint16_t)((f->ok_bmp ? 1 : 0) | (f->ok_ltr ? 2 : 0) |
                      (f->ok_tsl ? 4 : 0) | (f->ok_scd ? 8 : 0) |
                      (f->ok_gps ? 16 : 0) | (f->gps_fix ? 32 : 0) |
                      (f->ok_sd ? 64 : 0) | (f->ok_bmi ? 128 : 0) |
                      (f->ok_mlx ? 256 : 0) | (f->ok_sht ? 512 : 0) |
                      (f->ok_rad ? 1024 : 0));
}

/* Same JSON keys as before plus "n" (sequence), "h" (health) and the IMU /
   humidity / radiation fields. "uvi", "lux", "ir", the magnetometer and the
   SHT45 temperature stay SD-only (all derivable or low-value live) to keep
   the worst-case in-flight packet ~241 bytes, under the 253-byte LoRa
   payload limit at SF7 / 250 kHz (SF7/125 kHz caps at 222 B — do not
   downgrade the bandwidth without re-checking this budget). */
int telemetry_json(const flight_frame_t *f, char *buf, int n)
{
    return snprintf(buf, n,
        "{\"n\":%lu,\"t\":%lu,\"T\":%.1f,\"P\":%.1f,\"A\":%.1f,"
        "\"uv\":%.0f,"
        "\"co2\":%.0f,\"o3\":%.1f,"
        "\"la\":%.5f,\"lo\":%.5f,\"ga\":%.0f,\"s\":%u,"
        "\"ax\":%.1f,\"ay\":%.1f,\"az\":%.1f,"
        "\"gx\":%.0f,\"gy\":%.0f,\"gz\":%.0f,"
        "\"rh\":%.1f,\"rad\":%u,"
        "\"v\":%.2f,\"h\":%u}",
        (unsigned long)f->seq, (unsigned long)f->t_ms,
        f->temp_c, f->pressure_hpa, f->altitude_m,
        f->uv_raw,
        f->co2_ppm, f->o3_ppb,
        f->gps_lat, f->gps_lon, f->gps_alt_m, f->gps_sats,
        f->acc_x, f->acc_y, f->acc_z,
        f->gyr_x, f->gyr_y, f->gyr_z,
        f->hum_pct, (unsigned)f->rad_cps,
        f->vbat_v, telemetry_health(f));
}

const char *telemetry_csv_header(void)
{
    return "t_ms,temp_c,pressure_hpa,alt_m,uv_raw,uv_index,ir_counts,"
           "full_counts,lux,co2_ppm,o3_ppb,lat,lon,gps_alt_m,sats,vbat_v,"
           "acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,mag_x,mag_y,mag_z,"
           "rh_pct,hum_temp_c,rad_cps,health";
}

void telemetry_csv_row(const flight_frame_t *f, char *buf, int n)
{
    snprintf(buf, n,
        "%lu,%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.1f,%.1f,%.0f,%.1f,"
        "%.5f,%.5f,%.1f,%u,%.2f,"
        "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
        "%.1f,%.2f,%u,%u",
        (unsigned long)f->t_ms, f->temp_c, f->pressure_hpa, f->altitude_m,
        f->uv_raw, f->uv_index, f->ir_counts, f->full_counts, f->lux,
        f->co2_ppm, f->o3_ppb,
        f->gps_lat, f->gps_lon, f->gps_alt_m, f->gps_sats, f->vbat_v,
        f->acc_x, f->acc_y, f->acc_z, f->gyr_x, f->gyr_y, f->gyr_z,
        f->mag_x, f->mag_y, f->mag_z,
        f->hum_pct, f->hum_temp_c, (unsigned)f->rad_cps,
        telemetry_health(f));
}
