/**
 * main.c — Celestium CanSat flight firmware (STM32G474RE)
 *
 * Primary mission:   BMP280 temperature + pressure -> altitude, JSON
 *                    downlink at 1 Hz over 433 MHz LoRa (Rule 14 compliant).
 * Secondary mission: atmospheric radiation & gas profiling — UV, IR, CO2,
 *                    O3 vs altitude, logged to SD at 10 Hz.
 *
 * Hardware: custom PCB, STM32CubeMX config in stm32g474.ioc.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "board.h"
#include "config.h"
#include "telemetry.h"
#include "bmp280.h"
#include "ltr390.h"
#include "tsl2591.h"
#include "scd30.h"
#include "gps_sam_m8q.h"
#include "lora_sx1276.h"
#include "sdlog.h"
#include "servos.h"
#include "adc_read.h"

static flight_frame_t frame;
static uint32_t last_sample, last_telemetry;

static void sample_all(void);

int main(void)
{
    board_init();
    servos_start();

    frame.ok_bmp = (bmp280_init(&hi2c1) == 0);
    frame.ok_ltr = (ltr390_init(&hi2c1) == 0);
    frame.ok_tsl = (tsl2591_init(&hi2c1) == 0);
    frame.ok_scd = (scd30_init(&hi2c1) == 0);
    gps_init(&huart1);
    frame.ok_gps = 1;

    if (!frame.ok_bmp) {
        /* primary mission sensor dead — halt with fast LED heartbeat on PC6 */
        printf("[FAIL] BMP280 init — halting\r\n");
        while (1) { }
    }

    int lora_ok = (lora_init(&hspi1) == 0);
    sdlog_init();
    printf("[CELESTIUM] boot ok=%u%u%u%u lora=%u sd=%u\r\n",
           frame.ok_bmp, frame.ok_ltr, frame.ok_tsl, frame.ok_scd,
           lora_ok, sdlog_ok());

    while (1) {
        uint32_t now = HAL_GetTick();

        if (now - last_sample >= SAMPLE_PERIOD_MS) {
            last_sample = now;
            sample_all();

            char row[240];
            telemetry_csv_row(&frame, row, sizeof(row));
            sdlog_write(row);
            printf("t=%lus T=%.1fC P=%.1fhPa alt=%.1fm CO2=%.0f UVi=%.2f\r\n",
                   now / 1000U, frame.temp_c, frame.pressure_hpa,
                   frame.altitude_m, frame.co2_ppm, frame.uv_index);
        }

        if (now - last_telemetry >= TELEMETRY_PERIOD_MS) {   /* 1 Hz downlink */
            last_telemetry = now;
            char pkt[300];
            int n = telemetry_json(&frame, pkt, sizeof(pkt));
            if (lora_ok && n > 0) lora_send((const uint8_t *)pkt, (uint8_t)n);
        }
    }
}

static void sample_all(void)
{
    frame.t_ms = HAL_GetTick();

    bmp280_data_t b;  bmp280_read(&hi2c1, &b);
    frame.temp_c = b.temp_c;
    frame.pressure_hpa = b.pressure_hpa;
    frame.altitude_m = b.altitude_m;

    ltr390_data_t u;  ltr390_read(&hi2c1, &u);
    frame.uv_raw = u.uv_raw;
    frame.uv_index = u.uv_index;

    tsl2591_data_t l;  tsl2591_read(&hi2c1, &l);
    frame.ir_counts = l.ir_counts;
    frame.full_counts = l.full_counts;
    frame.lux = l.lux;

    scd30_data_t s;
    int r = scd30_read(&hi2c1, &s);
    if (r > 0) frame.co2_ppm = s.co2_ppm;

    gps_data_t g;  gps_poll(&g);
    frame.gps_lat = g.lat_deg;
    frame.gps_lon = g.lon_deg;
    frame.gps_alt_m = g.alt_m;
    frame.gps_sats = g.sats;
    frame.gps_fix = g.fix;

    /* O3-B4: differential working/auxiliary electrode in mV -> ppb */
    float work_mv = adc_to_v(adc_read_ch(O3_WORK_CHANNEL)) * 1000.0f;
    float aux_mv  = adc_to_v(adc_read_ch(O3_AUX_CHANNEL)) * 1000.0f;
    frame.o3_ppb = ((work_mv - aux_mv) - O3_ZERO_MV) / O3_MV_PER_PPB;

    frame.vbat_v = adc_to_v(adc_read_ch(VBAT_CHANNEL)) * VBAT_DIVIDER;
}
