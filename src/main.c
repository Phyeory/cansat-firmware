/**
 * main.c: Celestium CanSat flight firmware (STM32G474RE)
 *
 * Primary mission:   BMP280 temperature + pressure -> altitude, JSON
 *                    downlink at 1 Hz over 433 MHz LoRa (Rule 14 compliant).
 * Secondary mission: atmospheric radiation & gas profiling (UV, IR, CO2,
 *                    O3 vs altitude), motion profiling (BMI270, MLX90395),
 *                    humidity (SHT45) and BPW34 radiation counting, all
 *                    logged to SD at 10 Hz.
 *
 * Data path:  sensors --I2C/UART/ADC--> frame --JSON--> LoRa --> ground
 *                                             \--CSV---> SD card
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "board.h"
#include "config.h"
#include "telemetry.h"
#include "i2c_dev.h"
#include "bmp280.h"
#include "ltr390.h"
#include "tsl2591.h"
#include "scd30.h"
#include "bmi270.h"
#include "mlx90395.h"
#include "sht45.h"
#include "rad_counter.h"
#include "gps_sam_m8q.h"
#include "lora_sx1276.h"
#include "sdlog.h"
#include "servos.h"
#include "adc_read.h"

static flight_frame_t frame;
static uint32_t last_sample, last_telemetry, last_retry;
static uint32_t last_sht_trigger;
static uint8_t  sht_pending;

static void sample_all(void);

/* boot check: lists every device that ACKs on the sensor bus, so a wiring
   fault shows up on the debug console instead of as silent zeros */
static void i2c_scan(void)
{
    printf("[I2C] scan:");
    int found = 0;
    for (uint16_t a = 0x08; a < 0x78; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 2, 5) == HAL_OK) {
            printf(" 0x%02X", a);
            found++;
        }
    }
    printf(found ? "  (expect 0x0C 0x29 0x44 0x53 0x61 0x68 0x76)\r\n"
                 : "  NONE: check SDA/SCL, pull-ups, 3V3 and GND\r\n");
}

int main(void)
{
    board_init();
    servos_start();
    printf("\r\n[CELESTIUM] booting\r\n");

    i2c_scan();
    frame.ok_bmp = (bmp280_init(&hi2c1) == 0);
    frame.ok_ltr = (ltr390_init(&hi2c1) == 0);
    frame.ok_tsl = (tsl2591_init(&hi2c1) == 0);
    frame.ok_scd = (scd30_init(&hi2c1) == 0);
    frame.ok_bmi = (bmi270_init(&hi2c1) == 0);
    frame.ok_mlx = (mlx90395_init(&hi2c1) == 0);
    frame.ok_sht = (sht45_init(&hi2c1) == 0);
    rad_counter_start();
    gps_init(&huart1);

    int lora_ok = (lora_init(&hspi1) == 0);
    sdlog_init();
    frame.ok_sd = sdlog_ok();
    if (frame.ok_sd) sdlog_write(telemetry_csv_header());

    printf("[CELESTIUM] bmp=%u ltr=%u tsl=%u scd=%u bmi=%u mlx=%u sht=%u "
           "lora=%u sd=%u\r\n",
           frame.ok_bmp, frame.ok_ltr, frame.ok_tsl, frame.ok_scd,
           frame.ok_bmi, frame.ok_mlx, frame.ok_sht, lora_ok, frame.ok_sd);
    if (!frame.ok_bmi)
        printf("[BMI270] missing or config load failed: check addr 0x68 and the config upload\r\n");
    if (!lora_ok) printf("[LORA] no reply (RegVersion != 0x12): check SPI1, CS PB0, RST PB1\r\n");
    if (!frame.ok_bmp) printf("[BMP280] missing: primary mission degraded, will keep retrying\r\n");

    /* Never halt in flight: even with a dead sensor, keep sending whatever
       still works plus GPS so the can can be found. */
    while (1) {
        uint32_t now = HAL_GetTick();

        lora_busy();   /* services TxDone, cheap */

        if (now - last_sample >= SAMPLE_PERIOD_MS) {
            last_sample = now;
            sample_all();

            char row[340];
            telemetry_csv_row(&frame, row, sizeof(row));
            sdlog_write(row);
        }

        if (now - last_telemetry >= TELEMETRY_PERIOD_MS) {   /* 1 Hz downlink */
            last_telemetry = now;
            frame.seq++;
            frame.rad_cps = (uint16_t)rad_counter_latch();
            frame.ok_rad  = rad_counter_active();
            char pkt[256];
            int n = telemetry_json(&frame, pkt, sizeof(pkt));
            if (n > 0 && n < 256) {
                if (lora_ok && lora_send((const uint8_t *)pkt, (uint8_t)n) != 0)
                    printf("[LORA] still busy, packet %lu skipped\r\n",
                           (unsigned long)frame.seq);
                printf("%s\r\n", pkt);   /* same packet on the bench console */
            }
        }

        /* recovery for sensors that dropped off the bus (cracked joint,
           brown-out): every 2 s give them another shot */
        if (now - last_retry >= 2000U) {
            last_retry = now;
            if (!frame.ok_bmp) frame.ok_bmp = (bmp280_init(&hi2c1) == 0);
            if (!frame.ok_bmi) frame.ok_bmi = (bmi270_init(&hi2c1) == 0);
            if (!frame.ok_mlx) frame.ok_mlx = (mlx90395_init(&hi2c1) == 0);
            if (!frame.ok_sht) frame.ok_sht = (sht45_init(&hi2c1) == 0);
        }
    }
}

static void sample_all(void)
{
    uint32_t now = HAL_GetTick();
    frame.t_ms = now;

    if (frame.ok_bmp) {
        bmp280_data_t b;  bmp280_read(&hi2c1, &b);
        frame.temp_c       = b.temp_c;
        frame.pressure_hpa = b.pressure_hpa;
        frame.altitude_m   = b.altitude_m;
    }

    if (frame.ok_ltr) {
        ltr390_data_t u;  ltr390_read(&hi2c1, &u);
        frame.uv_raw   = u.uv_raw;
        frame.uv_index = u.uv_index;
    }

    if (frame.ok_tsl) {
        tsl2591_data_t l;  tsl2591_read(&hi2c1, &l);
        frame.ir_counts   = l.ir_counts;
        frame.full_counts = l.full_counts;
        frame.lux         = l.lux;
    }

    if (frame.ok_scd) {
        scd30_data_t s;
        if (scd30_read(&hi2c1, &s) > 0) frame.co2_ppm = s.co2_ppm;   /* new every 2 s */
    }

    if (frame.ok_bmi) {
        bmi270_data_t m;
        if (bmi270_read(&hi2c1, &m) > 0) {
            frame.acc_x = m.acc_x;  frame.acc_y = m.acc_y;  frame.acc_z = m.acc_z;
            frame.gyr_x = m.gyr_x;  frame.gyr_y = m.gyr_y;  frame.gyr_z = m.gyr_z;
        }
    }

    if (frame.ok_mlx) {
        mlx90395_data_t m;
        if (mlx90395_read(&hi2c1, &m) > 0) {
            frame.mag_x = m.mag_x;  frame.mag_y = m.mag_y;  frame.mag_z = m.mag_z;
        }
    }

    /* SHT45 single-shot: trigger on this tick, read on the next (100 ms
       later), so the 8.3 ms conversion never blocks the loop */
    if (frame.ok_sht) {
        if (now - last_sht_trigger >= 1000U) {
            last_sht_trigger = now;
            sht45_trigger(&hi2c1);
            sht_pending = 1;
        } else if (sht_pending) {
            sht45_data_t h;
            if (sht45_read(&hi2c1, &h) == 1) {
                frame.hum_pct    = h.rh_pct;
                frame.hum_temp_c = h.temp_c;
            }
            sht_pending = 0;
        }
    }

    gps_data_t g;  gps_poll(&g);
    frame.ok_gps    = (g.sentences > 0);   /* bytes arriving = wired right */
    frame.gps_fix   = g.fix;
    frame.gps_sats  = g.sats;
    if (g.fix) {
        frame.gps_lat   = g.lat_deg;
        frame.gps_lon   = g.lon_deg;
        frame.gps_alt_m = g.alt_m;
    }

    /* O3-B4 via potentiostat: working minus auxiliary electrode, mV -> ppb */
    float work_mv = adc_to_v(adc_read_ch(O3_WORK_CHANNEL)) * 1000.0f;
    float aux_mv  = adc_to_v(adc_read_ch(O3_AUX_CHANNEL)) * 1000.0f;
    frame.o3_ppb  = ((work_mv - aux_mv) - O3_ZERO_MV) / O3_MV_PER_PPB;

    frame.vbat_v = adc_to_v(adc_read_ch(VBAT_CHANNEL)) * VBAT_DIVIDER;
}
