#include <stdio.h>
#include "board.h"
#include "config.h"
#include "telemetry.h"
#include "i2c_dev.h"
#include "bmp581.h"
#include "lsm6dsv.h"
#include "ltr390.h"
#include "tsl2591.h"
#include "scd30.h"
#include "mlx90395.h"
#include "sht45.h"
#include "rad_counter.h"
#include "gps_sam_m8q.h"
#include "radio_e22.h"
#include "sdlog.h"
#include "servos.h"
#include "adc_read.h"

static flight_frame_t frame;
static uint32_t last_sample, last_telemetry, last_retry;

static void sample_all(void);

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
    printf(found ? "  (expect 0x29 0x44 0x53 0x61)\r\n"
                 : "  NONE: check SDA/SCL, pull-ups, 3.0 V and GND\r\n");
}

int main(void)
{
    board_init();
    servos_start();
    printf("\r\n[CELESTIUM] booting\r\n");

    i2c_scan();
    frame.ok_bmp = (bmp581_init(&SENSOR_SPI) == 0);
    frame.ok_ltr = (ltr390_init(&hi2c1) == 0);
    frame.ok_tsl = (tsl2591_init(&hi2c1) == 0);
    frame.ok_scd = (scd30_init(&hi2c1) == 0);
    frame.ok_imu = (lsm6dsv_init(&SENSOR_SPI) == 0);
    frame.ok_mlx = (mlx90395_init(&SENSOR_SPI) == 0);
    frame.ok_sht = (sht45_init(&hi2c1) == 0);
    rad_counter_start();
    gps_init(&huart1);

    int radio_ok = (e22_init(&E22_UART) == 0);
    sdlog_init();
    frame.ok_sd = sdlog_ok();
    if (frame.ok_sd) sdlog_write(telemetry_csv_header());

    printf("[CELESTIUM] baro=%u ltr=%u tsl=%u scd=%u imu=%u mlx=%u sht=%u "
           "radio=%u sd=%u\r\n",
           frame.ok_bmp, frame.ok_ltr, frame.ok_tsl, frame.ok_scd,
           frame.ok_imu, frame.ok_mlx, frame.ok_sht, radio_ok, frame.ok_sd);
    if (!frame.ok_imu)
        printf("[LSM6DSV] missing: check CS PB0 and SPI1 mode 3\r\n");
    if (!radio_ok) printf("[E22] UART init failed: check USART3 PC10/PC11\r\n");
    if (!frame.ok_bmp) printf("[BMP581] missing: primary mission degraded, will keep retrying\r\n");

    while (1) {
        uint32_t now = HAL_GetTick();

        e22_busy();

        if (now - last_sample >= SAMPLE_PERIOD_MS) {
            last_sample = now;
            sample_all();

            char row[340];
            telemetry_csv_row(&frame, row, sizeof(row));
            sdlog_write(row);
        }

        if (now - last_telemetry >= TELEMETRY_PERIOD_MS) {
            last_telemetry = now;
            frame.seq++;
            frame.rad_cps = (uint16_t)rad_counter_latch();
            frame.ok_rad  = rad_counter_active();
            char pkt[256];
            int n = telemetry_json(&frame, pkt, sizeof(pkt));
            if (n > 0 && n < 256) {
                if (radio_ok && e22_send((const uint8_t *)pkt, (uint16_t)n) != 0)
                    printf("[E22] packet %lu failed\r\n",
                           (unsigned long)frame.seq);
                printf("%s\r\n", pkt);
            }
        }

        if (now - last_retry >= 2000U) {
            last_retry = now;
            if (!frame.ok_bmp) frame.ok_bmp = (bmp581_init(&SENSOR_SPI) == 0);
            if (!frame.ok_imu) frame.ok_imu = (lsm6dsv_init(&SENSOR_SPI) == 0);
            if (!frame.ok_mlx) frame.ok_mlx = (mlx90395_init(&SENSOR_SPI) == 0);
            if (!frame.ok_sht) frame.ok_sht = (sht45_init(&hi2c1) == 0);
        }
    }
}

static void sample_all(void)
{
    frame.t_ms = HAL_GetTick();

    if (frame.ok_bmp) {
        bmp581_data_t b;  bmp581_read(&SENSOR_SPI, &b);
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
        if (scd30_read(&hi2c1, &s) > 0) frame.co2_ppm = s.co2_ppm;
    }

    if (frame.ok_imu) {
        lsm6dsv_data_t m;
        if (lsm6dsv_read(&SENSOR_SPI, &m) > 0) {
            frame.acc_x = m.acc_x;  frame.acc_y = m.acc_y;  frame.acc_z = m.acc_z;
            frame.gyr_x = m.gyr_x;  frame.gyr_y = m.gyr_y;  frame.gyr_z = m.gyr_z;
        }
    }

    if (frame.ok_mlx) {
        mlx90395_data_t m;
        if (mlx90395_read(&SENSOR_SPI, &m) > 0) {
            frame.mag_x = m.mag_x;  frame.mag_y = m.mag_y;  frame.mag_z = m.mag_z;
        }
    }

    if (frame.ok_sht) {
        sht45_data_t h;
        if (sht45_poll(&hi2c1, &h) == 1) {
            frame.rh_pct    = h.rh_pct;
            frame.sht_temp_c = h.temp_c;
        }
    }

    gps_data_t g;  gps_poll(&g);
    frame.ok_gps    = (g.sentences > 0);
    frame.gps_fix   = g.fix;
    frame.gps_sats  = g.sats;
    if (g.fix) {
        frame.gps_lat   = g.lat_deg;
        frame.gps_lon   = g.lon_deg;
        frame.gps_alt_m = g.alt_m;
    }

    float work_mv = adc_to_v(adc_read_ch(O3_WORK_CHANNEL)) * 1000.0f;
    float aux_mv  = adc_to_v(adc_read_ch(O3_AUX_CHANNEL)) * 1000.0f;
    frame.o3_ppb  = ((work_mv - aux_mv) - O3_ZERO_MV) / O3_MV_PER_PPB;

    frame.vbat_v = adc_to_v(adc_read_ch(VBAT_CHANNEL)) * VBAT_DIVIDER;
}
