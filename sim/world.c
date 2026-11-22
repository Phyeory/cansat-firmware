#include "world.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fake_hal.h"
#include "fake_devices.h"
#include "config.h"
#include "board.h"

#define GRAVITY          9.80665
#define PAD_S            5.0
#define BOOST_S          3.0
#define TARGET_APOGEE_M  750.0
#define DESCENT_MPS      10.0
#define SETTLE_S         2.0     /* run-on after touchdown before exiting */

#define GPS_LAT          48.1173
#define GPS_LON          11.5167
#define SEA_LEVEL_HPA    1013.25

void USART1_IRQHandler(void);    /* the firmware's own ISR, fed byte by byte */

/* ------------------------------------------------------------- profile --- */

static double boost_accel;       /* m/s^2, net (thrust - weight)             */
static double burn_speed, burn_alt;
static double apogee_alt, apogee_t, landing_t;

static sim_state_t st;
static uint32_t last_env_ms, last_gps_ms, last_rad_ms;
static int      initialised, finished, last_phase = -1;
static unsigned long rows, packets;

/* --------------------------------------------------------------- sinks --- */

static FILE *lora_file;

/* The E22 is a UART pipe: capture only USART3, not the debug console. */
static void uart_sink(UART_HandleTypeDef *h, const uint8_t *data, uint16_t len)
{
    if (h != &huart3) return;
    if (!lora_file) {
        char path[512];
        snprintf(path, sizeof(path), "%s/lora.jsonl", SIM_OUT_DIR);
        lora_file = fopen(path, "w");
        if (!lora_file) return;
    }
    fwrite(data, 1, len, lora_file);
    fputc('\n', lora_file);
    fflush(lora_file);
    packets++;
}

unsigned long sim_rows(void)    { return rows; }
unsigned long sim_packets(void) { return packets; }

void sim_count_row(void)    { rows++; }
void sim_count_packet(void) { packets++; }

/* ---------------------------------------------------------- flight model --- */

/* Solve  H = 0.5*a*T^2 + (a*T)^2/(2g)  for the boost acceleration a. */
static double solve_boost_accel(double target_alt, double burn_s)
{
    double a = burn_s * burn_s / (2.0 * GRAVITY);
    double b = burn_s * burn_s / 2.0;
    return (-b + sqrt(b * b + 4.0 * a * target_alt)) / (2.0 * a);
}

static void update_state(double t)
{
    st.t_s = t;

    if (t < PAD_S) {
        st.alt_m = 0.0;
        st.vel_mps = 0.0;
        st.accel_g = 1.0;
        st.phase = SIM_PAD;
    } else if (t < PAD_S + BOOST_S) {
        double tau = t - PAD_S;
        st.alt_m = 0.5 * boost_accel * tau * tau;
        st.vel_mps = boost_accel * tau;
        st.accel_g = (boost_accel + GRAVITY) / GRAVITY;
        st.phase = SIM_BOOST;
    } else if (t < apogee_t) {
        double tau = t - (PAD_S + BOOST_S);
        st.alt_m = burn_alt + burn_speed * tau - 0.5 * GRAVITY * tau * tau;
        st.vel_mps = burn_speed - GRAVITY * tau;
        st.accel_g = 0.0;                       /* free fall */
        st.phase = SIM_COAST;
    } else if (t < landing_t) {
        st.alt_m = apogee_alt - DESCENT_MPS * (t - apogee_t);
        st.vel_mps = -DESCENT_MPS;
        st.accel_g = 1.0;                       /* terminal velocity */
        st.phase = SIM_DESCENT;
    } else {
        st.alt_m = 0.0;
        st.vel_mps = 0.0;
        st.accel_g = 1.0;
        st.phase = SIM_LANDED;
    }

    st.temp_c = 15.0 - 0.0065 * st.alt_m;      /* ISA lapse rate */
    st.pressure_hpa = SEA_LEVEL_HPA *
        pow(1.0 - 2.25577e-5 * st.alt_m, 5.25588);
    st.rh_pct = 45.0 + 0.02 * st.alt_m;
}

static const char *phase_name(sim_phase_t p)
{
    switch (p) {
    case SIM_PAD:     return "PAD";
    case SIM_BOOST:   return "BOOST";
    case SIM_COAST:   return "COAST";
    case SIM_DESCENT: return "DESCENT";
    default:          return "LANDED";
    }
}

/* ------------------------------------------------------------ devices --- */

static uint16_t mv_to_raw(double mv)
{
    long raw = lrint(mv / 1000.0 / VREF_V * ADC_MAX);
    if (raw < 0) raw = 0;
    if (raw > 4095) raw = 4095;
    return (uint16_t)raw;
}

static void drive_devices(void)
{
    double alt = st.alt_m;
    double gx = 0.0, gy = 0.0, gz = 0.0;

    if (st.phase == SIM_BOOST) { gx = 12.0; gy = -4.0; gz = 3.0; }

    fdev_bmp581_set_env((float)st.temp_c, (float)st.pressure_hpa);
    fdev_lsm6dsv_set_motion(0.0f, 0.0f, (float)(st.accel_g * GRAVITY),
                            (float)gx, (float)gy, (float)gz);
    fdev_sht45_set((float)(st.temp_c + 0.8), (float)st.rh_pct);
    fdev_scd30_set(421.0f, (float)st.temp_c, (float)st.rh_pct);
    fdev_ltr390_set_uvs((uint32_t)(2000.0 + 40.0 * alt));
    fdev_tsl2591_set_counts((uint16_t)(20000.0 + 8.0 * alt),
                            (uint16_t)(5000.0 + 2.0 * alt));
    fdev_mlx90395_set(22.0f, -5.5f, 41.0f);

    fake_adc_set(O3_WORK_CHANNEL, mv_to_raw(300.0 + 0.05 * alt));
    fake_adc_set(O3_AUX_CHANNEL,  mv_to_raw(290.0));
    fake_adc_set(VBAT_CHANNEL,    mv_to_raw(2010.0));   /* 4.02 V pack */
}

/* ---------------------------------------------------------------- GPS --- */

static void nmea_send(const char *body)
{
    char line[160];
    uint8_t cs = 0;
    for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
    int n = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, cs);
    if (n > 0 && n < (int)sizeof(line))
        fake_uart_feed((const uint8_t *)line, (size_t)n);
}

static void emit_gps(double t)
{
    char body[128];
    int hh = (int)(t / 3600.0) % 24;
    int mm = (int)(t / 60.0) % 60;
    int ss = (int)t % 60;

    /* $GPGGA: fix quality 1, 9 satellites, altitude from the world model. */
    snprintf(body, sizeof(body),
             "GPGGA,%02d%02d%02d.00,4807.038,N,01131.000,E,1,09,0.9,%.1f,M,47.0,M,,",
             hh, mm, ss, st.alt_m);
    nmea_send(body);

    /* $GPRMC: status A (valid) so gps_data_t.fix goes high. */
    snprintf(body, sizeof(body),
             "GPRMC,%02d%02d%02d.00,A,4807.038,N,01131.000,E,%.2f,%.1f,010125,,,A",
             hh, mm, ss, fabs(st.vel_mps), st.vel_mps >= 0.0 ? 0.0 : 180.0);
    nmea_send(body);
}

/* ---------------------------------------------------------------- tick --- */

static void finish(void)
{
    if (finished) return;
    finished = 1;

    if (lora_file) { fflush(lora_file); fclose(lora_file); lora_file = NULL; }
    fflush(NULL);

    printf("[SIM] landed: apogee %.1f m at %.2f s, touchdown %.2f s\n",
           apogee_alt, apogee_t, landing_t);
    printf("[SIM] %lu csv rows, %lu lora packets, %.2f s simulated\n",
           rows, packets, st.t_s);
    fflush(stdout);
    exit(0);
}

static void sim_tick(void)
{
    uint32_t now = fake_tick();
    double t = now / 1000.0;

    update_state(t);

    if ((int)st.phase != last_phase) {
        last_phase = (int)st.phase;
        printf("[SIM] %6.2f s  %-7s alt=%7.1f m  vel=%7.1f m/s  %.1f hPa\n",
               t, phase_name(st.phase), st.alt_m, st.vel_mps, st.pressure_hpa);
        fflush(stdout);
    }

    if (now - last_env_ms >= 10U) {
        last_env_ms = now;
        drive_devices();
    }

    while (now - last_rad_ms >= 250U) {
        last_rad_ms += 250U;
        fake_gpio_exti_pulse(RAD_INPUT_PIN);      /* 4 counts/s */
    }

    if (now - last_gps_ms >= 1000U) {
        last_gps_ms += 1000U;
        emit_gps(t);
        fdev_scd30_set_ready(1);                  /* 1 Hz "new measurement" */
    }

    if (now >= (uint32_t)((landing_t + SETTLE_S) * 1000.0))
        finish();
}

/* --------------------------------------------------------------- setup --- */

void sim_init(void)
{
    if (initialised) return;
    initialised = 1;

    boost_accel = solve_boost_accel(TARGET_APOGEE_M, BOOST_S);
    burn_speed = boost_accel * BOOST_S;
    burn_alt = 0.5 * boost_accel * BOOST_S * BOOST_S;
    apogee_alt = burn_alt + burn_speed * burn_speed / (2.0 * GRAVITY);
    apogee_t = PAD_S + BOOST_S + burn_speed / GRAVITY;
    landing_t = apogee_t + apogee_alt / DESCENT_MPS;

    fake_hal_reset();
    fdev_reset();

    fdev_bmp581_enable();
    fdev_lsm6dsv_enable();
    fdev_scd30_enable();
    fdev_sht45_enable();
    fdev_mlx90395_enable();
    fdev_ltr390_enable();
    fdev_tsl2591_enable();
    fake_uart_set_tx_hook(uart_sink);

    fake_uart_set_rx_handler(USART1_IRQHandler);
    fake_hal_set_tick_step(1);          /* every HAL_GetTick() advances 1 ms */
    fake_hal_set_tick_hook(sim_tick);

    update_state(0.0);
}

__attribute__((constructor)) static void sim_ctor(void)
{
    sim_init();
}

const sim_state_t *sim_state(void) { return &st; }
double sim_apogee_time(void) { return apogee_t; }
double sim_landing_time(void) { return landing_t; }
