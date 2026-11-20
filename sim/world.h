/* Flight-simulator world: what the firmware sees out of the window.
 *
 * The firmware in src/ runs unmodified.  This module owns time: it installs a
 * tick hook that (a) solves a closed-form vertical flight profile, (b) pushes
 * that state into the device fakes, (c) raises the radiation pulses and the
 * 1 Hz NMEA stream, and (d) exits the process two seconds after landing,
 * because the real superloop never returns.
 *
 * Profile: 5 s on the pad, a 3 s boost sized so apogee is 750 m, ballistic
 * coast to apogee, then a 10 m/s parachute descent to the ground.
 */
#ifndef SIM_WORLD_H
#define SIM_WORLD_H

typedef enum {
    SIM_PAD = 0,
    SIM_BOOST,
    SIM_COAST,
    SIM_DESCENT,
    SIM_LANDED,
} sim_phase_t;

typedef struct {
    double      t_s;
    double      alt_m;
    double      vel_mps;      /* + up */
    double      accel_g;      /* specific force along +Z, in g */
    double      temp_c;
    double      pressure_hpa;
    double      rh_pct;
    sim_phase_t phase;
} sim_state_t;

/* Called automatically (constructor) before main(); safe to call twice. */
void sim_init(void);

const sim_state_t *sim_state(void);
double sim_apogee_time(void);
double sim_landing_time(void);

/* Output accounting, filled in by the sd/LoRa sinks. */
void sim_count_row(void);
void sim_count_packet(void);
unsigned long sim_rows(void);
unsigned long sim_packets(void);

#endif /* SIM_WORLD_H */
