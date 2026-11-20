/* Scriptable I2C/SPI device models.
 *
 * Each chip is modelled as the register file the driver already expects, not
 * as a reimplementation of the driver: writes move a register pointer, reads
 * return bytes from it.  A few chips need real protocol state (SHT45 needs a
 * measurement delay before it ACKs, MLX90395 gates data on status bits) and
 * that is the point - those are the paths the
 * firmware cannot test any other way.
 *
 * The models are shared by the Unity suites (included into a suite's TU) and
 * by the flight simulator (linked as separate objects).
 */
#ifndef FAKE_DEVICES_H
#define FAKE_DEVICES_H

#include <stdint.h>
#include <stddef.h>
#include "fake_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forget every model. */
void fdev_reset(void);

/* Raw register access, for assertions ("did the driver program CTRL=0x57?"). */
uint8_t fdev_reg(uint8_t addr7, uint8_t reg);
void    fdev_set_reg(uint8_t addr7, uint8_t reg, uint8_t val);

/* ------------------------------------------------------------------ BMP581 */

void fdev_bmp581_enable(void);
void fdev_bmp581_disable(void);
/* Drive the calibrated 24-bit data registers from an environment. */
void fdev_bmp581_set_env(float temp_c, float pressure_hpa);
/* Register file, as the driver left it (for assertions). */
uint8_t fdev_bmp581_reg(uint8_t reg);
void    fdev_bmp581_set_reg(uint8_t reg, uint8_t val);

/* ------------------------------------------------------------------- SCD30 */

void fdev_scd30_enable(void);
void fdev_scd30_disable(void);
void fdev_scd30_set(float co2_ppm, float temp_c, float rh_pct);
void fdev_scd30_set_ready(int ready);
/* Corrupt the measurement CRCs, to exercise the driver's -3 path. */
void fdev_scd30_corrupt_crc(int corrupt);
/* Last 16-bit command byte pair seen, and whether continuous mode is on. */
uint16_t fdev_scd30_last_cmd(void);
int      fdev_scd30_running(void);

/* ------------------------------------------------------------------- SHT45 */

void fdev_sht45_enable(void);
void fdev_sht45_disable(void);
void fdev_sht45_set(float temp_c, float rh_pct);
/* Measurement delay enforced by the model, 0 disables (default 10 ms). */
void fdev_sht45_set_delay_ms(uint32_t ms);
/* A measurement is armed and not yet read. */
int  fdev_sht45_pending(void);
/* Return bad CRCs on the next read, to exercise the driver's reject path. */
void fdev_sht45_corrupt_crc(int corrupt);

/* ---------------------------------------------------------------- MLX90395 */

void fdev_mlx90395_enable(void);
void fdev_mlx90395_disable(void);
void fdev_mlx90395_set(float x_ut, float y_ut, float z_ut);
/* Override the DRDY bit of a burst read (-1 = automatic). */
void fdev_mlx90395_set_drdy(int drdy);
/* Override the status byte returned for every command (-1 = automatic). */
void fdev_mlx90395_set_status(int status);
/* Make the 12-byte burst read NACK, to exercise the driver's -1 path. */
void fdev_mlx90395_fail_burst(int fail);

/* ------------------------------------------------------------------ LTR390 */

void fdev_ltr390_enable(void);
void fdev_ltr390_disable(void);
void fdev_ltr390_set_uvs(uint32_t raw18);

/* ----------------------------------------------------------------- TSL2591 */

void fdev_tsl2591_enable(void);
void fdev_tsl2591_disable(void);
void fdev_tsl2591_set_counts(uint16_t ch0, uint16_t ch1);

/* --------------------------------------------------------------- SPI bus */

/* Put every sensor chip select high, so the fake SPI mux has one owner. */
void fdev_spi_cs_idle(void);

#ifdef __cplusplus
}
#endif
#endif /* FAKE_DEVICES_H */
