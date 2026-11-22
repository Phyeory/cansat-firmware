/* Device models: one small register-level chip each.
 *
 * Layout convention: every I2C model begins with a `regfile_t rf` so the
 * generic fdev_reg()/fdev_set_reg() accessors can peek at any of them.
 */
#include "fake_devices.h"

#include <math.h>
#include <string.h>

#include "config.h"

static HAL_StatusTypeDef fdev_spi_dispatch(SPI_HandleTypeDef *h,
                                           uint8_t *tx, uint8_t *rx, uint16_t n);

/* ------------------------------------------------------------- reg file */

typedef struct {
    uint8_t regs[256];
    uint8_t ptr;
    uint8_t addr_mask;
} regfile_t;

static HAL_StatusTypeDef rf_write(fake_i2c_dev_t *d, const uint8_t *buf, uint16_t len)
{
    regfile_t *rf = (regfile_t *)d->ctx;
    if (len < 1) return HAL_ERROR;
    rf->ptr = (uint8_t)(buf[0] & rf->addr_mask);
    for (uint16_t i = 1; i < len; i++)
        rf->regs[rf->ptr++] = buf[i];
    return HAL_OK;
}

static HAL_StatusTypeDef rf_read(fake_i2c_dev_t *d, uint8_t *buf, uint16_t len)
{
    regfile_t *rf = (regfile_t *)d->ctx;
    for (uint16_t i = 0; i < len; i++)
        buf[i] = rf->regs[rf->ptr++];
    return HAL_OK;
}

/* Sensirion CRC-8, same polynomial as i2c_dev.c (kept private so this file
   has no link dependency on the firmware). */
static uint8_t crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

static void rf_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

uint8_t fdev_reg(uint8_t addr7, uint8_t reg)
{
    fake_i2c_dev_t *d = fake_i2c_device(addr7);
    if (!d->present || !d->ctx) return 0;
    regfile_t *rf = (regfile_t *)d->ctx;
    return rf->regs[reg];
}

void fdev_set_reg(uint8_t addr7, uint8_t reg, uint8_t val)
{
    fake_i2c_dev_t *d = fake_i2c_device(addr7);
    if (!d->present || !d->ctx) return;
    regfile_t *rf = (regfile_t *)d->ctx;
    rf->regs[reg] = val;
}

/* ================================================================= BMP581 */

typedef struct {
    uint8_t regs[256];
    int     present;
} bmp581_t;

static bmp581_t bmp581;

uint8_t fdev_bmp581_reg(uint8_t reg) { return bmp581.regs[reg]; }
void fdev_bmp581_set_reg(uint8_t reg, uint8_t val) { bmp581.regs[reg] = val; }

void fdev_bmp581_set_env(float temp_c, float pressure_hpa)
{
    int32_t  t_raw = (int32_t)lrintf(temp_c * 65536.0f);
    uint32_t p_raw = (uint32_t)lrintf(pressure_hpa * 100.0f * 64.0f);

    bmp581.regs[0x1D] = (uint8_t)(t_raw & 0xFF);
    bmp581.regs[0x1E] = (uint8_t)((t_raw >> 8) & 0xFF);
    bmp581.regs[0x1F] = (uint8_t)((t_raw >> 16) & 0xFF);
    bmp581.regs[0x20] = (uint8_t)(p_raw & 0xFF);
    bmp581.regs[0x21] = (uint8_t)((p_raw >> 8) & 0xFF);
    bmp581.regs[0x22] = (uint8_t)((p_raw >> 16) & 0xFF);
}

static HAL_StatusTypeDef bmp581_spi(uint8_t *tx, uint8_t *rx, uint16_t n)
{
    if (!bmp581.present) return HAL_ERROR;
    if (n < 1) return HAL_ERROR;

    uint8_t addr = (uint8_t)(tx[0] & 0x7F);
    if (tx[0] & 0x80) {
        if (rx) {
            rx[0] = 0;
            for (uint16_t i = 1; i < n; i++)
                rx[i] = bmp581.regs[(uint8_t)(addr + i - 1)];
        }
    } else {
        if (n >= 2) bmp581.regs[addr] = tx[1];
        if (rx) memset(rx, 0, n);
    }
    return HAL_OK;
}

void fdev_bmp581_enable(void)
{
    memset(&bmp581, 0, sizeof(bmp581));
    bmp581.present = 1;
    bmp581.regs[0x01] = 0x50;          /* CHIP_ID */
    fdev_bmp581_set_env(25.0f, 1013.25f);
    fake_spi_set_handler(fdev_spi_dispatch);
}

void fdev_bmp581_disable(void) { bmp581.present = 0; }

/* ================================================================ LSM6DSV */

typedef struct {
    uint8_t regs[256];
    int     present;
} lsm_t;

static lsm_t lsm;

void fdev_lsm6dsv_set_motion(float ax, float ay, float az,
                             float gx, float gy, float gz)
{
    int16_t raw[6];
    raw[0] = (int16_t)lrintf(ax / 9.80665f * 4096.0f);
    raw[1] = (int16_t)lrintf(ay / 9.80665f * 4096.0f);
    raw[2] = (int16_t)lrintf(az / 9.80665f * 4096.0f);
    raw[3] = (int16_t)lrintf(gx * 14.285714f);
    raw[4] = (int16_t)lrintf(gy * 14.285714f);
    raw[5] = (int16_t)lrintf(gz * 14.285714f);

    /* 0x22..0x27 = gyro, 0x28..0x2D = accel. */
    for (int i = 0; i < 3; i++) {
        lsm.regs[0x22 + 2 * i]     = (uint8_t)(raw[3 + i] & 0xFF);
        lsm.regs[0x22 + 2 * i + 1] = (uint8_t)((uint16_t)raw[3 + i] >> 8);
        lsm.regs[0x28 + 2 * i]     = (uint8_t)(raw[i] & 0xFF);
        lsm.regs[0x28 + 2 * i + 1] = (uint8_t)((uint16_t)raw[i] >> 8);
    }
}

uint8_t fdev_lsm6dsv_reg(uint8_t reg) { return lsm.regs[reg]; }
void fdev_lsm6dsv_set_reg(uint8_t reg, uint8_t val) { lsm.regs[reg] = val; }

static HAL_StatusTypeDef lsm_spi(uint8_t *tx, uint8_t *rx, uint16_t n)
{
    if (!lsm.present || n < 1) return HAL_ERROR;

    uint8_t addr = (uint8_t)(tx[0] & 0x7F);
    if (tx[0] & 0x80) {
        if (rx) {
            rx[0] = 0;
            for (uint16_t i = 1; i < n; i++)
                rx[i] = lsm.regs[(uint8_t)(addr + i - 1)];
        }
    } else {
        if (n >= 2) {
            for (uint16_t i = 1; i < n; i++)
                lsm.regs[(uint8_t)(addr + i - 1)] = tx[i];
        }
        if (rx) memset(rx, 0, n);
    }
    return HAL_OK;
}

void fdev_lsm6dsv_enable(void)
{
    memset(&lsm, 0, sizeof(lsm));
    lsm.present = 1;
    lsm.regs[0x0F] = 0x70;             /* WHO_AM_I */
    fdev_lsm6dsv_set_motion(0.0f, 0.0f, 9.80665f, 0.0f, 0.0f, 0.0f);
    fake_spi_set_handler(fdev_spi_dispatch);
}

void fdev_lsm6dsv_disable(void) { lsm.present = 0; }

/* ================================================================== SCD30 */

typedef struct {
    regfile_t rf;
    int       running;
    int       ready;
    uint8_t   last_cmd_hi, last_cmd_lo;
    float     co2, temp, rh;
    uint32_t  next_ready;
    int       corrupt;
} scd_t;

static scd_t scd;

void fdev_scd30_set(float co2_ppm, float temp_c, float rh_pct)
{
    scd.co2 = co2_ppm;
    scd.temp = temp_c;
    scd.rh = rh_pct;
}

void fdev_scd30_set_ready(int ready) { scd.ready = ready ? 1 : 0; }

static HAL_StatusTypeDef scd_write(fake_i2c_dev_t *d, const uint8_t *buf, uint16_t len)
{
    if (len < 2) return HAL_ERROR;
    uint16_t cmd = (uint16_t)((buf[0] << 8) | buf[1]);
    scd.last_cmd_hi = buf[0];
    scd.last_cmd_lo = buf[1];

    if (len >= 5) {
        /* argument word + CRC: a wrong CRC is a NACK on real silicon */
        if (crc8(&buf[2], 2) != buf[4]) return HAL_ERROR;
    }

    switch (cmd) {
    case 0x0010:                                  /* start continuous */
        scd.running = 1;
        scd.next_ready = fake_tick() + 2000u;
        break;
    case 0x0104:                                  /* stop */
        scd.running = 0;
        scd.ready = 0;
        break;
    case 0x0202:                                  /* data ready */
    case 0x0300:                                  /* read measurement */
    case 0x5306:                                  /* ASC on */
    case 0xD304:                                  /* soft reset */
        break;
    default:
        return HAL_ERROR;                         /* unsupported command */
    }
    (void)d;
    return HAL_OK;
}

static HAL_StatusTypeDef scd_read(fake_i2c_dev_t *d, uint8_t *buf, uint16_t len)
{
    uint16_t cmd = (uint16_t)((scd.last_cmd_hi << 8) | scd.last_cmd_lo);
    (void)d;

    if (cmd == 0x0202) {
        uint16_t v = scd.ready ? 1 : 0;
        rf_be16(buf, v);
        buf[2] = crc8(buf, 2);
        return len == 3 ? HAL_OK : HAL_ERROR;
    }

    if (cmd == 0x0300) {
        if (len != 18) return HAL_ERROR;
        float vals[3] = { scd.co2, scd.temp, scd.rh };
        for (int i = 0; i < 3; i++) {
            uint32_t bits;
            memcpy(&bits, &vals[i], 4);
            uint8_t *w = &buf[i * 6];
            w[0] = (uint8_t)(bits >> 24);
            w[1] = (uint8_t)(bits >> 16);
            w[2] = crc8(w, 2);
            w[3] = (uint8_t)(bits >> 8);
            w[4] = (uint8_t)bits;
            w[5] = crc8(w + 3, 2);
            if (scd.corrupt) {
                w[2] ^= 0xFF;
                w[5] ^= 0xFF;
            }
        }
        scd.ready = 0;
        return HAL_OK;
    }

    memset(buf, 0, len);
    return HAL_ERROR;
}

void fdev_scd30_enable(void)
{
    memset(&scd, 0, sizeof(scd));
    scd.rf.addr_mask = 0xFF;
    fdev_scd30_set(420.0f, 24.0f, 50.0f);

    fake_i2c_dev_t *d = fake_i2c_device(0x61);
    d->present = 1;
    d->ctx = &scd;
    d->on_write = scd_write;
    d->on_read = scd_read;
}

void fdev_scd30_disable(void)
{
    fake_i2c_device(0x61)->present = 0;
}

void fdev_scd30_corrupt_crc(int corrupt) { scd.corrupt = corrupt ? 1 : 0; }

uint16_t fdev_scd30_last_cmd(void)
{
    return (uint16_t)((scd.last_cmd_hi << 8) | scd.last_cmd_lo);
}

int fdev_scd30_running(void) { return scd.running; }

/* ------------------------------------------------------------------ SHT45 */

typedef struct {
    regfile_t rf;
    float     temp, rh;
    float     latched_t, latched_rh;
    uint32_t  ready_at;
    uint32_t  delay_ms;
    int       pending;
    int       corrupt;
} sht_t;

static sht_t sht;

void fdev_sht45_set(float temp_c, float rh_pct)
{
    sht.temp = temp_c;
    sht.rh = rh_pct;
}

void fdev_sht45_set_delay_ms(uint32_t ms) { sht.delay_ms = ms; }

static HAL_StatusTypeDef sht_write(fake_i2c_dev_t *d, const uint8_t *buf, uint16_t len)
{
    (void)d;
    if (len != 1) return HAL_ERROR;
    if (buf[0] == 0x94) {              /* soft reset */
        sht.pending = 0;
        sht.corrupt = 0;
        return HAL_OK;
    }
    if (buf[0] == 0xFD) {              /* measure T/RH, high repeatability */
        sht.latched_t = sht.temp;
        sht.latched_rh = sht.rh;
        sht.ready_at = fake_tick() + sht.delay_ms;
        sht.pending = 1;
        return HAL_OK;
    }
    return HAL_ERROR;
}

static HAL_StatusTypeDef sht_read(fake_i2c_dev_t *d, uint8_t *buf, uint16_t len)
{
    (void)d;
    if (len != 6) return HAL_ERROR;
    if (!sht.pending || (int32_t)(fake_tick() - sht.ready_at) < 0)
        return HAL_ERROR;              /* measurement not finished -> NACK */

    float t = sht.latched_t < -45.0f ? -45.0f : sht.latched_t;
    float rh = sht.latched_rh < 0.0f ? 0.0f : (sht.latched_rh > 100.0f ? 100.0f : sht.latched_rh);
    uint16_t traw = (uint16_t)lrintf((t + 45.0f) / 175.0f * 65535.0f);
    uint16_t hraw = (uint16_t)lrintf((rh + 6.0f) / 125.0f * 65535.0f);

    rf_be16(&buf[0], traw);
    buf[2] = crc8(&buf[0], 2);
    rf_be16(&buf[3], hraw);
    buf[5] = crc8(&buf[3], 2);
    if (sht.corrupt) {
        buf[2] ^= 0xFF;
        buf[5] ^= 0xFF;
    }
    sht.pending = 0;
    return HAL_OK;
}

void fdev_sht45_enable(void)
{
    memset(&sht, 0, sizeof(sht));
    sht.rf.addr_mask = 0xFF;
    sht.delay_ms = 10;
    fdev_sht45_set(25.0f, 50.0f);

    fake_i2c_dev_t *d = fake_i2c_device(0x44);
    d->present = 1;
    d->ctx = &sht;
    d->on_write = sht_write;
    d->on_read = sht_read;
}

void fdev_sht45_disable(void)
{
    fake_i2c_device(0x44)->present = 0;
}

int fdev_sht45_pending(void) { return sht.pending; }
void fdev_sht45_corrupt_crc(int corrupt) { sht.corrupt = corrupt ? 1 : 0; }

/* ================================================================ MLX90395 */

typedef struct {
    uint8_t status;
    int16_t x, y, z;
    int     have_meas;
    int     drdy;          /* -1 = follow have_meas */
    int     force_status;  /* -1 = follow the command */
    int     fail_burst;
    int     present;
} mlx_t;

static mlx_t mlx;

void fdev_mlx90395_set(float x_ut, float y_ut, float z_ut)
{
    mlx.x = (int16_t)lrintf(x_ut / 0.714f);
    mlx.y = (int16_t)lrintf(y_ut / 0.714f);
    mlx.z = (int16_t)lrintf(z_ut / 0.714f);
    mlx.have_meas = 1;
}

static HAL_StatusTypeDef mlx_spi(uint8_t *tx, uint8_t *rx, uint16_t n)
{
    if (!mlx.present) return HAL_ERROR;
    if (n < 1) return HAL_ERROR;

    uint8_t cmd = tx[0];

    if (cmd == 0xF0) {                  /* RT: no status byte */
        mlx.status = 0x02;
        if (rx) memset(rx, 0, n);
        return HAL_OK;
    }

    if (cmd == 0x4F) {                  /* RM, XYZT */
        if (mlx.fail_burst) return HAL_ERROR;
        if (n < 11) return HAL_ERROR;
        memset(rx, 0, n);
        rx[1] = (mlx.drdy >= 0) ? (uint8_t)mlx.drdy
                                : (mlx.have_meas ? 0x01 : 0x00);
        rx[2] = 0; rx[3] = 0;           /* temperature */
        rx[4] = (uint8_t)((uint16_t)mlx.x >> 8);
        rx[5] = (uint8_t)mlx.x;
        rx[6] = (uint8_t)((uint16_t)mlx.y >> 8);
        rx[7] = (uint8_t)mlx.y;
        rx[8] = (uint8_t)((uint16_t)mlx.z >> 8);
        rx[9] = (uint8_t)mlx.z;
        return HAL_OK;
    }

    switch (cmd) {
    case 0x80: mlx.status = 0x20; break;        /* exit mode */
    case 0x3F: mlx.status = 0x20; mlx.have_meas = 1; break;  /* single XYZT */
    default:   mlx.status = 0x00; break;
    }

    if (rx) {
        memset(rx, 0, n);
        if (n >= 2)
            rx[1] = (mlx.force_status >= 0) ? (uint8_t)mlx.force_status
                                            : mlx.status;
    }
    return HAL_OK;
}

void fdev_mlx90395_enable(void)
{
    memset(&mlx, 0, sizeof(mlx));
    mlx.present = 1;
    mlx.status = 0x20;
    mlx.drdy = -1;
    mlx.force_status = -1;
    fake_spi_set_handler(fdev_spi_dispatch);
}

void fdev_mlx90395_disable(void) { mlx.present = 0; }

void fdev_mlx90395_set_drdy(int drdy) { mlx.drdy = drdy; }
void fdev_mlx90395_set_status(int status) { mlx.force_status = status; }
void fdev_mlx90395_fail_burst(int fail) { mlx.fail_burst = fail ? 1 : 0; }

/* ================================================================== LTR390 */

typedef struct {
    regfile_t rf;
} ltr_t;

static ltr_t ltr;

void fdev_ltr390_set_uvs(uint32_t raw18)
{
    ltr.rf.regs[0x10] = (uint8_t)raw18;
    ltr.rf.regs[0x11] = (uint8_t)(raw18 >> 8);
    ltr.rf.regs[0x12] = (uint8_t)(raw18 >> 16);
}

void fdev_ltr390_enable(void)
{
    memset(&ltr, 0, sizeof(ltr));
    /* The driver addresses registers without the datasheet's command bit;
       masking bit 7 accepts both conventions (see TESTING.md). */
    ltr.rf.addr_mask = 0x7F;
    ltr.rf.regs[0x06] = 0xB1;          /* PART_ID, high nibble 0xB */

    fake_i2c_dev_t *d = fake_i2c_device(0x53);
    d->present = 1;
    d->ctx = &ltr;
    d->on_write = rf_write;
    d->on_read = rf_read;
}

void fdev_ltr390_disable(void)
{
    fake_i2c_device(0x53)->present = 0;
}

/* ================================================================= TSL2591 */

typedef struct {
    regfile_t rf;
} tsl_t;

static tsl_t tsl;

void fdev_tsl2591_set_counts(uint16_t ch0, uint16_t ch1)
{
    tsl.rf.regs[0x14] = (uint8_t)ch0;
    tsl.rf.regs[0x15] = (uint8_t)(ch0 >> 8);
    tsl.rf.regs[0x16] = (uint8_t)ch1;
    tsl.rf.regs[0x17] = (uint8_t)(ch1 >> 8);
}

void fdev_tsl2591_enable(void)
{
    memset(&tsl, 0, sizeof(tsl));
    tsl.rf.addr_mask = 0x1F;           /* 0xA0 command byte + address */
    tsl.rf.regs[0x12] = 0x50;          /* ID */

    fake_i2c_dev_t *d = fake_i2c_device(0x29);
    d->present = 1;
    d->ctx = &tsl;
    d->on_write = rf_write;
    d->on_read = rf_read;
}

void fdev_tsl2591_disable(void)
{
    fake_i2c_device(0x29)->present = 0;
}

/* =============================================================== SPI mux */

static int cs_low(GPIO_TypeDef *port, uint16_t pin)
{
    return fake_gpio_get_output(port, pin) == 0;
}

static HAL_StatusTypeDef fdev_spi_dispatch(SPI_HandleTypeDef *h,
                                           uint8_t *tx, uint8_t *rx, uint16_t n)
{
    (void)h;
    if (cs_low(BMP581_CS_GPIO, BMP581_CS_PIN))     return bmp581_spi(tx, rx, n);
    if (cs_low(LSM6DSV_CS_GPIO, LSM6DSV_CS_PIN))   return lsm_spi(tx, rx, n);
    if (cs_low(MLX90395_CS_GPIO, MLX90395_CS_PIN)) return mlx_spi(tx, rx, n);
    return HAL_OK;
}

void fdev_spi_cs_idle(void)
{
    HAL_GPIO_WritePin(BMP581_CS_GPIO, BMP581_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LSM6DSV_CS_GPIO, LSM6DSV_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MLX90395_CS_GPIO, MLX90395_CS_PIN, GPIO_PIN_SET);
}

/* =================================================================== reset */

void fdev_reset(void)
{
    memset(&bmp581, 0, sizeof(bmp581));
    memset(&lsm, 0, sizeof(lsm));
    memset(&scd, 0, sizeof(scd));
    memset(&sht, 0, sizeof(sht));
    memset(&mlx, 0, sizeof(mlx));
    memset(&ltr, 0, sizeof(ltr));
    memset(&tsl, 0, sizeof(tsl));
    fake_spi_set_handler(NULL);
    fdev_spi_cs_idle();

    for (int a = 0; a < 128; a++) {
        fake_i2c_dev_t *d = fake_i2c_device((uint8_t)a);
        d->present = 0;
        d->ctx = NULL;
        d->on_write = NULL;
        d->on_read = NULL;
        d->writes = d->reads = 0;
    }
}
