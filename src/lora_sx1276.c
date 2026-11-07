#include "lora_sx1276.h"
#include "config.h"

#define REG_FIFO            0x00
#define REG_OP_MODE         0x01
#define REG_FRF_MSB         0x06
#define REG_FRF_MID         0x07
#define REG_FRF_LSB         0x08
#define REG_PA_CONFIG       0x09
#define REG_OCP             0x0B
#define REG_LNA             0x0C
#define REG_FIFO_ADDR_PTR   0x0D
#define REG_FIFO_TX_BASE    0x0E
#define REG_FIFO_RX_BASE    0x0F
#define REG_IRQ_FLAGS       0x12
#define REG_MODEM_CONFIG1   0x1D
#define REG_MODEM_CONFIG2   0x1E
#define REG_PREAMBLE_MSB    0x20
#define REG_PREAMBLE_LSB    0x21
#define REG_PAYLOAD_LEN     0x22
#define REG_MODEM_CONFIG3   0x26
#define REG_SYNC_WORD       0x39
#define REG_DIO_MAPPING1    0x40
#define REG_VERSION         0x42
#define REG_PA_DAC          0x4D

#define MODE_LORA_SLEEP     0x80
#define MODE_LORA_STDBY     0x81
#define MODE_LORA_TX        0x83

#define IRQ_TX_DONE_MASK    0x08
#define PA_BOOST            0x80
#define TX_TIMEOUT_MS       1500U

static SPI_HandleTypeDef *s;
static uint8_t  ready = 0, busy = 0;
static uint32_t tx_started, n_tx = 0, n_fail = 0;

static void cs_low(void)  { HAL_GPIO_WritePin(LORA_CS_GPIO, LORA_CS_PIN, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(LORA_CS_GPIO, LORA_CS_PIN, GPIO_PIN_SET); }

static void reg_write(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { (uint8_t)(reg | 0x80), val };
    cs_low();
    HAL_SPI_Transmit(s, b, 2, 10);
    cs_high();
}

static uint8_t reg_read(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), 0x00 }, rx[2] = {0};
    cs_low();
    HAL_SPI_TransmitReceive(s, tx, rx, 2, 10);
    cs_high();
    return rx[1];
}

static void fifo_write(const uint8_t *data, uint8_t len)
{
    uint8_t addr = REG_FIFO | 0x80;
    cs_low();
    HAL_SPI_Transmit(s, &addr, 1, 10);
    HAL_SPI_Transmit(s, (uint8_t *)data, len, 50);
    cs_high();
}

static void reset(void)
{
    HAL_GPIO_WritePin(LORA_RST_GPIO, LORA_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(LORA_RST_GPIO, LORA_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

int lora_init(SPI_HandleTypeDef *spi)
{
    s = spi;
    ready = 0;
    reset();
    if (reg_read(REG_VERSION) != 0x12) return -1;

    reg_write(REG_OP_MODE, 0x00);
    reg_write(REG_OP_MODE, MODE_LORA_SLEEP);
    HAL_Delay(10);

    uint64_t frf = ((uint64_t)LORA_FREQ_HZ << 19) / 32000000U;
    reg_write(REG_FRF_MSB, (uint8_t)(frf >> 16));
    reg_write(REG_FRF_MID, (uint8_t)(frf >> 8));
    reg_write(REG_FRF_LSB, (uint8_t)frf);

    int p = LORA_TX_POWER_DB;
    if (p < 2) p = 2;
    if (p > 17) p = 17;
    reg_write(REG_PA_CONFIG, (uint8_t)(PA_BOOST | 0x70 | (p - 2)));
    reg_write(REG_PA_DAC, 0x84);
    reg_write(REG_OCP, 0x2B);
    reg_write(REG_LNA, 0x23);

    reg_write(REG_MODEM_CONFIG1, (uint8_t)((LORA_BW_CODE << 4) | (LORA_CR_CODE << 1)));
    reg_write(REG_MODEM_CONFIG2, (uint8_t)((LORA_SF << 4) | 0x04));
    reg_write(REG_MODEM_CONFIG3, (LORA_SF >= 11) ? 0x0C : 0x04);
    reg_write(REG_PREAMBLE_MSB, (uint8_t)(LORA_PREAMBLE >> 8));
    reg_write(REG_PREAMBLE_LSB, (uint8_t)(LORA_PREAMBLE & 0xFF));
    reg_write(REG_SYNC_WORD, LORA_SYNC_WORD);

    reg_write(REG_FIFO_TX_BASE, 0x00);
    reg_write(REG_FIFO_RX_BASE, 0x00);
    reg_write(REG_DIO_MAPPING1, 0x40);
    reg_write(REG_OP_MODE, MODE_LORA_STDBY);

    ready = 1;
    busy = 0;
    return 0;
}

uint8_t lora_busy(void)
{
    if (!busy) return 0;

    if (HAL_GPIO_ReadPin(LORA_DIO0_GPIO, LORA_DIO0_PIN) == GPIO_PIN_SET ||
        (reg_read(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)) {
        reg_write(REG_IRQ_FLAGS, 0xFF);
        busy = 0;
        n_tx++;
    } else if (HAL_GetTick() - tx_started > TX_TIMEOUT_MS) {
        reg_write(REG_OP_MODE, MODE_LORA_STDBY);
        reg_write(REG_IRQ_FLAGS, 0xFF);
        busy = 0;
        n_fail++;
    }
    return busy;
}

int lora_send(const uint8_t *data, uint8_t len)
{
    if (!ready || len == 0 || lora_busy()) return -1;

    reg_write(REG_OP_MODE, MODE_LORA_STDBY);
    reg_write(REG_FIFO_ADDR_PTR, 0x00);
    fifo_write(data, len);
    reg_write(REG_PAYLOAD_LEN, len);
    reg_write(REG_IRQ_FLAGS, 0xFF);
    reg_write(REG_OP_MODE, MODE_LORA_TX);

    tx_started = HAL_GetTick();
    busy = 1;
    return 0;
}

uint32_t lora_tx_count(void)   { return n_tx; }
uint32_t lora_fail_count(void) { return n_fail; }
