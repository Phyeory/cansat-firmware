#include "lora_sx1276.h"
#include "config.h"

/* SX1276 register set (LoRa mode) — subset needed for packet TX */
#define REG_FIFO            0x00
#define REG_OP_MODE         0x01
#define REG_FRF_MSB         0x06
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
#define REG_PAYLOAD_LEN     0x22
#define REG_MODEM_CONFIG3   0x26
#define REG_DIO_MAPPING1    0x40
#define REG_VERSION         0x42

#define MODE_LORA_SLEEP     0x80
#define MODE_LORA_STDBY     0x81
#define MODE_LORA_TX        0x83

#define IRQ_TX_DONE_MASK    0x08

static SPI_HandleTypeDef *s;

static void cs_low(void)  { HAL_GPIO_WritePin(LORA_CS_GPIO, LORA_CS_PIN, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(LORA_CS_GPIO, LORA_CS_PIN, GPIO_PIN_SET); }

static void reg_write(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { (uint8_t)(reg | 0x80), val };
    cs_low();
    HAL_SPI_Transmit(s, b, 1, 100);
    HAL_SPI_Transmit(s, &b[1], 1, 100);
    cs_high();
}

static uint8_t reg_read(uint8_t reg)
{
    uint8_t addr = (uint8_t)(reg & 0x7F), val = 0;
    cs_low();
    HAL_SPI_Transmit(s, &addr, 1, 100);
    HAL_SPI_Receive(s, &val, 1, 100);
    cs_high();
    return val;
}

static void reset(void)
{
    HAL_GPIO_WritePin(LORA_RST_GPIO, LORA_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RST_GPIO, LORA_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

int lora_init(SPI_HandleTypeDef *spi)
{
    s = spi;
    reset();
    if (reg_read(REG_VERSION) != 0x12) return -1;

    reg_write(REG_OP_MODE, MODE_LORA_SLEEP);       /* LoRa mode, sleep */
    HAL_Delay(10);

    /* frequency: FRF = F * 2^19 / 32 MHz */
    uint64_t frf = ((uint64_t)LORA_FREQ_HZ << 19) / 32000000U;
    reg_write(REG_FRF_MSB, (uint8_t)(frf >> 16));
    reg_write(REG_FRF_MSB + 1, (uint8_t)(frf >> 8));
    reg_write(REG_FRF_MSB + 2, (uint8_t)frf);

    /* PA: PA_BOOST, +13 dBm (OCP default 100 mA is fine) */
    reg_write(REG_PA_CONFIG, 0x4F);
    reg_write(REG_OCP, 0x3B);
    reg_write(REG_LNA, 0x23);

    /* BW=125 kHz, CR=4/5, explicit header; SF=7; low-data-rate-opt off */
    reg_write(REG_MODEM_CONFIG1, 0x72);
    reg_write(REG_MODEM_CONFIG2, 0x74);            /* SF7, normal TX single */
    reg_write(REG_MODEM_CONFIG3, 0x00);
    reg_write(REG_PREAMBLE_MSB, 0x00);
    reg_write(REG_PREAMBLE_MSB + 1, 0x08);

    reg_write(REG_FIFO_TX_BASE, 0x80);
    reg_write(REG_OP_MODE, MODE_LORA_STDBY);
    return 0;
}

int lora_send(const uint8_t *data, uint8_t len)
{
    reg_write(REG_OP_MODE, MODE_LORA_STDBY);
    reg_write(REG_FIFO_ADDR_PTR, reg_read(REG_FIFO_TX_BASE));
    for (uint8_t i = 0; i < len; i++) reg_write(REG_FIFO, data[i]);
    reg_write(REG_PAYLOAD_LEN, len);
    reg_write(REG_IRQ_FLAGS, 0xFF);                /* clear all */
    reg_write(REG_OP_MODE, MODE_LORA_TX);

    uint32_t t0 = HAL_GetTick();
    while (!(reg_read(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)) {
        if (HAL_GetTick() - t0 > 500) return -1;   /* radio jammed — bail out */
    }
    reg_write(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    return 0;
}
