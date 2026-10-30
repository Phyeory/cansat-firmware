#pragma once
// ===================================================================
// Celestium CanSat: STM32G474RE pin map & flight configuration
// Mirrors stm32g474.ioc. Edit BOTH files when the PCB routing changes.
// PCB checklist: WIRING.md
// ===================================================================

#include "stm32g4xx_hal.h"

// ---- clock ----
#define HCLK_HZ 170000000U      // PLL from HSI16 -> 170 MHz (max)

// ---- I2C1: BMP280 (0x76), LTR390 (0x53), TSL2591 (0x29), SCD30 (0x61) ----
// PB8=SCL (AF4), PB9=SDA (AF4), open-drain, 4.7k pull-ups to 3V3.
// Bus runs at 100 kHz because the SCD30 is only specified up to 100 kHz.
// I2C1 kernel clock = HSI16, so this timing value does not depend on SYSCLK.
#define I2C_SENSOR          hi2c1
#define I2C_TIMING_100K     0x00303D5BU    // 100 kHz @ 16 MHz kernel clock
#define BMP280_I2C_ADDR     (0x76U << 1)
#define LTR390_I2C_ADDR     (0x53U << 1)
#define TSL2591_I2C_ADDR    (0x29U << 1)
#define SCD30_I2C_ADDR      (0x61U << 1)

// ---- SPI1: RFM96W LoRa 433 MHz ----
#define LORA_SPI            hspi1          // PA5=SCK, PA6=MISO, PA7=MOSI (AF5)
#define LORA_CS_GPIO        GPIOB
#define LORA_CS_PIN         GPIO_PIN_0     // PB0  (NSS, active low)
#define LORA_RST_GPIO       GPIOB
#define LORA_RST_PIN        GPIO_PIN_1     // PB1
#define LORA_DIO0_GPIO      GPIOA
#define LORA_DIO0_PIN       GPIO_PIN_8     // PA8  (goes high on TxDone)

// Radio settings. The ground receiver (ground-station/lora_receiver) MUST
// use exactly the same values or it will hear nothing.
#define LORA_FREQ_HZ        434000000U     // 433 MHz ISM (UK); alterable, Rule 14
#define LORA_SF             7              // spreading factor 7..12
#define LORA_BW_CODE        0x07           // 0x07 = 125 kHz
#define LORA_CR_CODE        0x01           // 0x01 = coding rate 4/5
#define LORA_PREAMBLE       8
#define LORA_SYNC_WORD      0x12           // private-network default
// UK 433 MHz ISM band and the CanSat rules cap you at 10 mW (10 dBm).
// Check the current rule text before raising this.
#define LORA_TX_POWER_DB    10             // PA_BOOST, valid 2..17

// ---- SPI2: microSD (FatFs) ----
#define SD_SPI              hspi2          // PB13=SCK, PB14=MISO, PB15=MOSI (AF5)
#define SD_CS_GPIO          GPIOB
#define SD_CS_PIN           GPIO_PIN_12

// ---- USART1: u-blox SAM-M8Q GPS (NMEA, 9600 8N1, interrupt RX) ----
#define GPS_UART            huart1         // PA9=TX -> GPS RX, PA10=RX <- GPS TX (AF7)
#define GPS_BAUD            9600U

// ---- USART2: debug / bench serial ----
#define DEBUG_UART          huart2         // PA2=TX, PA3=RX (AF7)

// ---- ADC2: O3-B4 (via the PCB potentiostat) + battery monitor ----
// PC4 only exists on ADC2 (ADC2_IN5), so everything analog lives on ADC2.
// PA0 and PA1 are shared ADC1/ADC2 pins (IN1, IN2).
#define O3_ADC              hadc2
#define O3_WORK_CHANNEL     ADC_CHANNEL_1  // PA0 = working-electrode output
#define O3_AUX_CHANNEL      ADC_CHANNEL_2  // PA1 = auxiliary-electrode output
#define VBAT_CHANNEL        ADC_CHANNEL_5  // PC4 via 2:1 divider
#define VBAT_DIVIDER        2.0f
#define VREF_V              3.3f
#define ADC_MAX             4095.0f        // 12-bit

// ---- TIM8 CH1-4: 4x payload spoiler servos (50 Hz) ----
#define SERVO_TIM           htim8          // PC6=CH1, PC7=CH2, PC8=CH3, PC9=CH4 (AF4)
#define SERVO_TIM_CHANNEL_1 TIM_CHANNEL_1
#define SERVO_TIM_CHANNEL_2 TIM_CHANNEL_2
#define SERVO_TIM_CHANNEL_3 TIM_CHANNEL_3
#define SERVO_TIM_CHANNEL_4 TIM_CHANNEL_4
#define SERVO_STOWED_US     1000U
#define SERVO_DEPLOYED_US   2000U

// ---- timing ----
#define SAMPLE_PERIOD_MS    100U    // onboard logging rate (10 Hz)
#define TELEMETRY_PERIOD_MS 1000U   // primary mission: 1 Hz

// ---- barometric altitude ----
#define SEA_LEVEL_PRESSURE_HPA  1013.25f   // set from launch-day reading

// ---- O3-B4 conversion (calibrate before flight!) ----
#define O3_MV_PER_PPB   1.0f   // TODO: bench calibration vs known O3 source
#define O3_ZERO_MV      0.0f   // zero-current offset
