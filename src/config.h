#pragma once
// ===================================================================
// Celestium CanSat — STM32G474RE pin map & flight configuration
// Mirrors stm32g474.ioc. Edit BOTH files when the PCB routing changes.
// ===================================================================

#include "stm32g4xx_hal.h"

// ---- clock ----
#define HCLK_HZ 170000000U      // PLL from HSI16 -> 170 MHz (max)

// ---- I2C1: BMP280 (0x76), LTR390 (0x53), TSL2591 (0x29), SCD30 (0x61) ----
#define I2C_SENSOR          hi2c1          // PB8=SCL, PB9=SDA, 400 kHz
#define BMP280_I2C_ADDR     (0x76U << 1)
#define LTR390_I2C_ADDR     (0x53U << 1)
#define TSL2591_I2C_ADDR    (0x29U << 1)
#define SCD30_I2C_ADDR      (0x61U << 1)

// ---- SPI1: RFM96W LoRa 433 MHz ----
#define LORA_SPI            hspi1          // PA5=SCK, PA6=MISO, PA7=MOSI
#define LORA_CS_GPIO        GPIOB
#define LORA_CS_PIN         GPIO_PIN_0     // PB0  (CS, active low)
#define LORA_RST_GPIO       GPIOB
#define LORA_RST_PIN        GPIO_PIN_1     // PB1
#define LORA_DIO0_GPIO      GPIOA
#define LORA_DIO0_PIN       GPIO_PIN_8     // PA8 (poll for TxDone)
#define LORA_FREQ_HZ        434000000U     // 433 MHz ISM (UK); alterable, Rule 14
#define LORA_TX_POWER_DB    13

// ---- SPI2: microSD (FatFs) ----
#define SD_SPI              hspi2          // PB13=SCK, PB14=MISO, PB15=MOSI
#define SD_CS_GPIO          GPIOB
#define SD_CS_PIN           GPIO_PIN_12

// ---- USART1: u-blox SAM-M8Q GPS (NMEA, 9600 8N1) ----
#define GPS_UART            huart1         // PA9=TX, PA10=RX

// ---- USART2: debug / bench serial ----
#define DEBUG_UART          huart2         // PA2=TX, PA3=RX

// ---- ADC1: O3-B4 electrochemical cell + battery monitor ----
#define O3_ADC              hadc1
#define O3_WORK_CHANNEL     ADC_CHANNEL_1  // PA0
#define O3_AUX_CHANNEL      ADC_CHANNEL_2  // PA1
#define VBAT_CHANNEL        ADC_CHANNEL_15 // PC4 — VERIFY channel against .ioc
#define VBAT_DIVIDER        2.0f           // 2:1 resistor divider
#define VREF_V              3.3f
#define ADC_MAX             4095.0f        // 12-bit

// ---- TIM8 CH1-4: 4x payload spoiler servos (50 Hz) ----
#define SERVO_TIM           htim8          // PC6=CH1, PC7=CH2, PC8=CH3, PC9=CH4
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
