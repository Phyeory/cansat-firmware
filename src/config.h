#pragma once
// ===================================================================
// Celestium CanSat — pin map & flight configuration
// Custom PCB: edit these constants to match the final board routing.
// All I2C sensors share Wire (GP4=SDA, GP5=SCL on the Pico).
// ===================================================================

// ---- I2C ----
constexpr uint8_t PIN_SDA = 4;
constexpr uint8_t PIN_SCL = 5;

// ---- RFM96W LoRa 433 MHz (SPI0) ----
constexpr uint8_t PIN_LORA_CS   = 17;   // GP17  SPI0 CSn
constexpr uint8_t PIN_LORA_SCK  = 18;   // GP18  SPI0 SCK
constexpr uint8_t PIN_LORA_MOSI = 19;   // GP19  SPI0 TX
constexpr uint8_t PIN_LORA_MISO = 16;   // GP16  SPI0 RX
constexpr uint8_t PIN_LORA_RST  = 21;
constexpr uint8_t PIN_LORA_DIO0 = 22;
constexpr float   LORA_FREQ_MHZ = 434.0f;  // 433 MHz ISM band (UK); alterable, Rule 14
constexpr int16_t LORA_TX_POWER_DB = 13;

// ---- PA1010D GPS (UART1) ----
constexpr uint8_t PIN_GPS_TX = 8;    // Pico GP8 -> GPS RX
constexpr uint8_t PIN_GPS_RX = 9;    // Pico GP9 -> GPS TX
constexpr uint32_t GPS_BAUD  = 9600;

// ---- Alphasense O3-B4 electrochemical cell (analog via ADC0) ----
constexpr uint8_t PIN_O3_WORK = 26;  // GP26/ADC0 — working electrode
constexpr uint8_t PIN_O3_AUX  = 27;  // GP27/ADC1 — auxiliary electrode (–ve)
// O3-B4 sensitivity ≈ 15 nA/ppb at 20 °C; with the workshop TIA gain this
// maps to mV. Calibrate against a known O3 source and set the factor.
constexpr float O3_MV_PER_PPB = 1.0f;   // TODO calibrate before flight
constexpr float O3_ZERO_MV    = 0.0f;   // zero-current offset

// ---- microSD (SPI1) ----
constexpr uint8_t PIN_SD_CS   = 13;  // GP13
constexpr uint8_t PIN_SD_SCK  = 10;  // GP10
constexpr uint8_t PIN_SD_MOSI = 11;  // GP11
constexpr uint8_t PIN_SD_MISO = 12;  // GP12

// ---- battery monitor (ADC2) ----
constexpr uint8_t PIN_VBAT = 28;     // GP28/ADC2 through a 2:1 divider
constexpr float   VBAT_DIVIDER = 2.0f;

// ---- payload actuation (4x spoiler servos) ----
constexpr uint8_t PIN_SERVO_A = 2;
constexpr uint8_t PIN_SERVO_B = 3;
constexpr uint8_t PIN_SERVO_C = 6;
constexpr uint8_t PIN_SERVO_D = 7;
constexpr uint8_t SERVO_STOWED_DEG  = 0;
constexpr uint8_t SERVO_DEPLOYED_DEG = 90;

// ---- timing ----
constexpr uint32_t SAMPLE_PERIOD_MS = 100;   // onboard logging rate (10 Hz)
constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;  // primary mission: 1 Hz

// ---- barometric altitude ----
constexpr float SEA_LEVEL_PRESSURE_HPA = 1013.25f;  // set from launch-day reading
