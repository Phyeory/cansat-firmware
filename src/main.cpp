// ===================================================================
// Celestium CanSat — main firmware
// Brighton College, UK CanSat 2026-27
//
// Primary mission:   BMP280 temperature + pressure -> altitude,
//                    transmitted at 1 Hz over 433 MHz LoRa.
// Secondary mission: atmospheric radiation & gas profiling
//                    (UV, IR, CO2, O3) logged onboard to SD at 10 Hz.
// ===================================================================
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <Servo.h>
#include "config.h"
#include "sensors.h"
#include "telemetry.h"

SensorSuite sensors;
Telemetry   telemetry;
SdFat       sd;
FsFile      logfile;
Servo       spoilers[4];
const uint8_t SERVO_PINS[4] = {PIN_SERVO_A, PIN_SERVO_B, PIN_SERVO_C, PIN_SERVO_D};

uint32_t lastSample = 0, lastTelemetry = 0;
bool     sd_ok = false;

void setup() {
    Serial.begin(115200);
    // wait for USB only when attached (bench); fly blind otherwise
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);
    Serial.println("\n[CELESTIUM] boot");

    if (!sensors.begin()) {
        Serial.println("[FAIL] BMP280 (primary mission) — halting");
        while (true) { digitalWrite(LED_BUILTIN, HIGH); delay(100);
                       digitalWrite(LED_BUILTIN, LOW);  delay(900); }
    }

    if (!telemetry.begin()) Serial.println("[WARN] LoRa init failed — telemetry offline");
    else                    Serial.println("[OK]   LoRa 433 MHz up");

    SPI1.setRX(PIN_SD_MISO);
    SPI1.setTX(PIN_SD_MOSI);
    SPI1.setSCK(PIN_SD_SCK);
    sd_ok = sd.begin(SdSpiConfig(PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(25), &SPI1));
    if (sd_ok) {
        char name[24];
        snprintf(name, sizeof(name), "FL%08lu.csv", millis());
        logfile = sd.open(name, O_WRITE | O_CREAT | O_AT_END);
        logfile.println(Telemetry::csvHeader());
        logfile.flush();
        Serial.printf("[OK]   SD logging -> %s\n", name);
    } else Serial.println("[WARN] SD init failed — logging offline");

    for (int i = 0; i < 4; i++) {
        spoilers[i].attach(SERVO_PINS[i]);
        spoilers[i].write(SERVO_STOWED_DEG);
    }
}

void loop() {
    uint32_t now = millis();

    if (now - lastSample >= SAMPLE_PERIOD_MS) {
        lastSample = now;
        static SensorFrame f;
        sensors.poll(f);

        if (sd_ok && logfile.isOpen()) {
            char row[240];
            telemetry.csvRow(f, row, sizeof(row));
            logfile.println(row);
            if (now % 1000 < SAMPLE_PERIOD_MS) logfile.flush();  // ~1 Hz flush
        }

        Serial.printf("t=%lus T=%.1fC P=%.1fhPa alt=%.1fm CO2=%.0f UVi=%.2f\n",
                      now / 1000, f.temp_c, f.pressure_hpa, f.altitude_m,
                      f.co2_ppm, f.uv_index);
    }

    if (now - lastTelemetry >= TELEMETRY_PERIOD_MS) {   // primary mission: 1 Hz
        lastTelemetry = now;
        static SensorFrame f;
        telemetry.send(f);
    }
}
