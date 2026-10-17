#include "sensors.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LTR390.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_SCD30.h>
#include <Adafruit_GPS.h>

static Adafruit_BMP280 bmp(&Wire);
static Adafruit_LTR390 ltr;
static Adafruit_TSL2591 tsl(2591);
static Adafruit_SCD30  scd;
static Adafruit_GPS    gps(&Serial1);

static bool s_ok_bmp = false, s_ok_ltr = false, s_ok_tsl = false,
            s_ok_scd = false, s_ok_gps = false;

bool SensorSuite::begin() {
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

    s_ok_bmp = bmp.begin(0x76);                     // BMP280: T + P (primary)
    if (s_ok_bmp) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,   // temperature
                        Adafruit_BMP280::SAMPLING_X16,  // pressure
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_63);
    }

    s_ok_ltr = ltr.begin();                         // LTR390: UV
    if (s_ok_ltr) {
        ltr.setMode(LTR390_MODE_UVS);
        ltr.setGain(LTR390_GAIN_3);
        ltr.setResolution(LTR390_RESOLUTION_18BIT);
        ltr.enable(true);
    }

    s_ok_tsl = tsl.begin();                         // TSL2591: IR / light
    if (s_ok_tsl) {
        tsl.setGain(TSL2591_GAIN_MED);
        tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
    }

    s_ok_scd = scd.begin();                         // SCD30: CO2
    if (s_ok_scd) scd.startContinuousMeasurement();

    // PA1010D: GPS over UART1
    Serial1.setTX(PIN_GPS_TX);
    Serial1.setRX(PIN_GPS_RX);
    Serial1.begin(GPS_BAUD);
    gps.begin(GPS_BAUD);
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    s_ok_gps = true;

    // O3-B4 is a passive analog cell — no init needed, just ADC.
    pinMode(PIN_O3_WORK, INPUT);
    pinMode(PIN_O3_AUX,  INPUT);

    return s_ok_bmp;                                // only BMP280 is mission-critical
}

float SensorSuite::readO3mV() {
    // ADC returns 0..4095 for 0..3.3 V on the Pico (12-bit).
    float work = analogRead(PIN_O3_WORK) * (3.3f / 4095.0f) * 1000.0f;
    float aux  = analogRead(PIN_O3_AUX)  * (3.3f / 4095.0f) * 1000.0f;
    return work - aux;                              // differential = real signal
}

void SensorSuite::poll(SensorFrame& f) {
    f.t_ms   = millis();
    f.ok_bmp = f.ok_ltr = f.ok_tsl = f.ok_scd = f.ok_o3 = f.ok_gps = false;
    f.temp_c = f.pressure_hpa = f.altitude_m = NAN;
    f.uv_raw = f.uv_index = f.ir_lux = f.full_lux = f.co2_ppm = f.o3_ppb = NAN;
    f.gps_lat = f.gps_lon = f.gps_alt_m = 0.0f;
    f.gps_fix = false; f.sats = 0;

    if (s_ok_bmp) {
        float p = bmp.readPressure() / 100.0f;      // Pa -> hPa
        if (p > 0) {
            f.pressure_hpa = p;
            f.temp_c       = bmp.readTemperature();
            f.altitude_m   = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);
            f.ok_bmp = true;
        }
    }

    if (s_ok_ltr) {
        f.uv_raw = ltr.readUVS();
        // LTR390 18-bit / gain 3: ~2300 counts per UVI (datasheet scaling)
        f.uv_index = f.uv_raw / 2300.0f;
        f.ok_ltr   = true;
    }

    if (s_ok_tsl) {
        uint32_t lum = tsl.getFullLuminosity();
        uint16_t ir = lum & 0xFFFF, full = lum >> 16;
        f.ir_lux   = ir;
        f.full_lux = tsl.calculateLux(full, ir);
        f.ok_tsl   = true;
    }

    if (s_ok_scd && scd.dataReady()) {
        if (scd.read()) {
            f.co2_ppm = scd.CO2;
            f.ok_scd  = true;
        }
    }

    f.o3_ppb = (readO3mV() - O3_ZERO_MV) / O3_MV_PER_PPB;   // always report, calibrate offline
    f.ok_o3  = true;

    {   // drain a few NMEA bytes, then read the parsed fix
        uint32_t t0 = millis();
        while (millis() - t0 < 20) {
            if (gps.available()) gps.read();
        }
        gps.parse(gps.lastNMEA());
        f.ok_gps = s_ok_gps;
        if (gps.fix) {
            f.gps_lat = gps.latitudeDegrees;
            f.gps_lon = gps.longitudeDegrees;
            f.gps_alt_m = gps.altitude;
            f.gps_fix = true;
            f.sats = gps.satellites;
        }
    }

    f.vbat_v = analogRead(PIN_VBAT) * (3.3f / 4095.0f) * VBAT_DIVIDER;
}
