#include "telemetry.h"

bool Telemetry::begin() {
    pinMode(PIN_LORA_RST, OUTPUT);
    digitalWrite(PIN_LORA_RST, LOW);
    delay(10);
    digitalWrite(PIN_LORA_RST, HIGH);
    delay(10);

    rf95 = new RH_RF95(PIN_LORA_CS, PIN_LORA_DIO0);
    if (!rf95->init()) return false;
    if (!rf95->setFrequency(LORA_FREQ_MHZ)) return false;
    rf95->setTxPower(LORA_TX_POWER_DB, false);
    return true;
}

bool Telemetry::send(const SensorFrame& f) {
    // Compact single-line JSON — easy to parse on the ground station,
    // human-readable in the radio logs, ~250 bytes at 1 Hz is fine on LoRa.
    char pkt[300];
    int n = snprintf(pkt, sizeof(pkt),
        "{\"t\":%lu,\"T\":%.2f,\"P\":%.2f,\"A\":%.2f,"
        "\"uv\":%.1f,\"uvi\":%.2f,\"ir\":%.1f,\"lux\":%.1f,"
        "\"co2\":%.0f,\"o3\":%.1f,"
        "\"la\":%.5f,\"lo\":%.5f,\"ga\":%.1f,\"s\":%u,"
        "\"v\":%.2f}",
        (unsigned long)f.t_ms,
        f.temp_c, f.pressure_hpa, f.altitude_m,
        f.uv_raw, f.uv_index, f.ir_lux, f.full_lux,
        f.co2_ppm, f.o3_ppb,
        f.gps_lat, f.gps_lon, f.gps_alt_m, f.sats,
        f.vbat_v);
    if (n <= 0) return false;
    rf95->send((uint8_t*)pkt, n);
    rf95->waitPacketSent(100);
    return true;
}

const char* Telemetry::csvHeader() {
    return "t_ms,temp_c,pressure_hpa,alt_m,uv_raw,uv_index,ir_lux,full_lux,"
           "co2_ppm,o3_ppb,lat,lon,gps_alt_m,sats,vbat_v";
}

void Telemetry::csvRow(const SensorFrame& f, char* buf, size_t n) {
    snprintf(buf, n, "%lu,%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.1f,%.0f,%.1f,"
                     "%.5f,%.5f,%.1f,%u,%.2f",
        (unsigned long)f.t_ms, f.temp_c, f.pressure_hpa, f.altitude_m,
        f.uv_raw, f.uv_index, f.ir_lux, f.full_lux, f.co2_ppm, f.o3_ppb,
        f.gps_lat, f.gps_lon, f.gps_alt_m, f.sats, f.vbat_v);
}
