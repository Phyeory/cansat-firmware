#pragma once
#include <Arduino.h>
#include <RH_RF95.h>
#include "sensors.h"

// Telemetry downlink (RFM96W, 433 MHz) + CSV/JSON packet building.
class Telemetry {
public:
    bool begin();
    // Sends `f` as one compact JSON line. Returns true on radio ACK of TX.
    bool send(const SensorFrame& f);
    // Same data as a CSV header/row pair (for SD logging).
    static const char* csvHeader();
    void csvRow(const SensorFrame& f, char* buf, size_t n);

private:
    RH_RF95* rf95 = nullptr;
};
