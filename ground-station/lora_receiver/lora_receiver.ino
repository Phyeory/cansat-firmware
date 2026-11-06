/*
 * Celestium ground receiver: RFM96W -> USB serial bridge.
 *
 * Listens for the CanSat's LoRa packets and prints each one as a single
 * JSON line on USB serial, with "rssi" and "snr" added, for station.py.
 *
 * Board: Raspberry Pi Pico (arduino-pico core by Earle Philhower) or any
 * 3.3 V Arduino. Library: "LoRa" by Sandeep Mistry (Library Manager).
 *
 * Radio settings MUST match src/config.h on the CanSat.
 *
 * Wiring (Pico)            RFM96W
 *   GP18 (SPI0 SCK)   ->   SCK
 *   GP19 (SPI0 TX)    ->   MOSI
 *   GP16 (SPI0 RX)    <-   MISO
 *   GP17              ->   NSS / CS
 *   GP20              ->   RESET
 *   GP21              <-   DIO0
 *   3V3(OUT)          ->   VIN / 3.3V
 *   GND               ->   GND
 *   + 433 MHz antenna on ANT (17.3 cm wire or a proper Yagi for launch day)
 */
#include <SPI.h>
#include <LoRa.h>

// ---- must match the CanSat (src/config.h) ----
const long FREQ_HZ      = 434000000;
const int  SF           = 7;
const long BW_HZ        = 250000;   // config.h LORA_BW_CODE 0x08
const int  CR_DENOM     = 5;      // 4/5
const int  PREAMBLE     = 8;
const int  SYNC_WORD    = 0x12;

// ---- pins ----
const int PIN_CS   = 17;
const int PIN_RST  = 20;
const int PIN_DIO0 = 21;

const int LED = LED_BUILTIN;
char buf[256];

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }   // wait for USB, but not forever

  LoRa.setPins(PIN_CS, PIN_RST, PIN_DIO0);
  while (!LoRa.begin(FREQ_HZ)) {
    Serial.println("# LoRa init failed: check wiring (RegVersion != 0x12)");
    delay(1000);
  }
  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(BW_HZ);
  LoRa.setCodingRate4(CR_DENOM);
  LoRa.setPreambleLength(PREAMBLE);
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.enableCrc();
  LoRa.receive();
  Serial.println("# Celestium receiver ready");
}

void loop() {
  int size = LoRa.parsePacket();
  if (size <= 0) return;

  int n = 0;
  while (LoRa.available() && n < (int)sizeof(buf) - 1) buf[n++] = (char)LoRa.read();
  buf[n] = '\0';

  int   rssi = LoRa.packetRssi();
  float snr  = LoRa.packetSnr();

  // JSON packet: splice rssi/snr in before the closing brace
  if (n > 1 && buf[0] == '{' && buf[n - 1] == '}') {
    buf[n - 1] = '\0';
    Serial.print(buf);
    Serial.print(",\"rssi\":");
    Serial.print(rssi);
    Serial.print(",\"snr\":");
    Serial.print(snr, 1);
    Serial.println("}");
  } else {
    // not ours or damaged: comment line, station.py ignores it
    Serial.print("# junk rssi=");
    Serial.print(rssi);
    Serial.print(" len=");
    Serial.println(n);
  }

  digitalWrite(LED, !digitalRead(LED));   // blink per packet
}
