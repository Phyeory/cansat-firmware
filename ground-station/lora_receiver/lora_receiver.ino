#include <SPI.h>
#include <LoRa.h>

const long FREQ_HZ      = 434000000;
const int  SF           = 7;
const long BW_HZ        = 250000;
const int  CR_DENOM     = 5;
const int  PREAMBLE     = 8;
const int  SYNC_WORD    = 0x12;

const int PIN_CS   = 17;
const int PIN_RST  = 20;
const int PIN_DIO0 = 21;

const int LED = LED_BUILTIN;
char buf[256];

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }

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

  if (n > 1 && buf[0] == '{' && buf[n - 1] == '}') {
    buf[n - 1] = '\0';
    Serial.print(buf);
    Serial.print(",\"rssi\":");
    Serial.print(rssi);
    Serial.print(",\"snr\":");
    Serial.print(snr, 1);
    Serial.println("}");
  } else {
    Serial.print("# junk rssi=");
    Serial.print(rssi);
    Serial.print(" len=");
    Serial.println(n);
  }

  digitalWrite(LED, !digitalRead(LED));
}
