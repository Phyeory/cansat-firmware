# Celestium CanSat Firmware

Firmware for the Brighton College **Celestium** CanSat (UK CanSat 2026-27),
running on a **Raspberry Pi Pico (RP2040)** on a custom PCB.

**Language/toolchain: C++ (Arduino framework) built with PlatformIO.**
Chosen because every sensor in the suite has a mature, flight-proven Arduino
library, PlatformIO manages them all from `platformio.ini`, and C++ gives us
deterministic timing for the 1 Hz telemetry requirement.

## Hardware

| Subsystem | Part | Interface |
|---|---|---|
| Main computer | Raspberry Pi Pico (RP2040) | — |
| Temp + pressure (primary) | BMP280 | I²C |
| UV radiation | LTR390 | I²C |
| IR / light | TSL2591 | I²C |
| CO₂ | SCD30 | I²C |
| Ozone | Alphasense O3-B4 | ADC (differential) |
| GPS | Adafruit PA1010D | UART |
| Telemetry | RFM96W LoRa 433 MHz | SPI |
| Logging | microSD | SPI1 |
| Actuation | 4× spoiler servos | PWM |

The full pin map lives in [`src/config.h`](src/config.h) — one file to edit
when the custom PCB routing is finalised.

## Missions

- **Primary**: temperature + pressure → altitude, transmitted **once per
  second** as JSON over LoRa (rule-compliant, frequency alterable).
- **Secondary**: atmospheric radiation & gas profiling — UV/IR intensity and
  CO₂/O₃ concentration vs altitude, logged onboard to SD at 10 Hz so data
  survives a telemetry drop.

## Getting started

```bash
# one-time: install PlatformIO
pip install -U platformio        # or: brew install platformio

# clone, then from the repo root
pio run                          # compile everything (downloads libs on first run)
pio run -t upload                # flash a Pico in BOOTSEL mode
pio device monitor               # live serial output (115200 baud)
```

That's it — `pio run` pulls every pinned library in `platformio.ini` and
compiles. You're ready to start coding.

## Repository layout

```
platformio.ini        toolchain + all sensor libraries (pinned versions)
src/config.h          pin map & flight constants (edit for the PCB)
src/main.cpp          setup/loop: 10 Hz sampling, 1 Hz telemetry, SD logging
src/sensors.*         SensorSuite — initialises and polls every sensor
src/telemetry.*       LoRa downlink + CSV/JSON packet formats
ground-station/       Python starter: serial/LoRa listener -> CSV + live plot
docs/                 design notes
```

## Ground station

`ground-station/station.py` is a ready-to-run listener that receives the JSON
telemetry packets (via serial-to-LoRa bridge) and appends them to a CSV plus a
live altitude plot — the two deliverable graphs required by the rules.

```bash
python3 -m pip install pyserial matplotlib
python3 ground-station/station.py /dev/tty.usbmodem*
```

## Rules notes (2026-27)

- Telemetry is exactly 1 Hz — enforced by `TELEMETRY_PERIOD_MS` in `config.h`.
- LoRa frequency is a single constant (`LORA_FREQ_MHZ`), alterable and
  declared at launch (Rule 14), default 434.0 MHz (UK 433 MHz ISM band).
- Data redundancy: every transmitted packet is also written to the SD card.

## Licence

MIT — see [LICENSE](LICENSE).
