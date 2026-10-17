# Celestium CanSat Firmware

Firmware for the Brighton College **Celestium** CanSat (UK CanSat 2026-27),
running on an **STM32G474RE** on a custom PCB, configured with
**STM32CubeMX**.

**Language/toolchain: bare C on STM32 HAL, built with PlatformIO.**
The board is configured in `stm32g474.ioc` (CubeMX); the firmware talks to
the peripherals directly (`HAL_I2C`, `HAL_SPI`, `HAL_UART`, `HAL_ADC`,
`HAL_TIM`) with small self-contained drivers — no Arduino layer, no heavy
vendor libraries. Embedded protocols in use: **I²C** (sensors), **SPI**
(LoRa + SD), **UART** (GPS + debug), **PWM** (servos), analog **ADC** (O3
cell, battery).

## Hardware

| Subsystem | Part | Interface / pins |
|---|---|---|
| Main computer | STM32G474RE @ 170 MHz | — |
| Temp + pressure (primary) | BMP280 | I²C1 (PB8/PB9) @ 0x76 |
| UV radiation | LTR390 | I²C1 @ 0x53 |
| IR / light | TSL2591 | I²C1 @ 0x29 |
| CO₂ | SCD30 | I²C1 @ 0x61 |
| Ozone | Alphasense O3-B4 | ADC (PA0 work / PA1 aux, differential) |
| GPS | u-blox SAM-M8Q | USART1 (PA9/PA10) NMEA 9600 |
| Telemetry | RFM96W LoRa 433 MHz | SPI1 + PB0 CS, PB1 RST, PA8 DIO0 |
| Logging | microSD | SPI2 (PB13-15), PB12 CS |
| Actuation | 4× spoiler servos | TIM8 CH1-4 (PC6-PC9), 50 Hz PWM |
| Battery monitor | 2:1 divider | ADC PC4 |

The pin map lives in [`src/config.h`](src/config.h) and is mirrored in
[`stm32g474.ioc`](stm32g474.ioc) — edit **both** when the PCB changes.

## Missions

- **Primary**: temperature + pressure → altitude, transmitted **once per
  second** as JSON over LoRa (rule-compliant, frequency alterable).
- **Secondary**: atmospheric radiation & gas profiling — UV/IR intensity and
  CO₂/O₃ concentration vs altitude, logged onboard to SD at 10 Hz so data
  survives a telemetry drop.

## Getting started (CubeMX workflow)

1. **Configure the board**: open `stm32g474.ioc` in STM32CubeMX, verify the
   pin assignments against the PCB, press **GENERATE CODE** (Toolchain:
   "Generate peripheral initialization as a pair of .c/.h files").
2. **Build & flash from here**:

```bash
pip install -U platformio          # one-time
pio run                            # compiles everything
pio run -t upload                  # flash via ST-Link
pio device monitor                 # live debug output (115200, USART2)
```

CubeMX regeneration only replaces the peripheral init (mirrored in
`src/board.c`) — the drivers in `src/` are yours and won't be touched.

> **SD logging**: enable the FatFS middleware in CubeMX (with an SPI
> transport), then build with `-DHAS_FATFS` (add to `build_flags`). Without
> it `sdlog.c` compiles as a no-op so the rest of the firmware still runs.

## Repository layout

```
stm32g474.ioc          CubeMX board configuration (open this in CubeMX)
platformio.ini         toolchain (STM32G474RE, STM32Cube HAL)
inc/                   stm32g4xx_hal_conf.h — HAL module selection
src/config.h           pin map & flight constants (edit for the PCB)
src/board.c/.h         clock tree + all peripheral init (CubeMX mirror)
src/main.c             flight loop: 10 Hz sampling, 1 Hz telemetry
src/bmp280.c/.h        BMP280 driver with datasheet compensation
src/ltr390.c/.h        LTR390 UV driver
src/tsl2591.c/.h       TSL2591 light driver
src/scd30.c/.h         SCD30 CO2 driver (I2C, CRC-checked)
src/gps_sam_m8q.c/.h   SAM-M8Q NMEA parser (RMC/GGA) over USART1
src/lora_sx1276.c/.h   minimal SX1276 LoRa TX driver (RFM96W)
src/sdlog.c/.h         FatFs SD logging (no-op until HAS_FATFS)
src/servos.c/.h        spoiler servo PWM on TIM8
src/adc_read.c/.h      polled ADC helper (O3-B4, battery)
src/telemetry.c/.h     JSON downlink + CSV packet formats
src/stm32g4xx_it.c     interrupt handlers (SysTick/HAL tick)
ground-station/        Python starter: serial/LoRa listener -> CSV + live plot
```

## Ground station

Unchanged from the original build: `ground-station/station.py` receives the
JSON telemetry packets (via a serial-to-LoRa bridge) and produces the two
rule-required deliverables live — altitude vs time and temperature vs
altitude — plus a CSV for post-processing.

```bash
python3 -m pip install pyserial matplotlib
python3 ground-station/station.py /dev/tty.usbmodem*
```

## Rules notes (2026-27)

- Telemetry is exactly 1 Hz — enforced by `TELEMETRY_PERIOD_MS` in `config.h`.
- LoRa frequency is a single constant (`LORA_FREQ_HZ`), alterable and
  declared at launch (Rule 14), default 434.0 MHz (UK 433 MHz ISM band).
- Data redundancy: every transmitted packet is also written to the SD card.

## Pre-flight TODOs

- [ ] Verify `stm32g474.ioc` pins against the final PCB, regenerate in CubeMX.
- [ ] Bench-calibrate the O3-B4 constants (`O3_MV_PER_PPB`, `O3_ZERO_MV`).
- [ ] Set `SEA_LEVEL_PRESSURE_HPA` from a launch-day barometer reading.
- [ ] Enable + test FatFS/SD logging (`-DHAS_FATFS`).
- [ ] Descent state machine + spoiler-deploy trigger logic (servos are
      ready; the trigger decision is the remaining flight-code task).

## Licence

MIT — see [LICENSE](LICENSE).
