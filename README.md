# Celestium CanSat Firmware

Firmware for the Brighton College **Celestium** CanSat (UK CanSat 2026-27),
running on an **STM32G474RE** on a custom PCB, configured with
**STM32CubeMX**.

**Language/toolchain: bare C on STM32 HAL, built with PlatformIO.**
The board is configured in `stm32g474.ioc` (CubeMX); the firmware talks to
the peripherals directly (`HAL_I2C`, `HAL_SPI`, `HAL_UART`, `HAL_ADC`,
`HAL_TIM`) with small self-contained drivers: no Arduino layer, no heavy
vendor libraries. Embedded protocols in use: **SPI** (IMU, magnetometer,
barometer, SD), **I²C** (UV/light/CO₂/environment), **UART** (GPS, E22
telemetry, debug), **PWM** (servos), analog **ADC** (O3 cell, battery).

## Hardware

| Subsystem | Part | Interface / pins |
|---|---|---|
| Main computer | STM32G474RE @ 170 MHz | |
| Temp + pressure (primary) | Bosch **BMP581** | SPI1, CS PA8 (mode 3) |
| UV radiation | LTR390 | I²C1 @ 0x53 |
| IR / light | TSL2591 | I²C1 @ 0x29 |
| CO₂ | SCD30 | I²C1 @ 0x61 |
| Ozone | Alphasense O3-B4 | ADC (PA0 work / PA1 aux, differential) |
| IMU (accel + gyro) | ST **LSM6DSV** | SPI1, CS PB0 (mode 3) |
| Magnetometer | Melexis **MLX90395** | SPI1, CS PB1 (mode 3) |
| Humidity + 2nd temp | Sensirion SHT45 | I²C1 @ 0x44 |
| Radiation counter | BPW34 S-Z + comparator | EXTI PB10 (falling-edge pulses) |
| GPS | u-blox SAM-M8Q | USART1 (PA9/PA10) NMEA 9600 |
| Telemetry | EBYTE **E22-900T22D** 868.125 MHz | USART3 (PC10/PC11) transparent UART |
| Logging | microSD | SPI2 (PB13-15), PB12 CS |
| Actuation | 4× spoiler servos | TIM8 CH1-4 (PC6-PC9), 50 Hz PWM |
| Battery monitor | 2:1 divider | ADC PC4 |

PCB schematic and bring-up checklist: [`WIRING.md`](WIRING.md).

The pin map lives in [`src/config.h`](src/config.h) and is mirrored in
[`stm32g474.ioc`](stm32g474.ioc). Edit **both** when the PCB changes.

## Missions

- **Primary**: temperature + pressure → altitude, transmitted **once per
  second** as JSON over the E22 (rule-compliant, frequency alterable).
- **Secondary**: atmospheric radiation & gas profiling: UV/IR intensity and
  CO₂/O₃ concentration vs altitude, plus motion profiling (LSM6DSV IMU,
  MLX90395 magnetometer), humidity (SHT45 → dew point / absolute humidity
  post-flight) and BPW34 radiation pulse counting. Full 10 Hz stream goes to
  SD so data survives a telemetry drop; the 1 Hz downlink also carries
  accel, gyro, RH and pulses-per-second for live monitoring.

**Estimation status:** there is **no EKF in the firmware**. The online
baro/accel KF and the offline attitude/trajectory EKFs are design targets in
[`docs/DATA-PIPELINE.md`](docs/DATA-PIPELINE.md) §6 and
[`docs/KALMAN-NOISE.md`](docs/KALMAN-NOISE.md), not implemented code. The
firmware currently logs raw sensor data only.

## Getting started (CubeMX workflow)

1. **Configure the board**: open `stm32g474.ioc` in STM32CubeMX, verify the
   pin assignments against the PCB, press **GENERATE CODE** (Toolchain:
   "Generate peripheral initialization as a pair of .c/.h files").
2. **Build & flash from here**:

```bash
python3 -m pip install -r requirements.txt  # one-time
pio run                            # compiles everything
pio run -t upload                  # flash via ST-Link
pio device monitor                 # live debug output (115200, USART2)
```

CubeMX regeneration only replaces the peripheral init (mirrored in
`src/board.c`). The drivers in `src/` are yours and won't be touched.

> **SD logging**: enable the FatFS middleware in CubeMX (with an SPI
> transport), then build with `-DHAS_FATFS` (add to `build_flags`). Without
> it `sdlog.c` compiles as a no-op so the rest of the firmware still runs.

## Repository layout

```
stm32g474.ioc          CubeMX board configuration (open this in CubeMX)
platformio.ini         toolchain (STM32G474RE, STM32Cube HAL)
inc/                   stm32g4xx_hal_conf.h, HAL module selection
src/config.h           pin map & flight constants (edit for the PCB)
src/board.c/.h         clock tree + all peripheral init (CubeMX mirror)
src/main.c             flight loop: 10 Hz sampling, 1 Hz telemetry
src/bmp581.c/.h        BMP581 barometer driver (SPI, calibrated 24-bit words)
src/lsm6dsv.c/.h       LSM6DSV IMU driver (SPI, 120 Hz, ±8 g / ±2000 dps)
src/ltr390.c/.h        LTR390 UV driver
src/tsl2591.c/.h       TSL2591 light driver
src/scd30.c/.h         SCD30 CO2 driver (I2C, CRC-checked)
src/gps_sam_m8q.c/.h   SAM-M8Q NMEA parser (RMC/GGA) over USART1
src/radio_e22.c/.h     E22-900T22D transparent-UART telemetry driver
src/spi_dev.c/.h       shared SPI1 chip-select/transfer helper
src/sdlog.c/.h         FatFs SD logging (no-op until HAS_FATFS)
src/servos.c/.h        spoiler servo PWM on TIM8
src/adc_read.c/.h      polled ADC helper (O3-B4, battery)
src/telemetry.c/.h     JSON downlink + CSV packet formats
src/stm32g4xx_it.c     interrupt handlers (SysTick/HAL tick)
ground-station/        station.py (CSV + live plot; reads the E22 USB module)
```

## Ground station

The ground station uses the **E22-900T22U**, the USB-packaged sibling of the
flight radio. It arrives as a virtual serial port and outputs each transparent
UART packet as a JSON line. No Pico/RFM96W bridge is needed.
`ground-station/station.py` reads those lines and produces the two
rule-required deliverables live (altitude vs time and temperature vs
altitude), plus a CSV for post-processing. It also counts lost packets and
shows which sensors are healthy. If the USB module also emits RSSI/SNR, add
them as `rssi`/`snr` keys; the plotter falls back to zero otherwise.

```bash
python3 -m pip install -r requirements.txt  # one-time
python3 ground-station/station.py --list          # find the port
python3 ground-station/station.py /dev/tty.usbmodem*
```

VS Code will recommend the embedded C/C++ and Python extensions from
`.vscode/extensions.json` when the repository is opened.

## Rules notes (2026-27)

- Telemetry is exactly 1 Hz, enforced by `TELEMETRY_PERIOD_MS` in `config.h`.
- Radio frequency is a **module setting** (`E22_FREQ_HZ` records the intended
  value), alterable with EBYTE's tool and declared at launch (Rule 14),
  default **868.125 MHz**.
- Radio power is **14 dBm** (`E22_TX_POWER_DBM` records it), the
  868.0–868.6 MHz UK limit. The E22 must be configured for 62.5 kbps
  transparent mode; the firmware only writes bytes to USART3.
- Data redundancy: every transmitted packet is also written to the SD card.
- **Open RF check:** the JSON packet is ~200 B typical / ~250 B worst case.
  At 62.5 kbps that is ~26–32 ms of airtime, i.e. ~2.6–3.2 % duty at 1 Hz,
  above the 1 % limit for 868.0–868.6 MHz. The SD log is unaffected; if the
  rules are enforced strictly, the downlink needs a compact (≤78 B) packet
  or a different band.

## Pre-flight TODOs

- [ ] Set the PB8/BOOT0 option bytes on every board (see `WIRING.md`).
- [ ] Verify `stm32g474.ioc` pins against the final PCB, regenerate in CubeMX.
- [ ] Configure the E22-900T22D: 868.125 MHz, 14 dBm, 62.5 kbps, transparent
      mode (EBYTE tool or C0 command mode) and match the ground E22-900T22U.
- [ ] Confirm the 3.0 V rail (`VREF_V` in `config.h`) against the assembled
      board before trusting the ADC-derived O3/battery values.
- [ ] Bench-calibrate the O3-B4 constants (`O3_MV_PER_PPB`, `O3_ZERO_MV`).
- [ ] Set `SEA_LEVEL_PRESSURE_HPA` from a launch-day barometer reading.
- [ ] Enable + test FatFS/SD logging (`-DHAS_FATFS`).
- [ ] Descent state machine + spoiler-deploy trigger logic (servos are
      ready; the trigger decision is the remaining flight-code task).
      *(2026-11-07: the fin/petal brake system is archived — the competition
      descent is chute-only at ~10 m/s, statically stable with no descent-control
      hardware. Servo trigger logic on hold unless the v2 petal brake is revived;
      the chute-only state machine reduces to apogee → landing → power-down.)*

## Licence

MIT, see [LICENSE](LICENSE).
