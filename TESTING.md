# Testing without hardware

Three layers, each answering a different question:

| Layer | Command | Question it answers |
|---|---|---|
| Unit tests | `pio test -e native` | Does this driver do what its datasheet says, byte for byte? |
| Flight simulator | `make -C sim run` | Does the whole firmware survive a flight, with every fitted device attached? |
| Round-trip check | `python sim/check_roundtrip.py` | Can the ground-station analysis library read what we produced? |

The target firmware is untouched by all of this: `pio run -e stm32g474` still
builds the same image (~45.5 KB flash / 2.8 KB RAM, verified after the work
below).

---

## 1. Unit tests

```
pio test -e native                 # all 13 suites, 106 cases
pio test -e native -f test_bmp581  # one suite
```

`[env:native]` uses PlatformIO's `native` platform with Unity. Each suite is a
**directory** `test/test_<name>/test_main.c` with its own `main()`; a flat
`test/test_*.c` layout would compile every suite into one binary and collide on
`main`/`setUp`.

A suite compiles the code it tests by including it:

```c
#include "../fakes/fake_hal.c"          /* stub HAL + scriptable buses   */
#include "../fakes/fake_devices.c"      /* register-level chip models    */
#include "../../src/board.c"             /* hi2c1/hspi1/htim8 objects     */
#include "../../src/i2c_dev.c"
#include "../../src/bmp581.c"
```

The stub `test/fakes/stm32g4xx_hal.h` shadows the vendor header purely through
include order (`-I${PROJECT_DIR}/test/fakes` before `-I${PROJECT_DIR}/src`), so
`src/*.c` compiles unmodified on the host. When a new source file uses a HAL
symbol the stub lacks, add it there — never add host-only code to `src/`.

### What each suite pins down

| Suite | Cases | Golden vectors / behaviour |
|---|---|---|
| `test_i2c_crc8` | 7 | Sensirion CRC-8 (`0xBEEF`→0x92, `"123456789"`→0xF7) |
| `test_telemetry` | 8 | JSON/CSV field order, packet-size bound, health bitmask 0x3FF |
| `test_gps` | 8 | Real `USART1_IRQHandler` fed byte by byte; NMEA checksums, N/S/E/W, void and no-fix sentences |
| `test_bmp581` | 9 | Calibrated 24-bit words, OSR/ODR register image, ISA altitude sweep to 2 km |
| `test_scd30` | 10 | Command framing with CRC'd argument words, three-float measurement reassembly, CRC reject (`-3`) |
| `test_sht45` | 9 | Two-call poll state machine, 10 ms conversion NACK, 1 Hz cadence, CRC drop, clamping |
| `test_mlx90395` | 9 | Command/status protocol, SM-mode and DRDY gates, 0.714 µT/LSB scaling, signed full scale |
| `test_ltr390` | 7 | Part-ID gate, gain/rate registers, 18-bit UVS, UV index formula |
| `test_tsl2591` | 9 | Command-bit addressing, channel split, lux formula (34.816 at 1000/200 counts) |
| `test_radio_e22` | 7 | Packets leave on USART3 (not the debug UART), payload integrity, 255-byte packet, empty/NULL rejection |
| `test_rad_counter` | 8 | EXTI pulse counting, destructive latch, 60 s activity window, foreign pins ignored |
| `test_servos` | 8 | Stow/deploy presets, 500–2500 µs guard rail, invalid channel handling |
| `test_adc` | 7 | Per-call channel routing, one conversion per call, 0/4095 endpoints, VBAT divider |

### Device fakes

`test/fakes/fake_devices.c` models chips at the register/protocol level, not by
reimplementing the drivers, so a driver bug cannot be "fixed" by a matching
fake bug:

* **BMP581** — calibrated 24-bit temperature/pressure words synthesised from
  an environment; the OSR/ODR register image is asserted after init.
* **SCD30** — big-endian commands with CRC'd argument words; a bad CRC is a
  NACK, and measurement CRCs can be corrupted on demand.
* **SHT45** — genuinely NACKs reads until the 10 ms conversion has elapsed, so
  the driver's polling state machine is tested, not assumed.
* **E22** — the radio is a UART pipe, so the fake HAL records every
  `HAL_UART_Transmit` with its handle and the test asserts USART3 (not USART2)
  received the packet.
* **TSL2591 / LTR390** — register files with the protocol quirks the drivers
  depend on (command bit, status bytes).
* **MLX90395** — SPI command/status frames with the SMMODE and DRDY gates.

### Hardware corrections reflected in `src/`

The earlier BMP280 driver had a temperature scaling bug (the datasheet 0.01 °C integer was not divided by 100). That part has since been replaced by the SPI BMP581 driver, whose calibrated 24-bit words need no compensation maths. The current `src/` changes are the hardware-matching ones described in the changelog and `README.md`.

### Caveat to confirm on the bench: LTR390 command bit

The LTR390 driver addresses registers *without* the datasheet's `0x10` command
bit (`LTR390_PART_ID` is written as `0x06`, not `0x16`). Real silicon expects
the bit, so the real chip may not answer. The fake masks bit 7 so both
conventions work, which means **the unit tests cannot prove the bus sequence is
right** — only the arithmetic. If the LTR390 reads 0 on hardware, set
`#define LTR390_CMD 0xA0`-style addressing (command bit included) and re-run
`test_ltr390` after dropping the mask in the fake. The same question applies to
TSL2591, which the driver *does* address with `0xA0`.

---

## 2. Flight simulator

```
make -C sim run                  # builds and flies
make -C sim check                # fly, then load sim/out/* through the analysis library
python sim/check_roundtrip.py    # validate existing sim/out/* without re-flying
```

`sim/` links the real `src/*.c` (minus `sdlog.c`, replaced by a host file sink,
and `stm32g4xx_it.c`) against the same stub HAL and device models the unit
tests use. `src/main.c` still owns `main()` and still runs its own superloop;
`sim/world.c` installs a constructor that attaches the devices and a tick hook
that turns every `HAL_GetTick()` into 1 ms of simulated time.

Profile (closed form, solved in `sim/world.c`):

| Phase | Time | State |
|---|---|---|
| Pad | 0–5 s | 0 m, 1 g, 1013.25 hPa |
| Boost | 5–8 s | 35.8 m/s² net → 107.5 m/s at 161 m |
| Coast | 8–18.96 s | free fall, 0 g |
| Descent | 18.96–93.96 s | parachute, exactly 10 m/s, 750 m → 0 m |
| Landed | +2 s | 1 g, then the process exits |

Everything the firmware samples is driven from that state: BMP581 pressure and
temperature (ISA), SHT45/SCD30 temperature and humidity, LTR390 UV and
TSL2591 counts scaling with altitude,
a fixed magnetometer field, ADC channels for O3 and the 4.02 V pack, 4 rad
pulses/s on the EXTI line, and 1 Hz `$GPGGA`/`$GPRMC` sentences pushed byte by
byte through the firmware's real `USART1_IRQHandler`. The E22 is captured by a
UART TX hook, so `lora.jsonl` contains exactly what went to USART3.

Outputs (git-ignored):

* `sim/out/sd.csv` — exactly what `telemetry_csv_row()` would write to the card,
  ~10 Hz (960 rows for the nominal flight).
* `sim/out/lora.jsonl` — every packet the E22 driver actually wrote to USART3,
  ~1 Hz (95 frames), one JSON object per line.
* The console shows phase transitions, the firmware's boot banner and I2C scan,
  and each JSON packet as `main()` prints it.

The process exits 2 s after touchdown because the real superloop never returns.

---

## 3. Round-trip check

`sim/check_roundtrip.py` loads both artifacts through
`analysis/cansat_data.py` — the same `PACKET_MAP`/`SD_MAP` the ground station
uses — and asserts 16 properties, including:

* both files parse into `CanSatData`, timestamps strictly monotonic;
* apogee 750.1 m (within ±5 %), descent 10.00 m/s (within ±1 m/s);
* landing detected and 1–2 s of landed data present;
* E22 apogee agrees with the SD card and the frame rate is ~1 Hz;
* ISA temperature at apogee;
* GPS fix at the simulated site, altitude within 60 m of the barometer;
* all 10 health bits set in both channels (`0x3FF`).

It exits non-zero on the first failed check, so it can gate CI.

---

## Limits

This harness proves logic and integration, not electrical reality:

* No I2C/SPI timing, bus arbitration, clock-stretching or supply effects.
* No interrupt latency or preemption; the ISR is called synchronously.
* ADC is ideal (no noise, offset or reference error); pressure is quantised
  only by our BMP581 raw-word quantisation.
* FatFS, the SD card and the radio's RF performance are modelled only at the
  file/packet level.
* The LTR390 addressing caveat above.

What it does catch: wrong register values, wrong scaling or byte order, broken
CRC handling, off-by-one framing, state machines that never time out, JSON/CSV
drift, health-bit regressions and any change that breaks the flight profile
end to end.

## Adding a suite

1. `mkdir test/test_<name>` and add `test_main.c` with `setUp`, `tearDown` and
   `main()` (`UNITY_BEGIN(); RUN_TEST(...); return UNITY_END();`).
2. Include `fake_hal.c`, `fake_devices.c`, the `src` files under test, and
   `board.c` if you touch `hi2c1`/`hspi1`/`htim8`.
3. `fake_hal_reset()` + `fdev_reset()` + `fdev_<chip>_enable()` in `setUp`.
4. Run `pio test -e native -f test_<name>`.

Two traps worth knowing:

* PlatformIO reports a suite that exits with code *N* > 0 as
  `Program received signal SIG… (N)` and `[ERRORED]`, not `[FAILED]`. The
  per-test lines above it are still accurate; run
  `.pio/build/native/program` directly to see the real exit code.
* If a suite includes both `fake_hal.c` and a file that defines
  `HAL_GPIO_EXTI_Callback` (only `rad_counter.c`), define
  `FAKE_HAL_NO_DEFAULT_EXTI_CB` before the include — two definitions in one
  translation unit are an error even though both are weak.
