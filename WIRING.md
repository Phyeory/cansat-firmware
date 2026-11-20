# Celestium CanSat PCB checklist

The sensors, radio and GPS are soldered straight onto the custom PCB, so
there are no wires to move. Use this to check the **schematic before
ordering** and the **assembled board before first power-up**. Every net
here must match `src/config.h`; if the PCB differs, change `config.h`
(and `stm32g474.ioc`), not the copper.

Everything runs on the **3.0 V rail** with **one shared GND**, except the
servos (own 5 V supply, GND still shared). `VREF_V = 3.0f` in `config.h`
assumes VREF+ is tied to that rail.

## 1. STM32G474RE itself

- [ ] 100 nF on every VDD pin, plus one 4.7 µF bulk cap on 3.0 V.
- [ ] VDDA has its own 100 nF + 1 µF, and **VREF+ is tied to 3.0 V**
      (the firmware assumes a 3.0 V ADC reference: `VREF_V`).
- [ ] NRST has a 100 nF cap to GND.
- [ ] SWD header for flashing: SWDIO (PA13), SWCLK (PA14), NRST, 3.0 V, GND.
- [ ] Debug pads on PA2 (MCU TX) and PA3 (MCU RX) for the 115200 console.

### ⚠ PB8 is also BOOT0

PB8 is both I2C SCL and the BOOT0 pin. The I2C pull-up on the PCB holds it
high at reset, so the chip boots the ST bootloader and **your firmware
never runs**. Fix once per chip, before anything else, in
**STM32CubeProgrammer → Option bytes → User Configuration**: untick
**nSWBOOT0**, tick **nBOOT0**, Apply. After that the pin is ignored.

## 2. SPI sensor bus (SPI1, mode 3)

The magnetometer and barometer are **both SPI**, on one shared bus. They
each have their own chip select:

| Device | SPI signal | CS | Notes |
|---|---|---|---|
| MLX90395 | SCK = PA5, MISO = PA6, MOSI = PA7 | **PB1** | Mode 3, 16-bit command frames |
| BMP581 | same bus | **PA8** | Mode 3, 24-bit calibrated data |

PB0 is unused on this build (it was the BMI270 chip select before the IMU was
removed), so leave it unconnected or tie it high.

- [ ] One 100 nF decoupling cap per device, right at its VDD pin.
- [ ] Both CS lines have 10 kΩ pull-ups to 3.0 V, so they stay inactive
      while the MCU is resetting.
- [ ] The BMP581 starts in I²C mode and switches to SPI on the first CS edge;
      the driver performs the datasheet's dummy read before using it.
- [ ] Keep the SPI1 tracks short and away from the GNSS antenna feed.

## 3. I2C sensor bus (I2C1, 100 kHz)

The remaining I²C sensors share SCL = **PB8** and SDA = **PB9**.

- [ ] **One** 4.7 kΩ pull-up from SCL to 3.0 V and **one** from SDA to 3.0 V.
      Bare chips have none built in. The MCU's internal pull-ups are only
      a weak backup and are not enough on their own.
- [ ] 100 nF next to each sensor's VDD pin.

| Chip | SCL / SDA | Strapping pins that must be set | Address |
|---|---|---|---|
| LTR390 | SCK = PB8, SDI = PB9 | INT unused; leave unconnected or pulled up. | 0x53 (fixed) |
| TSL2591 | SCK = PB8, SDI = PB9 | INT unused; leave unconnected or pulled up. | 0x29 (fixed) |
| SCD30 | TX/SCL = PB8, RX/SDA = PB9 | **SEL → GND or floating** (I2C mode). RDY and PWM unused. | 0x61 (fixed) |
| SHT45 | SCK = PB8, SDI = PB9 | No strapping pins; VDD and VDDIO on 3.0 V. | 0x44 (fixed) |

The SCD30 is why the bus runs at 100 kHz: it isn't rated for 400 kHz and
it uses clock stretching.

## 4. Ozone (Alphasense O3-B4, ADC2)

The cell's electrodes **cannot** connect straight to the MCU. The PCB needs
a potentiostat front end (op-amps doing the job of Alphasense's ISB).

- [ ] Potentiostat circuit present for the working and auxiliary electrodes.
- [ ] Working-electrode output → **PA0** (ADC2_IN1).
- [ ] Auxiliary-electrode output → **PA1** (ADC2_IN2).
- [ ] Both outputs can never exceed 3.0 V.
- [ ] 100 nF from PA0 and PA1 to GND helps the ADC settle.

## 5. Radiation pulse counter (BPW34 S-Z + comparator)

A bare photodiode cannot drive a GPIO — the PCB needs a front end:

- [ ] BPW34 S-Z photodiode into a **transimpedance amplifier** (feedback
      resistor ~1-10 MΩ for pulse work) with a reverse-bias option.
- [ ] TIA output into a **comparator** (e.g. TLV7011) thresholded just above
      the noise floor; open-drain output preferred.
- [ ] Comparator output → **PB10** (EXTI10). Internal pull-up holds it high;
      every pulse pulls it low. Never drive it push-pull from an op-amp
      powered off 5 V.
- [ ] 100 nF at the TIA supply pin; guard-ring / keep the input node short.
- [ ] Bench check: tapping a bright light at the diode should make the
      `rad` field in the JSON climb.

## 6. Battery monitor

- [ ] Battery + → 100 kΩ → **PC4** → 100 kΩ → GND (2:1 divider).
- [ ] 100 nF from PC4 to GND.
- [ ] Battery never above 6.6 V.

PC4 is ADC2_IN5 and is **not** on ADC1.

## 7. GPS (u-blox SAM-M8Q, USART1, 9600 baud)

| SAM-M8Q pin | Net |
|---|---|
| TXD | **PA10** (MCU RX) |
| RXD | **PA9** (MCU TX) |
| VCC | 3.0 V, with decoupling per the u-blox integration manual |
| V_BCKP | 3.0 V or a backup cell, never floating |
| RESET_N, SAFEBOOT_N | leave unconnected (internal pull-ups) |
| GND | GND |

- [ ] TX goes to RX. If the PCB has them crossed the wrong way, **don't cut
      traces**: USART1 can swap TX/RX in firmware (`UART_ADVFEATURE_SWAP_INIT`).
- [ ] Ground plane and keep-out around the module follow the SAM-M8Q
      integration manual. Nothing tall or metal over the antenna face.
- [ ] The antenna faces the sky when the can is in flight orientation.

## 8. Radio (EBYTE E22-900T22D, USART3)

The E22 is a transparent UART module: no SPI, no RF layout, no library.

| E22 pin | Net |
|---|---|
| TXD | **PC11** (MCU RX, USART3_RX) |
| RXD | **PC10** (MCU TX, USART3_TX) |
| M0 | GND (transmission mode) |
| M1 | GND (transmission mode) |
| AUX | optional; leave unconnected (firmware uses a blocking UART write) |
| VCC | 3.0 V, with 10 µF + 100 nF right at the pin |
| GND | GND |

- [ ] Configure the module before flight: **868.125 MHz, 14 dBm, 62.5 kbps,
      transparent transmission**, UART 9600 8N1. Use EBYTE's PC tool or its
      `C0` command mode; settings are stored in the module.
- [ ] Configure the ground **E22-900T22U** identically.
- [ ] **Never transmit without an antenna attached.** It can damage the PA.
- [ ] Quarter-wave at 868 MHz is 8.6 cm; a helical fits in ~2 cm at some loss.
      Either needs a ground plane, and the antenna must stay well away from the
      GNSS patch (the 868 MHz 2nd harmonic at 1736 MHz is close to GPS L1).

## 9. microSD (SPI2)

| Socket pin | Net |
|---|---|
| CLK | PB13 |
| DAT0 / DO | PB14 (with 10 kΩ pull-up to 3.0 V) |
| CMD / DI | PB15 |
| DAT3 / CS | PB12 (with 10 kΩ pull-up to 3.0 V) |
| VDD | 3.0 V, with 10 µF + 100 nF |
| VSS | GND |

## 10. Spoiler servos (TIM8 PWM)

- [ ] Signal pads on **PC6, PC7, PC8, PC9** (CH1 to CH4).
- [ ] Servo power from a separate 5 V regulator/BEC, never the 3.0 V rail.
- [ ] Servo GND joined to board GND.

## First power-up

1. **Before power:** check with a multimeter that the 3.0 V rail isn't shorted to GND.
2. **Power on with nothing flashed:** check the 3.0 V rail measures ~3.0 V and nothing
   gets hot.
3. **Set the option bytes** (PB8/BOOT0 fix above) over SWD.
4. **Flash** (`pio run -t upload`) and open the console (`pio device monitor`).
5. **Read the boot report:**
   - I2C scan should list `0x29 0x44 0x53 0x61`. A missing address means that
     chip's solder joints, strapping pins or power.
   - `NONE` means the pull-ups, PB8/PB9 routing or the 3.0 V rail.
   - `baro=1`, `mlx=1` mean the two SPI devices answered. A zero usually
     means the CS line, the SPI1 mode/pins or that device's power.
   - `radio=1` means USART3 initialised. `radio=0` means check PC10/PC11.
6. **Watch the JSON lines.** `"h"` is the health bitmask: bit0 BMP581, 1 LTR,
   2 TSL, 3 SCD, 4 GPS data arriving, 5 GPS fix, 6 SD, 7 MLX90395,
   8 SHT45, 9 RAD counter (1 = a pulse seen in the last 60 s). All ten
   bits set reads as **1023 (0x3FF)**; bit 9 only lights once the BPW34
   front end pulses.
7. **Ground side:** connect the E22-900T22U USB module, run `station.py`,
   and check the same packets arrive.
