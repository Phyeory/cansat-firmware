# Celestium CanSat PCB checklist

The sensors, radio and GPS are soldered straight onto the custom PCB, so
there are no wires to move. Use this to check the **schematic before
ordering** and the **assembled board before first power-up**. Every net
here must match `src/config.h`; if the PCB differs, change `config.h`
(and `stm32g474.ioc`), not the copper.

Everything runs on the **3V3 rail** with **one shared GND**, except the
servos (own 5 V supply, GND still shared).

## 1. STM32G474RE itself

- [ ] 100 nF on every VDD pin, plus one 4.7 µF bulk cap on 3V3.
- [ ] VDDA has its own 100 nF + 1 µF, and **VREF+ is tied to 3V3**
      (the firmware assumes a 3.3 V ADC reference: `VREF_V`).
- [ ] NRST has a 100 nF cap to GND.
- [ ] SWD header for flashing: SWDIO (PA13), SWCLK (PA14), NRST, 3V3, GND.
- [ ] Debug pads on PA2 (MCU TX) and PA3 (MCU RX) for the 115200 console.

### ⚠ PB8 is also BOOT0

PB8 is both I2C SCL and the BOOT0 pin. The I2C pull-up on the PCB holds it
high at reset, so the chip boots the ST bootloader and **your firmware
never runs**. Fix once per chip, before anything else, in
**STM32CubeProgrammer → Option bytes → User Configuration**: untick
**nSWBOOT0**, tick **nBOOT0**, Apply. After that the pin is ignored.

## 2. I2C sensor bus (I2C1, 100 kHz)

All seven sensors share SCL = **PB8** and SDA = **PB9**.

- [ ] **One** 4.7 kΩ pull-up from SCL to 3V3 and **one** from SDA to 3V3.
      Bare chips have none built in. The MCU's internal pull-ups are only
      a weak backup and are not enough on their own.
- [ ] 100 nF next to each sensor's VDD pin.

| Chip | SCL / SDA | Strapping pins that must be set | Address |
|---|---|---|---|
| BMP280 | SCK = PB8, SDI = PB9 | **CSB → 3V3** (else it may start in SPI mode). **SDO → GND** for 0x76. VDD and VDDIO both on 3V3. | 0x76 (0x77 if SDO → 3V3: change `BMP280_I2C_ADDR`) |
| LTR390 | SCL = PB8, SDA = PB9 | INT unused; leave unconnected or pulled up. | 0x53 (fixed) |
| TSL2591 | SCL = PB8, SDA = PB9 | INT unused; leave unconnected or pulled up. | 0x29 (fixed) |
| SCD30 | TX/SCL = PB8, RX/SDA = PB9 | **SEL → GND or floating** (I2C mode). RDY and PWM unused. | 0x61 (fixed) |
| BMI270 | SCL = PB8, SDA = PB9 | **SDO → GND** for 0x68 (SDO → 3V3 = 0x69: change `BMI270_I2C_ADDR`). PS → 3V3. | 0x68 |
| MLX90395 | SCL = PB8, SDA = PB9 | **A1 → GND, A0 → GND** for 0x0C. INT unused. | 0x0C |
| SHT45 | SCL = PB8, SDA = PB9 | No strapping pins; VDD and VDDIO on 3V3. | 0x44 (fixed) |

The SCD30 is why the bus runs at 100 kHz: it isn't rated for 400 kHz and
it uses clock stretching.

## 3. Ozone (Alphasense O3-B4, ADC2)

The cell's electrodes **cannot** connect straight to the MCU. The PCB needs
a potentiostat front end (op-amps doing the job of Alphasense's ISB).

- [ ] Potentiostat circuit present for the working and auxiliary electrodes.
- [ ] Working-electrode output → **PA0** (ADC2_IN1).
- [ ] Auxiliary-electrode output → **PA1** (ADC2_IN2).
- [ ] Both outputs can never exceed 3.3 V.
- [ ] 100 nF from PA0 and PA1 to GND helps the ADC settle.

## 4. Radiation pulse counter (BPW34 S-Z + comparator)

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

## 5. Battery monitor

- [ ] Battery + → 100 kΩ → **PC4** → 100 kΩ → GND (2:1 divider).
- [ ] 100 nF from PC4 to GND.
- [ ] Battery never above 6.6 V.

PC4 is ADC2_IN5 and is **not** on ADC1.

## 6. GPS (u-blox SAM-M8Q, USART1, 9600 baud)

| SAM-M8Q pin | Net |
|---|---|
| TXD | **PA10** (MCU RX) |
| RXD | **PA9** (MCU TX) |
| VCC | 3V3, with decoupling per the u-blox integration manual |
| V_BCKP | 3V3 or a backup cell, never floating |
| RESET_N, SAFEBOOT_N | leave unconnected (internal pull-ups) |
| GND | GND |

- [ ] TX goes to RX. If the PCB has them crossed the wrong way, **don't cut
      traces**: USART1 can swap TX/RX in firmware (`UART_ADVFEATURE_SWAP_INIT`).
- [ ] Ground plane and keep-out around the module follow the SAM-M8Q
      integration manual. Nothing tall or metal over the antenna face.
- [ ] The antenna faces the sky when the can is in flight orientation.

## 7. Radio (RFM96W LoRa, SPI1)

| RFM96W pin | Net |
|---|---|
| SCK | PA5 |
| MISO | PA6 |
| MOSI | PA7 |
| NSS | PB0 |
| RESET | PB1 |
| DIO0 | PA8 |
| 3.3V | 3V3, with 10 µF + 100 nF right at the pin (TX pulls ~120 mA) |
| GND | GND, all GND pads soldered |
| ANA (antenna) | short, direct trace to an SMA/u.FL connector or a 17.3 cm wire pad |

- [ ] **Never transmit without an antenna attached.** It can damage the PA.

## 8. microSD (SPI2)

| Socket pin | Net |
|---|---|
| CLK | PB13 |
| DAT0 / DO | PB14 (with 10 kΩ pull-up to 3V3) |
| CMD / DI | PB15 |
| DAT3 / CS | PB12 (with 10 kΩ pull-up to 3V3) |
| VDD | 3V3, with 10 µF + 100 nF |
| VSS | GND |

## 9. Spoiler servos (TIM8 PWM)

- [ ] Signal pads on **PC6, PC7, PC8, PC9** (CH1 to CH4).
- [ ] Servo power from a separate 5 V regulator/BEC, never the 3V3 rail.
- [ ] Servo GND joined to board GND.

## First power-up

1. **Before power:** check with a multimeter that 3V3 isn't shorted to GND.
2. **Power on with nothing flashed:** check 3V3 measures ~3.3 V and nothing
   gets hot.
3. **Set the option bytes** (PB8/BOOT0 fix above) over SWD.
4. **Flash** (`pio run -t upload`) and open the console (`pio device monitor`).
5. **Read the boot report:**
   - I2C scan should list `0x0C 0x29 0x44 0x53 0x61 0x68 0x76`. A missing
     address means that chip's solder joints, strapping pins or power.
   - `NONE` means the pull-ups, PB8/PB9 routing or the 3V3 rail.
   - `lora=1` means the radio answered over SPI. `lora=0` means check
     SPI1, PB0 and PB1 joints.
6. **Watch the JSON lines.** `"h"` is the health bitmask: bit0 BMP, 1 LTR,
   2 TSL, 3 SCD, 4 GPS data arriving, 5 GPS fix, 6 SD, 7 BMI270, 8 MLX90395,
   9 SHT45, 10 RAD counter (1 = a pulse seen in the last 60 s). All
   sensors + GPS data with no fix yet = 991; with a fix = 1023. Bit 10
   only lights once the BPW34 front end pulses. The BMI270
   also refuses to produce data until its 8 KB config upload finishes —
   if `bmi=0` in the boot report, check the 0x68 ack first.
7. **Ground side:** flash `ground-station/lora_receiver`, run `station.py`,
   and check the same packets arrive with RSSI.
