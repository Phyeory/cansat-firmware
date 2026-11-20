# Kalman filter noise — Q / R / P₀

> Companion to [DATA-PIPELINE.md](DATA-PIPELINE.md) §6. ★ = planned online
> (STM32). **No IMU is fitted** — the BMI270 was removed on 2026-11-19, so
> accel/gyro are not available and no attitude or inertial-trajectory EKF can
> run. Only the barometric altitude/velocity filter is applicable. **Neither
> filter exists in `src/` yet — this file is a design target.**
>
> Values = **typ** from Bosch BST-BMP581-DS004, Melexis MLX90395-DS (rev 006)
> and u-blox SAM-M8Q-DS (UBX-16012619). Firmware config: BMP581 OSR pressure
> **×16**, temperature ×2, ODR **50 Hz**, normal mode (`bmp581.c` `0x61`/`0xBD`);
> log dt = **0.1 s**. 🔧 = not datasheet'd — tune on bench/flight data.
>
> **Form**: the online filter is genuinely linear → **constant Q/R is the
> correct form**. Only $R_{baro}(h)$ and $R_{gps}(\mathrm{DOP})$ vary with the
> state; the rest can stay constant.

## 0. Datasheet noise → per-sample σ

| Sensor | Datasheet spec | Per-sample σ | Feeds |
|---|---|---|---|
| BMP581 P | RMS 0.21 Pa (×16 OSR, full BW) | **σ_h = 0.0175 m** (hypsometric, ρg ≈ 12 Pa/m) | planned online R |
| BMP581 P | TC offset ±0.5 Pa/K ≡ ±0.042 m/K | slow h-bias walk **σ_tc = 0.042 m/√s** 🔧 | planned online Q |
| MLX90395 | RES-16 class ≈ 3 µT rms 🔧 (quantization floor 0.21 µT @ 0.714 µT/LSB) | **σ_B = 3 µT/axis** (post cal) | science only — no attitude EKF without an IMU |
| SAM-M8Q | 2.5 m CEP horizontal (autonomous); velocity 0.05 m/s | **σ_EN₀ = 2.5 m, σ_V₀ = 5 m** (2×H rule, not spec'd); σ_v = 0.05 m/s | offline trajectory/plots |

**Noise-function inventory** — the nonlinearity lives in the measurement model
H(x), and R inherits it:

- BMP581 → h: $R_{baro}(h) = \left[\dfrac{\sigma_P}{\rho(h)\,g}\right]^2, \quad \rho(h) = 1.225\,(1 - 2.2558\cdot10^{-5} h)^{4.2559}$ → σ_h = **0.018 / 0.019 / 0.021 m** at 0 / 1 / 2 km
- GPS NED: $R_{gps}(\mathrm{DOP}) = \mathrm{diag}\big[(2.5\,\mathrm{HDOP})^2,\ (2.5\,\mathrm{HDOP})^2,\ (5\,\mathrm{VDOP})^2\big]$ m² (HDOP/VDOP from NMEA $GSA; baselines = spec at DOP 1 🔧)

## 1. ★ Online linear KF — x = [h, v, b], dt = 0.1 s

State: altitude `h` (m), vertical velocity `v` (m/s), barometric altitude bias
`b` (m). The only measurement is the BMP581-derived altitude:

$$R = \sigma_h^2 = (0.0175)^2 = \mathbf{3.1\cdot10^{-4}\ m^2}, \qquad H = [1\ 0\ 0]$$

$$Q = \sigma_a^2\,GG^{\mathsf T} + \mathrm{diag}(\sigma_{tc}^2\,\mathrm{d}t,\ 0,\ \sigma_b^2\,\mathrm{d}t), \quad G = \begin{bmatrix}\mathrm{d}t^2/2\\ \mathrm{d}t\\ 0\end{bmatrix}$$

There is **no accelerometer**, so $\sigma_a$ is not a sensor noise density: it
represents unmodelled vertical motion (gusts, parachute snatch, deployment
jolts) and must be tuned from flight data — start around 0.1–0.5 m/s² 🔧.
$\sigma_b$ is the baro-bias random walk, also bench-tuned. The filter remains
linear (constant F/H); only $R_{baro}(h)$ should be re-evaluated per step
(3.1·10⁻⁴ → 3.7·10⁻⁴ m² over 0 → 1 km, +19 %/km).

$$P_0 = \mathrm{diag}\big((0.5\ \mathrm{m})^2,\ (0.1\ \mathrm{m/s})^2,\ (0.5\ \mathrm{m})^2\big) = \mathrm{diag}(0.25,\ 0.01,\ 0.25)$$

Only nonlinearity online is P → h (hypsometric): linearising it with
$\partial h/\partial P = -1/(\rho g)$ is what makes this an LKF — no other
state-dependent noise exists, constant Q/R is the exact form.

## 2. ☆ Attitude EKF — not available

An attitude EKF (MEKF) needs **both** accel (gravity direction) and gyro
(angular rate). With no IMU fitted, the MLX90395 magnetometer can only give a
heading once the platform is level and hard/soft-iron calibrated; it cannot
observe roll or pitch by itself. Do not implement §2-style fusion until an IMU
is added to the hardware and `flight_frame_t`.

## 3. ☆ Trajectory EKF + RTS — not available

An inertial trajectory EKF needs accel/gyro as the process input. The current
sensor set supports only baro + GNSS post-flight comparisons (Δh, drift, descent
rate), not a 3D inertial solution.

## Sources

- [Bosch BMP581 datasheet (BST-BMP581-DS004)](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf) — RMS noise 0.21 Pa (×16), TC offset ±0.5 Pa/K
- [Melexis MLX90395 datasheet (rev 004)](https://media.melexis.com/-/media/files/documents/datasheets/mlx90395-datasheet-melexis.pdf) — noise vs RES/OSR table; high-res ≈ 3 µT class 🔧 verify vs our GAIN_SEL
- [u-blox SAM-M8Q datasheet (UBX-16012619)](https://content.u-blox.com/sites/default/files/documents/SAM-M8Q_DataSheet_UBX-16012619.pdf) — 2.5 m CEP, 0.05 m/s
- Magnetic field values: WMM (UK 2026: \|B\| ≈ 49 µT, inclination ≈ 69°); ISA troposphere for ρ(h)
