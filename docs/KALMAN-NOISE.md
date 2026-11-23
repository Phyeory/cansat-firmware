# Kalman filter noise — Q / R / P₀

> Companion to [DATA-PIPELINE.md](DATA-PIPELINE.md) §6. ★ = planned online
> (STM32) · ☆ = planned offline (Python). **Neither the KF nor either EKF
> exists in `src/` yet — this file is a design target.**
>
> Values = **typ** from Bosch BST-BMP581-DS004, ST LSM6DSV datasheet,
> Melexis MLX90395-DS (rev 006) and u-blox SAM-M8Q-DS (UBX-16012619).
> Firmware config: BMP581 OSR pressure **×16**, temperature ×2, ODR **50 Hz**,
> normal mode (`bmp581.c` `0x61`/`0xBD`); LSM6DSV **120 Hz ODR**, ±8 g,
> ±2000 dps, BDU + IF_INC (`lsm6dsv.c` `0x10`/`0x11`/`0x12`/`0x15`/`0x17`);
> log dt = **0.1 s**. 🔧 = not datasheet'd — tune on bench/flight data.
>
> **Form**: the online filter is genuinely linear → **constant Q/R is the
> correct form** (§1); the offline EKFs get **noise functions** Q(x,t),
> R(x, z) evaluated at each update (§2–§3). Noise that is genuinely
> state-dependent: R_acc(â), R_ψ(x, m), Q_vv(P_θθ), R_gps(DOP), R_baro(h).
> Gyro process noise has **no** state dependence (sensor property) — the state
> enters through the transition matrix Φ, not through Q.

## 0. Datasheet noise → per-sample σ

| Sensor | Datasheet spec | Per-sample σ | Feeds |
|---|---|---|---|
| BMP581 P | RMS 0.21 Pa (×16 OSR, full BW) | **σ_h = 0.0175 m** (hypsometric, ρg ≈ 12 Pa/m) | planned online R · trajectory R |
| BMP581 P | TC offset ±0.5 Pa/K ≡ ±0.042 m/K | slow h-bias walk **σ_tc = 0.042 m/√s** 🔧 | planned online Q |
| LSM6DSV acc | noise density 90 µg/√Hz | **σ_a = 0.0044 m/s²** (= 90 µg·√25 Hz) | online Q · attitude R |
| LSM6DSV acc | zero-g offset ±20 mg | → b-state P₀ | online P₀ |
| LSM6DSV gyro | noise density 2.8 mdps/√Hz | **ARW = 4.89·10⁻⁵ rad/√s**; per-sample σ_ω = 0.014 °/s = 2.44·10⁻⁴ rad/s (BW 25 Hz) | attitude Q |
| LSM6DSV gyro | zero-rate offset ±0.5 °/s | → bg-state P₀ | attitude P₀ |
| MLX90395 | RES-16 class ≈ 3 µT rms 🔧 (quantization floor 0.21 µT @ 0.714 µT/LSB) | **σ_B = 3 µT/axis** (post cal) | attitude R |
| SAM-M8Q | 2.5 m CEP horizontal (autonomous); velocity 0.05 m/s | **σ_EN₀ = 2.5 m, σ_V₀ = 5 m** (2×H rule, not spec'd); σ_v = 0.05 m/s | trajectory R |

**Noise-function inventory** — the nonlinearity lives in the measurement model
H(x), and R inherits it:

- BMP581 → h: $R_{baro}(h) = \left[\dfrac{\sigma_P}{\rho(h)\,g}\right]^2, \quad \rho(h) = 1.225\,(1 - 2.2558\cdot10^{-5} h)^{4.2559}$ → σ_h = **0.018 / 0.019 / 0.021 m** at 0 / 1 / 2 km
- GPS NED: $R_{gps}(\mathrm{DOP}) = \mathrm{diag}\big[(2.5\,\mathrm{HDOP})^2,\ (2.5\,\mathrm{HDOP})^2,\ (5\,\mathrm{VDOP})^2\big]$ m² (HDOP/VDOP from NMEA $GSA; baselines = spec at DOP 1 🔧)
- Accel direction: $R_{acc}(\hat a) = \left(\dfrac{\sigma_a}{|a|}\right)^2 (I_3 - \hat a \hat a^{\mathsf T})$ (rank 2, no info along â)
- Mag → heading: $\sigma_\psi = \dfrac{\sigma_B}{m_H}$, tilt-coupled term below (§2)

## 1. ★ Online linear KF — x = [h, v, b], dt = 0.1 s

$$R = \sigma_h^2 = (0.0175)^2 = \mathbf{3.1\cdot10^{-4}\ m^2}, \qquad H = [1\ 0\ 0]$$

$$Q = \sigma_a^2\,GG^{\mathsf T} + \mathrm{diag}(\sigma_{tc}^2\,\mathrm{d}t,\ 0,\ \sigma_b^2\,\mathrm{d}t), \quad G = \begin{bmatrix}\mathrm{d}t^2/2\\ \mathrm{d}t\\ 0\end{bmatrix}$$

$\sigma_a = 0.0044$ m/s² is the LSM6DSV noise floor; in flight use a larger
buffeting term (≈0.05 m/s² 🔧) so the filter does not trust the accelerometer
through parachute snatch. **Offline (Python, `analysis/kalman.py`) the
smoother uses σ_a ≈ 0.5 🔧** — with the full record it can afford sharper
phase transitions (validated: v RMSE 0.08 m/s vs 0.99 at 0.05 on the
synthetic flight in `test_kalman.py`). $\sigma_b = 20\ \mathrm{\mu g}/\sqrt{\mathrm{s}}$ is
the accelerometer bias walk.

$$P_0 = \mathrm{diag}\big((0.5\ \mathrm{m})^2,\ (0.1\ \mathrm{m/s})^2,\ (20\ \mathrm{mg}=0.196\ \mathrm{m/s^2})^2\big) = \mathrm{diag}(0.25,\ 0.01,\ 0.038)$$

Only nonlinearity online is P → h (hypsometric): linearising it with
$\partial h/\partial P = -1/(\rho g)$ is what makes this an LKF — no other
state-dependent noise exists, constant Q/R is the exact form. Optional:
evaluate R = σ_h²(h) per step (3.1·10⁻⁴ → 3.7·10⁻⁴ m² over 0 → 1 km,
+19 %/km).

## 2. ☆ Attitude EKF (MEKF) — error state [δθ (rad), δ_bg (rad/s)]

**Process noise** — constant PSDs, closed-form dt-coupling (per axis); ω enters
Φ, not Q:

$$Q_k = \begin{bmatrix} \big(\sigma_{ARW}^2\,\mathrm{d}t + \tfrac{\sigma_{bg}^2\,\mathrm{d}t^3}{3}\big) I_3 & -\tfrac{\sigma_{bg}^2\,\mathrm{d}t^2}{2} I_3 \\[4pt] -\tfrac{\sigma_{bg}^2\,\mathrm{d}t^2}{2} I_3 & \sigma_{bg}^2\,\mathrm{d}t\, I_3 \end{bmatrix}$$

with σ_ARW = 4.89·10⁻⁵ rad/√s, σ_bg = 8.7·10⁻⁵ (rad/s)/√s 🔧, dt = 0.1 s.

**Observation 1 — accel (gravity direction), z = â = a/|a|:**

$$R_{acc}(\hat a) = \left(\frac{\sigma_a}{|a|}\right)^2 (I_3 - \hat a \hat a^{\mathsf T})$$

| condition | σ_a | σ per direction axis |
|---|---|---|
| bench (static) | 0.0044 m/s² | 4.5·10⁻⁴ rad = 0.026° |
| in flight (buffeting 🔧) | 0.05 m/s² | 5.1·10⁻³ rad = 0.29° |

Gate the update when $\big||a| - g\big| > 0.5\ \mathrm{m/s^2}$ (pendulum
swings, deployment jolts).

**Observation 2 — mag (tilt-compensated heading), z = ψ = atan2(−m_E, m_N):**

$$R_\psi(x, m) = \underbrace{\frac{\sigma_B^2}{m_H^2}}_{m_H = \sqrt{m_N^2 + m_E^2}} + \underbrace{\tan^2\theta_{dip}\;\big(P_{\theta\theta,\mathrm{roll}} + P_{\theta\theta,\mathrm{pitch}}\big)}_{\text{tilt uncertainty leaks in via dip}}, \qquad \theta_{dip} \approx 69^\circ \ (\mathrm{UK},\ |B| \approx 49\ \mathrm{\mu T})$$

| state | m_H | R_ψ | σ_ψ |
|---|---|---|---|
| level, converged (σ_tilt = 0.3°) | 17.6 µT | 0.029 rad² | 9.8° |
| converging (σ_tilt = 5°) | 17.6 µT | 0.081 rad² | 16.3° |

Use mag only when σ_tilt < 5° and \|ω\| small; hard/soft-iron calibrated.
(Full 3-axis vector update instead: R = σ_B²I₃ = 9 µT², constant.)

$$P_0 = \mathrm{diag}\big((5^\circ)^2 I_3,\ (1\,^\circ/\mathrm{s})^2 I_3\big) = \mathrm{diag}\big(7.6\cdot10^{-3} I_3,\ 3.05\cdot10^{-4} I_3\big)$$

## 3. ☆ Trajectory EKF + RTS — x = [p_NED, v_NED, δθ, δ_ba]

**Process noise** — accel noise is isotropic, but the **attitude error rotates
it** into the nav frame (genuine state-dependent Q):

$$\sigma_{a,\mathrm{eff}}(t) = \sqrt{\sigma_a^2 + g^2\,P_{\theta\theta}(t)}, \qquad Q_{pv}(t) = \sigma_{a,\mathrm{eff}}^2(t)\, GG^{\mathsf T}, \quad \sigma_{ba} = 20\ \mathrm{\mu g}/\sqrt{\mathrm{s}} \ 🔧$$

| t | P_θθ (per axis) | σ_a,eff | Q_vv per step (dt = 0.1) |
|---|---|---|---|
| t₀ (σ_θ = 5°) | 7.6·10⁻³ rad² | 0.86 m/s² | 7.3·10⁻² (m/s)² |
| converged (σ_θ = 0.3°) | 2.7·10⁻⁵ rad² | 0.052 m/s² | 2.7·10⁻⁴ (m/s)² |

**Observation noise** — baro @ 10 Hz, GPS @ 1 Hz (3D fix, sats ≥ 6):

$$R_{baro}(h) = \left[\frac{0.21\ \mathrm{Pa}}{\rho(h)\, g}\right]^2 = \begin{cases} 3.1\cdot10^{-4}\ \mathrm{m^2} & h = 0 \\ 3.7\cdot10^{-4}\ \mathrm{m^2} & h = 1\ \mathrm{km} \\ 4.5\cdot10^{-4}\ \mathrm{m^2} & h = 2\ \mathrm{km} \end{cases}$$

$$R_{gps}(\mathrm{DOP}) = \mathrm{diag}\big[(2.5\,\mathrm{HDOP})^2,\ (2.5\,\mathrm{HDOP})^2,\ (5\,\mathrm{VDOP})^2\big]\ \mathrm{m^2} \quad \xrightarrow{\mathrm{HDOP}\,1.2,\ \mathrm{VDOP}\,1.8} \quad \mathrm{diag}(9,\ 9,\ 81)$$

$$R_{vel} = (0.05)^2 I_3 = 2.5\cdot10^{-3}\ (\mathrm{m/s})^2$$

$$P_0 = \mathrm{diag}\big(6.25,\ 6.25,\ \mathbf{0.25}_{\text{baro-init }h},\ 0.04\,I_3,\ 7.6\cdot10^{-3} I_3,\ 0.038\,I_3\big)$$

## Sources

- [Bosch BMP581 datasheet (BST-BMP581-DS004)](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf) — RMS noise 0.21 Pa (×16), TC offset ±0.5 Pa/K
- [ST LSM6DSV datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv.pdf) — 90 µg/√Hz accel noise, 2.8 mdps/√Hz gyro noise, offsets
- [Melexis MLX90395 datasheet (rev 004)](https://media.melexis.com/-/media/files/documents/datasheets/mlx90395-datasheet-melexis.pdf) — noise vs RES/OSR table; high-res ≈ 3 µT class 🔧 verify vs our GAIN_SEL
- [u-blox SAM-M8Q datasheet (UBX-16012619)](https://content.u-blox.com/sites/default/files/documents/SAM-M8Q_DataSheet_UBX-16012619.pdf) — 2.5 m CEP, 0.05 m/s
- Magnetic field values: WMM (UK 2026: \|B\| ≈ 49 µT, inclination ≈ 69°); ISA troposphere for ρ(h)
