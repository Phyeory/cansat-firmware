# Report snippets — the maths-carrying code from `analysis/kalman.py`

Four snippets hold everything important; the rest of the module is logging
I/O and plumbing.  Each maps to a section of `docs/KALMAN-NOISE.md`:

| # | Snippet | Implements | Section |
|---|---|---|---|
| 1 | Altitude KF step | ★ linear KF x = [h, v, b], control input + Joseph-form update | §1 |
| 2 | RTS smoother | Rauch–Tung–Striebel backward pass (offline superpower, §6.2) | — |
| 3 | MEKF gravity update | multiplicative error-state update, rank-2 measurement noise | §2 |
| 4 | State-dependent Q | σ_a,eff = √(σ_a² + g²·P_θθ) — attitude error rotates accel | §3 |

If page space is tight, keep **1–3**; 4 is the one-line cherry.

---

## 1 — Altitude KF: one predict + update step

State x = [h, v, b] (altitude, vertical velocity, accel bias); the control
input u is the specific force rotated into the vertical by the *smoothed*
attitude, u = −((R(q)ᵀf)_z + g), which is a_true at any tilt.  F, Q are the
dt-coupled matrices from KALMAN-NOISE §1 (Q = σ_a² GGᵀ + diag(σ_tc² dt, 0,
σ_b² dt)); the baro measurement (σ_h = 0.21 Pa / ρg ≈ 1.75 cm) enters at
10 Hz.

```python
# predict: h' = h + v·dt − b·dt²/2 + u·dt²/2 ,  v' = v − b·dt + u·dt
F = np.array([[1, dt, -dt**2/2],
              [0, 1,     -dt ],
              [0, 0,       1]])
G = np.array([dt**2/2, dt, 0])
Q = sigma_a**2 * np.outer(G, G) + np.diag([sigma_tc**2*dt, 0, sigma_b**2*dt])
x = F @ x + G * u
P = F @ P @ F.T + Q

# update: baro altitude, Joseph form keeps P symmetric positive-definite
y = h_baro - H @ x                       # H = [1, 0, 0]
K = P @ H.T / (H @ P @ H.T + R)          # R = sigma_h**2
x = x + K * y
IKH = np.eye(3) - np.outer(K, H)
P = IKH @ P @ IKH.T + np.outer(K, K) * R
```

## 2 — RTS smoother: every estimate also uses future samples

Runs after the forward filter over the stored chain (x_f, P_f = filtered;
x_p, P_p = predicted; Phi = transition per step).  Six lines — this is why
the offline altitude/velocity beats anything onboard: h RMSE 0.29 m,
v RMSE 0.084 m/s on the synthetic flight.

```python
x_s, P_s = x_f.copy(), P_f.copy()
for k in range(n - 2, -1, -1):
    A = P_f[k] @ Phi[k+1].T @ np.linalg.inv(P_p[k+1])
    x_s[k] = x_f[k] + A @ (x_s[k+1] - x_p[k+1])
    P_s[k] = P_f[k] + A @ (P_s[k+1] - P_p[k+1]) @ A.T
```

## 3 — MEKF gravity-direction update (multiplicative, rank-2 R)

The accelerometer gives the gravity *direction* whenever ‖f‖ ≈ g (gated at
0.5 m/s² through chute snatch).  The measurement noise R_acc(â) =
(σ_a/‖f‖)²(I − ââᵀ) is rank 2 by design — no information along â — so the
gain needs pinv/regularisation or round-off corrupts the bias state.  The
solved 6-state error (δθ, δ_bg) is injected multiplicatively into the
quaternion, which keeps the attitude estimate exact to first order.

```python
s = -acc / np.linalg.norm(acc)           # measured down direction (body)
d = quat_rot(quat_conj(q), [0, 0, 1])    # predicted down direction (body)
Rm = (sigma_a/np.linalg.norm(acc))**2 * (np.eye(3) - np.outer(s, s)) \
     + 1e-6 * np.outer(s, s)             # +1e-6: regularise the null direction
H = np.zeros((3, 6))
H[:, :3] = skew(d)                       # δθ rotates the predicted direction
e, P = kf_update(np.zeros(6), P, s - d, H, Rm)
q = quat_mul(q, quat_exp(e[:3]))         # multiplicative injection: q ⊗ δq(δθ)
bg = bg + e[3:]                          # gyro bias corrected, error reset to 0
```

The mag update is the same shape with H[:, :3] = skew(m_pred),
R = σ_B²I₃ (full 3-axis field direction, σ_B = 3 µT).

## 4 — State-dependent process noise (trajectory EKF, §3)

Attitude error rotates the accel into the navigation frame, so the process
noise on (p, v) grows with the attitude covariance itself:

```python
sae = np.sqrt(sigma_a**2 + g**2 * np.clip(np.diag(P[6:9, 6:9]), 0, None))
for i in range(3):                       # per (p_i, v_i) pair, G = [dt²/2, dt]
    Q[i, i]   = sae[i]**2 * (dt**2/2)**2
    Q[i, 3+i] = sae[i]**2 * (dt**2/2) * dt
    Q[3+i, i] = Q[i, 3+i]
    Q[3+i, 3+i] = sae[i]**2 * dt**2
```

---

Source: `analysis/kalman.py` (validated vs a synthetic noisy flight with
known truth — `analysis/test_kalman.py`: h RMSE 0.29 m, v RMSE 0.084 m/s,
tilt < 2°, gyro bias recovered).  Accuracy numbers + grace-window rationale:
`analysis/ARCHITECTURE.md` §5.

---

## LaTeX versions (the two to include in the report)

Needs `\usepackage{listings}`. Suggested setup:

```latex
\lstset{basicstyle=\ttfamily\footnotesize, breaklines=true, frame=single,
        numbers=left, numberstyle=\tiny\color{gray},
        commentstyle=\itshape, language=Python}
```

```latex
\begin{lstlisting}[caption={Altitude Kalman filter: one predict/update step (state $x=[h,v,b]$, control input $u$ = attitude-rotated vertical accel)},label={lst:altkf}]
# predict: h' = h + v*dt - b*dt^2/2 + u*dt^2/2,  v' = v - b*dt + u*dt
F = np.array([[1, dt, -dt**2/2],
              [0, 1,     -dt ],
              [0, 0,       1]])
G = np.array([dt**2/2, dt, 0])
Q = sigma_a**2 * np.outer(G, G) + np.diag([sigma_tc**2*dt, 0, sigma_b**2*dt])
x = F @ x + G * u            # u = vertical accel, attitude-rotated IMU input
P = F @ P @ F.T + Q

# update with the barometric altitude (Joseph form keeps P symmetric)
y = h_baro - H @ x                        # H = [1, 0, 0]
K = P @ H.T / (H @ P @ H.T + R)           # R = sigma_h**2
x = x + K * y
IKH = np.eye(3) - np.outer(K, H)
P = IKH @ P @ IKH.T + np.outer(K, K) * R
\end{lstlisting}
```

```latex
\begin{lstlisting}[caption={Rauch--Tung--Striebel smoother: every estimate also uses future samples},label={lst:rts}]
x_s, P_s = x_f.copy(), P_f.copy()
for k in range(n - 2, -1, -1):
    A = P_f[k] @ Phi[k + 1].T @ np.linalg.inv(P_p[k + 1])
    x_s[k] = x_f[k] + A @ (x_s[k + 1] - x_p[k + 1])
    P_s[k] = P_f[k] + A @ (P_s[k + 1] - P_p[k + 1]) @ A.T
\end{lstlisting}
```

---

## LaTeX versions (part 2) — NN processing, plotting, transmission/collect/receive

### 1 — Data processing into the NN dataset (nn_dataset.py)

```latex
\begin{lstlisting}[caption={Neural-network dataset construction: uniform grid, z-score, sliding windows with a leakage-free chronological split},label={lst:nndata}]
# every channel resampled onto one uniform 10 Hz grid (gaps interpolated)
grid = np.arange(0.0, t[-1], 1.0 / fs)
for j, c in enumerate(cols):
    y = pd.Series(df[c]).interpolate(limit_direction="both").to_numpy()
    X[:, j] = np.interp(grid, t, y)

# per-channel z-score; the stats ship with the dataset so the network's
# outputs can be mapped back to physical units
mu, sd = X.mean(0), X.std(0)
sd[sd < 1e-9] = 1.0
Xn = ((X - mu) / sd).astype(np.float32)

# sliding windows, channels-first X_win[N, C, W]; the chronological
# train/val/test split happens BEFORE windowing, so no segment leaks
i1, i2 = int(T * 0.70), int(T * 0.85)
for a, b in ((0, i1), (i1, i2), (i2, T)):
    for s in range(a, b - W + 1, stride):
        wins.append(Xn[s:s + W].T)          # [C, W] window
        ywin.append(phase[s + W - 1])       # label at the window end
\end{lstlisting}
```

### 2 — Plotting system (plot_flight.py)

```latex
\begin{lstlisting}[caption={Plotting system: every channel against the Kalman-smoothed altitude, split into ascent and descent at the smoothed apogee},label={lst:plot}]
# altitude axis = linear KF + RTS smoothed barometric altitude
h = alt["h_smooth"].to_numpy(float)
i_ap = int(np.nanargmax(h))          # smoothed apogee index
asc = np.arange(len(h)) <= i_ap

ok = np.isfinite(h) & np.isfinite(y) # NaN-safe mask per channel
ax.plot(y[ok &  asc], h[ok &  asc], "-",  label="ascent")
ax.plot(y[ok & ~asc], h[ok & ~asc], "--", label="descent")
ax.set_xlabel(label)
ax.set_ylabel("Altitude (m)")
\end{lstlisting}
```

### 3 — Transmission, collection and receiving

**(a) Onboard: 10 Hz collection to SD + 1 Hz JSON transmission (src/main.c)**

```latex
\begin{lstlisting}[language=C,caption={Onboard loop: sensors sampled at 10 Hz and logged to the SD card as CSV; every 1 s the flight frame is serialised to JSON and transmitted over the E22 LoRa radio (868.125 MHz, 14 dBm)},label={lst:tx}]
while (1) {
    uint32_t now = HAL_GetTick();

    if (now - last_sample >= SAMPLE_PERIOD_MS) {         /* 100 ms */
        last_sample = now;
        sample_all();                        /* read every sensor into frame */
        char row[340];
        telemetry_csv_row(&frame, row, sizeof(row));
        sdlog_write(row);                    /* full-rate SD backup: 10 Hz */
    }

    if (now - last_telemetry >= TELEMETRY_PERIOD_MS) {   /* 1 s */
        last_telemetry = now;
        frame.seq++;                         /* packet number for loss count */
        frame.rad_cps = (uint16_t)rad_counter_latch();
        char pkt[256];
        int n = telemetry_json(&frame, pkt, sizeof(pkt));
        if (n > 0 && n < 256)
            e22_send((const uint8_t *)pkt, (uint16_t)n);
    }
}
\end{lstlisting}
```

**(b) Ground station: receiving, parsing and loss counting (ground-station/station.py)**

```latex
\begin{lstlisting}[caption={Ground station: reads the E22 USB radio, splits JSON lines, appends each packet to a CSV, and counts lost packets from the sequence number},label={lst:rx}]
buf = b""
while True:
    buf += ser.read(ser.in_waiting or 1)
    while b"\n" in buf:
        raw, buf = buf.split(b"\n", 1)
        try:
            pkt = json.loads(raw.decode(errors="replace").strip())
        except ValueError:
            continue
        writer.writerow([pkt.get(k, "") for k in FIELDS])  # CSV telemetry log

        if isinstance(pkt.get("n"), int):       # "n" = packet sequence number
            if last_n is not None and pkt["n"] > last_n + 1:
                print(f"[!] lost {pkt['n'] - last_n - 1} packet(s)")
            last_n = pkt["n"]
\end{lstlisting}
```

---

## Brief explanations (report text, one per listing)

**Listing 1 (altkf) — Altitude Kalman filter.**
The filter fuses two noisy sources into one clean altitude estimate: the
barometer gives absolute altitude but noisy from sample to sample, while the
accelerometer gives smooth short-term velocity but drifts when integrated
alone.  Each step, the filter predicts the next state from the current one
(predict) and then corrects that prediction against the barometer, weighted
by how much it trusts each source (update).  The third state b absorbs the
accelerometer's constant offset so the drift never accumulates.  Vertical
acceleration enters as the control input u, taken from the specific force
rotated into the vertical by the attitude filter, so it stays correct at any
tilt.  Accuracy against a simulated flight with known truth: 0.29 m in
altitude, 0.08 m/s in velocity.

**Listing 2 (rts) — RTS smoother.**
The forward filter above only knows the past.  Because this runs after
landing, the smoother re-walks the flight backwards and blends each estimate
with the corrected one that follows it — so every point also uses information
from the future.  This is what makes the offline result strictly better than
anything the onboard filter could achieve, at the cost of six lines of code.

**Listing 3 (nndata) — NN dataset construction.**
The sensors log at their own rates with occasional gaps, but a network needs
every channel on one clean, uniform time grid — so everything is resampled to
10 Hz and gaps are interpolated.  Channels are then z-scored (shifted and
scaled by their mean and standard deviation) so no channel dominates the
training just because its units are bigger, and the statistics are saved so
outputs can be converted back to physical units.  Sliding windows cut the
flight into training samples; the train/validation/test split is made in
time order before windowing, so the network is tested on data it has never
seen parts of — no leakage.

**Listing 4 (plot) — Plotting system.**
The altitude axis is the Kalman-smoothed altitude, not the raw barometer
(which is noisy at the centimetre scale and would make every curve fuzzy).
Each channel is split at the smoothed apogee into ascent (solid) and descent
(dashed): at the same altitude the two legs genuinely differ — temperature
and ozone hysteresis, pendulum swings on descent — and overlaying them makes
sensor faults obvious as gaps between the curves.

**Listing 5 (tx) — Onboard collection and transmission.**
Two independent data paths, on purpose.  Every 100 ms all sensors are read
into one flight frame and written to the SD card at full rate — the SD copy
is the complete record and survives any radio failure.  Every 1 s the same
frame is serialised to a compact JSON packet (the competition rule requires
telemetry once per second) and transmitted by the E22 LoRa radio on
868.125 MHz at 14 dBm, the UK-legal limit.  Each packet carries an
incrementing sequence number n and an 11-bit health mask h, one bit per
sensor, so the ground can count lost packets and spot a dead sensor live.

**Listing 6 (rx) — Ground station.**
The E22 USB radio delivers a serial byte stream; the station buffers it,
splits complete lines, and parses each as JSON.  Every packet is appended to
a timestamped CSV (the same column order as the onboard SD log, plus the
received signal-strength fields), and gaps in the sequence number n are
reported immediately as lost packets.  The two rule-required plots —
altitude vs time and temperature vs altitude — are drawn live from the same
stream.

---

## Expanded subsection text: NN dataset (LaTeX, paste-ready)

```latex
\subsection{Data processing into the NN dataset}

The sensors log at their own rates with occasional gaps, but a network needs
every channel on one clean uniform time grid, therefore everything is
resampled to 10\,Hz and gaps interpolated.

\textbf{Channel selection.}  The dataset is not just the raw sensor stream:
every quantity the pipeline computes is included as a channel, so the network
trains on physics that is already worked out rather than having to re-learn
sensor integration from raw counts.  The 63 channels fall into five groups:
(i) all 29 raw logged channels, including GNSS latitude/longitude, the raw
battery voltage and the 11-bit health mask expanded into one binary flag per
sensor; (ii) the filter outputs --- Kalman-smoothed altitude and vertical
velocity, roll/pitch/yaw and the estimated gyro biases; (iii) the
trajectory-filter position and velocity in north--east--down axes; (iv) the
per-sample derived quantities --- air density, dew point, absolute humidity,
the GNSS-minus-baro altitude error, and the dominant oscillation frequency
and amplitude from the gyro FFT; and (v) the flight-phase label itself.
Each channel's data coverage is recorded in the dataset metadata, and the
final matrix is asserted NaN-free.

\textbf{Labels.}  Supervised training needs a target, and labelling 958
samples by hand is neither practical nor reproducible.  Instead every sample
inherits a flight-phase label 0--4 (pad, boost, coast, descent, landed)
computed from the filter output: launch, apogee and landing come from the
event detection on the smoothed altitude and velocity, and the boost/coast
boundary from a thrust gate (specific force above $1g$ by more than
5\,m\,s$^{-2}$).  The labels are therefore exactly as trustworthy as the
filters validated in the previous section.

\textbf{Normalisation and windows.}  Each channel is z-scored with its own
mean and standard deviation, so channels with large numeric ranges (pressure
in hPa, counts in thousands) cannot dominate channels with small ones
(battery volts), and the statistics are stored with the dataset so any
network output can be mapped back to physical units.  Sliding windows of
$W=100$ samples (10\,s) with a stride of 5 turn the flight into overlapping
training examples, channels-first ($N$, $C$, $W$); the window label is the
phase at its final sample.

\textbf{Leakage-free evaluation.}  The train/validation/test split --- 70\%,
15\%, 15\% --- is made \emph{chronologically, before windowing}: the three
segments are disjoint parts of the flight and no window straddles a
boundary, so the network is validated and tested on flight time it has
never seen during training rather than on randomly shuffled neighbours of
its training data.

\textbf{Artefacts.}  One flight produces a single compressed
\texttt{.npz} file (raw and normalised sequences, window tensors, phase
labels, normalisation statistics), a flat CSV for inspection, a metadata
JSON (channel order, statistics, coverage, split boundaries, phase legend),
and six binned side tables (50\,m profiles, radiation count rate, GNSS--baro
$\Delta h$, lapse rate, oscillation, drift) --- one folder, everything a
training run needs.  On the simulated flight the dataset is 63 channels
$\times$ 958 samples with 115/9/9 windows.
```

---

## Compressed subsection (the one above, halved)

```latex
\subsection{Data processing into the NN dataset}

The sensors log at their own rates with occasional gaps, so every channel is
resampled onto one uniform 10\,Hz grid and gaps interpolated.  The dataset
holds 63 channels: all raw logged channels (including the health mask as
per-sensor flags) plus everything the pipeline computes --- Kalman-smoothed
altitude and velocity, attitude, trajectory, air density, GNSS--baro
$\Delta h$, oscillation --- so the network trains on physics already worked
out rather than raw counts.  Each sample carries a flight-phase label 0--4
(pad, boost, coast, descent, landed) taken from the filter-derived launch,
apogee and landing events, so no hand-labelling is needed.  Channels are
z-scored with the statistics stored in the dataset, and sliding windows of
10\,s turn the flight into training examples; the 70/15/15
train/validation/test split is made chronologically before windowing, so the
network is tested on flight time it has never seen (no leakage).  One flight
yields a \texttt{.npz} (raw + normalised sequences, windows, labels,
statistics) with a metadata JSON and six binned side tables.
```

---

## Compressed subsection, altitude-regression version (current)

```latex
\subsection{Data processing into the NN dataset}

The sensors log at their own rates with occasional gaps, so every channel is
resampled onto one uniform 10\,Hz grid and gaps interpolated.  The trained
model predicts \emph{altitude from environment data}: its inputs are the 14
environment-science channels (temperature, humidity, CO$_2$, ozone, UV, IR,
illuminance, radiation, and the derived density, dew point and absolute
humidity), and its regression label is the Kalman-filtered altitude in
metres.  Barometric pressure is deliberately excluded from the inputs ---
it is itself an altimeter ($P\!\to\!h$ is the barometric formula), so it
would trivially leak the answer; the same guard excludes every altitude,
velocity and attitude channel.  Flight-phase labels 0--4 (pad, boost, coast,
descent, landed) from the filter events ride along for optional
classification tasks.  Channels are z-scored with the statistics stored in
the dataset, and sliding windows of 10\,s turn the flight into training
examples; the 70/15/15 train/validation/test split is made chronologically
before windowing, so the network is tested on flight time it has never seen
(no leakage).  One flight yields a \texttt{.npz} (environment inputs,
altitude labels, windows, statistics) with a metadata JSON and six binned
side tables.
```

---

## Compressed subsection, FINAL (altitude + band, no phase)

```latex
\subsection{Data processing into the NN dataset}

The sensors log at their own rates with occasional gaps, so every channel is
resampled onto one uniform 10\,Hz grid and gaps interpolated.  The trained
model predicts \emph{altitude from atmospheric composition}: its inputs are
the 14 environment-science channels (temperature, humidity, CO$_2$, ozone,
UV, IR, illuminance, radiation, and the derived density, dew point and
absolute humidity), and its labels are the Kalman-filtered altitude, both
exact in metres (regression) and as 50\,m altitude-band classes
(classification).  Barometric pressure is deliberately excluded from the
inputs --- it is itself an altimeter ($P\!\to\!h$ is the barometric
formula), so it would trivially leak the answer; the same guard excludes
every altitude, velocity and attitude channel.  Channels are z-scored with
the statistics stored in the dataset, and sliding windows of 10\,s turn the
flight into training examples; the 70/15/15 train/validation/test split is
made chronologically before windowing, so the network is tested on flight
time it has never seen (no leakage).  One flight yields a \texttt{.npz}
(atmospheric inputs, exact and banded altitude labels, windows, statistics)
with a metadata JSON and six binned side tables.
```
