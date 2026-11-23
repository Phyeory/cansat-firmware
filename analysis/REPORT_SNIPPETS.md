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
