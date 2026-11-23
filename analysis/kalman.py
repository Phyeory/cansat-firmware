"""Offline Kalman filtering for CanSat logs — DATA-PIPELINE.md §6, now in Python.

The three filters designed in `docs/KALMAN-NOISE.md` (datasheet Q/R/P₀), all
offline on the 10 Hz SD log:

  ★ altitude   linear KF   x = [h, v, b]        baro h + accel-z, + RTS smoother
  ☆ attitude   MEKF        x = [δθ, δ_bg]       gyro + accel + mag (quaternions)
  ☆ trajectory EKF + RTS   x = [p, v, δθ, δ_ba] GNSS + baro + accel in NED

Terms: KF = Kalman filter (optimal recursive estimator); RTS = Rauch–Tung–Striebel
backward pass, the offline superpower (DATA-PIPELINE.md §6.2) — it re-reads the
flight backwards so every estimate also uses *future* samples; MEKF = multiplicative
error-state KF (the estimated state is a small attitude error, not the quaternion
itself); NED = north-east-down axes; Q = process-noise covariance, R =
measurement-noise covariance, P = state covariance.

numpy/pandas only, no scipy.  Importable functions AND one CLI:

    python3 kalman.py altitude   SD_LOG.csv --csv out/
    python3 kalman.py attitude   SD_LOG.csv --csv out/
    python3 kalman.py trajectory SD_LOG.csv --csv out/
    python3 kalman.py events     SD_LOG.csv
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from cansat_data import CanSatData

G0 = 9.80665                    # m/s², standard gravity (sim world.c constant)
D2R = np.pi / 180.0             # °/s → rad/s
R2D = 180.0 / np.pi


# --------------------------------------------------------------------- quaternions ---
# Convention: q = (w, x, y, z) maps BODY → NED, v_ned = R(q) v_body, Hamilton product.
# Body frame = sensor frame as logged: z reads +1 g at rest (z up), (x, y, z) right-handed.

def quat_mul(q: np.ndarray, p: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = np.moveaxis(q, -1, 0)
    w2, x2, y2, z2 = np.moveaxis(p, -1, 0)
    return np.stack([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2], axis=-1)


def quat_exp(rv: np.ndarray) -> np.ndarray:
    """Rotation vector (rad) → unit quaternion; exact, series near zero."""
    th = float(np.linalg.norm(rv))
    if th < 1e-10:
        return np.r_[1.0, 0.5 * rv]
    return np.r_[np.cos(0.5 * th), np.sin(0.5 * th) * rv / th]


def quat_rot(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vectors: R(q) @ v.  q (…,4), v (…,3)."""
    q = np.asarray(q, float)
    v = np.asarray(v, float)
    w = q[..., 0:1]
    u = q[..., 1:4]
    tvec = 2.0 * np.cross(u, v)
    return v + w * tvec + np.cross(u, tvec)


def quat_conj(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, float).copy()
    q[..., 1:] *= -1.0
    return q


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix → unit quaternion (Shepperd's branch method)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                      (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] >= R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                      0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                      (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    return q / np.linalg.norm(q)


def rpy_from_quat(q: np.ndarray) -> np.ndarray:
    """(roll, pitch, yaw) rad — z-y-x Euler of R = Rz(ψ)Ry(θ)Rx(φ), yaw CW+ from north."""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                     cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])


def align_quat(f_body: np.ndarray, m_body: np.ndarray) -> np.ndarray:
    """Initial attitude from one averaged accel + mag sample (pad, stationary).

    f_body: mean specific force (points UP in body coords at rest, |f| ≈ g);
    m_body: mean magnetic field µT (dips DOWN).  Builds the NED basis in body
    coordinates: up = f̂, magnetic north = horizontal part of m̂, east = D×N."""
    up = f_body / np.linalg.norm(f_body)              # body-frame up
    n = m_body - np.dot(m_body, up) * up              # horizontal (north) part
    n = n / np.linalg.norm(n)
    east = np.cross(-up, n)                           # D×N = E (NED right-handed)
    R = np.stack([n, east, -up])                      # rows = N, E, D in body coords
    return rot_to_quat(R)


# ---------------------------------------------------------------------- KF kernels ---

def expm(A: np.ndarray) -> np.ndarray:
    """Matrix exponential, scaling-and-squaring + Taylor (14 terms)."""
    nrm = np.linalg.norm(A, np.inf)
    s = max(0, int(np.ceil(np.log2(nrm))) + 1) if nrm > 0.5 else 0
    A = A / (2.0 ** s)
    E = np.eye(A.shape[0])
    T = np.eye(A.shape[0])
    for k in range(1, 15):
        T = T @ A / k
        E = E + T
    for _ in range(s):
        E = E @ E
    return E


def kf_update(x, P, z, H, R):
    """Kalman measurement update (Joseph form keeps P symmetric PSD).

    pinv, not inv: R may be singular by design — the gravity-direction update
    has R_acc = (σ_a/|a|)²(I − ââᵀ), rank 2 with no information along â (§2),
    and S = H P Hᵀ + R is then singular in that direction too.  pinv gives the
    correct rank-2 gain instead of amplifying round-off."""
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.pinv(S)
    x = x + K @ y
    IKH = np.eye(len(x)) - K @ H
    P = IKH @ P @ IKH.T + K @ R @ K.T
    return x, (P + P.T) / 2.0


def rts_smooth(x_f, P_f, x_p, P_p, Phi):
    """Rauch–Tung–Striebel backward pass for a (possibly time-varying) linear KF.

    x_f/P_f: filtered per step; x_p/P_p: predicted (step k+1 uses Phi[k+1]).
    Returns smoothed x/P.  Also serves the error-state EKFs whose mean error is
    injected to zero each step (MEKF / trajectory EKF) — pass zeros for x_f/x_p
    and apply the returned smoothed correction to the stored reference states."""
    n = len(x_f)
    x_s, P_s = x_f.copy(), P_f.copy()
    for k in range(n - 2, -1, -1):
        A = P_f[k] @ Phi[k + 1].T @ np.linalg.inv(P_p[k + 1])
        x_s[k] = x_f[k] + A @ (x_s[k + 1] - x_p[k + 1])
        P_s[k] = P_f[k] + A @ (P_s[k + 1] - P_p[k + 1]) @ A.T
        P_s[k] = (P_s[k] + P_s[k].T) / 2.0
    return x_s, P_s


def _dt_chain(t: np.ndarray, dt_max: float = 0.5) -> np.ndarray:
    """Per-step dt from the ms timestamps (first step = median), clamped."""
    dt = np.diff(t, prepend=t[0])
    dt[0] = np.median(dt[1:]) if len(dt) > 1 else 0.1
    return np.clip(dt, 1e-3, dt_max)


def _ffill(a: np.ndarray) -> np.ndarray:
    """Forward-fill NaN (brief sensor dropouts), leading NaN → first valid."""
    idx = np.where(np.isfinite(a), np.arange(len(a)), -1)
    idx = np.maximum.accumulate(idx)
    idx[idx < 0] = 0
    out = a[idx]
    out[~np.isfinite(out)] = 0.0
    return out


def _ffill_cols(A: np.ndarray) -> np.ndarray:
    return np.column_stack([_ffill(A[:, i]) for i in range(A.shape[1])])


def pad_zero(h: np.ndarray, t: np.ndarray, pad_s: float) -> float:
    """Barometric offset: mean raw altitude over the pre-launch pad window.

    The firmware's alt_m is referenced to sea level (bmp581.c); every offline
    estimate works AGL (above ground level) by subtracting the pad mean."""
    n = max(3, int(np.searchsorted(t, pad_s)))
    return float(np.nanmean(h[:min(n, max(3, len(h) // 10))]))


# =========================================================== ★ altitude: linear KF ==

def altitude_filter(d: CanSatData, *, att: pd.DataFrame | None = None,
                    pad_s: float = 5.0,
                    sigma_h: float = 0.0175,     # BMP581 ×16 → 0.21 Pa (KALMAN-NOISE §0)
                    sigma_a: float = 0.5,        # 🔧 offline: sharper phase transitions;
                                                 # the doc's 0.05 protects the ONLINE filter
                                                 # through chute snatch (0.084 m/s v RMSE here
                                                 # vs 0.99 at 0.05 — sweep in test_kalman)
                    sigma_tc: float = 0.042,     # baro TC offset walk, m/√s
                    sigma_b: float = 20e-6 * G0,  # accel bias walk, (m/s²)/√s
                    g: float = G0) -> pd.DataFrame:
    """★ 3-state linear KF  x = [h, v, b]  + RTS smoother (KALMAN-NOISE §1).

    Process: h' = v, v' = u − b with control u = vertical accel; b = accel
    bias.  Measurement: baro altitude @ 10 Hz.

    u comes from the §1 online recipe u = acc_z − g, valid only while the z
    axis is near-vertical (pad, boost, chute pendulum — the online STM32 has
    no attitude).  Offline we can do better: pass `att` (attitude_filter
    output) and u = −((R(q)ᵀ f)_z + g) — the specific force rotated into NED
    with the *smoothed* attitude, correct at any tilt.

    Returns: t, h_baro (pad-zeroed raw), acc_z, h_kf, v_kf, b_kf (forward),
    h_smooth, v_smooth, b_smooth (RTS), sigma_h, sigma_v."""
    t = d.t
    z = d.df["alt_m"].to_numpy(float) - pad_zero(d.df["alt_m"].to_numpy(float), t, pad_s)
    f = _ffill(d.df["acc_z"].to_numpy(float))
    n = len(t)
    dt = _dt_chain(t)

    if att is not None:
        qs = att[["qw", "qx", "qy", "qz"]].to_numpy()
        RbN = np.array([_rot_mat(qq) for qq in qs])
        f3 = _ffill_cols(d.df[["acc_x", "acc_y", "acc_z"]].to_numpy(float))
        f_ned = np.einsum("kij,kj->ki", RbN, f3)     # R(q) f_body → NED
        u = -(f_ned[:, 2] + g)                       # a_up = −(f_ned_z + g)
    else:
        u = f - g
    u = np.r_[u[0], u[:-1]]          # ZOH: interval ending at k uses sample k−1

    x = np.array([z[0], 0.0, 0.0])
    P = np.diag([0.25, 0.01, 0.038])          # (0.5 m)², (0.1 m/s)², (20 mg)² — §1
    H = np.array([1.0, 0.0, 0.0]).reshape(1, 3)
    R = np.array([[sigma_h ** 2]])

    x_f, P_f, x_p, P_p, Phis = [], [], [], [], []
    for k in range(n):
        dtk = dt[k]
        # F = [[1,dt,−dt²/2],[0,1,−dt],[0,0,1]], control B = [dt²/2, dt, 0]
        F = np.array([[1.0, dtk, -0.5 * dtk ** 2],
                      [0.0, 1.0, -dtk],
                      [0.0, 0.0, 1.0]])
        Gcol = np.array([0.5 * dtk ** 2, dtk, 0.0])
        Q = sigma_a ** 2 * np.outer(Gcol, Gcol) \
            + np.diag([sigma_tc ** 2 * dtk, 0.0, sigma_b ** 2 * dtk])
        x = F @ x + Gcol * u[k]
        P = F @ P @ F.T + Q
        P = (P + P.T) / 2.0
        x_p.append(x.copy()); P_p.append(P.copy()); Phis.append(F)
        if np.isfinite(z[k]):
            x, P = kf_update(x, P, np.array([z[k]]), H, R)
        x_f.append(x.copy()); P_f.append(P.copy())

    x_s, P_s = rts_smooth(np.array(x_f), np.array(P_f),
                          np.array(x_p), np.array(P_p), Phis)
    return pd.DataFrame({
        "t": t, "h_baro": z, "acc_z": f,
        "h_kf": [xx[0] for xx in x_f], "v_kf": [xx[1] for xx in x_f],
        "b_kf": [xx[2] for xx in x_f],
        "h_smooth": x_s[:, 0], "v_smooth": x_s[:, 1], "b_smooth": x_s[:, 2],
        "sigma_h": np.sqrt(P_s[:, 0, 0]), "sigma_v": np.sqrt(P_s[:, 1, 1]),
    })


# =========================================================== ☆ attitude: MEKF =======

ATT_DEFAULTS = dict(
    sigma_arw=4.89e-5,       # rad/√s, LSM6DSV 2.8 mdps/√Hz  (§0)
    sigma_bg=8.7e-5,         # (rad/s)/√s 🔧                  (§2)
    sigma_a_static=0.0044,   # m/s², 90 µg/√Hz bench floor    (§0)
    sigma_a_flight=0.05,     # m/s², buffeting 🔧             (§2)
    sigma_mag=3.0,           # µT/axis, MLX90395 RES-16       (§0)
    g=G0,
    mag_ned=(17.6, 0.0, 45.7),   # µT, UK WMM: |B| 49 µT, dip 69°, decl 0 (§2)
    gate_ms2=0.5,            # accel update gate, |‖f‖−g| (§2)
    mag_tilt_deg=5.0,        # use mag only when σ_tilt < 5°  (§2)
    mag_omega_dps=20.0,      # … and ‖ω‖ < 20 °/s             (§2)
    mag_norm_gate=10.0,      # µT, skip mag when |‖m‖−|B‖| past this (uncalibrated iron)
    pad_s=5.0,
)


def attitude_filter(d: CanSatData, **cfg) -> pd.DataFrame:
    """☆ MEKF  error state x = [δθ(3), δ_bg(3)]  (KALMAN-NOISE §2).

    Gyro propagates the quaternion (body→NED); accel gives the gravity
    direction (gated when |‖f‖−g| > 0.5 m/s² — pendulum swings, deployment
    jolts); mag gives the full 3-axis field direction with R = σ_B²I₃ (the §2
    "full 3-axis vector update" option — needs the site field `mag_ned` and
    hard/soft-iron calibrated data).  Forward MEKF injects the error each step;
    the RTS backward pass then smooths the whole flight via the stored
    covariance chain.  All rates in rad/s internally; the log's °/s converted.

    Returns: t, qw..qz, roll_deg, pitch_deg, yaw_deg (smoothed), bgx..bgz (°/s),
    sigma_tilt_deg, and the raw gyro columns for the oscillation FFT."""
    c = {**ATT_DEFAULTS, **cfg}
    t = d.t
    n = len(t)
    dt = _dt_chain(t)
    gyr = d.df[["gyr_x", "gyr_y", "gyr_z"]].to_numpy(float) * D2R   # rad/s
    gyr = np.vstack([gyr[:1], gyr[:-1]])   # ZOH: interval [k−1,k] uses sample k−1
    acc = _ffill_cols(d.df[["acc_x", "acc_y", "acc_z"]].to_numpy(float))
    mag = _ffill_cols(d.df[["mag_x", "mag_y", "mag_z"]].to_numpy(float))  # µT
    B_ned = np.asarray(c["mag_ned"], float)
    I3 = np.eye(3)
    gN = np.array([0.0, 0.0, 1.0])                   # NED down unit vector

    # --- initial attitude + gyro bias from the stationary pad window ---
    n0 = max(3, min(int(np.searchsorted(t, c["pad_s"])), n // 10))
    f0, m0, w0 = acc[:n0].mean(0), mag[:n0].mean(0), gyr[:n0].mean(0)
    still = abs(np.linalg.norm(f0) - c["g"]) < c["gate_ms2"] \
        and np.linalg.norm(w0) < 5 * D2R
    q = align_quat(f0 if still else np.array([0.0, 0.0, c["g"]]),
                   m0 if still else np.r_[B_ned[:2], -B_ned[2]])
    bg = w0.copy() if still else np.zeros(3)         # pad gyro mean = bias

    P = np.diag(np.r_[np.full(3, (5 * D2R) ** 2),     # (5°)²
                      np.full(3, (1.0 * D2R) ** 2)])  # (1 °/s)²      — §2 P₀

    ref = np.zeros((n, 7))                            # qw..qz, bgx..bgz (post-inject)
    P_f, P_p, Phis = [], [], []

    for k in range(n):
        dtk = dt[k]
        w = gyr[k] - bg                               # bias-corrected body rate
        q = quat_mul(q, quat_exp(w * dtk))            # body-frame rotation → right-mult
        q /= np.linalg.norm(q)

        F = np.zeros((6, 6))
        F[:3, :3] = -_skew(w)                         # δθ' = −[ω×]δθ − δbg
        F[:3, 3:] = -I3
        Phi = expm(F * dtk)
        Qd = np.zeros((6, 6))
        Qd[:3, :3] = (c["sigma_arw"] ** 2 * dtk
                      + c["sigma_bg"] ** 2 * dtk ** 3 / 3) * I3
        Qd[:3, 3:] = -c["sigma_bg"] ** 2 * dtk ** 2 / 2 * I3
        Qd[3:, :3] = Qd[:3, 3:]
        Qd[3:, 3:] = c["sigma_bg"] ** 2 * dtk * I3
        P = Phi @ P @ Phi.T + Qd
        P = (P + P.T) / 2.0
        P_p.append(P.copy()); Phis.append(Phi)

        # accel update — gravity direction, R(â) per §2, gated through jolts
        fa = np.linalg.norm(acc[k])
        if abs(fa - c["g"]) <= c["gate_ms2"]:
            s_hat = -acc[k] / fa                      # measured down direction
            s_cap = s_hat / np.linalg.norm(s_hat)
            d_pred = quat_rot(quat_conj(q), gN)       # predicted down in body
            sa = c["sigma_a_static"] if abs(fa - c["g"]) < 0.2 else c["sigma_a_flight"]
            # +1e-6 along ŝ: the innovation has no component along ŝ (both
            # vectors are unit length), so this only regularises the rank-2
            # null direction of R_acc — without it S is singular to round-off
            # and the gain blows up in that direction.
            Rm = (sa / fa) ** 2 * (I3 - np.outer(s_cap, s_cap)) \
                + 1e-6 * np.outer(s_cap, s_cap)
            H = np.zeros((3, 6)); H[:, :3] = _skew(d_pred)
            q, bg, P = _mekf_update(q, bg, P, s_cap - d_pred, H, Rm)

        # mag update — full 3-axis vector, only when tilt converged & quiet
        sig_tilt = np.sqrt(max(np.trace(P[:3, :3]), 1e-12))
        if sig_tilt < c["mag_tilt_deg"] * D2R \
                and np.linalg.norm(w) < c["mag_omega_dps"] * D2R \
                and abs(np.linalg.norm(mag[k]) - np.linalg.norm(B_ned)) < c["mag_norm_gate"]:
            m_pred = quat_rot(quat_conj(q), B_ned)
            H = np.zeros((3, 6)); H[:, :3] = _skew(m_pred)
            q, bg, P = _mekf_update(q, bg, P, mag[k] - m_pred,
                                    H, c["sigma_mag"] ** 2 * I3)

        ref[k] = np.r_[q, bg]
        P_f.append(P.copy())

    P_f = np.array(P_f)
    dth_s, P_s = rts_smooth(np.zeros((n, 6)), P_f, np.zeros((n, 6)),
                            np.array(P_p), Phis)
    out = ref.copy()
    for k in range(n):                                # apply smoothed corrections
        qs = quat_mul(out[k, :4], quat_exp(dth_s[k, :3]))
        out[k, :4] = qs / np.linalg.norm(qs)
        out[k, 4:] += dth_s[k, 3:]

    rpy = np.array([rpy_from_quat(o[:4]) for o in out])
    sig = np.sqrt(np.clip(np.trace(P_s[:, :3, :3], axis1=1, axis2=2), 0, None))
    return pd.DataFrame({
        "t": t, "qw": out[:, 0], "qx": out[:, 1], "qy": out[:, 2], "qz": out[:, 3],
        "roll_deg": rpy[:, 0] * R2D, "pitch_deg": rpy[:, 1] * R2D,
        "yaw_deg": rpy[:, 2] * R2D,
        "bgx_dps": out[:, 4] / D2R, "bgy_dps": out[:, 5] / D2R,
        "bgz_dps": out[:, 6] / D2R,
        "sigma_tilt_deg": sig * R2D,
        "gyr_x_dps": d.df["gyr_x"].to_numpy(float), "gyr_y_dps": d.df["gyr_y"].to_numpy(float),
        "gyr_z_dps": d.df["gyr_z"].to_numpy(float),
    })


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def _mekf_update(q, bg, P, innov, H, R):
    """One multiplicative error-state update; injects the mean error immediately
    (δθ → quaternion, δbg → bias) and resets the error mean to zero."""
    x, P = kf_update(np.zeros(6), P, innov, H, R)
    q = quat_mul(q, quat_exp(x[:3]))
    q /= np.linalg.norm(q)
    return q, bg + x[3:], P


# ===================================================== ☆ trajectory: EKF + RTS ======

TRAJ_DEFAULTS = dict(
    sigma_a=0.05,            # m/s² sensor noise under buffeting 🔧 (§3)
    sigma_ba=20e-6 * G0,     # accel bias walk (m/s²)/√s            (§3)
    sigma_arw=4.89e-5,       # rad/√s                               (§0)
    sigma_P=0.21,            # Pa BMP581 RMS                        (§0)
    g=G0,
    hdop=1.2, vdop=1.8,      # → R_gps = diag(9, 9, 81) m²          (§3)
    gps_gate_sats=4,
    pad_s=5.0,
)


def _geo_ned(lat, lon, alt, lat0, lon0, alt0):
    """lat/lon/alt (deg, deg, m) → local tangent-plane NED m (WGS84 local radii)."""
    a, e2 = 6378137.0, 6.69437999014e-3
    s = np.sin(np.deg2rad(lat0))
    Rm = a * (1 - e2) / (1 - e2 * s * s) ** 1.5      # meridional radius
    Rn = a / np.sqrt(1 - e2 * s * s)                 # transverse radius
    n = np.deg2rad(lat - lat0) * Rm
    e = np.deg2rad(lon - lon0) * Rn * np.cos(np.deg2rad(lat0))
    return np.array([n, e, -(alt - alt0)])


def trajectory_filter(d: CanSatData, att: pd.DataFrame | None = None,
                      **cfg) -> pd.DataFrame:
    """☆ 3D-trajectory error-state EKF + RTS smoother (KALMAN-NOISE §3).

    State x = [p_NED(3), v_NED(3), δθ(3), δ_ba(3)] around the reference
    (p, v, quaternion, accel bias).  Accel propagates p/v through the
    attitude; baro h @ 10 Hz and GNSS position @ 1 Hz (sats ≥ gate) correct
    it.  Q carries the §3 state-dependent term σ_a,eff = √(σ_a² + g²·P_θθ)
    per axis — attitude error rotates the accel into the nav frame.  RTS
    smoother on the stored covariance chain.

    att: attitude_filter() output — supplies the initial quaternion and the
    per-step gyro bias (removed from ω before propagation).  GNSS altitude is
    aligned to the baro pad-zero frame by the median GNSS−baro Δh.

    Returns: t, p_N/p_E/p_D, v_N/v_E/v_D (all smoothed), h_smooth = −p_D,
    roll/pitch/yaw of the smoothed attitude, sigma_p, sigma_v, sigma_aeff."""
    c = {**TRAJ_DEFAULTS, **cfg}
    t = d.t
    n = len(t)
    dt = _dt_chain(t)
    acc = _ffill_cols(d.df[["acc_x", "acc_y", "acc_z"]].to_numpy(float))
    acc = np.vstack([acc[:1], acc[:-1]])   # ZOH: interval [k−1,k] uses sample k−1
    if att is None:
        att = attitude_filter(d)
    q = att[["qw", "qx", "qy", "qz"]].iloc[0].to_numpy(float)
    bg = att[["bgx_dps", "bgy_dps", "bgz_dps"]].to_numpy(float) * D2R  # per step
    gyr = d.df[["gyr_x", "gyr_y", "gyr_z"]].to_numpy(float) * D2R
    gyr = np.vstack([gyr[:1], gyr[:-1]])

    h_baro = d.df["alt_m"].to_numpy(float)
    z0 = pad_zero(h_baro, t, c["pad_s"])
    h_baro = h_baro - z0

    fix = ((d.df["sats"].fillna(0) > c["gps_gate_sats"])
           & d.df["lat"].notna()).to_numpy()
    lat = d.df["lat"].to_numpy(float); lon = d.df["lon"].to_numpy(float)
    galt = d.df["gps_alt_m"].to_numpy(float)
    has_gps = bool(fix.any())
    if has_gps:
        i0 = int(np.argmax(fix))
        lat0, lon0 = lat[i0], lon[i0]
        galt = galt - np.nanmedian(galt[fix] - h_baro[fix])  # align GNSS→baro frame
        alt0 = galt[i0]
        p = np.array([0.0, 0.0, -(galt[i0] - z0)])
    else:
        lat0 = lon0 = alt0 = 0.0
        p = np.array([0.0, 0.0, -h_baro[0]])

    v = np.zeros(3)
    ba = np.zeros(3)
    P = np.diag(np.r_[[6.25, 6.25, 0.25],            # p: 2.5 m GNSS, 0.5 m baro-init
                      np.full(3, 0.04),              # v: (0.2 m/s)²
                      np.full(3, (5 * D2R) ** 2),    # δθ (5°)²
                      np.full(3, 0.038)])            # δba (20 mg)² — §3 P₀
    gN = np.array([0.0, 0.0, c["g"]])
    I3 = np.eye(3)

    ref_p = np.zeros((n, 3)); ref_v = np.zeros((n, 3)); ref_q = np.zeros((n, 4))
    P_f, P_p, Phis, saeff_out = [], [], [], []

    for k in range(n):
        dtk = dt[k]
        RbN = _rot_mat(q)
        w = gyr[k] - bg[k]
        a_nav = RbN @ (acc[k] - ba) + gN              # NED accel from specific force

        p = p + v * dtk + 0.5 * a_nav * dtk ** 2
        v = v + a_nav * dtk
        q = quat_mul(q, quat_exp(w * dtk)); q /= np.linalg.norm(q)

        # 12-state error Jacobian: δṗ=δv; δv̇=−R[f×]δθ−Rδba; δθ̇=−[ω×]δθ; δba'=0
        F = np.zeros((12, 12))
        F[0:3, 3:6] = I3
        F[3:6, 6:9] = -RbN @ _skew(acc[k] - ba)
        F[3:6, 9:12] = -RbN
        F[6:9, 6:9] = -_skew(w)
        Phi = expm(F * dtk)

        # §3: σ_a,eff(t) = √(σ_a² + g²·P_θθ) per axis — attitude error rotates accel
        Pth = np.clip(np.diag(P[6:9, 6:9]), 0, None)
        sae = np.sqrt(c["sigma_a"] ** 2 + c["g"] ** 2 * Pth)
        Q = np.zeros((12, 12))
        G2 = np.array([0.5 * dtk ** 2, dtk])
        for i in range(3):
            Q[i, i] = sae[i] ** 2 * G2[0] ** 2
            Q[i, 3 + i] = sae[i] ** 2 * G2[0] * G2[1]
            Q[3 + i, i] = Q[i, 3 + i]
            Q[3 + i, 3 + i] = sae[i] ** 2 * G2[1] ** 2
        Q[6:9, 6:9] = c["sigma_arw"] ** 2 * dtk * I3
        Q[9:12, 9:12] = c["sigma_ba"] ** 2 * dtk * I3

        P = Phi @ P @ Phi.T + Q
        P = (P + P.T) / 2.0
        P_p.append(P.copy()); Phis.append(Phi)
        saeff_out.append(float(sae.max()))

        # baro update @ 10 Hz — R_baro(h) = (σ_P/(ρ(h)·g))²  (§3)
        if np.isfinite(h_baro[k]):
            h_pred = -p[2]
            rho = 1.225 * (1 - 2.2558e-5 * max(h_pred, 0.0)) ** 4.2559
            R_baro = (c["sigma_P"] / max(rho * c["g"], 1e-6)) ** 2
            H = np.zeros((1, 12)); H[0, 2] = -1.0     # h = −p_D → δh = −δp_D
            dx, Pn = kf_update(np.zeros(12), P,
                               np.array([h_baro[k] - h_pred]), H,
                               np.array([[R_baro]]))
            p, v, q, ba = _traj_inject(p, v, q, ba, dx)
            P = Pn

        # GNSS update @ 1 Hz — R_gps(DOP) = diag((2.5·HDOP)², (2.5·HDOP)², (5·VDOP)²)
        if has_gps and fix[k]:
            z = _geo_ned(lat[k], lon[k], galt[k], lat0, lon0, alt0)
            H = np.zeros((3, 12)); H[0:3, 0:3] = I3
            Rg = np.diag([(2.5 * c["hdop"]) ** 2, (2.5 * c["hdop"]) ** 2,
                          (5.0 * c["vdop"]) ** 2])
            dx, Pn = kf_update(np.zeros(12), P, z - p, H, Rg)
            p, v, q, ba = _traj_inject(p, v, q, ba, dx)
            P = Pn

        ref_p[k] = p; ref_v[k] = v; ref_q[k] = q
        P_f.append(P.copy())

    P_f = np.array(P_f)
    dx_s, P_s = rts_smooth(np.zeros((n, 12)), P_f, np.zeros((n, 12)),
                           np.array(P_p), Phis)
    ps, vs, qs = ref_p.copy(), ref_v.copy(), ref_q.copy()
    for k in range(n):                                # apply smoothed corrections
        ps[k] += dx_s[k, 0:3]; vs[k] += dx_s[k, 3:6]
        qs[k] = quat_mul(qs[k], quat_exp(dx_s[k, 6:9]))
        qs[k] /= np.linalg.norm(qs[k])
    rpy = np.array([rpy_from_quat(qq) for qq in qs])
    return pd.DataFrame({
        "t": t,
        "p_N": ps[:, 0], "p_E": ps[:, 1], "p_D": ps[:, 2],
        "v_N": vs[:, 0], "v_E": vs[:, 1], "v_D": vs[:, 2],
        "h_smooth": -ps[:, 2],
        "roll_deg": rpy[:, 0] * R2D, "pitch_deg": rpy[:, 1] * R2D,
        "yaw_deg": rpy[:, 2] * R2D,
        "sigma_p": np.sqrt(P_s[:, 0, 0] + P_s[:, 1, 1] + P_s[:, 2, 2]),
        "sigma_v": np.sqrt(P_s[:, 3, 3] + P_s[:, 4, 4] + P_s[:, 5, 5]),
        "sigma_aeff": saeff_out,
    })


def _rot_mat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]])


def _traj_inject(p, v, q, ba, dx):
    p = p + dx[0:3]; v = v + dx[3:6]
    q = quat_mul(q, quat_exp(dx[6:9])); q /= np.linalg.norm(q)
    return p, v, q, ba + dx[9:12]


# =========================================================== flight events ==========

def flight_events(d: CanSatData, alt: pd.DataFrame) -> pd.DataFrame:
    """Phase timeline from the *smoothed* KF output — the §6.2 consumer of h/v
    (apogee/landing detection, and chute deployment as the peak deceleration
    jerk after apogee).

    launch: v first exceeds +2 m/s; apogee: max h_smooth; deploy: largest
    positive dv/dt (fall slowing) within 10 s of apogee; landing: first t after
    apogee where |v| < 0.5 m/s and h < 5 m AGL for ≥ 1 s."""
    t = alt["t"].to_numpy(float)
    h = alt["h_smooth"].to_numpy(float)
    v = alt["v_smooth"].to_numpy(float)

    i_launch = int(np.argmax(v > 2.0)) if (v > 2.0).any() else -1
    if i_launch < 0:
        return pd.DataFrame(columns=["event", "t_s", "detail"])
    i_ap = i_launch + int(np.argmax(h[i_launch:]))

    dvdt = np.gradient(v, t)
    win = (t >= t[i_ap]) & (t <= t[i_ap] + 10.0)
    # |dv/dt| peak catches both snatch styles: sim deploys AT apogee (fall
    # starts, dv/dt < 0); real freefall deploy decelerates the fall (dv/dt > 0)
    i_dep = int(np.argmax(np.where(win, np.abs(dvdt), -np.inf)))

    landed = (np.abs(v) < 0.5) & (h < 5.0)
    need = max(3, int(round(1.0 / max(np.median(np.diff(t)), 1e-3))))
    i_land = -1
    run = 0
    for k in range(i_ap, len(t)):
        run = run + 1 if landed[k] else 0
        if run >= need:
            i_land = k - run + 1
            break

    rows = [("launch", t[i_launch], f"v ≥ 2 m/s, h = {h[i_launch]:.1f} m"),
            ("apogee", t[i_ap], f"h = {h[i_ap]:.1f} m"),
            ("chute_deploy", t[i_dep], f"jerk peak dv/dt = {dvdt[i_dep]:+.1f} m/s²"),
            ("landing", t[i_land] if i_land >= 0 else np.nan,
             f"h = {h[i_land]:.1f} m" if i_land >= 0 else "not detected")]
    return pd.DataFrame(rows, columns=["event", "t_s", "detail"])


# ------------------------------------------------------------------------- CLI -------

def _write(df: pd.DataFrame, path: Path, tag: str, stem: str) -> None:
    path.mkdir(parents=True, exist_ok=True)
    f = path / f"{stem}_{tag}.csv"
    df.to_csv(f, index=False, float_format="%.4f")
    print(f"  wrote {f}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cmd", choices=("altitude", "attitude", "trajectory", "events"))
    ap.add_argument("log")
    ap.add_argument("--csv", metavar="DIR")
    ap.add_argument("--pad-s", type=float, default=5.0, help="pre-launch pad window, s")
    a = ap.parse_args(argv)

    d = CanSatData(a.log)
    stem = Path(a.log).stem
    show = lambda df, cols: print(df[cols].iloc[::max(1, len(df) // 10)]
                                  .round(2).to_string(index=False))
    if a.cmd == "altitude":
        df = altitude_filter(d, att=attitude_filter(d, pad_s=a.pad_s),
                             pad_s=a.pad_s)
        show(df, ["t", "h_smooth", "v_smooth", "sigma_h"])
        if a.csv:
            _write(df, Path(a.csv), "altkf", stem)
    elif a.cmd == "attitude":
        df = attitude_filter(d, pad_s=a.pad_s)
        show(df, ["t", "roll_deg", "pitch_deg", "yaw_deg", "bgz_dps"])
        if a.csv:
            _write(df, Path(a.csv), "attitude", stem)
    elif a.cmd == "trajectory":
        df = trajectory_filter(d, pad_s=a.pad_s)
        show(df, ["t", "p_N", "p_E", "h_smooth", "v_D"])
        if a.csv:
            _write(df, Path(a.csv), "traj", stem)
    else:
        ev = flight_events(d, altitude_filter(d, pad_s=a.pad_s))
        print(ev.to_string(index=False))
        if a.csv:
            _write(ev, Path(a.csv), "events", stem)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
