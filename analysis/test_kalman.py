"""Tests for analysis/kalman.py — the §6 offline filters, validated against truth.

Runs each filter on a synthetic flight with known ground truth: the same
profile as sim/world.c (pad 5 s → 3 s boost → coast → −10 m/s chute descent),
realistic sensor noise from docs/KALMAN-NOISE.md §0 (BMP581 0.21 Pa, LSM6DSV
90 µg/√Hz + 2.8 mdps/√Hz + offsets, MLX90395 3 µT, SAM-M8Q 2.5 m CEP), plus
the ugly real-world bits: sensor biases, a baro/GNSS altitude offset, and a
GNSS MSL−baro offset.  Accuracy = RMSE of the smoothed estimate vs truth.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).parent))

import kalman as kf
from cansat_data import CanSatData

G0 = 9.80665
SD_HEADER = ("t_ms,temp_c,pressure_hpa,alt_m,uv_raw,uv_index,ir_counts,"
             "full_counts,lux,co2_ppm,o3_ppb,lat,lon,gps_alt_m,sats,vbat_v,"
             "acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,mag_x,mag_y,mag_z,rh_pct,"
             "sht_temp_c,rad_cps,health")


# ----------------------------------------------------------- synthetic flight -------

def _profile(apogee=750.0, burn_s=3.0, pad_s=5.0, descent=10.0, t_end=None):
    """Kinematic truth: same profile family as sim/world.c."""
    g = G0
    if apogee <= 0 or burn_s <= 0:               # static/pad-only profile
        a_boost = 0.0
    else:
        ac = burn_s ** 2 / (2 * g)
        bc = burn_s ** 2 / 2
        a_boost = (-bc + np.sqrt(bc ** 2 + 4 * ac * apogee)) / (2 * ac)
    burn_v = a_boost * burn_s
    burn_h = 0.5 * a_boost * burn_s ** 2
    t_ap = pad_s + burn_s + burn_v / g
    t_land = t_ap + apogee / descent
    t_end = (t_land + 5.0) if t_end is None else t_end

    def state(t):
        if t < pad_s:
            return 0.0, 0.0, 0.0, "PAD"
        if t < pad_s + burn_s:
            tau = t - pad_s
            return 0.5 * a_boost * tau ** 2, a_boost * tau, a_boost, "BOOST"
        if t < t_ap:
            tau = t - (pad_s + burn_s)
            return (burn_h + burn_v * tau - 0.5 * g * tau ** 2,
                    burn_v - g * tau, -g, "COAST")
        if t < t_land:
            return apogee - descent * (t - t_ap), -descent, 0.0, "DESCENT"
        return 0.0, 0.0, 0.0, "LANDED"

    return dict(a_boost=a_boost, t_ap=t_ap, t_land=t_land, t_end=t_end,
                pad_s=pad_s, burn_s=burn_s, state=state)


def make_flight(seed=7, noise=True, fs=10.0, static_s=None, tmpdir=None):
    """SD-format CSV with datasheet noise + biases; returns (path, truth dict).

    static_s: pad-only log (attitude static test).  Truth carries h, v, the
    per-sample quaternion, the sensor biases and the event times."""
    rng = np.random.default_rng(seed)
    prof = _profile() if static_s is None else _profile(apogee=0.0, burn_s=0.0,
                                                        pad_s=0.0, t_end=static_s)
    t = np.arange(0.0, prof["t_end"], 1.0 / fs)
    n = len(t)
    h = np.empty(n); v = np.empty(n); a = np.empty(n)
    for k, tk in enumerate(t):
        h[k], v[k], a[k], _ = prof["state"](tk)

    # truth attitude: pad tilt (2°, −3°, 40°) + sim-like rates (12,−4,3)°/s in boost.
    # Sensor frame is z-UP (accel reads +1 g at rest): R_sensor = R_F·T with
    # T = diag(−1,1,−1) flipping FRD→z-up (proper rotation, det +1).
    q = np.zeros((n, 4))
    R_F = kf._rot_mat(kf.quat_from_rpy(2 * kf.D2R, -3 * kf.D2R, 40 * kf.D2R))
    qi = kf.rot_to_quat(R_F @ np.diag([-1.0, 1.0, -1.0]))
    w_true = np.zeros((n, 3))
    boost = (t >= prof["pad_s"]) & (t < prof["pad_s"] + prof["burn_s"])
    w_true[boost] = np.array([12.0, -4.0, 3.0]) * kf.D2R
    for k in range(n):
        if k:
            qi = kf.quat_mul(qi, kf.quat_exp(w_true[k - 1] / fs))
            qi /= np.linalg.norm(qi)
        q[k] = qi
    RbN = np.array([kf._rot_mat(qq) for qq in q])

    B_ned = np.array(kf.ATT_DEFAULTS["mag_ned"])
    bias_a = np.array([0.02, -0.01, 0.03])          # m/s², accel biases
    bias_g = np.array([0.2, -0.1, 0.3]) * kf.D2R    # rad/s,  gyro biases

    # specific force: f_ned = a_ned − g_ned, a_ned = (0,0,−a_kin)
    f_ned = np.tile(np.array([0.0, 0.0, -G0]), (n, 1))
    f_ned[:, 2] -= a
    f_body = np.einsum("kji,kj->ki", RbN, f_ned)    # R(q)ᵀ f_ned
    buffeting = np.where(boost | (h > 0), 0.05, 0.0044) if noise else 0.0
    acc = f_body + bias_a
    if noise:
        acc += rng.normal(0, 0.0044, (n, 3)) \
            + rng.normal(0, 1, (n, 3)) * buffeting[:, None]
    gyr = w_true + bias_g
    if noise:
        gyr += rng.normal(0, 0.014 * kf.D2R, (n, 3))
    mag = np.einsum("kji,j->ki", RbN, B_ned)    # R(q)ᵀ B_ned
    if noise:
        mag += rng.normal(0, 3.0, (n, 3))

    # baro: sensor noise + TC walk + a pad-zero-able mount offset
    walk = np.cumsum(rng.normal(0, 0.042 * np.sqrt(1 / fs), n)) if noise \
        else np.zeros(n)
    alt_m = h + walk + (1.5 if noise else 0.0)
    if noise:
        alt_m += rng.normal(0, 0.0175, n)
    temp_c = 15.0 - 0.0065 * h + (rng.normal(0, 0.15, n) if noise else 0)
    pressure = 1013.25 * (1 - 2.25577e-5 * h) ** 5.25588
    sht_temp = temp_c + 0.8
    rh = np.clip(45.0 + 0.02 * h + (rng.normal(0, 2, n) if noise else 0), 1, 100)

    # GNSS @ 1 Hz: 2.5 m CEP horizontal, 4.5 m vertical (VDOP 1.8), +12 m MSL offset
    gps_fix = t >= 1.0
    lat0, lon0 = 51.5, -1.3
    dN = rng.normal(0, 2.5, n) if noise else np.zeros(n)
    dE = rng.normal(0, 2.5, n) if noise else np.zeros(n)
    dU = rng.normal(0, 4.5, n) if noise else np.zeros(n)
    lat = lat0 + np.where(gps_fix, dN / 6.371e6 * 180 / np.pi, np.nan)
    lon = lon0 + np.where(gps_fix, dE / (6.371e6 * np.cos(np.deg2rad(lat0)))
                          * 180 / np.pi, np.nan)
    gps_alt = h + 12.0 + np.where(gps_fix, dU, np.nan)
    sats = np.where(gps_fix, 9, 0)

    rad = np.clip(np.round(4 + (rng.normal(0, 2, n) if noise else 0)), 0, None)
    df = pd.DataFrame({
        "t_ms": (t * 1000).astype(int) + 10,
        "temp_c": temp_c, "pressure_hpa": pressure, "alt_m": alt_m,
        "uv_raw": 2000 + 40 * h, "uv_index": (2000 + 40 * h) / 96.0,
        "ir_counts": 5000 + 2 * h, "full_counts": 20000 + 8 * h,
        "lux": 612 + 0.05 * h, "co2_ppm": 420.0, "o3_ppb": 12 + 0.01 * h,
        "lat": lat, "lon": lon, "gps_alt_m": gps_alt, "sats": sats,
        "vbat_v": 4.02 - 0.01 * t / 60, "acc_x": acc[:, 0], "acc_y": acc[:, 1],
        "acc_z": acc[:, 2], "gyr_x": gyr[:, 0] / kf.D2R,
        "gyr_y": gyr[:, 1] / kf.D2R, "gyr_z": gyr[:, 2] / kf.D2R,
        "mag_x": mag[:, 0], "mag_y": mag[:, 1], "mag_z": mag[:, 2],
        "rh_pct": rh, "sht_temp_c": sht_temp, "rad_cps": rad,
        "health": np.where(gps_fix, 1023, 975),
    })
    path = Path(tmpdir or "data") / f"synth_flight_{seed}.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(path, index=False, float_format="%.5f", columns=SD_HEADER.split(","))

    truth = dict(t=t, h=h, v=v, a=a, q=q, w_true=w_true, bias_g=bias_g,
                 bias_a=bias_a, B_ned=B_ned,
                 t_launch=prof["pad_s"], t_ap=prof["t_ap"], t_land=prof["t_land"],
                 lat0=lat0, lon0=lon0)
    return path, truth


def _rmse(est, true):
    return float(np.sqrt(np.mean((np.asarray(est) - np.asarray(true)) ** 2)))


def _tilt_err_deg(q_est, q_true):
    """Angle between the predicted and true down directions, deg, per sample."""
    down_e = np.array([kf.quat_rot(kf.quat_conj(qq), [0, 0, 1]) for qq in q_est])
    down_t = np.array([kf.quat_rot(kf.quat_conj(qq), [0, 0, 1]) for qq in q_true])
    d = np.clip(np.sum(down_e * down_t, axis=1), -1, 1)
    return np.degrees(np.arccos(d))


def _yaw_err_deg(att, truth):
    yaw_e = att["yaw_deg"].to_numpy()
    yaw_t = np.unwrap(np.array([kf.rpy_from_quat(qq)[2] for qq in truth["q"]])) * kf.R2D
    return (yaw_e - yaw_t + 180) % 360 - 180


# ------------------------------------------------------------- quaternion algebra ---

def test_quaternion_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.normal(size=4); q /= np.linalg.norm(q)
        R = kf._rot_mat(q)
        assert np.allclose(kf.rot_to_quat(R), q, atol=1e-9) or \
            np.allclose(kf.rot_to_quat(R), -q, atol=1e-9)
        assert np.allclose(kf._rot_mat(kf.quat_conj(q)), R.T, atol=1e-9)
        v = rng.normal(size=3)
        assert np.allclose(kf.quat_rot(q, v), R @ v, atol=1e-9)
    rpy = np.array([0.3, -0.2, 1.1])
    assert np.allclose(kf.rpy_from_quat(kf.quat_from_rpy(*rpy)), rpy, atol=1e-9)


def test_meas_jacobians():
    """MEKF accel/mag Jacobians H = [ŝ×, 0] vs finite differences."""
    rng = np.random.default_rng(3)
    q = rng.normal(size=4); q /= np.linalg.norm(q)
    eps = 1e-6
    for vec in (np.array([0.0, 0.0, 1.0]), np.array(kf.ATT_DEFAULTS["mag_ned"])):
        pred = kf.quat_rot(kf.quat_conj(q), vec)
        H = kf._skew(pred)
        for i in range(3):
            dq = np.zeros(3); dq[i] = eps
            q2 = kf.quat_mul(q, kf.quat_exp(dq))
            pert = kf.quat_rot(kf.quat_conj(q2), vec)
            assert np.allclose((pert - pred) / eps, H[:, i], atol=1e-5)


def test_expm():
    A = np.diag([1.0, -2.0, 0.5])
    assert np.allclose(kf.expm(A), np.diag(np.exp(np.diag(A))), atol=1e-12)
    B = np.array([[0.0, 1.0], [-1.0, 0.0]])          # generator of rotation
    E = kf.expm(B * 0.7)
    assert np.allclose(E, np.array([[np.cos(.7), np.sin(.7)],
                                    [-np.sin(.7), np.cos(.7)]]), atol=1e-12)


# ---------------------------------------------------------------- ★ altitude -------

def test_velocity_rts(tmp_path):
    """Smoothed h/v beat raw baro and the Δh/Δt recipe against truth.  The
    synthetic can tilts ~39° in boost (sim rates) and stays tilted, so the
    attitude-rotated control is what makes v accurate — the z-only §1 online
    recipe degrades exactly as it should offline.

    The truth model (sim/world.c) switches chute and touchdown with an
    *instant* 10 m/s Δv — no physical deceleration.  Any smooth process model
    smears such a step over ±1–2 s (correct behaviour), so the accuracy metric
    excludes those two grace windows."""
    path, tr = make_flight(tmpdir=tmp_path)
    d = CanSatData(path)
    att = kf.attitude_filter(d)
    alt = kf.altitude_filter(d, att=att)
    alt_z = kf.altitude_filter(d)               # z-only fallback (online recipe)
    _, v_raw = d.get_vel()

    t = alt["t"].to_numpy()
    ok = ~(((t > tr["t_ap"] - 1) & (t < tr["t_ap"] + 1))
           | ((t > tr["t_land"] - 1) & (t < tr["t_land"] + 2)))

    e_s = _rmse(alt["v_smooth"][ok], tr["v"][ok])
    assert e_s < 0.5                                        # m/s
    assert _rmse(alt["h_smooth"][ok], tr["h"][ok]) < 0.5    # m
    assert _rmse(alt["h_kf"][ok], tr["h"][ok]) < 1.0        # forward still decent
    assert _rmse(alt["h_smooth"][ok], tr["h"][ok]) < _rmse(alt["h_baro"][ok], tr["h"][ok])
    # attitude-rotated control beats the z-only recipe on tilted data
    assert e_s < _rmse(alt_z["v_smooth"][ok], tr["v"][ok])
    # and better than the raw central-difference velocity of cansat_data.get_vel
    assert e_s < _rmse(v_raw[ok], tr["v"][ok])
    # accel bias state is a nuisance state, weakly observable at σ_a = 0.5
    # (the smoother puts the freedom in v, which is what actually matters)
    assert abs(alt["b_smooth"].iloc[-20:].mean() - 0.03) < 0.15
    # touchdown recovery: |v| < 3 m/s within 2 s of the instant stop
    k = np.searchsorted(t, tr["t_land"] + 2.0)
    assert abs(alt["v_smooth"].iloc[k]) < 3.0


# ------------------------------------------------------------- ☆ attitude ----------

def test_attitude_static(tmp_path):
    path, tr = make_flight(static_s=30.0, tmpdir=tmp_path)
    d = CanSatData(path)
    att = kf.attitude_filter(d)
    sel = att["t"].to_numpy() > 5.0
    qe = att[["qw", "qx", "qy", "qz"]].to_numpy()[sel]
    assert _tilt_err_deg(qe, tr["q"][sel]).mean() < 1.0     # deg
    assert np.abs(_yaw_err_deg(att, tr)[sel]).mean() < 8.0  # mag-noise limited
    bg = att[["bgx_dps", "bgy_dps", "bgz_dps"]].to_numpy()[-10:] * kf.D2R
    assert np.abs(bg - tr["bias_g"]).max() < 0.1 * kf.D2R  # mag-noise-limited


def test_attitude_synthetic(tmp_path):
    path, tr = make_flight(tmpdir=tmp_path)
    d = CanSatData(path)
    att = kf.attitude_filter(d)
    sel = att["t"].to_numpy() > 1.0
    qe = att[["qw", "qx", "qy", "qz"]].to_numpy()[sel]
    assert _tilt_err_deg(qe, tr["q"][sel]).mean() < 2.0
    assert np.abs(_yaw_err_deg(att, tr)[sel]).mean() < 8.0  # mag-noise limited
    bg = att[["bgx_dps", "bgy_dps", "bgz_dps"]].to_numpy()[-10:] * kf.D2R
    assert np.abs(bg.mean(0) - tr["bias_g"]).max() < 0.05 * kf.D2R


# ------------------------------------------------------------- ☆ trajectory --------

def test_trajectory(tmp_path):
    path, tr = make_flight(tmpdir=tmp_path)
    d = CanSatData(path)
    att = kf.attitude_filter(d)
    traj = kf.trajectory_filter(d, att)
    t = traj["t"].to_numpy()

    # NED position from the truth lat/lon-free vertical flight + tilt offsets:
    # horizontal truth = 0 by construction (baro/GNSS frame aligned)
    assert _rmse(traj["p_D"] * -1, tr["h"]) < 3.0
    assert _rmse(traj["p_N"], 0.0) < 5.0
    assert _rmse(traj["p_E"], 0.0) < 5.0
    assert _rmse(traj["v_D"] * -1, tr["v"]) < 1.0
    assert traj["sigma_p"].iloc[-1] > 0                     # covariance sane
    assert t[-1] > tr["t_land"]                             # full flight processed


# ---------------------------------------------------------------- events / deploy --

def test_events(tmp_path):
    path, tr = make_flight(tmpdir=tmp_path)
    d = CanSatData(path)
    ev = kf.flight_events(d, kf.altitude_filter(d))
    get = dict(zip(ev["event"], ev["t_s"]))
    assert abs(get["launch"] - tr["t_launch"]) < 0.3
    assert abs(get["apogee"] - tr["t_ap"]) < 0.3
    assert abs(get["landing"] - tr["t_land"]) < 1.0


def test_deploy_detection(tmp_path):
    """Chute-out = the |dv/dt| spike within 10 s of apogee (sim deploys at
    apogee: v jumps 0 → −10 m/s in one 0.1 s step)."""
    path, tr = make_flight(tmpdir=tmp_path)
    d = CanSatData(path)
    alt = kf.altitude_filter(d)
    ev = kf.flight_events(d, alt)
    t_dep = float(dict(zip(ev["event"], ev["t_s"]))["chute_deploy"])
    assert abs(t_dep - tr["t_ap"]) < 1.0
    # v_smooth right after the spike already sits at chute terminal velocity
    v = alt["v_smooth"].to_numpy()
    k = np.searchsorted(alt["t"].to_numpy(), t_dep + 0.5)
    assert -12.0 < v[k] < -8.0


# ---------------------------------------------------------------- plots / NN -------

def test_plot_flight(tmp_path):
    path, tr = make_flight(tmpdir=tmp_path)
    import plot_flight
    out = tmp_path / "plots"
    written = plot_flight.make_plots(CanSatData(path), out, stem=path.stem)
    names = {f.name for f in written}
    assert f"{path.stem}_altitude_time.png" in names          # P1 compulsory
    assert f"{path.stem}_temperature_altitude.png" in names   # P2 compulsory
    assert f"{path.stem}_descent_rate_time.png" in names      # P3
    assert f"{path.stem}_science_heatmap.png" in names        # P4
    assert f"{path.stem}_battery_health_time.png" in names    # P5
    assert f"{path.stem}_oscillation_time.png" in names       # P6
    assert f"{path.stem}_dh_time.png" in names                # GNSS−baro
    assert f"{path.stem}_drift_map.png" in names              # trajectory map
    assert f"{path.stem}_alt_rho_kgm3.png" in names           # derived channels
    assert f"{path.stem}_alt_dh_gps_baro.png" in names
    assert f"{path.stem}_overview.png" in names
    assert len(written) >= 45                                 # every channel


def test_derived_estimators(tmp_path):
    """§3 last two rows: oscillation FFT + wind proxy."""
    import derived_estimator as de
    # 1 Hz single-axis wobble → freq 1 Hz, amp ≈ 5 °/s (Hann-corrected)
    t = np.arange(0, 20, 0.1)
    df = pd.DataFrame({"t_ms": (t * 1000).astype(int) + 10,
                       "gyr_x": 5 * np.sin(2 * np.pi * 1.0 * t),
                       "gyr_y": 0.0, "gyr_z": 0.0, "acc_x": 0.0,
                       "acc_y": 0.0, "acc_z": kf.G0, "alt_m": 0.0,
                       "temp_c": 15.0, "pressure_hpa": 1013.25,
                       "rh_pct": 45.0, "sht_temp_c": 15.0})
    f = tmp_path / "osc.csv"
    df.to_csv(f, index=False)
    osc = de.oscillation(CanSatData(f))
    fin = osc.dropna()
    assert abs(fin["osc_freq_hz"].median() - 1.0) < 0.2
    assert 3.5 < fin["osc_amp_dps"].max() < 6.5
    # wind proxy on the synthetic flight: truth horizontal position = 0
    path, tr = make_flight(tmpdir=tmp_path)
    d = CanSatData(path)
    wind = de.wind_proxy(d)
    assert len(wind) > 3
    assert wind["drift_from_pad_m"].max() < 15.0              # inertial only


def test_nn_dataset(tmp_path):
    path, tr = make_flight(tmpdir=tmp_path)
    import nn_dataset as nn
    ds = nn.build_dataset(CanSatData(path), fs=10.0, window=50, stride=5)
    assert np.isfinite(ds["X_seq"]).all() and np.isfinite(ds["X_raw"]).all()
    # every §2 raw channel is in (lat/lon/health included) + every §3 per-sample
    for c in ("lat", "lon", "health", "dh_gps_baro", "osc_freq_hz",
              "rho_kgm3", "t_dew_c", "abs_hum_gm3", "p_N", "p_E"):
        assert c in ds["channels"]
    # normalisation is invertible from the saved stats (float32 storage)
    assert np.allclose(ds["X_seq"] * ds["std"] + ds["mu"], ds["X_raw"],
                       rtol=1e-5, atol=1e-3)
    y = ds["y_seq"]
    assert set(np.unique(y)) <= {0, 1, 2, 3, 4}
    firsts = [int(np.argmax(y == p)) for p in range(5) if (y == p).any()]
    assert firsts == sorted(firsts)                # pad→boost→coast→descent→landed
    assert len(ds["X_win"]["train"]) > 0 and len(ds["X_win"]["val"]) > 0
    # windows are channels-first [N, C, W] with the chronological split
    assert ds["X_win"]["train"].shape[1] == len(ds["channels"])
    assert ds["X_win"]["train"].shape[2] == 50
    # CLI smoke + artifacts
    assert nn.main([str(path), "--out", str(tmp_path / "nn"), "--window", "50"]) == 0
    z = np.load(tmp_path / "nn" / f"{path.stem}_dataset.npz")
    assert "X_win_train" in z.files and "mu" in z.files
    meta = json.load(open(tmp_path / "nn" / f"{path.stem}_metadata.json"))
    assert meta["channels"] == ds["channels"]
    for k in ("profile", "radiation", "dh", "lapse", "oscillation", "wind"):
        assert (tmp_path / "nn" / f"{path.stem}_{k}.csv").exists()


# ---------------------------------------------------------------------- CLI --------

def test_cli_smoke(tmp_path, capsys):
    path, _ = make_flight(tmpdir=tmp_path)
    for cmd in ("altitude", "attitude", "trajectory", "events"):
        assert kf.main([cmd, str(path), "--csv", str(tmp_path / "out")]) == 0
    import derived_estimator as de
    assert de.main([str(path), "--csv", str(tmp_path / "out")]) == 0
    out = tmp_path / "out"
    assert (out / f"{path.stem}_altkf.csv").exists()
    assert (out / f"{path.stem}_attitude.csv").exists()
    assert (out / f"{path.stem}_traj.csv").exists()
    assert (out / f"{path.stem}_events.csv").exists()
    capsys.readouterr()
