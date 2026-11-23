"""Turn a CanSat flight log into a neural-network-ready dataset.

Pipeline: SD/ground log → KF stack (altitude, attitude, trajectory) → every
channel + derived quantities resampled onto a uniform time grid → z-score
normalisation (stats saved) → sliding windows → chronological train/val/test
split → one .npz + one flat CSV + one metadata JSON per flight.

Outputs (per log, in --out):
    <stem>_dataset.npz    X_seq [T,C] normalised · X_seq_raw [T,C] ·
                          y_seq [T] phase label 0-4 · X_win [N,C,W] normalised
                          (channels-first: transpose to [N,W,C] for TF/Keras) ·
                          y_win [N] · mu, std [C] · win_train/win_val/win_test
    <stem>_flat.csv       t + every raw (interpolated) channel + phase — for
                          eyeballing in pandas
    <stem>_metadata.json  channel order, normalisation stats, coverage,
                          window/stride/split, phase legend

Phase labels (estimated from the KF output, see flight_events + boost gate):
    0 pad · 1 boost (powered ascent) · 2 coast (unpowered ascent) ·
    3 descent · 4 landed

    python3 nn_dataset.py SD_LOG.csv --out nn/ [--fs 10] [--window 100]
                              [--stride 5] [--pad-s 5]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from cansat_data import CanSatData, HEALTH_BITS
from atmosphere_estimator import estimate as atmosphere_estimate, lapse_rate
from profile_estimator import (binned_profile, radiation_profile,
                               gnss_baro_dh)
from derived_estimator import oscillation, wind_proxy
import kalman as kf

# EVERY channel the SD log (or ground CSV) can carry — §2 of DATA-PIPELINE.md
MEASURED = ["seq", "temp_c", "pressure_hpa", "alt_m", "uv_index", "uv_raw",
            "ir_counts", "full_counts", "lux", "co2_ppm", "o3", "rh_pct",
            "sht_temp_c", "rad_cps", "vbat_v", "acc_x", "acc_y", "acc_z",
            "gyr_x", "gyr_y", "gyr_z", "mag_x", "mag_y", "mag_z",
            "lat", "lon", "gps_alt_m", "sats", "health"]
KF_COLS = ["h_smooth", "v_smooth", "sigma_h", "sigma_v"]
ATT_COLS = ["roll_deg", "pitch_deg", "yaw_deg", "sigma_tilt_deg",
            "bgx_dps", "bgy_dps", "bgz_dps"]
TRAJ_COLS = ["p_N", "p_E", "p_D", "v_N", "v_E", "v_D", "sigma_p", "sigma_v"]
EST_COLS = ["rho_kgm3", "t_dew_c", "abs_hum_gm3"]
DER_COLS = ["dh_gps_baro", "osc_freq_hz", "osc_amp_dps"]
# model INPUTS: environment-science channels only.  Pressure is excluded on
# purpose (it is an altimeter: P -> h is the barometric formula, the task
# would be trivial); altitude/velocity/attitude/trajectory channels are
# labels or auxiliary, never environment inputs.
ENV_INPUTS = ["temp_c", "rh_pct", "sht_temp_c", "co2_ppm", "o3", "uv_index",
              "uv_raw", "ir_counts", "full_counts", "lux", "rad_cps",
              "rho_kgm3", "t_dew_c", "abs_hum_gm3"]
BAND_M = 50.0                 # altitude-band width for the classification label


def build_features(d: CanSatData, pad_s: float = 5.0):
    """EVERY column of DATA-PIPELINE.md §2 (raw) + §3 (derived, per-sample):
    measured channels + health bits + the three KF outputs + atmosphere
    formulas + GNSS−baro Δh + oscillation FFT.  Returns (df, aux) with the
    filter outputs so callers don't re-run them."""
    att = kf.attitude_filter(d, pad_s=pad_s)
    alt = kf.altitude_filter(d, att=att, pad_s=pad_s)
    traj = kf.trajectory_filter(d, att=att, pad_s=pad_s)

    df = pd.DataFrame({"t": d.t})
    for c in MEASURED:
        if c in d.df.columns:
            df[c] = d.df[c].to_numpy(float)
    for j, bit in enumerate(HEALTH_BITS):
        df[f"h_{bit}"] = (d.df["health"].fillna(0).astype(int)
                          & (1 << j)).astype(float)
    for c in KF_COLS:
        df[c] = alt[c].to_numpy(float)
    for c in ATT_COLS:
        df[c] = att[c].to_numpy(float)
    for c in TRAJ_COLS:
        df[c] = traj[c].to_numpy(float)
    est = atmosphere_estimate(d)                       # §3: ρ, T_d, AH
    for c in EST_COLS:
        df[c] = est[c].to_numpy(float)
    # §3 ERR per sample: Δh = h_GNSS − h_baro (raw, same as the ERR plot),
    # 0 without a fix (sats carries the fix flag)
    fix = (d.df["sats"].fillna(0) > 0).to_numpy()
    dh = d.df["gps_alt_m"].to_numpy(float) - d.df["alt_m"].to_numpy(float)
    df["dh_gps_baro"] = np.where(fix, np.nan_to_num(dh, nan=0.0), 0.0)
    der = oscillation(d)                               # §3: oscillation FFT
    der = der.set_index("t").reindex(df["t"]).interpolate(
        limit_direction="both").reset_index(drop=True)
    for c in ["osc_freq_hz", "osc_amp_dps"]:
        df[c] = der[c].to_numpy(float)
    return df, dict(att=att, alt=alt, traj=traj)


def phase_labels(t: np.ndarray, acc_z: np.ndarray, ev: pd.DataFrame) -> np.ndarray:
    """0 pad · 1 boost · 2 coast · 3 descent · 4 landed — KF events + the
    powered-ascent gate |acc_z − g| > 5 m/s² for the boost/coast split."""
    g = kf.G0
    get = dict(zip(ev["event"], ev["t_s"]))
    t_launch = get.get("launch", np.nan)
    t_ap = get.get("apogee", np.nan)
    t_land = get.get("landing", np.inf)
    ph = np.full(len(t), 2, dtype=np.int8)                 # coast default
    ph[t < t_launch] = 0
    boosted = (t >= t_launch) & (t < t_ap) & (acc_z > g + 5.0)  # thrust > 1 g

    ph[boosted] = 1
    ph[(t >= t_ap) & (t < t_land)] = 3
    ph[t >= t_land] = 4
    return ph


def resample_grid(df: pd.DataFrame, fs: float) -> tuple[np.ndarray, np.ndarray, dict]:
    """Uniform fs grid → (grid_t, X [T,C], per-channel coverage)."""
    t = df["t"].to_numpy(float)
    grid = np.arange(0.0, t[-1], 1.0 / fs)
    cols = [c for c in df.columns if c != "t"]
    X = np.empty((len(grid), len(cols)))
    cov = {}
    for j, c in enumerate(cols):
        y = df[c].to_numpy(float)
        cov[c] = float(np.isfinite(y).mean())
        y = pd.Series(y).interpolate(limit_direction="both").to_numpy()
        X[:, j] = np.interp(grid, t, y)
    return grid, X, cov


def build_dataset(d: CanSatData, fs: float = 10.0, window: int = 100,
                  stride: int = 5, pad_s: float = 5.0, band_m: float = BAND_M,
                  split=(0.7, 0.15, 0.15)) -> dict:
    """Full pipeline → dict of arrays + metadata (saved by main)."""
    feats, aux = build_features(d, pad_s=pad_s)
    grid, X, cov = resample_grid(feats, fs)
    channels = [c for c in feats.columns if c != "t"]

    # normalisation (std-guarded; stats are part of the deliverable)
    mu = X.mean(0)
    sd = X.std(0)
    sd[sd < 1e-9] = 1.0
    Xn = ((X - mu) / sd).astype(np.float32)

    # --- the task: atmospheric composition -> altitude -------------------
    env_names = [c for c in ENV_INPUTS if c in channels]
    env_idx = [channels.index(c) for c in env_names]
    X_env = Xn[:, env_idx]                        # normalised env inputs
    y_alt = X[:, channels.index("h_smooth")].astype(np.float32)  # metres
    alt_mu, alt_sd = float(y_alt.mean()), float(y_alt.std() or 1.0)
    y_alt_n = ((y_alt - alt_mu) / alt_sd).astype(np.float32)
    n_bands = max(1, int(np.ceil(max(float(y_alt.max()), 1.0) / band_m)))
    y_band = np.clip((y_alt // band_m).astype(np.int16), 0, n_bands - 1)

    # chronological split BEFORE windowing — no leakage across segments
    T = len(grid)
    i1, i2 = int(T * split[0]), int(T * (split[0] + split[1]))
    segs = {"train": (0, i1), "val": (i1, i2), "test": (i2, T)}

    W = min(window, T)
    wins = {k: [] for k in segs}
    wenv = {k: [] for k in segs}
    yalt = {k: [] for k in segs}
    ybnd = {k: [] for k in segs}
    for k, (a, b) in segs.items():
        for s in range(a, max(a, b - W + 1), stride):
            if s + W <= b:
                wins[k].append(Xn[s:s + W].T)      # [C, W] channels-first
                wenv[k].append(X_env[s:s + W].T)   # env-only model input
                yalt[k].append(y_alt[s + W - 1])   # exact altitude, metres
                ybnd[k].append(y_band[s + W - 1])  # altitude band class

    X_win = {k: np.stack(v) if v else np.zeros((0, len(channels), W), np.float32)
             for k, v in wins.items()}
    X_win_env = {k: np.stack(v) if v else np.zeros((0, len(env_names), W),
                                                   np.float32)
                 for k, v in wenv.items()}
    y_win_alt = {k: np.array(v, dtype=np.float32) for k, v in yalt.items()}
    y_win_band = {k: np.array(v, dtype=np.int16) for k, v in ybnd.items()}

    return dict(
        grid=grid, X_raw=X, X_seq=Xn,
        channels=channels, mu=mu, std=sd, coverage=cov,
        window=W, stride=stride, fs=fs, split=segs,
        X_win=X_win, _d=d, _aux=aux,
        env_names=env_names, X_env=X_env, y_alt=y_alt, y_alt_n=y_alt_n,
        alt_mu=alt_mu, alt_sd=alt_sd, X_win_env=X_win_env, y_win_alt=y_win_alt,
        band_m=band_m, n_bands=n_bands, y_band=y_band, y_win_band=y_win_band,
    )


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log")
    ap.add_argument("--out", default="nn")
    ap.add_argument("--fs", type=float, default=10.0, help="uniform grid, Hz")
    ap.add_argument("--window", type=int, default=100, help="window samples")
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--pad-s", type=float, default=5.0)
    ap.add_argument("--band-m", type=float, default=BAND_M,
                    help="altitude band width for the classification label, m")
    a = ap.parse_args(argv)

    d = CanSatData(a.log)
    stem = Path(a.log).stem
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    ds = build_dataset(d, fs=a.fs, window=a.window, stride=a.stride,
                       pad_s=a.pad_s, band_m=a.band_m)

    np.savez_compressed(
        out / f"{stem}_dataset.npz",
        X_seq=ds["X_seq"], X_seq_raw=ds["X_raw"].astype(np.float32),
        t_grid=ds["grid"].astype(np.float32),
        mu=ds["mu"].astype(np.float32), std=ds["std"].astype(np.float32),
        # the task: atmospheric composition -> exact altitude / altitude band
        X_env_seq=ds["X_env"], y_alt=ds["y_alt"], y_alt_n=ds["y_alt_n"],
        alt_mu=np.float32(ds["alt_mu"]), alt_sd=np.float32(ds["alt_sd"]),
        y_band=ds["y_band"], band_m=np.float32(ds["band_m"]),
        **{f"X_win_{k}": v for k, v in ds["X_win"].items()},
        **{f"X_win_env_{k}": v for k, v in ds["X_win_env"].items()},
        **{f"y_win_alt_{k}": v for k, v in ds["y_win_alt"].items()},
        **{f"y_win_band_{k}": v for k, v in ds["y_win_band"].items()},
    )
    flat = pd.DataFrame(ds["X_raw"], columns=ds["channels"])
    flat.insert(0, "t", ds["grid"].round(2))
    flat["alt_m"] = ds["y_alt"]
    flat["alt_band"] = ds["y_band"]
    flat.to_csv(out / f"{stem}_flat.csv", index=False, float_format="%.4f")

    # the binned/aggregated §3 tables ride along, so one folder holds
    # everything a training run or judge pack needs
    d, aux = ds["_d"], ds["_aux"]
    side: dict[str, pd.DataFrame] = {}
    side["profile"] = binned_profile(d)                # §3 BIN (50 m means)
    side["radiation"] = radiation_profile(d)           # §3 RADC
    side["dh"] = gnss_baro_dh(d)                       # §3 ERR
    lap, gamma = lapse_rate(d)                         # §3 LAP
    side["lapse"] = lap.assign(gamma_c_per_km=gamma)
    side["oscillation"] = oscillation(d)               # §3 P6
    side["wind"] = wind_proxy(d, aux["traj"])          # §3 WIND
    for name, tab in side.items():
        tab.to_csv(out / f"{stem}_{name}.csv", index=False, float_format="%.4f")

    meta = {
        "source_log": a.log, "fs_hz": ds["fs"], "window": ds["window"],
        "stride": ds["stride"],
        "task": "predict altitude from atmospheric composition: exact "
                "altitude (regression, y_alt m) or altitude band "
                "(classification, y_band, band_m classes)",
        "env_input_channels": ds["env_names"],
        "label": "y_alt = h_smooth in metres (y_alt_n normalised with "
                 "alt_mu/alt_sd); y_band = floor(y_alt / band_m)",
        "band_m": ds["band_m"], "n_bands": ds["n_bands"],
        "excluded_from_inputs": "pressure_hpa and every altitude/velocity/"
                                "attitude/trajectory channel (leak guards)",
        "channels": ds["channels"], "n_samples": len(ds["grid"]),
        "n_windows": {k: int(len(ds["X_win"][k])) for k in ds["X_win"]},
        "split_segments_samples": {k: [int(v[0]), int(v[1])]
                                   for k, v in ds["split"].items()},
        "coverage": ds["coverage"],
        "normalisation": "X_norm = (X − mu) / std, per channel, from the full flight",
        "mu": ds["mu"].tolist(), "std": ds["std"].tolist(),
        "shapes": {"X_seq": list(ds["X_seq"].shape),
                   "X_win": [None, len(ds["channels"]), ds["window"]]},
        "side_tables": [f"{stem}_{k}.csv" for k in side],
        "lapse_rate_c_per_km": None if np.isnan(gamma) else float(gamma),
        "note": "X_win / X_win_env are channels-first [N, C, W]; transpose "
                "for TF/Keras. Pressure is excluded from env inputs on "
                "purpose (it is an altimeter). The binned §3 tables "
                "(profile/radiation/dh/lapse/oscillation/wind) sit "
                "alongside as CSVs.",
    }
    (out / f"{stem}_metadata.json").write_text(json.dumps(meta, indent=2))

    n_ch = len(ds["channels"])
    print(f"  {stem}: {n_ch} channels × {len(ds['grid'])} samples @ {a.fs} Hz, "
          f"windows train/val/test = "
          f"{[len(ds['X_win'][k]) for k in ('train', 'val', 'test')]}")
    low = [f"{c} ({v:.0%})" for c, v in ds["coverage"].items() if v < 0.5]
    if low:
        print(f"  ⚠ low-coverage channels: {', '.join(low)}")
    print(f"  wrote {out}/{stem}_dataset.npz, _flat.csv, _metadata.json "
          f"+ {len(side)} side tables")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
