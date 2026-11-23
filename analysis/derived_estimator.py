"""Oscillation FFT + horizontal drift/wind proxy — DATA-PIPELINE.md §3 rows
"Descent oscillation frequency" (plot P6) and "Horizontal drift / wind proxy".

Formula-only, numpy/pandas, no scipy:

- `oscillation`   sliding-window per-axis FFT of the angular rate (window
                  mean removed, so steady spin/tumble doesn't mask the
                  pendulum) → strongest frequency in [fmin, fmax] + amplitude.
- `wind_proxy`    per-altitude-bin displacement of the smoothed trajectory
                  (kalman.py trajectory_filter, p_N/p_E) — pad-referenced
                  drift, per-bin step, and heading.  Without GNSS the
                  trajectory N/E is inertial-only and drifts with it.

    python3 derived_estimator.py SD_LOG.csv --csv out/
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from cansat_data import CanSatData
import kalman as kf

R2D = kf.R2D


def oscillation(d: CanSatData, win_s: float = 5.0, hop_s: float = 1.0,
                fmin: float = 0.2, fmax: float = 5.0) -> pd.DataFrame:
    """Node P6: dominant angular-oscillation frequency (Hz) + amplitude (°/s)
    per hop.  The window mean is removed per window (detrend), the rfft peak
    inside [fmin, fmax] picked on the anomaly magnitude ‖gyr − mean‖."""
    t = d.t
    gyr = d.df[["gyr_x", "gyr_y", "gyr_z"]].to_numpy(float)
    fs = 1.0 / float(np.median(np.diff(t)))
    n_win = max(4, int(round(win_s * fs)))
    hop = max(1, int(round(hop_s * fs)))
    freqs = np.fft.rfftfreq(n_win, d=1.0 / fs)
    band = np.flatnonzero((freqs >= fmin) & (freqs <= fmax))
    win_fn = np.hanning(n_win)

    rows = []
    for s in range(0, max(1, len(t) - n_win + 1), hop):
        w = gyr[s:s + n_win]
        if np.isfinite(w).sum() < 0.8 * w.size:
            rows.append((t[s + n_win // 2], np.nan, np.nan))
            continue
        # per-axis detrended rfft — a magnitude FFT would full-wave-rectify a
        # single-axis wobble and double its frequency
        specs = np.abs(np.fft.rfft((w - np.nanmean(w, 0)) * win_fn[:, None],
                                   axis=0))          # [freq, axis]
        flat = int(np.argmax(specs[band, :]))        # argmax is over the
        j, i_ax = band[flat // 3], flat % 3          # flattened block

        # Hann coherent gain 0.5: amp = 2·|X|/(0.5·N) → 4·|X|/N
        rows.append((t[s + n_win // 2], float(freqs[j]),
                     float(4.0 * specs[j, i_ax] / n_win)))
    return pd.DataFrame(rows, columns=["t", "osc_freq_hz", "osc_amp_dps"])


def wind_proxy(d: CanSatData, traj: pd.DataFrame | None = None,
               bin_m: float = 50.0, min_n: int = 3) -> pd.DataFrame:
    """Node WIND: horizontal drift per altitude bin from the smoothed
    trajectory.  pad-referenced drift (hypot of p_N, p_E), per-bin step,
    and heading (deg from north).  Inertial-only drift when no GNSS."""
    if traj is None:
        traj = kf.trajectory_filter(d, att=kf.attitude_filter(d))
    h = traj["h_smooth"].to_numpy(float)
    pN = traj["p_N"].to_numpy(float)
    pE = traj["p_E"].to_numpy(float)
    ok = np.isfinite(h)
    edges = np.arange(np.floor(np.nanmin(h[ok]) / bin_m) * bin_m,
                      np.nanmax(h[ok]) + bin_m, bin_m)
    which = np.digitize(h[ok], edges) - 1
    N, E = pN[ok], pE[ok]
    rows = []
    for k in range(len(edges) - 1):
        m = which == k
        if m.sum() < min_n:
            continue
        n_m, e_m = float(np.mean(N[m])), float(np.mean(E[m]))
        prev = rows[-1] if rows else {"p_N_mean": 0.0, "p_E_mean": 0.0}
        dN, dE = n_m - prev["p_N_mean"], e_m - prev["p_E_mean"]
        rows.append({
            "bin_m": 0.5 * (edges[k] + edges[k + 1]), "n": int(m.sum()),
            "p_N_mean": n_m, "p_E_mean": e_m,
            "step_N_m": dN, "step_E_m": dE,
            "step_drift_m": float(np.hypot(dN, dE)),
            "drift_from_pad_m": float(np.hypot(n_m, e_m)),
            "dir_deg": float(np.degrees(np.arctan2(e_m, n_m))),
        })
    return pd.DataFrame(rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log")
    ap.add_argument("--csv", metavar="DIR")
    ap.add_argument("--win", type=float, default=5.0, help="FFT window, s")
    a = ap.parse_args(argv)

    d = CanSatData(a.log)
    stem = Path(a.log).stem
    osc = oscillation(d, win_s=a.win)
    fin = osc["osc_freq_hz"].dropna()
    print("  oscillation (windowed FFT of gyro anomaly):")
    if len(fin):
        top = osc.loc[fin.idxmax()]
        print(f"    median {fin.median():.2f} Hz, max amp "
              f"{osc['osc_amp_dps'].max():.1f} °/s @ {top['osc_freq_hz']:.2f} Hz")
    traj = kf.trajectory_filter(d, att=kf.attitude_filter(d))
    wind = wind_proxy(d, traj)
    if len(wind):
        print("  horizontal drift vs altitude (smoothed trajectory):")
        print(wind[["bin_m", "n", "drift_from_pad_m", "dir_deg"]]
              .round(1).to_string(index=False))
    if a.csv:
        out = Path(a.csv)
        out.mkdir(parents=True, exist_ok=True)
        f1 = out / f"{stem}_oscillation.csv"
        f2 = out / f"{stem}_wind.csv"
        osc.to_csv(f1, index=False, float_format="%.4f")
        wind.to_csv(f2, index=False, float_format="%.4f")
        print(f"  wrote {f1}\n  wrote {f2}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
