"""Plot every CanSat channel and every §3 derived quantity — plain matplotlib.

Altitude axis = the ★ linear-KF + RTS `h_smooth` (analysis/kalman.py).
Per-channel PNGs put **altitude on Y** (ascent solid / descent dashed — the
split at the smoothed-apogee index).  Plus every DATA-PIPELINE.md §4 plot:
P1 altitude-time, P2 temperature-altitude, P3 descent-rate-time, P4 science
heatmap, P5 battery+health timeline, P6 oscillation, GNSS−baro Δh, and the
horizontal-drift map.

    python3 plot_flight.py SD_LOG.csv --out plots/ [--pad-s 5]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from cansat_data import CanSatData, HEALTH_BITS
from atmosphere_estimator import estimate as atmosphere_estimate
from profile_estimator import binned_profile
from derived_estimator import oscillation
import kalman as kf

# (canonical column, axis label) — altitude always on Y, channel on X
CHANNELS = [
    # §2 raw
    ("temp_c",      "Temperature (°C)"),
    ("pressure_hpa", "Pressure (hPa)"),
    ("alt_m",       "Raw baro altitude (m)"),
    ("rh_pct",      "Relative humidity (%)"),
    ("sht_temp_c",  "SHT45 temp (°C)"),
    ("co2_ppm",     "CO₂ (ppm)"),
    ("o3",          "Ozone (ppb)"),
    ("uv_index",    "UV index"),
    ("uv_raw",      "UV raw counts"),
    ("ir_counts",   "IR counts"),
    ("full_counts", "Full-spectrum counts"),
    ("lux",         "Illuminance (lux)"),
    ("rad_cps",     "Radiation (counts/s)"),
    ("acc_x",       "Accel X (m/s²)"),
    ("acc_y",       "Accel Y (m/s²)"),
    ("acc_z",       "Accel Z (m/s²)"),
    ("gyr_x",       "Gyro X (°/s)"),
    ("gyr_y",       "Gyro Y (°/s)"),
    ("gyr_z",       "Gyro Z (°/s)"),
    ("mag_x",       "Mag X (µT)"),
    ("mag_y",       "Mag Y (µT)"),
    ("mag_z",       "Mag Z (µT)"),
    ("vbat_v",      "Battery (V)"),
    ("sats",        "GNSS satellites"),
    ("gps_alt_m",   "GNSS altitude (m)"),
    ("lat",         "GNSS latitude (°)"),
    ("lon",         "GNSS longitude (°)"),
    ("seq",         "Radio packet number"),
    ("health",      "Health bitmask"),
    # §3 derived — KF stack
    ("v_smooth",    "Vertical velocity (m/s)"),
    ("sigma_h",     "Altitude 1σ (m)"),
    ("sigma_v",     "Velocity 1σ (m/s)"),
    ("roll_deg",    "Roll (°)"),
    ("pitch_deg",   "Pitch (°)"),
    ("yaw_deg",     "Yaw (°)"),
    ("sigma_tilt_deg", "Attitude 1σ tilt (°)"),
    ("bgx_dps",     "Gyro bias X (°/s)"),
    ("bgy_dps",     "Gyro bias Y (°/s)"),
    ("bgz_dps",     "Gyro bias Z (°/s)"),
    ("p_N",         "Trajectory north (m)"),
    ("p_E",         "Trajectory east (m)"),
    ("v_N",         "Velocity north (m/s)"),
    ("v_E",         "Velocity east (m/s)"),
    ("v_D",         "Velocity down (m/s)"),
    ("sigma_p",     "Position 1σ (m)"),
    ("sigma_v_traj", "Trajectory velocity 1σ (m/s)"),
    # §3 derived — formulas
    ("rho_kgm3",    "Air density (kg/m³)"),
    ("t_dew_c",     "Dew point (°C)"),
    ("abs_hum_gm3", "Absolute humidity (g/m³)"),
    ("dh_gps_baro", "GNSS − baro Δh (m)"),
    ("osc_freq_hz", "Oscillation frequency (Hz)"),
    ("osc_amp_dps", "Oscillation amplitude (°/s)"),
]

HEATMAP_CHANS = ["temp_c", "rh_pct", "co2_ppm", "o3", "uv_index",
                 "ir_counts", "lux", "rad_cps"]


def _asc_desc_split(h: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    i_ap = int(np.nanargmax(h))
    asc = np.arange(len(h)) <= i_ap
    return asc, ~asc


def _plot_channel(ax, h, y, label):
    """One channel vs altitude: ascent solid, descent dashed, NaN-safe."""
    ok = np.isfinite(h) & np.isfinite(y)
    asc, desc = _asc_desc_split(h)
    if ok[asc].sum() > 1:
        ax.plot(y[asc & ok], h[asc & ok], "-", label="ascent")
    if ok[desc].sum() > 1:
        ax.plot(y[desc & ok], h[desc & ok], "--", label="descent")
    ax.set_xlabel(label)
    ax.set_ylabel("Altitude (m)")
    ax.grid(alpha=0.3)


def make_plots(d: CanSatData, out: Path, pad_s: float = 5.0,
               stem: str = "flight") -> list[Path]:
    """Run the KF stack, write every PNG, return the paths written."""
    out.mkdir(parents=True, exist_ok=True)

    att = kf.attitude_filter(d, pad_s=pad_s)
    alt = kf.altitude_filter(d, att=att, pad_s=pad_s)
    traj = kf.trajectory_filter(d, att=att, pad_s=pad_s)
    est = atmosphere_estimate(d)
    der = oscillation(d)          # window-centred → align to the 10 Hz samples
    der = der.set_index("t").reindex(d.t).interpolate(
        limit_direction="both").reset_index()
    h = alt["h_smooth"].to_numpy(float)
    t = alt["t"].to_numpy(float)
    written: list[Path] = []

    # assemble every series by canonical name
    series: dict[str, np.ndarray] = {}
    for frame in (d.df, alt, att, traj, est, der):
        for c in frame.columns:
            if c != "t":
                series[c] = frame[c].to_numpy(float)
    fix = (d.df["sats"].fillna(0) > 0).to_numpy()
    series["dh_gps_baro"] = np.where(
        fix, d.df["gps_alt_m"].to_numpy(float) - d.df["alt_m"].to_numpy(float),
        np.nan)
    series["sigma_v_traj"] = traj["sigma_v"].to_numpy(float)

    def save(fig, name):
        f = out / f"{stem}_{name}.png"
        fig.tight_layout(); fig.savefig(f, dpi=150); plt.close(fig)
        written.append(f)

    # ---- §4 P1: altitude vs time (compulsory) ----
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(t, alt["h_baro"].to_numpy(float), ".", ms=2, alpha=0.4,
            label="baro raw (pad-zeroed)")
    ax.plot(t, h, "-", lw=1.5, label="KF + RTS smooth")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Altitude (m)")
    ax.set_title("P1 · Altitude vs time")
    ax.grid(alpha=0.3); ax.legend()
    save(fig, "altitude_time")

    # ---- §4 P2: temperature vs altitude (compulsory) ----
    fig, ax = plt.subplots(figsize=(5, 7))
    _plot_channel(ax, h, series["temp_c"], "Temperature (°C)")
    ax.set_title("P2 · Temperature vs altitude (compulsory)")
    ax.legend()
    save(fig, "temperature_altitude")

    # ---- §4 P3: descent rate vs time ----
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(t, alt["v_kf"].to_numpy(float), lw=0.8, alpha=0.5, label="KF forward")
    ax.plot(t, alt["v_smooth"].to_numpy(float), lw=1.5, label="KF + RTS smooth")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Vertical velocity (m/s)")
    ax.set_title("P3 · Descent rate vs time")
    ax.grid(alpha=0.3); ax.legend()
    save(fig, "descent_rate_time")

    # ---- §4 P4: science heatmap (50 m bins × channels) ----
    prof = binned_profile(d, 50.0, HEATMAP_CHANS)
    if len(prof):
        cols = [c for c in prof.columns if c.endswith("_mean")]
        grid = np.column_stack([prof[c].to_numpy(float) for c in cols])
        mu = np.nanmean(grid, 0)
        sd = np.where(np.nanstd(grid, 0) > 0, np.nanstd(grid, 0), 1.0)
        fig, ax = plt.subplots(figsize=(1.6 * len(cols) + 3, 6))
        im = ax.imshow((grid - mu) / sd, aspect="auto", cmap="viridis")
        ax.set_yticks(range(len(prof)))
        ax.set_yticklabels([f"{b:.0f}" for b in prof["bin_m"]])
        ax.set_ylabel("Altitude bin centre (m)")
        ax.set_xticks(range(len(cols)))
        ax.set_xticklabels([c.replace("_mean", "") for c in cols],
                           rotation=30, ha="right")
        ax.set_title("P4 · Science channels vs altitude (z-scored 50 m bins)")
        fig.colorbar(im, ax=ax, label="z-score")
        save(fig, "science_heatmap")

    # ---- §4 P5: battery + health timeline ----
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax1.plot(t, series["vbat_v"], lw=1.2)
    ax1.set_ylabel("Battery (V)"); ax1.grid(alpha=0.3)
    ax1.set_title("P5 · Battery + health timeline")
    for j, bit in enumerate(HEALTH_BITS):
        col = (d.df["health"].fillna(0).astype(int) & (1 << j)) > 0
        ax2.step(t, col.astype(float) + j * 1.5, where="post", lw=0.8,
                 label=bit)
    ax2.set_xlabel("Time (s)"); ax2.set_ylabel("bit (stacked)")
    ax2.grid(alpha=0.3); ax2.legend(fontsize=6, ncol=4, loc="best")
    save(fig, "battery_health_time")

    # ---- §4 P6: oscillation ω vs t ----
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax1.plot(der["t"], der["osc_freq_hz"], ".", ms=3)
    ax1.set_ylabel("Dominant frequency (Hz)"); ax1.grid(alpha=0.3)
    ax1.set_title("P6 · Oscillation (FFT of gyro anomaly)")
    ax2.plot(der["t"], der["osc_amp_dps"], ".", ms=3)
    ax2.set_ylabel("Amplitude (°/s)")
    ax2.set_xlabel("Time (s)"); ax2.grid(alpha=0.3)
    save(fig, "oscillation_time")

    # ---- §4: GNSS vs baro Δh ----
    if np.isfinite(series["dh_gps_baro"]).any():
        fig, ax = plt.subplots(figsize=(7, 5))
        ax.plot(t, series["dh_gps_baro"], ".", ms=3)
        ax.axhline(0, color="k", lw=0.5)
        ax.set_xlabel("Time (s)"); ax.set_ylabel("GNSS − baro Δh (m)")
        ax.set_title("GNSS vs baro altitude error")
        ax.grid(alpha=0.3)
        save(fig, "dh_time")

    # ---- horizontal drift map (smoothed trajectory) ----
    fig, ax = plt.subplots(figsize=(6, 6))
    pN, pE = traj["p_N"].to_numpy(float), traj["p_E"].to_numpy(float)
    asc, desc = _asc_desc_split(h)
    ax.plot(pE[asc], pN[asc], "-", label="ascent")
    ax.plot(pE[desc], pN[desc], "--", label="descent")
    ax.plot(pE[0], pN[0], "ko", ms=4, label="pad")
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title("Horizontal drift (KF + RTS trajectory)")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend()
    save(fig, "drift_map")

    # ---- per-channel vs altitude ----
    for c, label in CHANNELS:
        if c not in series or np.isfinite(series[c]).sum() < 2:
            continue
        fig, ax = plt.subplots(figsize=(5, 7))
        _plot_channel(ax, h, series[c], label)
        ax.set_title(f"{label} vs altitude")
        ax.legend(fontsize=8)
        save(fig, f"alt_{c}")

    # ---- overview sheet: science channels in one grid ----
    sci = [c for c, _ in CHANNELS if c in series
           and c not in ("acc_x", "acc_y", "acc_z", "gyr_x", "gyr_y",
                         "gyr_z", "mag_x", "mag_y", "mag_z", "roll_deg",
                         "pitch_deg", "yaw_deg", "lat", "lon", "seq",
                         "health", "bgx_dps", "bgy_dps", "bgz_dps",
                         "sigma_h", "sigma_v", "sigma_p", "sigma_v_traj",
                         "sigma_tilt_deg", "alt_m", "gps_alt_m")]
    ncol = 4
    nrow = int(np.ceil(len(sci) / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(16, 4 * nrow), sharey=True)
    for ax, c in zip(np.atleast_1d(axes).ravel(), sci):
        label = dict(CHANNELS)[c]
        _plot_channel(ax, h, series[c], label)
        ax.set_title(label, fontsize=9)
        ax.legend(fontsize=7)
    for ax in np.atleast_1d(axes).ravel()[len(sci):]:
        ax.axis("off")
    fig.suptitle("Science channels vs altitude (KF-smoothed)")
    save(fig, "overview")
    return written


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log")
    ap.add_argument("--out", default="plots", help="output directory")
    ap.add_argument("--pad-s", type=float, default=5.0)
    a = ap.parse_args(argv)

    d = CanSatData(a.log)
    written = make_plots(d, Path(a.out), pad_s=a.pad_s,
                         stem=Path(a.log).stem)
    print(f"  {len(written)} plots written to {a.out}/ "
          f"(altitude = KF-smoothed, ascent solid / descent dashed)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
