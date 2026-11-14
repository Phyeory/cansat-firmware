"""plot_flight.py: flight-profile graphs from a CanSat log.

Reads any log CanSatData understands (SD card FLxxxxxxxx.csv, the ground
station's flight_*.csv, or JSON lines) and renders:

  01_altitude_vs_time.png        compulsory plot (baro + GPS altitude)
  02_temperature_vs_altitude.png compulsory plot (raw points + 50 m bin means)
  03_flight_dashboard.png        altitude, descent rate, temp, ground track,
                                 battery, sensor health on one page
  04_science_profiles.png        UV / IR / CO2 / O3 / RH / radiation vs altitude

Usage:
    python3 plot_flight.py FL00012345.csv --out plots/
    python3 plot_flight.py flight_20270315_101500.csv --out plots/
    python3 plot_flight.py --demo --out plots/      # simulated flight, for testing

Needs: numpy, pandas, matplotlib, seaborn
"""
from __future__ import annotations

import argparse
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import seaborn as sns

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cansat_data import CanSatData, HEALTH_BITS  # noqa: E402

# ---------------------------------------------------------------- style ----
BG, INK, INK2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0"
BLUE, ORANGE, AQUA, YELLOW, MAGENTA, GREEN, VIOLET = (
    "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6")
PHASE_CLR = {"Pad": "#c9c8c2", "Ascent": ORANGE, "Descent": BLUE, "Landed": "#c9c8c2"}
BIN_M = 50

sns.set_theme(style="whitegrid", font="DejaVu Sans", rc={
    "figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG,
    "axes.edgecolor": GRID, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "text.color": INK, "axes.spines.top": False, "axes.spines.right": False,
    "axes.titlesize": 12, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "axes.titlecolor": INK, "font.size": 10.5, "legend.frameon": False,
})


def header(fig, title, sub, src):
    fig.text(0.03, 0.975, title, fontsize=18, fontweight="bold", va="top")
    fig.text(0.03, 0.935, sub, fontsize=11, color=INK2, va="top")
    fig.text(0.97, 0.008, src, fontsize=8.5, color=MUTED, ha="right", va="bottom")


def has(df, col):
    return col in df.columns and df[col].notna().sum() > 3


# --------------------------------------------------------------- phases ----
def find_phases(t, alt, v):
    """Pad / Ascent / Descent / Landed from altitude + smoothed velocity.
    Plain thresholds, no filtering: good enough to colour the plots."""
    alt0 = np.nanmedian(alt[: max(5, len(alt) // 20)])
    rel = alt - alt0
    i_apo = int(np.nanargmax(rel))
    climbing = np.where((rel[:i_apo + 1] > 10) & (v[:i_apo + 1] > 2))[0]
    i_launch = int(climbing[0]) if len(climbing) else 0
    while i_launch > 0 and v[i_launch - 1] > 0.5:
        i_launch -= 1
    after = np.where((rel[i_apo:] < 10) & (np.abs(v[i_apo:]) < 1.0))[0]
    i_land = i_apo + int(after[0]) if len(after) else len(t) - 1
    phase = np.full(len(t), "Descent", dtype=object)
    phase[:i_launch] = "Pad"
    phase[i_launch:i_apo] = "Ascent"
    phase[i_land:] = "Landed"
    return phase, dict(launch=i_launch, apogee=i_apo, land=i_land, alt0=alt0)


def shade_phases(ax, t, ev, labels=True):
    ax.axvspan(t[ev["launch"]], t[ev["apogee"]], color=ORANGE, alpha=0.07, lw=0)
    ax.axvspan(t[ev["apogee"]], t[ev["land"]], color=BLUE, alpha=0.06, lw=0)
    if labels:
        y = ax.get_ylim()[1]
        for i, name in ((ev["launch"], "launch"), (ev["apogee"], "apogee"),
                        (ev["land"], "landed")):
            ax.axvline(t[i], color=MUTED, lw=0.8, ls=":")
            ax.text(t[i], y, f" {name}", fontsize=8.5, color=INK2, va="top")


# ---------------------------------------------------------------- plots ----
def plot_altitude(d, df, ev, out, src):
    fig, ax = plt.subplots(figsize=(12, 6.5))
    fig.subplots_adjust(left=0.08, right=0.97, top=0.82, bottom=0.11)
    ax.plot(d.t, df.alt_m, color=BLUE, lw=2, label="Barometric (BMP280)", zorder=4)
    if has(df, "gps_alt_m"):
        g = (df.sats.fillna(0) > 0) & df.gps_alt_m.notna()
        g &= df.gps_alt_m.diff().ne(0) | (np.arange(len(df)) % 10 == 0)
        g &= np.r_[True, np.diff(np.floor(d.t)) > 0]          # ~1 Hz, like the GPS
        ax.scatter(d.t[g], df.gps_alt_m[g], s=16, color=ORANGE, alpha=0.9,
                   label="GPS (SAM-M8Q)", zorder=5, edgecolor=BG, linewidths=0.6)
    ax.set_ylim(min(0, df.alt_m.min()) - 20, df.alt_m.max() * 1.12)
    shade_phases(ax, d.t, ev)
    ia = ev["apogee"]
    ax.scatter([d.t[ia]], [df.alt_m.iloc[ia]], s=70, color=BLUE, edgecolor=BG,
               linewidth=2, zorder=4)
    ax.annotate(f"apogee {df.alt_m.iloc[ia]:.0f} m", (d.t[ia], df.alt_m.iloc[ia]),
                xytext=(10, 6), textcoords="offset points", fontsize=10, color=INK)
    ax.set_xlabel("Time since log start (s)")
    ax.set_ylabel("Altitude (m)")
    ax.legend(loc="upper right")
    dur = d.t[ev["land"]] - d.t[ev["apogee"]]
    header(fig, "Altitude vs time",
           f"Apogee {df.alt_m.iloc[ia]:.0f} m, descent took {dur:.0f} s.", src)
    fig.savefig(out / "01_altitude_vs_time.png", dpi=200)
    plt.close(fig)


def binned(df, col, mask):
    sub = df.loc[mask, ["alt_m", col]].dropna()
    if sub.empty:
        return pd.DataFrame(columns=["alt", col])
    b = (sub.alt_m.clip(lower=0) // BIN_M) * BIN_M + BIN_M / 2
    return sub.groupby(b)[col].mean().rename_axis("alt").reset_index()


def plot_temperature(df, phase, out, src):
    fig, ax = plt.subplots(figsize=(9, 8.5))
    fig.subplots_adjust(left=0.11, right=0.96, top=0.85, bottom=0.08)
    for ph in ("Ascent", "Descent"):
        m = phase == ph
        ax.scatter(df.temp_c[m], df.alt_m[m], s=6, color=PHASE_CLR[ph], alpha=0.35,
                   linewidths=0, label=f"{ph} samples")
    desc = phase == "Descent"
    bm = binned(df, "temp_c", desc)
    ax.plot(bm.temp_c, bm.alt, color=INK, lw=2, marker="o", ms=5,
            label=f"Descent, {BIN_M} m bin mean")
    lapse = ""
    if len(bm) > 2:
        k = np.polyfit(bm.alt, bm.temp_c, 1)[0]
        lapse = f" Mean lapse rate {-k * 1000:.1f} °C/km."
    ax.set_xlabel("Temperature (°C)")
    ax.set_ylabel("Altitude (m)")
    ax.legend(loc="upper right")
    header(fig, "Temperature vs altitude",
           f"BMP280 raw temperature, descent binned every {BIN_M} m.{lapse}", src)
    fig.savefig(out / "02_temperature_vs_altitude.png", dpi=200)
    plt.close(fig)


def plot_dashboard(d, df, phase, ev, v, out, src):
    fig = plt.figure(figsize=(16, 10.5))
    gs = fig.add_gridspec(3, 3, left=0.06, right=0.97, top=0.87, bottom=0.07,
                          hspace=0.55, wspace=0.38)
    # altitude
    ax = fig.add_subplot(gs[0, :2])
    for ph, c in PHASE_CLR.items():
        m = phase == ph
        ax.plot(np.where(m, d.t, np.nan), np.where(m, df.alt_m, np.nan), color=c, lw=2)
    ax.set_title("Altitude")
    ax.set_ylabel("m")
    shade_phases(ax, d.t, ev, labels=False)
    # descent rate
    ax = fig.add_subplot(gs[1, :2])
    ax.plot(d.t, v, color=VIOLET, lw=1.6)
    ax.axhline(0, color=MUTED, lw=0.8)
    desc = phase == "Descent"
    if desc.any():
        vd = np.nanmedian(v[desc])
        ax.axhline(vd, color=BLUE, lw=1, ls="--")
        ax.text(d.t[-1], vd, f"median descent {abs(vd):.1f} m/s ", color=INK2,
                fontsize=9, ha="right", va="top")
    ax.set_title("Vertical velocity")
    ax.set_ylabel("m/s")
    shade_phases(ax, d.t, ev, labels=False)
    # temperature vs altitude
    ax = fig.add_subplot(gs[:2, 2])
    for ph in ("Ascent", "Descent"):
        m = phase == ph
        ax.scatter(df.temp_c[m], df.alt_m[m], s=4, color=PHASE_CLR[ph], alpha=0.35,
                   linewidths=0, label=ph)
    bm = binned(df, "temp_c", desc)
    ax.plot(bm.temp_c, bm.alt, color=INK, lw=1.8, label="bin mean")
    ax.set_title("Temperature profile")
    ax.set_xlabel("°C")
    ax.set_ylabel("Altitude (m)")
    ax.legend(loc="upper right", fontsize=9, markerscale=2.5)
    # ground track
    ax = fig.add_subplot(gs[2, 0])
    if has(df, "lat") and (df.sats.fillna(0) > 0).sum() > 3:
        g = (df.sats.fillna(0) > 0) & df.lat.notna()
        lat0, lon0 = df.lat[g].iloc[0], df.lon[g].iloc[0]
        x = (df.lon[g] - lon0) * 111_320 * np.cos(np.radians(lat0))
        y = (df.lat[g] - lat0) * 110_540
        cmap = matplotlib.colors.LinearSegmentedColormap.from_list("alt", ["#a9c9ef", BLUE, "#0d3a73"])
        sc = ax.scatter(x, y, c=df.alt_m[g], cmap=cmap, s=10, linewidths=0)
        ax.scatter([x.iloc[-1]], [y.iloc[-1]], marker="X", s=80, color=ORANGE, zorder=3)
        ax.set_aspect("equal", adjustable="datalim")
        cb = fig.colorbar(sc, ax=ax, fraction=0.05, pad=0.02)
        cb.ax.set_title("alt m", fontsize=8.5, color=INK2)
        cb.outline.set_visible(False)
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.set_title(f"Ground track, drift {np.hypot(x.iloc[-1], y.iloc[-1]):.0f} m")
    else:
        ax.text(0.5, 0.5, "no GPS fix in log", ha="center", va="center", color=MUTED)
        ax.set_axis_off()
        ax.set_title("Ground track")
    # battery
    ax = fig.add_subplot(gs[2, 1])
    if has(df, "vbat_v"):
        ax.plot(d.t, df.vbat_v, color=GREEN, lw=1.6)
        ax.set_title(f"Battery, {df.vbat_v.iloc[0]:.2f} → {df.vbat_v.iloc[-1]:.2f} V")
        ax.set_ylabel("V")
        ax.set_xlabel("Time (s)")
    # health
    ax = fig.add_subplot(gs[2, 2])
    if has(df, "health"):
        h = df.health.fillna(0).astype(int).to_numpy()
        grid = np.array([(h >> i) & 1 for i in range(len(HEALTH_BITS))])
        ax.imshow(grid, aspect="auto", interpolation="nearest",
                  cmap=matplotlib.colors.ListedColormap(["#f2b8a2", "#bfe8d6"]),
                  extent=[d.t[0], d.t[-1], len(HEALTH_BITS) - 0.5, -0.5], vmin=0, vmax=1)
        ax.set_yticks(range(len(HEALTH_BITS)), HEALTH_BITS, fontsize=8.5)
        ax.grid(False)
        ok = grid.mean(axis=1) * 100
        worst = HEALTH_BITS[int(np.argmin(ok))]
        ax.set_title(f"Sensor health, weakest {worst} {ok.min():.0f}% up")
        ax.set_xlabel("Time (s)")
    ia = ev["apogee"]
    header(fig, "Flight profile",
           f"{len(df)} records over {d.t[-1]:.0f} s. Apogee {df.alt_m.iloc[ia]:.0f} m "
           f"at t = {d.t[ia]:.0f} s.", src)
    fig.savefig(out / "03_flight_dashboard.png", dpi=180)
    plt.close(fig)


SCIENCE = [("uv_index", "UV index", YELLOW), ("ir_counts", "IR (counts)", ORANGE),
           ("co2_ppm", "CO₂ (ppm)", MUTED), ("o3", "O₃", AQUA),
           ("rh_pct", "Relative humidity (%)", BLUE), ("rad_cps", "Radiation (counts/s)", VIOLET)]


def plot_science(df, phase, out, src):
    chans = [c for c in SCIENCE if has(df, c[0])]
    if not chans:
        return
    n = len(chans)
    cols = min(3, n)
    rows = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 5.2 * rows), sharey=True,
                             squeeze=False)
    fig.subplots_adjust(left=0.07, right=0.97, top=0.84 if rows > 1 else 0.76,
                        bottom=0.08, hspace=0.35, wspace=0.12)
    desc = phase == "Descent"
    for ax, (col, lab, c) in zip(axes.flat, chans):
        ax.scatter(df[col][desc], df.alt_m[desc], s=4, color=c, alpha=0.25, linewidths=0)
        bm = binned(df, col, desc)
        ax.plot(bm[col], bm.alt, color=INK, lw=1.8, marker="o", ms=3.5)
        ax.set_title(lab)
    for ax in axes[:, 0]:
        ax.set_ylabel("Altitude (m)")
    for ax in list(axes.flat)[n:]:
        ax.set_axis_off()
    header(fig, "Science channels vs altitude",
           f"Descent only. Dots are raw samples, black line is the {BIN_M} m bin mean.", src)
    fig.savefig(out / "04_science_profiles.png", dpi=180)
    plt.close(fig)


# ------------------------------------------------------------------ demo ----
def make_demo_log(path):
    """Simulated SD log in the firmware's column layout. NOT real data."""
    rng = np.random.default_rng(7)
    dt = 0.1
    t = np.arange(0, 175, dt)
    pad, burn, apo_t, apo_h = 20.0, 2.0, 30.0, 1020.0
    h = np.zeros_like(t)
    a = (t >= pad) & (t < apo_t)
    s = (t[a] - pad) / (apo_t - pad)
    h[a] = apo_h * (1 - (1 - s) ** 2)
    d = t >= apo_t
    vfall = 10.0 + 4 * np.exp(-(t[d] - apo_t) / 4)
    h[d] = np.maximum(apo_h - np.cumsum(vfall) * dt, 0)
    tz = 9.5 - 0.0068 * h + 1.2 * np.exp(-((h - 450) / 80) ** 2)
    p = 1013.25 * (1 - 0.0065 * (h + 35) / 288.15) ** 5.2559
    alt = 44330 * (1 - (p / 1013.25) ** 0.1903) - 35 + rng.normal(0, 0.6, len(t))
    n = len(t)
    sats = np.where(t > 8, 9, 0)
    drift = np.cumsum(np.where(d, 3.2, 0.0)) * dt
    lat = 52.0 + drift * 0.6 / 110_540
    lon = -1.0 + drift * 0.8 / (111_320 * np.cos(np.radians(52)))
    health = np.full(n, 0x7FF)
    health[t < 8] &= ~(1 << 5)
    drop = (t > 95) & (t < 104)
    health[drop] &= ~(1 << 3)
    df = pd.DataFrame({
        "t_ms": (t * 1000).astype(int),
        "temp_c": tz + rng.normal(0, 0.08, n),
        "pressure_hpa": p,
        "alt_m": alt,
        "uv_raw": 0, "uv_index": 2.1 + h / 900 + rng.normal(0, 0.08, n),
        "ir_counts": 3200 + 0.6 * h + rng.normal(0, 40, n),
        "full_counts": 0, "lux": 0,
        "co2_ppm": 418 - h / 300 + rng.normal(0, 3, n),
        "o3_ppb": 32 + h / 120 + rng.normal(0, 1.5, n),
        "lat": np.where(sats > 0, lat, np.nan), "lon": np.where(sats > 0, lon, np.nan),
        "gps_alt_m": np.where(sats > 0, h + rng.normal(0, 4, n), np.nan),
        "sats": sats, "vbat_v": 8.25 - t * 0.0016 + rng.normal(0, 0.005, n),
        "health": health,
        "acc_x": 0, "acc_y": 0, "acc_z": 9.81, "gyr_x": 0, "gyr_y": 0, "gyr_z": 0,
        "rh_pct": 62 + h / 40 + rng.normal(0, 0.7, n), "sht_temp_c": tz,
        "rad_cps": rng.poisson(0.35 + h / 2500, n).astype(float),
    })
    df.to_csv(path, index=False)
    return path


# ------------------------------------------------------------------ main ----
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?", help="SD CSV, ground-station CSV or JSONL")
    ap.add_argument("--out", default="plots", help="output folder (default plots/)")
    ap.add_argument("--demo", action="store_true", help="plot a simulated flight")
    a = ap.parse_args()
    if not a.log and not a.demo:
        ap.error("give a log file or --demo")

    if a.demo:
        path = make_demo_log(Path(tempfile.gettempdir()) / "cansat_demo_SIMULATED.csv")
        src = "SIMULATED FLIGHT (plot_flight.py --demo), not real data"
    else:
        path = Path(a.log)
        src = f"Log: {path.name}"

    d = CanSatData(path)
    df = d.df
    if not has(df, "alt_m"):
        sys.exit("log has no altitude column; nothing to plot")
    _, v = d.get_vel()
    phase, ev = find_phases(d.t, df.alt_m.to_numpy(float), v)

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    plot_altitude(d, df, ev, out, src)
    if has(df, "temp_c"):
        plot_temperature(df, phase, out, src)
    plot_dashboard(d, df, phase, ev, v, out, src)
    plot_science(df, phase, out, src)
    print(f"{d!r}: apogee {df.alt_m.iloc[ev['apogee']]:.0f} m. Plots in {out}/")


if __name__ == "__main__":
    main()
