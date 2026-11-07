"""Altitude-binned science profiles + GNSS−baro altitude error —
DATA-PIPELINE.md nodes BIN, RADC, ERR.

Pure formula plug-ins: per-bin statistics of every logged science channel
against barometric altitude (50 m default, per the doc), the radiation
count rate with a standard-error bar (node RADC), and the one-subtraction
GNSS−baro Δh (node ERR).  No filters — slow science channels get bins,
not KFs (§6.1).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from cansat_data import CanSatData

SCIENCE = ("temp_c", "pressure_hpa", "rh_pct", "co2_ppm", "o3",
           "uv_index", "uv_raw", "ir_counts", "full_counts", "lux",
           "rad_cps")


def gnss_baro_dh(d: CanSatData) -> pd.DataFrame:
    """Node ERR: Δh = h_GNSS − h_baro, NaN where there is no fix
    (sats > 0, same rule as cansat_data.get_gps)."""
    fix = (d.df["sats"].fillna(0) > 0).to_numpy()
    dh = d.df["gps_alt_m"].to_numpy(float) - d.df["alt_m"].to_numpy(float)
    dh[~fix] = np.nan
    return pd.DataFrame({"t": d.t, "dh_m": dh})


def binned_profile(d: CanSatData, bin_m: float = 50.0, channels=None,
                   min_n: int = 3, phase: str = "all") -> pd.DataFrame:
    """DataFrame with one row per altitude bin: bin_m (centre), n, and
    <channel>_mean / _std for every requested channel present in the log.
    phase: 'all' | 'ascent' | 'descent' (split at the raw altitude peak —
    science profiles usually want 'descent')."""
    alt = d.df["alt_m"].to_numpy(float)
    m = np.ones(len(d.t), bool)
    if phase != "all":
        i_ap = int(np.nanargmax(alt))
        m = (np.arange(len(d.t)) <= i_ap) if phase == "ascent" \
            else (np.arange(len(d.t)) > i_ap)
    chans = [c for c in (channels or SCIENCE) if c in d.df.columns]
    edges = np.arange(np.floor(np.nanmin(alt) / bin_m) * bin_m,
                      np.nanmax(alt) + bin_m, bin_m)
    which = np.digitize(alt, edges) - 1
    rows = []
    for k in range(len(edges) - 1):
        sel = m & (which == k)
        if sel.sum() >= min_n:
            row = {"bin_m": 0.5 * (edges[k] + edges[k + 1]), "n": int(sel.sum())}
            for c in chans:
                v = d.df[c].to_numpy(float)[sel]
                v = v[np.isfinite(v)]
                if v.size:
                    row[f"{c}_mean"] = float(v.mean())
                    row[f"{c}_std"] = float(v.std()) if v.size > 1 else 0.0
            rows.append(row)
    return pd.DataFrame(rows)


def radiation_profile(d: CanSatData, bin_m: float = 50.0,
                      min_n: int = 3) -> pd.DataFrame:
    """Node RADC: rad_cps per bin with the standard error of the mean.
    The per-sample Poisson sigma of a 1 s latch is √cps, so std/√n is the
    honest uncertainty for the count-rate-vs-altitude plot (P7)."""
    prof = binned_profile(d, bin_m, ["rad_cps"], min_n)
    if prof.empty:
        return prof
    prof["rad_cps_sem"] = prof["rad_cps_std"] / np.sqrt(prof["n"])
    return prof[["bin_m", "n", "rad_cps_mean", "rad_cps_sem"]]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log")
    ap.add_argument("--bin", type=float, default=50.0, help="bin size, m")
    ap.add_argument("--phase", choices=("all", "ascent", "descent"),
                    default="all")
    ap.add_argument("--dh", action="store_true",
                    help="print the GNSS−baro Δh stats too")
    ap.add_argument("--csv", metavar="DIR")
    a = ap.parse_args(argv)

    d = CanSatData(a.log)
    prof = binned_profile(d, a.bin, phase=a.phase)
    cols = ["bin_m", "n"] + [c for c in prof.columns if c.endswith("_mean")]
    print(f"  {a.bin:.0f} m binned profile ({a.phase}):")
    print(prof[cols].round(2).to_string(index=False))
    rad = radiation_profile(d, a.bin)
    if len(rad):
        print("  radiation count rate vs altitude:")
        print(rad.round(3).to_string(index=False))
    if a.dh:
        dh = gnss_baro_dh(d)["dh_m"].to_numpy(float)
        dh = dh[np.isfinite(dh)]
        if len(dh):
            print(f"  GNSS − baro Δh: mean {dh.mean():+.1f} m, "
                  f"std {dh.std():.1f} m, max |Δh| {np.abs(dh).max():.1f} m "
                  f"({len(dh)} fixes)")
    if a.csv:
        out = Path(a.csv)
        out.mkdir(parents=True, exist_ok=True)
        f = out / f"{Path(a.log).stem}_profile.csv"
        prof.to_csv(f, index=False, float_format="%.3f")
        print(f"  wrote {f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
