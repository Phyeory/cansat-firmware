"""Atmosphere estimators for logged CanSat data."""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from cansat_data import CanSatData

R_D = 287.05
_MAG_A, _MAG_B = 17.62, 243.12


def air_density(p_hpa, t_c):
    """ρ = P / (Rs·T) in kg/m³ (P in hPa, T in °C)."""
    return np.asarray(p_hpa, float) * 100.0 / (R_D * (np.asarray(t_c, float) + 273.15))


def vapour_pressure(t_c, rh_pct):
    """Water-vapour pressure e = RH/100 · es(T) in hPa (Magnus)."""
    rh = np.clip(np.asarray(rh_pct, float), 1e-3, 100.0)
    return rh / 100.0 * 6.112 * np.exp(_MAG_A * t_c / (_MAG_B + t_c))


def dew_point(t_c, rh_pct):
    """Magnus dew point (°C): γ = ln(RH/100) + aT/(b+T), Td = bγ/(a−γ)."""
    rh = np.clip(np.asarray(rh_pct, float), 1e-3, 100.0)
    gam = np.log(rh / 100.0) + _MAG_A * t_c / (_MAG_B + t_c)
    return _MAG_B * gam / (_MAG_A - gam)


def abs_humidity(t_c, rh_pct):
    """Absolute humidity AH = 216.7·es/(T+273.15) in g/m³."""
    return 216.7 * vapour_pressure(t_c, rh_pct) / (np.asarray(t_c, float) + 273.15)


def lapse_rate(d: CanSatData, bin_m: float = 50.0, min_n: int = 3):
    """Γ = −ΔT/Δz between consecutive 50 m-bin mean temperatures (°C/km).

    Descent-only by default — the samples between the raw altitude peak
    and the raw minimum.  The pad and ground-rest clusters pile hundreds
    of samples into the lowest bins and would bias their means.
    Returns (per-bin DataFrame, Γ from the first to the last bin)."""
    alt = d.df["alt_m"].to_numpy(float)
    t = d.df["temp_c"].to_numpy(float)
    ok = np.isfinite(alt) & np.isfinite(t)
    alt, t = alt[ok], t[ok]
    i_ap = int(np.argmax(alt))
    i_min = i_ap + int(np.argmin(alt[i_ap:]))
    alt, t = alt[i_ap:i_min], t[i_ap:i_min]
    edges = np.arange(np.floor(alt.min() / bin_m) * bin_m, alt.max() + bin_m, bin_m)
    which = np.digitize(alt, edges) - 1
    rows = []
    for k in range(len(edges) - 1):
        m = which == k
        if m.sum() >= min_n:
            rows.append({"bin_m": 0.5 * (edges[k] + edges[k + 1]),
                         "n": int(m.sum()),
                         "temp_mean_c": float(t[m].mean())})
    table = pd.DataFrame(rows)
    gamma = float("nan")
    if len(table) >= 2:
        dt = table["temp_mean_c"].diff(-1)
        dz = table["bin_m"].diff(-1)
        table["gamma_c_per_km"] = -dt / dz * 1000.0
        dz_all = table["bin_m"].iloc[-1] - table["bin_m"].iloc[0]
        if dz_all > 0:
            gamma = (-(table["temp_mean_c"].iloc[-1]
                       - table["temp_mean_c"].iloc[0]) / dz_all) * 1000.0
    return table, gamma


def estimate(d: CanSatData) -> pd.DataFrame:
    """Per-sample derived columns: rho_kgm3, and (if RH is logged)
    t_dew_c + abs_hum_gm3 on the SHT45 pair (BMP280 T as fallback)."""
    use_sht = ("sht_temp_c" in d.df.columns
               and d.df["sht_temp_c"].notna().mean() > 0.5)
    t_hum = d.df["sht_temp_c" if use_sht else "temp_c"].to_numpy(float)
    out = pd.DataFrame({"t": d.t})
    out["rho_kgm3"] = air_density(d.df["pressure_hpa"].to_numpy(float),
                                  d.df["temp_c"].to_numpy(float))
    if "rh_pct" in d.df.columns:
        rh = d.df["rh_pct"].to_numpy(float)
        out["t_dew_c"] = dew_point(t_hum, rh)
        out["abs_hum_gm3"] = abs_humidity(t_hum, rh)
    return out


def _line(d: CanSatData, est: pd.DataFrame, label: str, i: int) -> str:
    """One summary line at sample index i (5-sample mean around it)."""
    sl = slice(max(0, i - 2), i + 3)
    line = (f"  {label:<8} t={d.t[i]:6.1f} s"
            f"  T={d.df['temp_c'].to_numpy(float)[sl].mean():6.1f} °C"
            f"  P={d.df['pressure_hpa'].to_numpy(float)[sl].mean():7.1f} hPa"
            f"  rho={est['rho_kgm3'].iloc[sl].mean():6.3f} kg/m³")
    if "t_dew_c" in est:
        line += (f"  Td={est['t_dew_c'].iloc[sl].mean():5.1f} °C"
                 f"  AH={est['abs_hum_gm3'].iloc[sl].mean():5.2f} g/m³")
    return line


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log")
    ap.add_argument("--bin", type=float, default=50.0, help="lapse bin, m")
    ap.add_argument("--csv", metavar="DIR")
    a = ap.parse_args(argv)

    d = CanSatData(a.log)
    est = estimate(d)
    i_ap = int(np.nanargmax(d.df["alt_m"].to_numpy(float)))
    print(_line(d, est, "surface", 0))
    print(_line(d, est, "apogee", i_ap))
    print(_line(d, est, "landing", len(d.t) - 1))
    table, gamma = lapse_rate(d, a.bin)
    print(f"  lapse rate (first→last bin): {gamma:.1f} °C/km")
    print(table.round(1).to_string(index=False))
    if a.csv:
        out = Path(a.csv)
        out.mkdir(parents=True, exist_ok=True)
        f = out / f"{Path(a.log).stem}_atmosphere.csv"
        est.to_csv(f, index=False, float_format="%.4f")
        print(f"  wrote {f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
