from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd

# telemetry JSON keys (and station.py CSV columns) -> canonical names,
PACKET_MAP = {
    "n": "seq", "t": "t_ms", "T": "temp_c", "P": "pressure_hpa", "A": "alt_m",
    "uv": "uv_raw", "uvi": "uv_index", "ir": "ir_counts", "lux": "lux",
    "co2": "co2_ppm", "o3": "o3", "la": "lat", "lo": "lon", "ga": "gps_alt_m",
    "s": "sats", "v": "vbat_v", "h": "health",
    "ax": "acc_x", "ay": "acc_y", "az": "acc_z",
    "gx": "gyr_x", "gy": "gyr_y", "gz": "gyr_z",
    "rh": "rh_pct", "rad": "rad_cps",
}
# onboard SD CSV columns -> canonical (same schema at 10 Hz, plus full_counts)
SD_MAP = {
    "t_ms": "t_ms", "temp_c": "temp_c", "pressure_hpa": "pressure_hpa",
    "alt_m": "alt_m", "uv_raw": "uv_raw", "uv_index": "uv_index",
    "ir_counts": "ir_counts", "full_counts": "full_counts", "lux": "lux",
    "co2_ppm": "co2_ppm", "o3_ppb": "o3", "lat": "lat", "lon": "lon",
    "gps_alt_m": "gps_alt_m", "sats": "sats", "vbat_v": "vbat_v",
    "health": "health",
    "acc_x": "acc_x", "acc_y": "acc_y", "acc_z": "acc_z",
    "gyr_x": "gyr_x", "gyr_y": "gyr_y", "gyr_z": "gyr_z",
    "mag_x": "mag_x", "mag_y": "mag_y", "mag_z": "mag_z",
    "rh_pct": "rh_pct", "sht_temp_c": "sht_temp_c", "rad_cps": "rad_cps",
}
# health bitmask, bit order from telemetry.h
HEALTH_BITS = ("BMP", "LTR", "TSL", "SCD", "GPSdata", "GPSfix", "SD",
               "BMI", "MLX", "SHT", "RAD")


class CanSatData:
    """One flight's log, with per-channel accessors.

    `d.t` is seconds since the first record; every getter takes an optional
    time: omit it to get the whole channel as (time, values) arrays for
    plotting, or pass a time to get the interpolated value at that instant.
    """

    def __init__(self, path):
        self.df = self._load(Path(path))
        self.t = (self.df["t_ms"] - self.df["t_ms"].iloc[0]).to_numpy() / 1000.0

    def __len__(self):
        return len(self.df)

    def __repr__(self):
        return f"CanSatData({len(self)} records, {self.t[-1]:.0f} s)"

    # ---- loading ----------------------------------------------------------

    @staticmethod
    def _load(path: Path) -> pd.DataFrame:
        """Read SD CSV / ground CSV / JSON lines into one canonical table."""
        with path.open("r", errors="replace") as fh:
            head = next((ln.strip() for ln in fh
                         if ln.strip() and not ln.strip().startswith("#")), "")
        if head.startswith("{"):                       # raw JSON packets
            rows = []
            for ln in path.open("r", errors="replace"):
                ln = ln.strip()
                try:
                    pkt = json.loads(ln)
                except ValueError:
                    continue
                if isinstance(pkt, dict) and "t" in pkt:
                    rows.append(pkt)
            df = pd.DataFrame(rows).rename(columns=PACKET_MAP)
        elif "t_ms" in head or "rx_time" in head:      # SD log or station.py CSV
            raw = pd.read_csv(path, dtype=str, comment="#", skip_blank_lines=True)
            raw.columns = [c.strip() for c in raw.columns]
            m = SD_MAP if "t_ms" in head else PACKET_MAP
            df = pd.DataFrame({canon: pd.to_numeric(raw[src], errors="coerce")
                               for src, canon in m.items() if src in raw.columns})
            for meta in ("rx_time", "rssi", "snr"):    # ground-station extras
                if meta in raw.columns:
                    df[meta] = (pd.to_datetime(raw[meta], errors="coerce")
                                if meta == "rx_time"
                                else pd.to_numeric(raw[meta], errors="coerce"))
        else:
            raise ValueError(f"{path.name}: not a CanSat log (header: {head[:60]!r})")
        for c in df.columns:
            if c != "rx_time":
                df[c] = pd.to_numeric(df[c], errors="coerce")
        df = df.dropna(subset=["t_ms"]).sort_values("t_ms").reset_index(drop=True)
        if df.empty:
            raise ValueError(f"{path.name}: no telemetry rows found")
        return df

    # ---- channel accessors ---------------------------------------------------

    def get(self, channel: str, t=None):
        """Any column by canonical name; same t=None / t semantics as below."""
        y = self.df[channel].to_numpy(dtype=float)
        if t is None:
            return self.t, y
        return float(np.interp(t, self.t, y, left=np.nan, right=np.nan))

    def get_temp(self, t=None):        # BMP280, °C
        return self.get("temp_c", t)

    def get_pressure(self, t=None):    # BMP280, hPa
        return self.get("pressure_hpa", t)

    def get_alt(self, t=None):         # barometric altitude, m
        return self.get("alt_m", t)

    def get_vel(self, t=None):         # vertical speed, m/s
        """d(alt)/dt, smoothed over ~1 s — raw single-step gradients of a
        10 Hz barometric series are pure noise."""
        alt = self.df["alt_m"].to_numpy(dtype=float)
        dt = float(np.median(np.diff(self.t))) if len(self.t) > 1 else 0.1
        w = max(5, int(round(1.0 / max(dt, 1e-3))))
        w += w % 2 == 0
        sm = pd.Series(alt).rolling(w, center=True, min_periods=1).median()
        v = pd.Series(np.gradient(sm.to_numpy(), self.t))
        v = v.rolling(w, center=True, min_periods=1).mean().to_numpy()
        if t is None:
            return self.t, v
        return float(np.interp(t, self.t, v, left=np.nan, right=np.nan))

    def get_uv(self, t=None):          # UV index (LTR390)
        return self.get("uv_index", t)

    def get_ir(self, t=None):          # IR counts (TSL2591)
        return self.get("ir_counts", t)

    def get_lux(self, t=None):         # lux (TSL2591)
        return self.get("lux", t)

    def get_co2(self, t=None):         # ppm (SCD30, new value every 2 s)
        return self.get("co2_ppm", t)

    def get_acc(self, axis="z", t=None):    # BMI270, m/s^2
        return self.get(f"acc_{axis}", t)

    def get_gyr(self, axis="z", t=None):    # BMI270, deg/s
        return self.get(f"gyr_{axis}", t)

    def get_mag(self, axis="z", t=None):    # MLX90395, uT
        return self.get(f"mag_{axis}", t)

    def get_rh(self, t=None):          # relative humidity % (SHT45)
        return self.get("rh_pct", t)

    def get_rad(self, t=None):         # BPW34 pulse rate, counts/s
        return self.get("rad_cps", t)

    def get_o3(self, t=None):          # O3-B4; electrode mV until calibrated
        return self.get("o3", t)

    def get_vbat(self, t=None):        # battery, V
        return self.get("vbat_v", t)

    def get_gps(self, t=None, max_age_s: float = 5.0):
        """(lat, lon, alt_m) from the nearest fix within max_age_s of the
        requested time; None before the first fix or if the fix is stale.
        The firmware reports 0 satellites until it has a lock."""
        ok = (self.df["sats"].fillna(0) > 0) & self.df["lat"].notna()
        if not ok.any():
            return None
        idx = ok.to_numpy().nonzero()[0]
        t_want = self.t[-1] if t is None else t
        i = idx[int(np.argmin(np.abs(self.t[idx] - t_want)))]
        if t is not None and abs(self.t[i] - t_want) > max_age_s:
            return None
        return (float(self.df["lat"].iloc[i]), float(self.df["lon"].iloc[i]),
                float(self.df["gps_alt_m"].iloc[i]))

    def get_health(self, t=None) -> dict:
        """Sensor flags from the 'h' bitmask at the nearest sample."""
        i = int(np.argmin(np.abs(self.t - (self.t[-1] if t is None else t))))
        h = int(self.df["health"].fillna(0).iloc[i])
        return {b.lower(): bool(h & (1 << j)) for j, b in enumerate(HEALTH_BITS)}
