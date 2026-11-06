#!/usr/bin/env python3
"""Self-test for cansat_data.py against a synthetic flight with known truth.

Truth: 300 s log at 10 Hz, pad 0 m, liftoff t=10 s at 12 m/s, apogee 800 m
at t=76.67 s, descent 8 m/s, touchdown t=176.67 s. Ground link drops packets
with seq 50..54. O3 sensor truth: sens 0.85 mV/ppb, zero 2.0 mV.

Run:  python3 test_cansat_data.py          (pytest also works)
"""
from __future__ import annotations

import csv
import json
import sys
import tempfile
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from cansat_data import CanSatData  # noqa: E402

RNG = np.random.default_rng(42)
P0 = 1013.25
O3_SENS, O3_ZERO = 0.85, 2.0          # the "real" bench calibration
GROUND_LOST_SEQ = {50, 51, 52, 53, 54}


def _pressure(alt_m):
    """Inverse of the firmware barometric formula."""
    return P0 * np.power(1.0 - alt_m / 44330.0, 1.0 / 0.190294957)


def build_synthetic(tmp: Path):
    """Write SD_LOG.csv, ground.csv (station.py format) and packets.jsonl."""
    t = np.arange(0.0, 300.0 + 0.05, 0.1)
    ascent = np.clip(12.0 * (t - 10.0), 0.0, 800.0)
    descent = np.clip(800.0 - 8.0 * (t - 76.667), 0.0, 800.0)
    alt = np.where(t < 76.667, ascent, descent)
    press = _pressure(alt) + RNG.normal(0, 0.1, t.size)
    temp = 15.0 - 6.5 * alt / 1000.0 + RNG.normal(0, 0.1, t.size)
    o3_mv = O3_ZERO + O3_SENS * (20.0 + 30.0 * alt / 800.0) + RNG.normal(0, 0.3, t.size)
    co2_base = 430.0 + RNG.normal(0, 4.0, t.size)
    co2 = co2_base[np.searchsorted(t, np.floor(t / 2) * 2)]      # SCD: new every 2 s
    uvi = 0.3 + 3.2 * alt / 800.0 + RNG.normal(0, 0.05, t.size)
    vbat = 4.1 - 0.4 * t / 300.0
    gps_fix = t > 25.0
    # bits 0..10; RAD (bit 10) stays off: no pulses on the bench
    health = (1 | 2 | 4 | 8 | 64 | 128 | 256 | 512) \
        | np.where(t > 5, 16, 0) | np.where(gps_fix, 32, 0)
    acc_z = np.where(t < 10.0, 9.81, np.where(t < 76.667, 12.0 + 9.81,
                                             9.81 - 8.0)) \
        + RNG.normal(0, 0.2, t.size)
    rh = np.clip(60.0 - 30.0 * alt / 800.0 + RNG.normal(0, 1.0, t.size), 0, 100)
    rad = np.round(2.0 + 10.0 * alt / 800.0 + RNG.normal(0, 0.5, t.size))

    # --- SD log (firmware telemetry_csv_header order) ---
    sd = tmp / "SD_LOG.csv"
    sd.write_text(
        "t_ms,temp_c,pressure_hpa,alt_m,uv_raw,uv_index,ir_counts,full_counts,"
        "lux,co2_ppm,o3_ppb,lat,lon,gps_alt_m,sats,vbat_v,"
        "acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,mag_x,mag_y,mag_z,"
        "rh_pct,sht_temp_c,rad_cps,health\n"
        + "\n".join(
            f"{int(round(t_i*1000))},{T:.2f},{P:.2f},{h:.2f},{u*1000:.1f},{u:.2f},"
            f"{ir:.1f},{full:.1f},{lux:.1f},{c:.0f},{o:.1f},"
            f"{50.85:.5f},{-0.35:.5f},{h_i+8.0:.1f},{sats},{v:.2f},"
            f"{0.3:.2f},{-0.1:.2f},{az:.2f},{gx:.1f},{gy:.1f},{gz:.1f},"
            f"{48.0:.1f},{2.0:.1f},{-41.0:.1f},{r:.1f},{T+0.3:.2f},{rd:.0f},{hh}"
            for t_i, T, P, h, u, ir, full, lux, c, o, h_i, sats, v,
                az, gx, gy, gz, r, rd, hh in zip(
                t, temp, press, alt,
                uvi, 500 + 2000 * alt / 800 + RNG.normal(0, 5, t.size),
                3000 + 9000 * alt / 800, 300 + 9000 * alt / 800,
                co2, o3_mv, alt, np.where(gps_fix, 7, 0), vbat,
                acc_z,
                RNG.normal(0, 2.0, t.size), RNG.normal(0, 2.0, t.size),
                RNG.normal(0, 2.0, t.size),
                rh, rad, health))
        + "\n")

    # --- ground packets (1 Hz, station.py JSON keys) ---
    pkts = []
    for i in range(0, len(t), 10):                    # every 10th sample = 1 Hz
        seq = i // 10 + 1
        if seq in GROUND_LOST_SEQ:                    # the link dropped these
            continue
        pkts.append({
            "n": seq, "t": int(round(t[i] * 1000)),
            "T": round(float(temp[i]), 1), "P": round(float(press[i]), 1),
            "A": round(float(alt[i]), 1), "uv": round(float(uvi[i] * 1000), 0),
            "co2": round(float(co2[i]), 0), "o3": round(float(o3_mv[i]), 1),
            "la": 50.85, "lo": -0.35, "ga": round(float(alt[i] + 8), 0),
            "s": int(np.where(gps_fix[i], 7, 0)),
            "ax": 0.3, "ay": -0.1, "az": round(float(acc_z[i]), 1),
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
            "rh": round(float(rh[i]), 1), "rad": int(rad[i]),
            "v": round(float(vbat[i]), 2), "h": int(health[i]),
        })
    ground = tmp / "ground.csv"
    fields = ["rx_time", "n", "t", "T", "P", "A", "uv",
              "co2", "o3", "la", "lo", "ga", "s",
              "ax", "ay", "az", "gx", "gy", "gz", "rh", "rad",
              "v", "h", "rssi", "snr"]
    with ground.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(fields)
        base = datetime(2026, 6, 14, 10, 0, 0)
        for p in pkts:
            rx = (base + timedelta(milliseconds=p["t"])).isoformat(timespec="milliseconds")
            w.writerow([rx] + [p[f] for f in fields[1:-2]]
                       + [-95 + RNG.integers(-5, 6), 8.5 + RNG.integers(-2, 3)])
    jsonl = tmp / "packets.jsonl"
    jsonl.write_text("\n".join(json.dumps(p) for p in pkts) + "\n")
    return sd, ground, jsonl


# ---- tests -----------------------------------------------------------------

def test_load_autodetect(tmp):
    sd, ground, jsonl = build_synthetic(tmp)
    d_sd, d_g, d_j = CanSatData(sd), CanSatData(ground), CanSatData(jsonl)
    assert len(d_sd) == 3001 and len(d_g) == len(d_j) == 296  # 301 sent, 5 lost
    assert d_sd.df["full_counts"].notna().all()      # SD-only columns
    assert "full_counts" not in d_g.df.columns       # the link never carries them
    assert "mag_x" not in d_g.df.columns             # mag stays SD-only too
    assert d_g.df["acc_x"].notna().all()             # accel is downlinked
    assert d_sd.t[0] == 0.0 and abs(d_sd.t[-1] - 300.0) < 0.01
    assert d_g.df["rssi"].notna().all()              # ground extras preserved


def test_getters(tmp):
    sd, *_ = build_synthetic(tmp)
    d = CanSatData(sd)
    t, alt = d.get_alt()                             # whole channel for plotting
    assert t is d.t and len(alt) == len(d)
    # point lookup against the synthetic truth at t=100 (descent phase)
    alt_truth = 800.0 - 8.0 * (100.0 - 76.667)
    assert abs(d.get_alt(100.0) - alt_truth) < 3.0
    assert abs(d.get_temp(100.0) - (15.0 - 6.5 * alt_truth / 1000.0)) < 0.5
    assert abs(d.get_co2(57.0) - 430.0) < 10.0
    assert abs(d.get_vbat(300.0) - 3.70) < 0.05
    assert np.isnan(d.get_alt(-5.0))                 # outside the log
    assert abs(d.get_o3(100.0) - (O3_ZERO + O3_SENS * (20 + 30 * alt_truth / 800))) < 1.0
    assert abs(d.get_acc("z", 100.0) - (9.81 - 8.0)) < 0.5
    assert abs(d.get_rh(100.0) - (60.0 - 30.0 * alt_truth / 800.0)) < 2.0
    assert abs(d.get_rad(100.0) - (2.0 + 10.0 * alt_truth / 800.0)) < 2.0


def test_get_vel(tmp):
    sd, *_ = build_synthetic(tmp)
    d = CanSatData(sd)
    t, v = d.get_vel()                               # whole channel
    assert len(v) == len(d)
    assert abs(d.get_vel(120.0) - (-8.0)) < 1.0      # mid-descent
    assert abs(d.get_vel(45.0) - 12.0) < 1.0         # mid-ascent
    assert abs(d.get_vel(250.0)) < 1.5               # landed


def test_gps_and_health(tmp):
    sd, ground, _ = build_synthetic(tmp)
    d = CanSatData(sd)
    assert d.get_gps(10.0) is None                   # no fix before t=25
    assert d.get_gps(26.0) is not None               # fresh fix after lock
    lat, lon, galt = d.get_gps(200.0)
    assert abs(lat - 50.85) < 1e-4 and abs(lon + 0.35) < 1e-4
    assert d.get_health(0.0) == {"bmp": True, "ltr": True, "tsl": True,
                                 "scd": True, "gpsdata": False,
                                 "gpsfix": False, "sd": True,
                                 "bmi": True, "mlx": True, "sht": True,
                                 "rad": False}
    assert d.get_health(200.0)["gpsfix"] is True
    g = CanSatData(ground)
    assert g.get_gps(200.0) is not None              # same API on link data


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        with tempfile.TemporaryDirectory() as d:
            fn(Path(d))
        print(f"  PASS {fn.__name__}")
    print(f"{len(tests)}/{len(tests)} tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
