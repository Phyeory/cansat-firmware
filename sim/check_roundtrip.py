#!/usr/bin/env python3
"""Round-trip check: simulator output -> analysis/cansat_data.py -> assertions.

The simulator writes the two artifacts a real flight produces: the SD card CSV
(10 Hz, every sensor) and the E22 JSON lines (1 Hz, the radio subset).  This
script loads both through the same parser the ground station uses and checks
that the flight profile survived the whole chain - firmware, device models,
telemetry formatting, file sinks and the analysis library.

    python3 sim/check_roundtrip.py [--sd PATH] [--lora PATH]

Exits 0 when every check passes, 1 otherwise.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis"))

from cansat_data import CanSatData  # noqa: E402

TARGET_APOGEE_M = 750.0
APOGEE_TOL = 0.05          # plan: +-5 %
DESCENT_MPS = 10.0
DESCENT_TOL = 1.0
REQUIRED_BITS = ("baro", "ltr", "tsl", "scd", "gpsdata", "gpsfix",
                 "sd", "mlx90395", "sht45", "rad")

results: list[tuple[bool, str, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    results.append((bool(ok), name, detail))


def monotonic(d: CanSatData) -> tuple[bool, str]:
    dt = np.diff(d.t)
    return bool(dt.size and np.all(dt > 0)), f"min dt={dt.min() * 1000:.1f} ms, n={dt.size}"


def flight_times(d: CanSatData):
    """(t_apogee, t_land) from the barometric series, seconds since first row."""
    alt = d.df["alt_m"].to_numpy(dtype=float)
    i_apogee = int(np.argmax(alt))
    after = np.nonzero(alt[i_apogee:] < 5.0)[0]
    i_land = int(i_apogee + after[0]) if after.size else None
    return d.t[i_apogee], (None if i_land is None else d.t[i_land])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sd", default=str(ROOT / "sim" / "out" / "sd.csv"))
    ap.add_argument("--lora", default=str(ROOT / "sim" / "out" / "lora.jsonl"))
    args = ap.parse_args()

    sd_path, lora_path = Path(args.sd), Path(args.lora)
    for p in (sd_path, lora_path):
        if not p.exists():
            print(f"missing {p}; run `make -C sim run` first", file=sys.stderr)
            return 2

    sd = CanSatData(sd_path)
    lora = CanSatData(lora_path)

    # ---- both artifacts parse -------------------------------------------
    check("sd.csv parses", len(sd) > 500, f"{len(sd)} rows, {sd.t[-1]:.1f} s")
    check("lora.jsonl parses", len(lora) >= 60,
          f"{len(lora)} frames, {lora.t[-1]:.1f} s")

    # ---- time base -------------------------------------------------------
    ok, detail = monotonic(sd)
    check("sd timestamps strictly monotonic", ok, detail)
    ok, detail = monotonic(lora)
    check("lora timestamps strictly monotonic", ok, detail)

    # ---- flight profile --------------------------------------------------
    t_apogee, t_land = flight_times(sd)
    apogee = float(sd.df["alt_m"].max())
    check(f"apogee {apogee:.1f} m within {APOGEE_TOL:.0%} of {TARGET_APOGEE_M:.0f} m",
          abs(apogee - TARGET_APOGEE_M) <= APOGEE_TOL * TARGET_APOGEE_M,
          f"apogee at t={t_apogee:.2f} s")

    t, vel = sd.get_vel()
    if t_land is not None and t_land - t_apogee > 20.0:
        win = (t > t_apogee + 5.0) & (t < t_land - 5.0)
        rate = float(np.median(vel[win]))
        check(f"descent {abs(rate):.2f} m/s ~= {DESCENT_MPS:.0f} m/s",
              abs(abs(rate) - DESCENT_MPS) <= DESCENT_TOL,
              f"{int(win.sum())} samples in the steady-descent window")
    else:
        check("descent rate measurable", False, "no steady-descent window")

    landed_alt = float(sd.df["alt_m"].iloc[-1])
    check("landing detected", t_land is not None and landed_alt < 5.0,
          f"final altitude {landed_alt:.2f} m at t={sd.t[-1]:.2f} s")
    check("simulator runs on after touchdown",
          t_land is not None and sd.t[-1] > t_land + 1.0,
          "1-2 s of landed data" if t_land is not None else "")

    # ---- the radio link sees the same flight -----------------------------
    lora_apogee = float(lora.df["alt_m"].max())
    check("lora apogee agrees with sd",
          abs(lora_apogee - apogee) <= APOGEE_TOL * TARGET_APOGEE_M,
          f"{lora_apogee:.1f} m vs {apogee:.1f} m")
    check("lora frames are ~1 Hz", 0.7 <= len(lora) / lora.t[-1] <= 1.3,
          f"{len(lora) / lora.t[-1]:.2f} Hz")

    # ---- sensor plausibility at apogee -----------------------------------
    temp_apogee = sd.get_temp(t_apogee)
    isa_temp = 15.0 - 0.0065 * apogee
    check(f"temperature at apogee {temp_apogee:.1f} C follows ISA",
          abs(temp_apogee - isa_temp) < 2.0, f"expected {isa_temp:.1f} C")

    # ---- GPS path (NMEA -> USART ISR -> parser -> telemetry) -------------
    fix = sd.get_gps(t_apogee, max_age_s=5.0)
    check("gps fix available", fix is not None,
          "" if fix is None else f"lat={fix[0]:.5f} lon={fix[1]:.5f} alt={fix[2]:.0f} m")
    if fix is not None:
        check("gps position is the simulated site",
              abs(fix[0] - 48.1173) < 1e-3 and abs(fix[1] - 11.5167) < 1e-3,
              f"{fix[0]:.5f}, {fix[1]:.5f}")
        check("gps altitude tracks the barometer near apogee",
              abs(fix[2] - apogee) < 60.0, f"{fix[2]:.0f} m vs {apogee:.0f} m")

    # ---- health ----------------------------------------------------------
    health = sd.get_health()
    missing = [b for b in REQUIRED_BITS if not health.get(b, False)]
    check("all health bits set on the sd channel", not missing,
          "0x%03X" % int(sd.df["health"].iloc[-1]) if not missing
          else "missing " + ", ".join(missing))

    lora_health = lora.get_health()
    missing = [b for b in REQUIRED_BITS if not lora_health.get(b, False)]
    check("all health bits set on the lora channel", not missing,
          "0x%03X" % int(lora.df["health"].iloc[-1]) if not missing
          else "missing " + ", ".join(missing))

    # ---- report ----------------------------------------------------------
    width = max(len(name) for _, name, _ in results)
    failed = 0
    print(f"round trip: {sd_path.name} ({len(sd)} rows) + {lora_path.name} "
          f"({len(lora)} frames)")
    for ok, name, detail in results:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<{width}}  {detail}")
        failed += not ok
    print(f"\n{len(results) - failed}/{len(results)} checks passed")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
