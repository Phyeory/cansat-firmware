"""Ozone estimator — O3-B4 electrode mV → ppb (DATA-PIPELINE.md §3 ⚠).

The firmware logs the raw differential electrode signal in `o3` (mV)
until the bench calibration is done.  With a calibration this converts
to ppb — the doc's C_ppb = (I_work − I_aux)/S, here (mv − zero)/sens —
and median-despikes it; without one it passes mV through and says so.
Save the bench numbers once with --save-cal and reuse the JSON forever.
"""
from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import pandas as pd

from cansat_data import CanSatData


@dataclass
class O3Cal:
    """Bench calibration: sensitivity (mV/ppb) and zero offset (mV)."""
    sens_mv_per_ppb: float
    zero_mv: float
    name: str = "O3-B4"

    def save(self, path):
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n")

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text()))


def estimate(d: CanSatData, cal: O3Cal | None = None,
             despike: int = 11) -> pd.DataFrame:
    """Frame with t, o3_mv, o3_ppb (NaN without a cal) and, when
    calibrated, o3_ppb_med — the ~1 s median despiked copy."""
    mv = d.df["o3"].to_numpy(float)
    out = pd.DataFrame({"t": d.t, "o3_mv": mv})
    if cal is not None:
        ppb = (mv - cal.zero_mv) / cal.sens_mv_per_ppb
        out["o3_ppb"] = ppb
        out["o3_ppb_med"] = (pd.Series(ppb)
                             .rolling(despike, center=True, min_periods=1)
                             .median().to_numpy())
    else:
        out["o3_ppb"] = np.nan
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log")
    ap.add_argument("--cal", help="calibration JSON (from --save-cal)")
    ap.add_argument("--sens", type=float, help="sensitivity, mV/ppb")
    ap.add_argument("--zero", type=float, default=0.0, help="zero offset, mV")
    ap.add_argument("--save-cal", help="write --sens/--zero as JSON here")
    ap.add_argument("--csv", metavar="DIR")
    a = ap.parse_args(argv)

    cal = O3Cal.load(a.cal) if a.cal else None
    if cal is None and a.sens is not None:
        cal = O3Cal(a.sens, a.zero)
        if a.save_cal:
            cal.save(a.save_cal)
            print(f"  saved calibration to {a.save_cal}")
    d = CanSatData(a.log)
    est = estimate(d, cal)
    if cal is None:
        print("  NOT calibrated — pass --sens/--zero (mV/ppb, mV) or --cal; "
              "showing raw electrode mV")
        mv = est["o3_mv"]
        print(f"  o3_mv: min {np.nanmin(mv):.1f}, mean {np.nanmean(mv):.1f}, "
              f"max {np.nanmax(mv):.1f}")
    else:
        ppb = est["o3_ppb"]
        print(f"  cal: {cal.name} sens {cal.sens_mv_per_ppb} mV/ppb, "
              f"zero {cal.zero_mv} mV")
        print(f"  o3_ppb: min {np.nanmin(ppb):.1f}, mean {np.nanmean(ppb):.1f}, "
              f"max {np.nanmax(ppb):.1f}")
    if a.csv:
        out = Path(a.csv)
        out.mkdir(parents=True, exist_ok=True)
        f = out / f"{Path(a.log).stem}_ozone.csv"
        est.to_csv(f, index=False, float_format="%.3f")
        print(f"  wrote {f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
