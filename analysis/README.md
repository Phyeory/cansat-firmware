# analysis/ — post-flight data pipeline

The Python side of [`docs/DATA-PIPELINE.md`](../docs/DATA-PIPELINE.md): the
purple ④ DERIVED box.  Every estimator takes a `CanSatData` (SD log,
ground CSV or JSONL — see `cansat_data.py`) and returns a pandas
DataFrame, and every one is also a CLI.  numpy/pandas only, no scipy.

**Formula plug-ins only** — straight from the §3 table.  The quantities
that need a filter or fusion (vertical-velocity re-filter, orientation,
3D trajectory, wind proxy, deploy detection) are the online/estimation
work and deliberately live elsewhere (DATA-PIPELINE.md §6).

| Script | Pipeline nodes | What it computes |
|---|---|---|
| `cansat_data.py` | — | loader: SD CSV / ground CSV / JSONL → canonical columns; `get_vel` = Δh/Δt central difference + median filter |
| `atmosphere_estimator.py` | DEN, LAP, HUM | air density, lapse rate, dew point, absolute humidity |
| `profile_estimator.py` | BIN, RADC, ERR | 50 m-binned science profiles; radiation rate ± sem; GNSS−baro Δh |
| `ozone_estimator.py` | §3 ⚠ | O3-B4 electrode mV → ppb (needs the bench cal JSON) |
| `test_cansat_data.py`, `test_estimators.py` | — | self-tests against a synthetic flight with known truth |

Run the tests (direct runner or pytest):

    python3 test_estimators.py
    pytest

Typical use:

    python3 atmosphere_estimator.py SD_LOG.csv --csv out/
    python3 profile_estimator.py SD_LOG.csv --phase descent --dh
    python3 ozone_estimator.py SD_LOG.csv --sens 0.85 --zero 2.0 --save-cal o3cal.json
