# analysis/ — post-flight data pipeline

The Python side of [`docs/DATA-PIPELINE.md`](../docs/DATA-PIPELINE.md): the
purple ④ DERIVED box plus the §6 offline filter stack.  Every estimator takes
a `CanSatData` (SD log, ground CSV or JSONL — see `cansat_data.py`) and
returns a pandas DataFrame, and every one is also a CLI.  numpy/pandas only,
no scipy.

**System architecture** (input → output diagram, data products, design
decisions): [`ARCHITECTURE.md`](ARCHITECTURE.md).

**Two layers:**

1. **Formula plug-ins** — means and differences straight from the §3 table.
2. **Offline Kalman stack** (2026-11-22) — the §6.2 ☆ filters, validated
   against a synthetic noisy flight in `test_kalman.py` (h RMSE 0.29 m,
   v RMSE 0.08 m/s, tilt < 2°, bias recovered).

| Script | Pipeline nodes | What it computes |
|---|---|---|
| `cansat_data.py` | — | loader: SD CSV / ground CSV / JSONL → canonical columns; `get_vel` = Δh/Δt central difference + median filter |
| `kalman.py` | §6 ★/☆ | offline KF stack: linear KF `[h,v,b]` + RTS smoother (altitude/velocity), MEKF (attitude), error-state EKF + RTS (trajectory), flight events — each importable *and* a CLI subcommand |
| `atmosphere_estimator.py` | DEN, LAP, HUM | air density; lapse rate = ΔT/Δz between 50 m-bin means (descent-only by default); dew point; absolute humidity |
| `profile_estimator.py` | BIN, RADC, ERR | 50 m-binned science-channel means; radiation count rate; GNSS−baro Δh |
| `ozone_estimator.py` | §3 ⚠ | O3-B4 electrode mV → ppb (needs the bench cal JSON) |
| `derived_estimator.py` | §3 P6, WIND | oscillation FFT (dominant freq + amplitude) and horizontal drift per 50 m bin |
| `plot_flight.py` | ⑤ PLOTS | every channel vs the KF-smoothed altitude (ascent solid / descent dashed) + the two compulsory plots + overview sheet |
| `nn_dataset.py` | ④ → NN | uniform 10 Hz grid, z-score, sliding windows, chronological split → `.npz` + flat CSV + metadata JSON |
| `test_kalman.py` | — | pytest suite: filter accuracy vs a synthetic noisy flight with known truth |

Typical use:

    python3 kalman.py altitude SD_LOG.csv --csv out/     # ★ h, v (+ attitude first)
    python3 kalman.py attitude SD_LOG.csv --csv out/
    python3 kalman.py trajectory SD_LOG.csv --csv out/
    python3 kalman.py events SD_LOG.csv
    python3 plot_flight.py SD_LOG.csv --out plots/
    python3 nn_dataset.py SD_LOG.csv --out nn/
    python3 atmosphere_estimator.py SD_LOG.csv --csv out/
    python3 profile_estimator.py SD_LOG.csv --phase descent --dh
    python3 ozone_estimator.py SD_LOG.csv --sens 0.85 --zero 2.0 --save-cal o3cal.json
