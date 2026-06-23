# Parameter Sweep Summary

This sweep compares three short live runs of the RGB-D mapping plus occupancy-grid pipeline.

The goal is to show the speed/detail tradeoff between map density, accumulation rate, and occupancy-grid resolution.

## Sweep cases

| Case          | Map voxel size | Accumulate every N frames | Occupancy resolution | Expected update rate | Final map points | Occupied cells | PCD size |
| ------------- | -------------: | ------------------------: | -------------------: | -------------------: | ---------------: | -------------: | -------: |
| baseline      |         0.05 m |                         3 |          0.05 m/cell |            ~1.667 Hz |           14,741 |          4,310 |   232 KB |
| dense_map     |         0.03 m |                         3 |          0.05 m/cell |            ~1.667 Hz |           43,233 |          4,219 |   676 KB |
| faster_sparse |         0.08 m |                         1 |          0.10 m/cell |            ~5.000 Hz |            6,159 |          1,278 |   100 KB |

## Interpretation

* `baseline` matches the normal Version 1.2-style configuration.
* `dense_map` uses a smaller map voxel size, preserving more geometric detail and increasing final map size.
* `faster_sparse` accumulates more frequently but uses coarser map and occupancy-grid resolution, giving a faster expected update rate with fewer retained points and fewer occupied cells.
* This demonstrates the practical robotics tradeoff between map detail, update rate, and memory/storage footprint.

## Generated artifacts

* `outputs/metrics/parameter_sweep.csv`
* `outputs/metrics/parameter_sweep_summary.md`
* local raw logs/maps under `outputs/sweeps/` ignored by git

## Run command

```bash
DURATION_SECONDS=35 bash scripts/run_parameter_sweep.sh
```

## Notes

* The sweep is intentionally short and practical for local verification.
* `expected_update_hz` is computed from replay FPS, accumulation interval, and publish interval.
* `final_map_points` is parsed from the generated PCD header.
* `last_occupied_cells` is parsed from the occupancy-grid node logs.
* Raw per-case logs and generated maps are local artifacts and are not committed.
