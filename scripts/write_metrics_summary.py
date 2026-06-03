#!/usr/bin/env python3
"""Write the Version 1 metrics summary for the RGB-D mapping pipeline.

The values here are the measured first-pass results from the TUM
rgbd_dataset_freiburg1_xyz run used for the README and sample outputs.
Keeping this as a script makes the metrics artifact reproducible instead
of only hand-editing CSV/Markdown files.
"""

from __future__ import annotations

import csv
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
METRICS_DIR = REPO_ROOT / "outputs" / "metrics"
METRICS_DIR.mkdir(parents=True, exist_ok=True)

HEADERS = [
    "sequence",
    "rgbd_frames",
    "raw_cloud_rate_hz",
    "filtered_cloud_rate_hz",
    "map_cloud_rate_hz",
    "raw_points_typical",
    "filtered_points_typical",
    "point_reduction_typical",
    "map_points_final",
    "pcd_size_kb",
    "ply_size_kb",
]

ROW = {
    "sequence": "rgbd_dataset_freiburg1_xyz",
    "rgbd_frames": "798",
    "raw_cloud_rate_hz": "10.0",
    "filtered_cloud_rate_hz": "10.0",
    "map_cloud_rate_hz": "1.66",
    "raw_points_typical": "14000-15500",
    "filtered_points_typical": "1500-6000",
    "point_reduction_typical": "60-90%",
    "map_points_final": "17810",
    "pcd_size_kb": "279",
    "ply_size_kb": "262",
}


def write_csv() -> None:
    csv_path = METRICS_DIR / "results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=HEADERS)
        writer.writeheader()
        writer.writerow(ROW)
    print(f"Wrote {csv_path}")


def write_markdown() -> None:
    md_path = METRICS_DIR / "metrics_summary.md"
    summary = """# Metrics Summary

Sequence: `rgbd_dataset_freiburg1_xyz`

| Metric | Observed value |
|---|---:|
| RGB-D replay rate | ~10 Hz |
| Raw cloud rate | ~10 Hz |
| Filtered cloud rate | ~10 Hz |
| Map cloud publish rate | ~1.66 Hz |
| Raw points per frame | ~14k–15.5k |
| Filtered points per frame | ~1.5k–6k |
| Typical point reduction | ~60–90% |
| Final accumulated map points | ~17.8k |
| Final PCD size | ~279 KB |
| Final PLY size | ~262 KB |

Notes:
- The map publish rate is lower than the frame rate by design.
- The C++ map accumulator processes every 3rd filtered cloud and publishes every 2 processed frames.
- The final map uses TUM ground-truth poses for world-frame accumulation.
- These are first-pass Version 1 metrics from one TUM RGB-D sequence.
"""
    md_path.write_text(summary, encoding="utf-8")
    print(f"Wrote {md_path}")


def main() -> None:
    write_csv()
    write_markdown()


if __name__ == "__main__":
    main()
