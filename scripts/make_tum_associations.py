#!/usr/bin/env python3
"""
Create RGB-depth association files for the TUM RGB-D dataset.

Input:
  rgb.txt
  depth.txt

Output:
  associations.txt

Each output line:
  rgb_timestamp rgb_path depth_timestamp depth_path

This script intentionally stays simple and readable because it is part of the
portfolio project and should be easy to explain in an interview.
"""

from __future__ import annotations

import argparse
from pathlib import Path


def read_tum_list(path: Path) -> list[tuple[float, str]]:
    """Read a TUM timestamp file such as rgb.txt or depth.txt."""
    entries: list[tuple[float, str]] = []

    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) < 2:
                continue

            timestamp = float(parts[0])
            relative_path = parts[1]
            entries.append((timestamp, relative_path))

    return entries


def associate(
    rgb_entries: list[tuple[float, str]],
    depth_entries: list[tuple[float, str]],
    max_time_diff: float,
) -> list[tuple[float, str, float, str]]:
    """
    Pair each RGB frame with the closest depth frame.

    This is a greedy nearest-neighbor timestamp matcher. For this project, that is
    enough because the dataset streams are already close in time.
    """
    matches: list[tuple[float, str, float, str]] = []

    depth_index = 0
    num_depth = len(depth_entries)

    for rgb_time, rgb_path in rgb_entries:
        # Move forward until the next depth timestamp would be farther forward.
        while (
            depth_index + 1 < num_depth
            and abs(depth_entries[depth_index + 1][0] - rgb_time)
            < abs(depth_entries[depth_index][0] - rgb_time)
        ):
            depth_index += 1

        depth_time, depth_path = depth_entries[depth_index]
        time_diff = abs(depth_time - rgb_time)

        if time_diff <= max_time_diff:
            matches.append((rgb_time, rgb_path, depth_time, depth_path))

    return matches


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset-dir",
        required=True,
        type=Path,
        help="Path to extracted TUM dataset folder.",
    )
    parser.add_argument(
        "--max-time-diff",
        type=float,
        default=0.02,
        help="Maximum allowed RGB-depth timestamp difference in seconds.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output association file. Defaults to dataset_dir/associations.txt.",
    )
    args = parser.parse_args()

    dataset_dir: Path = args.dataset_dir
    rgb_file = dataset_dir / "rgb.txt"
    depth_file = dataset_dir / "depth.txt"

    if not rgb_file.exists():
        raise FileNotFoundError(f"Missing RGB index file: {rgb_file}")

    if not depth_file.exists():
        raise FileNotFoundError(f"Missing depth index file: {depth_file}")

    output_file = args.output or dataset_dir / "associations.txt"

    rgb_entries = read_tum_list(rgb_file)
    depth_entries = read_tum_list(depth_file)
    matches = associate(rgb_entries, depth_entries, args.max_time_diff)

    with output_file.open("w", encoding="utf-8") as f:
        f.write("# rgb_timestamp rgb_path depth_timestamp depth_path\n")
        for rgb_time, rgb_path, depth_time, depth_path in matches:
            f.write(f"{rgb_time:.6f} {rgb_path} {depth_time:.6f} {depth_path}\n")

    print(f"RGB entries:        {len(rgb_entries)}")
    print(f"Depth entries:      {len(depth_entries)}")
    print(f"Associations made:  {len(matches)}")
    print(f"Output file:        {output_file}")

    if matches:
        first = matches[0]
        print()
        print("First association:")
        print(f"  RGB:   {first[0]:.6f} {first[1]}")
        print(f"  Depth: {first[2]:.6f} {first[3]}")


if __name__ == "__main__":
    main()
