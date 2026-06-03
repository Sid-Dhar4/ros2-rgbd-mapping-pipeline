#!/usr/bin/env python3
"""
Create RGB-D-pose association files for the TUM RGB-D dataset.

Input:
  associations.txt
    rgb_timestamp rgb_path depth_timestamp depth_path

  groundtruth.txt
    timestamp tx ty tz qx qy qz qw

Output:
  rgbd_pose_associations.txt
    rgb_timestamp rgb_path depth_timestamp depth_path gt_timestamp tx ty tz qx qy qz qw

This file is used by the ROS publisher so every replayed RGB-D frame can also
publish the nearest ground-truth camera pose.
"""

from __future__ import annotations

import argparse
from pathlib import Path


RgbdAssociation = tuple[float, str, float, str]
Pose = tuple[float, float, float, float, float, float, float, float]


def read_rgbd_associations(path: Path) -> list[RgbdAssociation]:
    rows: list[RgbdAssociation] = []

    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) != 4:
                continue

            rows.append(
                (
                    float(parts[0]),
                    parts[1],
                    float(parts[2]),
                    parts[3],
                )
            )

    return rows


def read_groundtruth(path: Path) -> list[Pose]:
    poses: list[Pose] = []

    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) != 8:
                continue

            poses.append(tuple(float(x) for x in parts))  # type: ignore[arg-type]

    return poses


def associate_poses(
    rgbd_rows: list[RgbdAssociation],
    poses: list[Pose],
    max_time_diff: float,
) -> list[tuple[RgbdAssociation, Pose]]:
    matches: list[tuple[RgbdAssociation, Pose]] = []

    pose_index = 0
    num_poses = len(poses)

    for row in rgbd_rows:
        rgb_time = row[0]

        while (
            pose_index + 1 < num_poses
            and abs(poses[pose_index + 1][0] - rgb_time)
            < abs(poses[pose_index][0] - rgb_time)
        ):
            pose_index += 1

        pose = poses[pose_index]
        pose_time = pose[0]
        time_diff = abs(pose_time - rgb_time)

        if time_diff <= max_time_diff:
            matches.append((row, pose))

    return matches


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset-dir",
        required=True,
        type=Path,
        help="Path to extracted TUM RGB-D dataset directory.",
    )
    parser.add_argument(
        "--max-time-diff",
        type=float,
        default=0.05,
        help="Maximum allowed RGB-to-groundtruth timestamp difference in seconds.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output file. Defaults to dataset_dir/rgbd_pose_associations.txt.",
    )
    args = parser.parse_args()

    dataset_dir: Path = args.dataset_dir

    rgbd_path = dataset_dir / "associations.txt"
    gt_path = dataset_dir / "groundtruth.txt"
    output_path = args.output or dataset_dir / "rgbd_pose_associations.txt"

    if not rgbd_path.exists():
        raise FileNotFoundError(f"Missing RGB-D associations file: {rgbd_path}")

    if not gt_path.exists():
        raise FileNotFoundError(f"Missing groundtruth file: {gt_path}")

    rgbd_rows = read_rgbd_associations(rgbd_path)
    poses = read_groundtruth(gt_path)
    matches = associate_poses(rgbd_rows, poses, args.max_time_diff)

    with output_path.open("w", encoding="utf-8") as f:
        f.write("# rgb_timestamp rgb_path depth_timestamp depth_path ")
        f.write("gt_timestamp tx ty tz qx qy qz qw\n")

        for row, pose in matches:
            rgb_time, rgb_path, depth_time, depth_path = row
            gt_time, tx, ty, tz, qx, qy, qz, qw = pose

            f.write(
                f"{rgb_time:.6f} {rgb_path} "
                f"{depth_time:.6f} {depth_path} "
                f"{gt_time:.6f} "
                f"{tx:.9f} {ty:.9f} {tz:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )

    print(f"RGB-D associations: {len(rgbd_rows)}")
    print(f"Ground-truth poses: {len(poses)}")
    print(f"RGB-D-pose matches: {len(matches)}")
    print(f"Output file:        {output_path}")

    if matches:
        row, pose = matches[0]
        print()
        print("First RGB-D-pose association:")
        print(f"  RGB:   {row[0]:.6f} {row[1]}")
        print(f"  Depth: {row[2]:.6f} {row[3]}")
        print(
            "  Pose:  "
            f"{pose[0]:.6f} "
            f"t=({pose[1]:.3f}, {pose[2]:.3f}, {pose[3]:.3f}) "
            f"q=({pose[4]:.3f}, {pose[5]:.3f}, {pose[6]:.3f}, {pose[7]:.3f})"
        )


if __name__ == "__main__":
    main()
