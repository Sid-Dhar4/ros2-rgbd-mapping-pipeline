"""Utilities for projecting 3D point clouds into 2D occupancy grids.

The ROS node layer should handle subscriptions and message publishing.
This file keeps the geometry/testable logic separate.

Convention:
- Input points are Nx3 XYZ in a fixed world/map frame.
- The output grid is a 2D numpy array with shape (height, width).
- grid[row, col] corresponds to y/x map cells.
- Occupied cells use 100.
- Free cells use 0 by default.
- Unknown cells can use -1 if unknown_as_free=False.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class OccupancyGridConfig:
    """Configuration for point-cloud to occupancy-grid projection."""

    resolution_m: float = 0.05
    min_x_m: float = -3.0
    max_x_m: float = 3.0
    min_y_m: float = -3.0
    max_y_m: float = 3.0
    min_z_m: float = 0.05
    max_z_m: float = 1.50
    occupied_threshold_points: int = 1
    inflate_radius_cells: int = 0
    unknown_as_free: bool = True


def grid_shape(config: OccupancyGridConfig) -> tuple[int, int]:
    """Return (height, width) for the configured metric map bounds."""
    _validate_config(config)
    width = int(math.ceil((config.max_x_m - config.min_x_m) / config.resolution_m))
    height = int(math.ceil((config.max_y_m - config.min_y_m) / config.resolution_m))
    return height, width


def points_to_occupancy_grid(
    points_xyz: np.ndarray,
    config: OccupancyGridConfig,
) -> np.ndarray:
    """Project XYZ points into a 2D occupancy grid.

    Args:
        points_xyz: Nx3 array of points in a fixed world/map frame.
        config: occupancy-grid projection parameters.

    Returns:
        np.int8 grid with shape (height, width), using 100 for occupied,
        0 for free, and optionally -1 for unknown.
    """
    _validate_config(config)
    points = _as_points_array(points_xyz)

    height, width = grid_shape(config)
    fill_value = 0 if config.unknown_as_free else -1
    grid = np.full((height, width), fill_value, dtype=np.int8)

    if points.shape[0] == 0:
        return grid

    finite_mask = np.isfinite(points).all(axis=1)
    points = points[finite_mask]
    if points.shape[0] == 0:
        return grid

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    obstacle_mask = (
        (x >= config.min_x_m)
        & (x < config.max_x_m)
        & (y >= config.min_y_m)
        & (y < config.max_y_m)
        & (z >= config.min_z_m)
        & (z <= config.max_z_m)
    )

    obstacle_points = points[obstacle_mask]
    if obstacle_points.shape[0] == 0:
        return grid

    cols = np.floor((obstacle_points[:, 0] - config.min_x_m) / config.resolution_m).astype(
        np.int64
    )
    rows = np.floor((obstacle_points[:, 1] - config.min_y_m) / config.resolution_m).astype(
        np.int64
    )

    counts = np.zeros((height, width), dtype=np.int32)
    np.add.at(counts, (rows, cols), 1)

    occupied = counts >= config.occupied_threshold_points
    if config.inflate_radius_cells > 0:
        occupied = inflate_occupied_cells(occupied, config.inflate_radius_cells)

    grid[occupied] = 100
    return grid


def flatten_grid_for_ros(grid: np.ndarray) -> list[int]:
    """Flatten a 2D occupancy grid into ROS OccupancyGrid row-major data."""
    if grid.ndim != 2:
        raise ValueError("grid must be a 2D array")
    return grid.astype(np.int8).reshape(-1).astype(int).tolist()


def origin_xy(config: OccupancyGridConfig) -> tuple[float, float]:
    """Return the metric map origin for nav_msgs/OccupancyGrid metadata."""
    _validate_config(config)
    return config.min_x_m, config.min_y_m


def inflate_occupied_cells(occupied: np.ndarray, radius_cells: int) -> np.ndarray:
    """Inflate occupied cells by a square radius in grid-cell units."""
    if occupied.ndim != 2:
        raise ValueError("occupied must be a 2D boolean array")
    if radius_cells < 0:
        raise ValueError("radius_cells must be non-negative")
    if radius_cells == 0:
        return occupied.copy()

    inflated = occupied.copy()
    rows, cols = np.where(occupied)
    height, width = occupied.shape

    for row, col in zip(rows, cols):
        row_min = max(0, row - radius_cells)
        row_max = min(height, row + radius_cells + 1)
        col_min = max(0, col - radius_cells)
        col_max = min(width, col + radius_cells + 1)
        inflated[row_min:row_max, col_min:col_max] = True

    return inflated


def _as_points_array(points_xyz: np.ndarray) -> np.ndarray:
    points = np.asarray(points_xyz, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_xyz must have shape Nx3")
    return points


def _validate_config(config: OccupancyGridConfig) -> None:
    if config.resolution_m <= 0.0:
        raise ValueError("resolution_m must be positive")
    if config.max_x_m <= config.min_x_m:
        raise ValueError("max_x_m must be greater than min_x_m")
    if config.max_y_m <= config.min_y_m:
        raise ValueError("max_y_m must be greater than min_y_m")
    if config.max_z_m < config.min_z_m:
        raise ValueError("max_z_m must be greater than or equal to min_z_m")
    if config.occupied_threshold_points < 1:
        raise ValueError("occupied_threshold_points must be at least 1")
    if config.inflate_radius_cells < 0:
        raise ValueError("inflate_radius_cells must be non-negative")
