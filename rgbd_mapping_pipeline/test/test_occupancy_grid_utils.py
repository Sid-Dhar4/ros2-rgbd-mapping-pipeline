import numpy as np
import pytest

from rgbd_mapping_pipeline.occupancy_grid_utils import (
    OccupancyGridConfig,
    flatten_grid_for_ros,
    grid_shape,
    origin_xy,
    points_to_occupancy_grid,
)


def test_grid_shape_and_origin():
    config = OccupancyGridConfig(
        resolution_m=0.5,
        min_x_m=-1.0,
        max_x_m=1.0,
        min_y_m=-2.0,
        max_y_m=2.0,
    )

    assert grid_shape(config) == (8, 4)
    assert origin_xy(config) == (-1.0, -2.0)


def test_height_filter_marks_only_obstacle_band():
    points = np.array(
        [
            [0.0, 0.0, 0.01],  # below min height
            [0.2, 0.1, 0.5],   # valid obstacle
            [0.4, 0.1, 2.0],   # above max height
        ],
        dtype=np.float32,
    )
    config = OccupancyGridConfig(
        resolution_m=0.1,
        min_x_m=0.0,
        max_x_m=1.0,
        min_y_m=0.0,
        max_y_m=1.0,
        min_z_m=0.05,
        max_z_m=1.0,
    )

    grid = points_to_occupancy_grid(points, config)

    assert grid.shape == (10, 10)
    assert grid[1, 2] == 100
    assert np.count_nonzero(grid == 100) == 1


def test_points_outside_xy_bounds_are_ignored():
    points = np.array(
        [
            [-0.1, 0.5, 0.5],
            [0.5, -0.1, 0.5],
            [1.0, 0.5, 0.5],
            [0.5, 1.0, 0.5],
            [0.5, 0.5, 0.5],
        ],
        dtype=np.float32,
    )
    config = OccupancyGridConfig(
        resolution_m=0.1,
        min_x_m=0.0,
        max_x_m=1.0,
        min_y_m=0.0,
        max_y_m=1.0,
        min_z_m=0.05,
        max_z_m=1.0,
    )

    grid = points_to_occupancy_grid(points, config)

    assert np.count_nonzero(grid == 100) == 1
    assert grid[5, 5] == 100


def test_occupied_threshold_requires_multiple_points_in_cell():
    points = np.array(
        [
            [0.21, 0.21, 0.5],
            [0.22, 0.22, 0.5],
            [0.70, 0.70, 0.5],
        ],
        dtype=np.float32,
    )
    config = OccupancyGridConfig(
        resolution_m=0.1,
        min_x_m=0.0,
        max_x_m=1.0,
        min_y_m=0.0,
        max_y_m=1.0,
        min_z_m=0.05,
        max_z_m=1.0,
        occupied_threshold_points=2,
    )

    grid = points_to_occupancy_grid(points, config)

    assert grid[2, 2] == 100
    assert grid[7, 7] == 0
    assert np.count_nonzero(grid == 100) == 1


def test_inflation_expands_occupied_cells():
    points = np.array([[0.2, 0.2, 0.5]], dtype=np.float32)
    config = OccupancyGridConfig(
        resolution_m=0.1,
        min_x_m=0.0,
        max_x_m=1.0,
        min_y_m=0.0,
        max_y_m=1.0,
        min_z_m=0.05,
        max_z_m=1.0,
        inflate_radius_cells=1,
    )

    grid = points_to_occupancy_grid(points, config)

    assert np.count_nonzero(grid == 100) == 9
    assert grid[1, 1] == 100
    assert grid[2, 2] == 100
    assert grid[3, 3] == 100


def test_unknown_mode_uses_minus_one_for_unobserved_cells():
    points = np.empty((0, 3), dtype=np.float32)
    config = OccupancyGridConfig(
        resolution_m=0.1,
        min_x_m=0.0,
        max_x_m=0.2,
        min_y_m=0.0,
        max_y_m=0.2,
        unknown_as_free=False,
    )

    grid = points_to_occupancy_grid(points, config)

    assert grid.shape == (2, 2)
    assert np.all(grid == -1)


def test_flatten_grid_for_ros_is_row_major():
    grid = np.array([[0, 100], [-1, 0]], dtype=np.int8)

    assert flatten_grid_for_ros(grid) == [0, 100, -1, 0]


def test_invalid_config_raises():
    config = OccupancyGridConfig(resolution_m=0.0)

    with pytest.raises(ValueError):
        points_to_occupancy_grid(np.empty((0, 3), dtype=np.float32), config)
