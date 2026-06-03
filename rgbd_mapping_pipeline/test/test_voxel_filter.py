import numpy as np

from rgbd_mapping_pipeline.cloud_filter_node import voxel_downsample


def test_voxel_downsample_merges_points_in_same_voxel():
    points = np.array(
        [
            [0.00, 0.00, 1.00],
            [0.01, 0.01, 1.01],
            [0.20, 0.20, 1.20],
        ],
        dtype=np.float32,
    )
    colors = np.array(
        [
            [100, 100, 100],
            [200, 200, 200],
            [10, 20, 30],
        ],
        dtype=np.uint8,
    )

    filtered_points, filtered_colors = voxel_downsample(
        points_xyz=points,
        colors_rgb=colors,
        voxel_size_m=0.05,
    )

    assert filtered_points.shape[0] == 2
    assert filtered_colors.shape[0] == 2


def test_voxel_downsample_empty_cloud():
    points = np.empty((0, 3), dtype=np.float32)
    colors = np.empty((0, 3), dtype=np.uint8)

    filtered_points, filtered_colors = voxel_downsample(points, colors, 0.05)

    assert filtered_points.shape == (0, 3)
    assert filtered_colors.shape == (0, 3)
