"""
PointCloud2 helper functions.

ROS PointCloud2 is powerful but a little awkward because the point data is stored
as a compact binary layout. This file keeps that formatting code away from the
main node logic.
"""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


def pack_rgb_uint32(rgb: np.ndarray) -> np.ndarray:
    """
    Pack Nx3 uint8 RGB colors into one uint32 field.

    Layout:
      rgb_uint32 = R << 16 | G << 8 | B
    """
    rgb_u32 = rgb.astype(np.uint32)
    return (rgb_u32[:, 0] << 16) | (rgb_u32[:, 1] << 8) | rgb_u32[:, 2]


def make_colored_pointcloud2(header, points_xyz: np.ndarray, rgb: np.ndarray) -> PointCloud2:
    """
    Build a colored PointCloud2 message with fields x, y, z, rgb.

    Args:
        header:
            ROS Header containing stamp and frame_id.
        points_xyz:
            Nx3 float32 array.
        rgb:
            Nx3 uint8 RGB array.
    """
    if points_xyz.shape[0] != rgb.shape[0]:
        raise ValueError("points_xyz and rgb must have the same number of rows")

    rgb_packed = pack_rgb_uint32(rgb)

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]

    points = [
        (
            float(points_xyz[i, 0]),
            float(points_xyz[i, 1]),
            float(points_xyz[i, 2]),
            int(rgb_packed[i]),
        )
        for i in range(points_xyz.shape[0])
    ]

    return point_cloud2.create_cloud(header, fields, points)
