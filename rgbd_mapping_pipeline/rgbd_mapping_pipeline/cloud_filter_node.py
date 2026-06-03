#!/usr/bin/env python3
"""
Filter and voxel-downsample a colored PointCloud2.

Subscribes:
  /rgbd/cloud_raw

Publishes:
  /rgbd/cloud_filtered

This node is where the project starts showing engineering tradeoffs:
we reduce point count while measuring how much data we removed and how long
the filtering step takes.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from rgbd_mapping_pipeline.pointcloud2_utils import make_colored_pointcloud2


def unpack_rgb_uint32(rgb_packed: np.ndarray) -> np.ndarray:
    """Convert packed uint32 RGB values into Nx3 uint8 RGB colors."""
    rgb_u32 = rgb_packed.astype(np.uint32)
    r = (rgb_u32 >> 16) & 255
    g = (rgb_u32 >> 8) & 255
    b = rgb_u32 & 255
    return np.stack((r, g, b), axis=1).astype(np.uint8)


def pointcloud2_to_arrays(msg: PointCloud2) -> tuple[np.ndarray, np.ndarray]:
    """
    Convert PointCloud2 fields x, y, z, rgb into numpy arrays.

    Returns:
      points_xyz: Nx3 float32
      colors_rgb: Nx3 uint8
    """
    cloud = point_cloud2.read_points(
        msg,
        field_names=("x", "y", "z", "rgb"),
        skip_nans=True,
    )

    if isinstance(cloud, np.ndarray) and cloud.dtype.names is not None:
        x = cloud["x"].astype(np.float32)
        y = cloud["y"].astype(np.float32)
        z = cloud["z"].astype(np.float32)
        rgb_packed = cloud["rgb"].astype(np.uint32)
    else:
        rows = list(cloud)
        if not rows:
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)

        arr = np.asarray(rows)
        x = arr[:, 0].astype(np.float32)
        y = arr[:, 1].astype(np.float32)
        z = arr[:, 2].astype(np.float32)
        rgb_packed = arr[:, 3].astype(np.uint32)

    points_xyz = np.stack((x, y, z), axis=1).astype(np.float32)
    colors_rgb = unpack_rgb_uint32(rgb_packed)

    return points_xyz, colors_rgb


def voxel_downsample(
    points_xyz: np.ndarray,
    colors_rgb: np.ndarray,
    voxel_size_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Downsample a point cloud using voxel-grid averaging.

    All points that fall in the same voxel are replaced by one average XYZ point
    and one average RGB color.
    """
    if points_xyz.shape[0] == 0:
        return points_xyz, colors_rgb

    if voxel_size_m <= 0.0:
        raise ValueError("voxel_size_m must be > 0")

    voxel_indices = np.floor(points_xyz / voxel_size_m).astype(np.int64)

    _, inverse, counts = np.unique(
        voxel_indices,
        axis=0,
        return_inverse=True,
        return_counts=True,
    )

    num_voxels = counts.shape[0]

    xyz_sum = np.zeros((num_voxels, 3), dtype=np.float64)
    rgb_sum = np.zeros((num_voxels, 3), dtype=np.float64)

    np.add.at(xyz_sum, inverse, points_xyz)
    np.add.at(rgb_sum, inverse, colors_rgb.astype(np.float64))

    xyz_mean = (xyz_sum / counts[:, None]).astype(np.float32)
    rgb_mean = np.clip(rgb_sum / counts[:, None], 0, 255).astype(np.uint8)

    return xyz_mean, rgb_mean


class CloudFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("cloud_filter")

        self.declare_parameter("input_cloud_topic", "/rgbd/cloud_raw")
        self.declare_parameter("output_cloud_topic", "/rgbd/cloud_filtered")
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 5.0)
        self.declare_parameter("voxel_size_m", 0.03)

        input_topic = self.get_parameter("input_cloud_topic").value
        output_topic = self.get_parameter("output_cloud_topic").value

        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.voxel_size_m = float(self.get_parameter("voxel_size_m").value)

        self.frame_count = 0

        self.cloud_pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self._on_cloud,
            10,
        )

        self.get_logger().info("Cloud filter node started")
        self.get_logger().info(f"Subscribing cloud: {input_topic}")
        self.get_logger().info(f"Publishing cloud:  {output_topic}")
        self.get_logger().info(f"min_depth_m={self.min_depth_m}")
        self.get_logger().info(f"max_depth_m={self.max_depth_m}")
        self.get_logger().info(f"voxel_size_m={self.voxel_size_m}")

    def _on_cloud(self, msg: PointCloud2) -> None:
        start = time.perf_counter()

        points_xyz, colors_rgb = pointcloud2_to_arrays(msg)
        raw_count = points_xyz.shape[0]

        if raw_count == 0:
            return

        finite_mask = np.isfinite(points_xyz).all(axis=1)
        depth_mask = (
            (points_xyz[:, 2] >= self.min_depth_m)
            & (points_xyz[:, 2] <= self.max_depth_m)
        )
        keep_mask = finite_mask & depth_mask

        range_points = points_xyz[keep_mask]
        range_colors = colors_rgb[keep_mask]
        range_count = range_points.shape[0]

        filtered_points, filtered_colors = voxel_downsample(
            range_points,
            range_colors,
            self.voxel_size_m,
        )

        out_msg = make_colored_pointcloud2(
            msg.header,
            filtered_points,
            filtered_colors,
        )
        self.cloud_pub.publish(out_msg)

        filtered_count = filtered_points.shape[0]
        reduction_ratio = 1.0 - (filtered_count / raw_count)
        elapsed_ms = (time.perf_counter() - start) * 1000.0

        self.frame_count += 1

        if self.frame_count % 30 == 1:
            self.get_logger().info(
                f"frame={self.frame_count} "
                f"raw_points={raw_count} "
                f"range_points={range_count} "
                f"filtered_points={filtered_count} "
                f"reduction={reduction_ratio * 100.0:.1f}% "
                f"latency_ms={elapsed_ms:.2f}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CloudFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Shutting down.", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
