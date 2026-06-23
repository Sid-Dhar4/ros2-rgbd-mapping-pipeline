"""ROS 2 node that projects a PointCloud2 map into a 2D occupancy grid."""

from __future__ import annotations

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from rgbd_mapping_pipeline.occupancy_grid_utils import (
    OccupancyGridConfig,
    flatten_grid_for_ros,
    grid_shape,
    origin_xy,
    points_to_occupancy_grid,
)


def pointcloud2_to_xyz_array(msg: PointCloud2) -> np.ndarray:
    """Convert PointCloud2 x/y/z fields into an Nx3 float32 array."""
    points = point_cloud2.read_points(
        msg,
        field_names=("x", "y", "z"),
        skip_nans=True,
    )

    if isinstance(points, np.ndarray):
        if points.dtype.names:
            return np.column_stack(
                [points["x"], points["y"], points["z"]]
            ).astype(np.float32)
        return np.asarray(points, dtype=np.float32).reshape(-1, 3)

    rows = []
    for point in points:
        rows.append([float(point[0]), float(point[1]), float(point[2])])

    if not rows:
        return np.empty((0, 3), dtype=np.float32)

    return np.asarray(rows, dtype=np.float32)


class OccupancyGridNode(Node):
    """Project an accumulated point-cloud map into nav_msgs/OccupancyGrid."""

    def __init__(self) -> None:
        super().__init__("occupancy_grid_node")

        self.declare_parameter("input_cloud_topic", "/map/cloud")
        self.declare_parameter("output_grid_topic", "/map/occupancy_grid")
        self.declare_parameter("frame_id", "world")

        self.declare_parameter("resolution_m", 0.05)
        self.declare_parameter("min_x_m", -3.0)
        self.declare_parameter("max_x_m", 3.0)
        self.declare_parameter("min_y_m", -3.0)
        self.declare_parameter("max_y_m", 3.0)
        self.declare_parameter("min_z_m", 0.05)
        self.declare_parameter("max_z_m", 1.50)
        self.declare_parameter("occupied_threshold_points", 1)
        self.declare_parameter("inflate_radius_cells", 0)
        self.declare_parameter("unknown_as_free", True)

        input_topic = self.get_parameter("input_cloud_topic").value
        output_topic = self.get_parameter("output_grid_topic").value

        self.publisher = self.create_publisher(OccupancyGrid, output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.cloud_callback,
            10,
        )

        self.callback_count = 0

        self.get_logger().info(
            f"Occupancy grid node listening on {input_topic} "
            f"and publishing {output_topic}"
        )

    def cloud_callback(self, msg: PointCloud2) -> None:
        config = self._config_from_parameters()
        points_xyz = pointcloud2_to_xyz_array(msg)
        grid = points_to_occupancy_grid(points_xyz, config)

        grid_msg = self._make_occupancy_grid_msg(msg, grid, config)
        self.publisher.publish(grid_msg)

        self.callback_count += 1
        if self.callback_count == 1 or self.callback_count % 10 == 0:
            occupied_count = int(np.count_nonzero(grid == 100))
            self.get_logger().info(
                f"Published occupancy grid {grid_msg.info.width}x"
                f"{grid_msg.info.height}, occupied cells={occupied_count}, "
                f"input points={points_xyz.shape[0]}"
            )

    def _config_from_parameters(self) -> OccupancyGridConfig:
        return OccupancyGridConfig(
            resolution_m=float(self.get_parameter("resolution_m").value),
            min_x_m=float(self.get_parameter("min_x_m").value),
            max_x_m=float(self.get_parameter("max_x_m").value),
            min_y_m=float(self.get_parameter("min_y_m").value),
            max_y_m=float(self.get_parameter("max_y_m").value),
            min_z_m=float(self.get_parameter("min_z_m").value),
            max_z_m=float(self.get_parameter("max_z_m").value),
            occupied_threshold_points=int(
                self.get_parameter("occupied_threshold_points").value
            ),
            inflate_radius_cells=int(self.get_parameter("inflate_radius_cells").value),
            unknown_as_free=bool(self.get_parameter("unknown_as_free").value),
        )

    def _make_occupancy_grid_msg(
        self,
        cloud_msg: PointCloud2,
        grid: np.ndarray,
        config: OccupancyGridConfig,
    ) -> OccupancyGrid:
        height, width = grid_shape(config)
        origin_x, origin_y = origin_xy(config)

        frame_id = str(self.get_parameter("frame_id").value)
        if not frame_id:
            frame_id = cloud_msg.header.frame_id

        msg = OccupancyGrid()
        msg.header.stamp = cloud_msg.header.stamp
        msg.header.frame_id = frame_id

        msg.info.resolution = float(config.resolution_m)
        msg.info.width = int(width)
        msg.info.height = int(height)

        origin = Pose()
        origin.position.x = float(origin_x)
        origin.position.y = float(origin_y)
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        msg.info.origin = origin

        msg.data = flatten_grid_for_ros(grid)
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
