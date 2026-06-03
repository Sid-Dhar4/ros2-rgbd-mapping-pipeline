#!/usr/bin/env python3
"""
Convert synchronized RGB-D images into a colored PointCloud2.

Subscribes:
  /camera/color/image_raw      sensor_msgs/Image
  /camera/depth/image_raw      sensor_msgs/Image
  /camera/color/camera_info    sensor_msgs/CameraInfo

Publishes:
  /rgbd/cloud_raw              sensor_msgs/PointCloud2
"""

from __future__ import annotations

import time

import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from rgbd_mapping_pipeline.camera_model import depth_to_xyz
from rgbd_mapping_pipeline.pointcloud2_utils import make_colored_pointcloud2


class DepthToCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_to_cloud")

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("cloud_topic", "/rgbd/cloud_raw")

        self.declare_parameter("depth_scale", 5000.0)
        self.declare_parameter("max_depth_m", 5.0)
        self.declare_parameter("pixel_step", 4)
        self.declare_parameter("sync_slop_s", 0.03)

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        cloud_topic = self.get_parameter("cloud_topic").value

        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.pixel_step = int(self.get_parameter("pixel_step").value)
        sync_slop_s = float(self.get_parameter("sync_slop_s").value)

        self.bridge = CvBridge()
        self.frame_count = 0

        self.cloud_pub = self.create_publisher(PointCloud2, cloud_topic, 10)

        self.color_sub = message_filters.Subscriber(self, Image, color_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, camera_info_topic)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=20,
            slop=sync_slop_s,
        )
        self.sync.registerCallback(self._on_rgbd)

        self.get_logger().info("Depth-to-cloud node started")
        self.get_logger().info(f"Subscribing color:       {color_topic}")
        self.get_logger().info(f"Subscribing depth:       {depth_topic}")
        self.get_logger().info(f"Subscribing camera info: {camera_info_topic}")
        self.get_logger().info(f"Publishing cloud:        {cloud_topic}")
        self.get_logger().info(f"depth_scale={self.depth_scale}")
        self.get_logger().info(f"max_depth_m={self.max_depth_m}")
        self.get_logger().info(f"pixel_step={self.pixel_step}")

    def _on_rgbd(
        self,
        color_msg: Image,
        depth_msg: Image,
        camera_info_msg: CameraInfo,
    ) -> None:
        start = time.perf_counter()

        rgb = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        if depth_raw.dtype != np.uint16:
            self.get_logger().warn(
                f"Expected uint16 depth image, got {depth_raw.dtype}. Continuing anyway."
            )

        k = camera_info_msg.k
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])

        points_xyz, v_idx, u_idx = depth_to_xyz(
            depth_raw=depth_raw,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            depth_scale=self.depth_scale,
            max_depth_m=self.max_depth_m,
            pixel_step=self.pixel_step,
        )

        colors = rgb[v_idx, u_idx, :].astype(np.uint8)

        header = depth_msg.header
        header.frame_id = camera_info_msg.header.frame_id

        cloud_msg = make_colored_pointcloud2(header, points_xyz, colors)
        self.cloud_pub.publish(cloud_msg)

        elapsed_ms = (time.perf_counter() - start) * 1000.0
        self.frame_count += 1

        if self.frame_count % 30 == 1:
            total_sampled = (depth_raw.shape[0] // self.pixel_step) * (
                depth_raw.shape[1] // self.pixel_step
            )
            self.get_logger().info(
                f"Published cloud frame={self.frame_count} "
                f"points={points_xyz.shape[0]} "
                f"sampled_pixels≈{total_sampled} "
                f"latency_ms={elapsed_ms:.2f}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthToCloudNode()

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
