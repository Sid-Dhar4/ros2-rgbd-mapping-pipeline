#!/usr/bin/env python3
"""
Replay a TUM RGB-D dataset sequence as ROS 2 image topics.

Publishes:
  /camera/color/image_raw     sensor_msgs/Image, encoding rgb8
  /camera/depth/image_raw     sensor_msgs/Image, encoding 16UC1
  /camera/color/camera_info   sensor_msgs/CameraInfo
  /tf                         world -> camera_color_optical_frame, optional

This node acts like a dataset-backed RGB-D camera driver. When a pose association
file is provided, it also publishes the matched ground-truth camera pose as TF.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


@dataclass(frozen=True)
class RgbdFrame:
    rgb_time: float
    rgb_path: str
    depth_time: float
    depth_path: str
    gt_time: Optional[float] = None
    tx: Optional[float] = None
    ty: Optional[float] = None
    tz: Optional[float] = None
    qx: Optional[float] = None
    qy: Optional[float] = None
    qz: Optional[float] = None
    qw: Optional[float] = None


class RgbdDatasetPublisher(Node):
    def __init__(self) -> None:
        super().__init__("rgbd_dataset_publisher")

        self.declare_parameter("dataset_dir", "")
        self.declare_parameter("association_file", "associations.txt")
        self.declare_parameter("pose_association_file", "")
        self.declare_parameter("fps", 10.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("world_frame_id", "world")
        self.declare_parameter("publish_tf", False)

        # TUM freiburg1 RGB camera intrinsics.
        self.declare_parameter("fx", 517.3)
        self.declare_parameter("fy", 516.5)
        self.declare_parameter("cx", 318.6)
        self.declare_parameter("cy", 255.3)

        self.dataset_dir = Path(
            self.get_parameter("dataset_dir").get_parameter_value().string_value
        ).expanduser()

        association_name = (
            self.get_parameter("association_file").get_parameter_value().string_value
        )
        pose_association_name = (
            self.get_parameter("pose_association_file").get_parameter_value().string_value
        )

        self.fps = self.get_parameter("fps").get_parameter_value().double_value
        self.loop = self.get_parameter("loop").get_parameter_value().bool_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.world_frame_id = (
            self.get_parameter("world_frame_id").get_parameter_value().string_value
        )
        self.publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value

        self.fx = self.get_parameter("fx").get_parameter_value().double_value
        self.fy = self.get_parameter("fy").get_parameter_value().double_value
        self.cx = self.get_parameter("cx").get_parameter_value().double_value
        self.cy = self.get_parameter("cy").get_parameter_value().double_value

        if not self.dataset_dir.exists():
            raise FileNotFoundError(f"Dataset directory does not exist: {self.dataset_dir}")

        if pose_association_name:
            self.association_path = self.dataset_dir / pose_association_name
        else:
            self.association_path = self.dataset_dir / association_name

        if not self.association_path.exists():
            raise FileNotFoundError(
                f"Association file does not exist: {self.association_path}"
            )

        self.frames = self._load_frames(self.association_path)

        if not self.frames:
            raise RuntimeError(f"No frames found in {self.association_path}")

        if self.publish_tf and not self.frames[0].has_pose:
            raise RuntimeError(
                "publish_tf:=true requires pose_association_file with pose columns."
            )

        self.bridge = CvBridge()
        self.index = 0

        self.color_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        period_s = 1.0 / max(self.fps, 0.1)
        self.timer = self.create_timer(period_s, self._on_timer)

        self.get_logger().info("RGB-D dataset publisher started")
        self.get_logger().info(f"Dataset: {self.dataset_dir}")
        self.get_logger().info(f"Association file: {self.association_path}")
        self.get_logger().info(f"Frames loaded: {len(self.frames)}")
        self.get_logger().info(f"Replay FPS: {self.fps}")
        self.get_logger().info(f"Frame ID: {self.frame_id}")
        self.get_logger().info(f"Publish TF: {self.publish_tf}")
        if self.publish_tf:
            self.get_logger().info(f"TF: {self.world_frame_id} -> {self.frame_id}")

    def _load_frames(self, path: Path) -> list[RgbdFrame]:
        frames: list[RgbdFrame] = []

        with path.open("r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()

                if not line or line.startswith("#"):
                    continue

                parts = line.split()

                if len(parts) == 4:
                    frames.append(
                        RgbdFrame(
                            rgb_time=float(parts[0]),
                            rgb_path=parts[1],
                            depth_time=float(parts[2]),
                            depth_path=parts[3],
                        )
                    )
                elif len(parts) == 12:
                    frames.append(
                        RgbdFrame(
                            rgb_time=float(parts[0]),
                            rgb_path=parts[1],
                            depth_time=float(parts[2]),
                            depth_path=parts[3],
                            gt_time=float(parts[4]),
                            tx=float(parts[5]),
                            ty=float(parts[6]),
                            tz=float(parts[7]),
                            qx=float(parts[8]),
                            qy=float(parts[9]),
                            qz=float(parts[10]),
                            qw=float(parts[11]),
                        )
                    )
                else:
                    self.get_logger().warn(f"Skipping malformed line: {line}")

        return frames

    def _make_camera_info(self, stamp) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.width = 640
        msg.height = 480

        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        msg.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0,
        ]

        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        msg.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        return msg

    def _publish_tf(self, frame: RgbdFrame, stamp) -> None:
        if not frame.has_pose:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.world_frame_id
        tf_msg.child_frame_id = self.frame_id

        tf_msg.transform.translation.x = float(frame.tx)
        tf_msg.transform.translation.y = float(frame.ty)
        tf_msg.transform.translation.z = float(frame.tz)

        tf_msg.transform.rotation.x = float(frame.qx)
        tf_msg.transform.rotation.y = float(frame.qy)
        tf_msg.transform.rotation.z = float(frame.qz)
        tf_msg.transform.rotation.w = float(frame.qw)

        self.tf_broadcaster.sendTransform(tf_msg)

    def _on_timer(self) -> None:
        if self.index >= len(self.frames):
            if self.loop:
                self.index = 0
            else:
                self.get_logger().info("Reached end of dataset. Stopping timer.")
                self.timer.cancel()
                return

        frame = self.frames[self.index]

        rgb_path = self.dataset_dir / frame.rgb_path
        depth_path = self.dataset_dir / frame.depth_path

        rgb_bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)

        if rgb_bgr is None:
            self.get_logger().error(f"Failed to read RGB image: {rgb_path}")
            self.index += 1
            return

        if depth is None:
            self.get_logger().error(f"Failed to read depth image: {depth_path}")
            self.index += 1
            return

        rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)

        stamp = self.get_clock().now().to_msg()

        if self.publish_tf:
            self._publish_tf(frame, stamp)

        color_msg = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = self.frame_id

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="16UC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.frame_id

        info_msg = self._make_camera_info(stamp)

        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(info_msg)

        if self.index % 50 == 0:
            if frame.has_pose:
                self.get_logger().info(
                    f"Published frame {self.index + 1}/{len(self.frames)} "
                    f"rgb_time={frame.rgb_time:.6f} depth_time={frame.depth_time:.6f} "
                    f"gt_time={frame.gt_time:.6f}"
                )
            else:
                self.get_logger().info(
                    f"Published frame {self.index + 1}/{len(self.frames)} "
                    f"rgb_time={frame.rgb_time:.6f} depth_time={frame.depth_time:.6f}"
                )

        self.index += 1


@property
def _has_pose(self: RgbdFrame) -> bool:
    return (
        self.gt_time is not None
        and self.tx is not None
        and self.ty is not None
        and self.tz is not None
        and self.qx is not None
        and self.qy is not None
        and self.qz is not None
        and self.qw is not None
    )


RgbdFrame.has_pose = _has_pose  # type: ignore[attr-defined]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RgbdDatasetPublisher()

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
