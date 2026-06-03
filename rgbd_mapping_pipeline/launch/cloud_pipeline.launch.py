#!/usr/bin/env python3
"""
Launch the RGB-D replay + depth-to-point-cloud pipeline.

This is the first end-to-end perception pipeline launch file:

TUM RGB-D dataset
  -> RGB/depth/CameraInfo ROS topics
  -> colored PointCloud2 cloud

Published output:
  /rgbd/cloud_raw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_dataset_dir = PathJoinSubstitution(
        [
            EnvironmentVariable("HOME"),
            "ros2_rgbd_ws",
            "src",
            "ros2-rgbd-mapping-pipeline",
            "data",
            "rgbd_dataset_freiburg1_xyz",
        ]
    )

    dataset_dir_arg = DeclareLaunchArgument(
        "dataset_dir",
        default_value=default_dataset_dir,
        description="Path to extracted TUM RGB-D dataset directory.",
    )

    replay_fps_arg = DeclareLaunchArgument(
        "replay_fps",
        default_value="10.0",
        description="RGB-D dataset replay rate in frames per second.",
    )

    loop_arg = DeclareLaunchArgument(
        "loop",
        default_value="true",
        description="Whether to loop the dataset replay.",
    )

    pixel_step_arg = DeclareLaunchArgument(
        "pixel_step",
        default_value="4",
        description="Use every Nth pixel when creating the first raw point cloud.",
    )

    max_depth_arg = DeclareLaunchArgument(
        "max_depth_m",
        default_value="5.0",
        description="Maximum depth in meters to keep when creating the point cloud.",
    )

    depth_scale_arg = DeclareLaunchArgument(
        "depth_scale",
        default_value="5000.0",
        description="TUM depth scale. Raw depth / 5000.0 = meters.",
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="camera_color_optical_frame",
        description="Optical camera frame used for images and raw point cloud.",
    )

    rgbd_publisher = Node(
        package="rgbd_mapping_pipeline",
        executable="rgbd_dataset_publisher",
        name="rgbd_dataset_publisher",
        output="screen",
        parameters=[
            {
                "dataset_dir": LaunchConfiguration("dataset_dir"),
                "association_file": "associations.txt",
                "pose_association_file": "rgbd_pose_associations.txt",
                "publish_tf": True,
                "world_frame_id": "world",
                "fps": ParameterValue(LaunchConfiguration("replay_fps"), value_type=float),
                "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
                "frame_id": LaunchConfiguration("frame_id"),
                "fx": 517.3,
                "fy": 516.5,
                "cx": 318.6,
                "cy": 255.3,
            }
        ],
    )

    depth_to_cloud = Node(
        package="rgbd_mapping_pipeline",
        executable="depth_to_cloud",
        name="depth_to_cloud",
        output="screen",
        parameters=[
            {
                "color_topic": "/camera/color/image_raw",
                "depth_topic": "/camera/depth/image_raw",
                "camera_info_topic": "/camera/color/camera_info",
                "cloud_topic": "/rgbd/cloud_raw",
                "depth_scale": ParameterValue(
                    LaunchConfiguration("depth_scale"), value_type=float
                ),
                "max_depth_m": ParameterValue(
                    LaunchConfiguration("max_depth_m"), value_type=float
                ),
                "pixel_step": ParameterValue(
                    LaunchConfiguration("pixel_step"), value_type=int
                ),
                "sync_slop_s": 0.03,
            }
        ],
    )

    return LaunchDescription(
        [
            dataset_dir_arg,
            replay_fps_arg,
            loop_arg,
            pixel_step_arg,
            max_depth_arg,
            depth_scale_arg,
            frame_id_arg,
            rgbd_publisher,
            depth_to_cloud,
        ]
    )
