#!/usr/bin/env python3
"""
Launch RGB-D dataset replay.

This launch file starts the dataset publisher node and makes the project
reproducible with one command.
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

    fps_arg = DeclareLaunchArgument(
        "fps",
        default_value="10.0",
        description="Replay rate in frames per second.",
    )

    loop_arg = DeclareLaunchArgument(
        "loop",
        default_value="true",
        description="Whether to loop the dataset replay.",
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="camera_color_optical_frame",
        description="Frame ID used for published RGB, depth, and CameraInfo messages.",
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
                "fps": ParameterValue(LaunchConfiguration("fps"), value_type=float),
                "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
                "frame_id": LaunchConfiguration("frame_id"),
                "fx": 517.3,
                "fy": 516.5,
                "cx": 318.6,
                "cy": 255.3,
            }
        ],
    )

    return LaunchDescription(
        [
            dataset_dir_arg,
            fps_arg,
            loop_arg,
            frame_id_arg,
            rgbd_publisher,
        ]
    )
