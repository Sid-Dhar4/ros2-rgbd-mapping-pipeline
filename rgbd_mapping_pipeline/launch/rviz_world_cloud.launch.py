#!/usr/bin/env python3
"""
Launch filtered RGB-D cloud pipeline with TF and RViz fixed in world frame.

This validates that dataset ground-truth poses are being published as TF:

  world -> camera_color_optical_frame

RViz displays /rgbd/cloud_filtered transformed into the world frame.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("rgbd_mapping_pipeline")

    filtered_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    package_share,
                    "launch",
                    "filtered_cloud_pipeline.launch.py",
                ]
            )
        )
    )

    rviz_config = PathJoinSubstitution(
        [
            package_share,
            "rviz",
            "rgbd_world_cloud.rviz",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            filtered_pipeline_launch,
            rviz_node,
        ]
    )
