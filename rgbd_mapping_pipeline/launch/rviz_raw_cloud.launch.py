#!/usr/bin/env python3
"""
Launch the RGB-D cloud pipeline with RViz.

This is the first screenshot-ready demo launch:

  ros2 launch rgbd_mapping_pipeline rviz_raw_cloud.launch.py

It starts:
  1. RGB-D dataset replay
  2. depth image -> colored PointCloud2
  3. RViz configured for /rgbd/cloud_raw
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("rgbd_mapping_pipeline")

    cloud_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    package_share,
                    "launch",
                    "cloud_pipeline.launch.py",
                ]
            )
        )
    )

    rviz_config = PathJoinSubstitution(
        [
            package_share,
            "rviz",
            "rgbd_raw_cloud.rviz",
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
            cloud_pipeline_launch,
            rviz_node,
        ]
    )
