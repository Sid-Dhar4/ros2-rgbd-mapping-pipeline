#!/usr/bin/env python3
"""
Launch filtered RGB-D cloud pipeline with RViz.

Starts:
  1. RGB-D dataset replay
  2. raw cloud generation
  3. voxel cloud filtering
  4. RViz showing raw vs filtered clouds
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
            "rgbd_filtered_cloud.rviz",
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
