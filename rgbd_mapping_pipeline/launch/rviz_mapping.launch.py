#!/usr/bin/env python3
"""
Launch the full RGB-D mapping pipeline with RViz.

This is the main demo command:

  ros2 launch rgbd_mapping_pipeline rviz_mapping.launch.py

It starts:
  1. RGB-D dataset replay
  2. depth-to-cloud
  3. cloud filtering
  4. C++ world-frame map accumulation
  5. RViz showing /map/cloud
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("rgbd_mapping_pipeline")

    mapping_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    package_share,
                    "launch",
                    "mapping_pipeline.launch.py",
                ]
            )
        )
    )

    rviz_config = PathJoinSubstitution(
        [
            package_share,
            "rviz",
            "rgbd_mapping.rviz",
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
            mapping_pipeline_launch,
            rviz_node,
        ]
    )
