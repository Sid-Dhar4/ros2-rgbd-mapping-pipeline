#!/usr/bin/env python3
"""Launch RViz configured for the accumulated map and occupancy grid."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("rgbd_mapping_pipeline"),
            "rviz",
            "rgbd_occupancy_grid.rviz",
        ]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_occupancy_grid",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([rviz])
