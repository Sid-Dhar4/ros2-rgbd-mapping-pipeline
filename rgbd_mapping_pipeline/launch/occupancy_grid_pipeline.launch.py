#!/usr/bin/env python3
"""
Launch the RGB-D mapping pipeline plus a navigation-relevant occupancy grid.

Pipeline:

TUM RGB-D dataset
  -> RGB/depth/CameraInfo topics
  -> /rgbd/cloud_raw
  -> /rgbd/cloud_filtered
  -> C++ map accumulator
  -> /map/cloud
  -> occupancy_grid node
  -> /map/occupancy_grid

This is not full Nav2. It is a costmap-style bridge that projects the
accumulated RGB-D point-cloud map into a 2D nav_msgs/OccupancyGrid.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("rgbd_mapping_pipeline")

    resolution_arg = DeclareLaunchArgument(
        "resolution_m",
        default_value="0.05",
        description="Occupancy grid resolution in meters per cell.",
    )

    min_z_arg = DeclareLaunchArgument(
        "min_z_m",
        default_value="0.05",
        description="Minimum obstacle height included in the occupancy grid.",
    )

    max_z_arg = DeclareLaunchArgument(
        "max_z_m",
        default_value="1.50",
        description="Maximum obstacle height included in the occupancy grid.",
    )

    inflate_arg = DeclareLaunchArgument(
        "inflate_radius_cells",
        default_value="1",
        description="Square cell inflation radius around occupied cells.",
    )

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

    occupancy_grid = Node(
        package="rgbd_mapping_pipeline",
        executable="occupancy_grid",
        name="occupancy_grid_node",
        output="screen",
        parameters=[
            {
                "input_cloud_topic": "/map/cloud",
                "output_grid_topic": "/map/occupancy_grid",
                "frame_id": "world",
                "resolution_m": ParameterValue(
                    LaunchConfiguration("resolution_m"),
                    value_type=float,
                ),
                "min_x_m": -3.0,
                "max_x_m": 3.0,
                "min_y_m": -3.0,
                "max_y_m": 3.0,
                "min_z_m": ParameterValue(
                    LaunchConfiguration("min_z_m"),
                    value_type=float,
                ),
                "max_z_m": ParameterValue(
                    LaunchConfiguration("max_z_m"),
                    value_type=float,
                ),
                "occupied_threshold_points": 1,
                "inflate_radius_cells": ParameterValue(
                    LaunchConfiguration("inflate_radius_cells"),
                    value_type=int,
                ),
                "unknown_as_free": True,
            }
        ],
    )

    return LaunchDescription(
        [
            resolution_arg,
            min_z_arg,
            max_z_arg,
            inflate_arg,
            mapping_pipeline_launch,
            occupancy_grid,
        ]
    )
