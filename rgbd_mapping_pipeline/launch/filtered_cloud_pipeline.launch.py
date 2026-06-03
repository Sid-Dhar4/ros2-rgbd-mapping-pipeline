#!/usr/bin/env python3
"""
Launch the RGB-D replay, raw point cloud generation, and cloud filtering.

Pipeline:

TUM RGB-D dataset
  -> /camera/color/image_raw
  -> /camera/depth/image_raw
  -> /camera/color/camera_info
  -> /rgbd/cloud_raw
  -> /rgbd/cloud_filtered

This launch file produces the first measured perception pipeline output:
a filtered/downsampled RGB-D point cloud.
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

    pixel_step_arg = DeclareLaunchArgument(
        "pixel_step",
        default_value="4",
        description="Use every Nth image pixel when creating /rgbd/cloud_raw.",
    )

    raw_max_depth_arg = DeclareLaunchArgument(
        "raw_max_depth_m",
        default_value="5.0",
        description="Maximum depth used by depth_to_cloud.",
    )

    voxel_size_arg = DeclareLaunchArgument(
        "voxel_size_m",
        default_value="0.03",
        description="Voxel size in meters for cloud_filter.",
    )

    filter_min_depth_arg = DeclareLaunchArgument(
        "filter_min_depth_m",
        default_value="0.2",
        description="Minimum Z depth kept by cloud_filter.",
    )

    filter_max_depth_arg = DeclareLaunchArgument(
        "filter_max_depth_m",
        default_value="5.0",
        description="Maximum Z depth kept by cloud_filter.",
    )

    cloud_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    package_share,
                    "launch",
                    "cloud_pipeline.launch.py",
                ]
            )
        ),
        launch_arguments={
            "pixel_step": LaunchConfiguration("pixel_step"),
            "max_depth_m": LaunchConfiguration("raw_max_depth_m"),
        }.items(),
    )

    cloud_filter = Node(
        package="rgbd_mapping_pipeline",
        executable="cloud_filter",
        name="cloud_filter",
        output="screen",
        parameters=[
            {
                "input_cloud_topic": "/rgbd/cloud_raw",
                "output_cloud_topic": "/rgbd/cloud_filtered",
                "min_depth_m": ParameterValue(
                    LaunchConfiguration("filter_min_depth_m"),
                    value_type=float,
                ),
                "max_depth_m": ParameterValue(
                    LaunchConfiguration("filter_max_depth_m"),
                    value_type=float,
                ),
                "voxel_size_m": ParameterValue(
                    LaunchConfiguration("voxel_size_m"),
                    value_type=float,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            pixel_step_arg,
            raw_max_depth_arg,
            voxel_size_arg,
            filter_min_depth_arg,
            filter_max_depth_arg,
            cloud_pipeline_launch,
            cloud_filter,
        ]
    )
