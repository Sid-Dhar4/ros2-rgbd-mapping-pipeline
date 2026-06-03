#!/usr/bin/env python3
"""
Launch the full RGB-D mapping pipeline.

Pipeline:

TUM RGB-D dataset
  -> RGB/depth/CameraInfo topics
  -> /rgbd/cloud_raw
  -> /rgbd/cloud_filtered
  -> C++ map accumulator
  -> /map/cloud
  -> outputs/maps/final_map_cpp.pcd and final_map_cpp.ply on shutdown

This is the main reproducible mapping launch command:

  ros2 launch rgbd_mapping_pipeline mapping_pipeline.launch.py
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

    map_voxel_size_arg = DeclareLaunchArgument(
        "map_voxel_size_m",
        default_value="0.05",
        description="Voxel size in meters for the accumulated global map.",
    )

    accumulate_every_arg = DeclareLaunchArgument(
        "accumulate_every_n_frames",
        default_value="3",
        description="Accumulate every Nth filtered cloud.",
    )

    publish_every_arg = DeclareLaunchArgument(
        "publish_every_n_processed_frames",
        default_value="2",
        description="Publish /map/cloud every N processed frames.",
    )

    max_map_points_arg = DeclareLaunchArgument(
        "max_map_points",
        default_value="150000",
        description="Maximum number of points retained in the accumulated map.",
    )

    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="/home/sudha/ros2_rgbd_ws/src/ros2-rgbd-mapping-pipeline/outputs/maps",
        description="Directory where final PCD/PLY map files are saved.",
    )

    output_basename_arg = DeclareLaunchArgument(
        "output_basename",
        default_value="final_map_cpp",
        description="Basename for saved final map files.",
    )

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

    map_accumulator = Node(
        package="map_accumulator_cpp",
        executable="map_accumulator_node",
        name="map_accumulator_cpp",
        output="screen",
        parameters=[
            {
                "input_cloud_topic": "/rgbd/cloud_filtered",
                "map_cloud_topic": "/map/cloud",
                "target_frame": "world",
                "map_voxel_size_m": ParameterValue(
                    LaunchConfiguration("map_voxel_size_m"),
                    value_type=float,
                ),
                "accumulate_every_n_frames": ParameterValue(
                    LaunchConfiguration("accumulate_every_n_frames"),
                    value_type=int,
                ),
                "publish_every_n_processed_frames": ParameterValue(
                    LaunchConfiguration("publish_every_n_processed_frames"),
                    value_type=int,
                ),
                "max_map_points": ParameterValue(
                    LaunchConfiguration("max_map_points"),
                    value_type=int,
                ),
                "use_latest_tf_fallback": True,
                "save_final_map_on_shutdown": True,
                "output_dir": LaunchConfiguration("output_dir"),
                "output_basename": LaunchConfiguration("output_basename"),
            }
        ],
    )

    return LaunchDescription(
        [
            map_voxel_size_arg,
            accumulate_every_arg,
            publish_every_arg,
            max_map_points_arg,
            output_dir_arg,
            output_basename_arg,
            filtered_pipeline_launch,
            map_accumulator,
        ]
    )
