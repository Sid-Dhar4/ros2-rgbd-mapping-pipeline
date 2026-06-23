# Debugging and Failure Modes

This document summarizes the main engineering issues encountered while building the ROS 2 RGB-D mapping pipeline and how they were debugged.

## 1. RGB/depth timestamp association

Problem: RGB and depth frames in public RGB-D datasets are not always timestamp-identical.

Resolution: added association scripts to pair RGB, depth, and pose messages before replay. This prevents mismatched color/depth projection and unstable point clouds.

## 2. Camera model correctness

Problem: incorrect intrinsics or depth scaling can stretch, flatten, or distort the point cloud.

Resolution: implemented and tested the pinhole projection path separately so the depth-to-cloud conversion can be verified without running the full ROS graph.

## 3. TF frame consistency

Problem: point clouds must be transformed into a consistent world frame before accumulation. A missing or incorrect transform causes the global map to drift, duplicate, or appear in the wrong location.

Resolution: the dataset replay publishes a `world -> camera_color_optical_frame` transform from TUM ground-truth poses, and the C++ map accumulator uses TF lookup before adding each filtered cloud.

## 4. Point-cloud density and runtime tradeoff

Problem: raw RGB-D point clouds can be too dense for fast accumulation and visualization.

Resolution: added pixel stepping, range filtering, and voxel-style downsampling. Version 1 reduces typical per-frame point count by approximately 60-90 percent.

## 5. Map publish rate vs camera rate

Problem: the map topic updates slower than the camera stream.

Resolution: this is intentional. The C++ accumulator processes every 3rd filtered cloud and publishes every 2 processed frames, giving approximately 1.66 Hz map updates from an approximately 10 Hz input stream.

## 6. Scope limitation

Version 1 is not a full SLAM system. It uses TUM ground-truth poses to isolate RGB-D projection, point-cloud filtering, TF integration, C++/PCL accumulation, and map export.

Future versions can compare ground-truth-pose mapping against estimated-pose mapping from RGB-D odometry, RTAB-Map, Open3D odometry, or ICP.
