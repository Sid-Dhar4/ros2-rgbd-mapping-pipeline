# How to Read the Results

This project produces three main point-cloud outputs.

## `/rgbd/cloud_raw`

A colored point cloud created directly from each RGB-D frame. Points are in `camera_color_optical_frame`. This validates depth projection and camera intrinsics.

## `/rgbd/cloud_filtered`

A reduced point cloud after range filtering and voxel downsampling. This is used to reduce computation before mapping.

## `/map/cloud`

The accumulated world-frame point-cloud map. The C++/PCL accumulator uses TF from `world` to `camera_color_optical_frame`, voxel-downsamples the global map, publishes `/map/cloud`, and exports PCD/PLY files.

## Important interpretation

- This is not a full SLAM system.
- TUM ground-truth poses are used to isolate and evaluate the RGB-D mapping pipeline.
- The output is a colored point-cloud map, not a dense mesh or CAD model.
- Holes, sparse surfaces, and some ghosting are expected due to depth noise, invalid pixels, voxel filtering, and RGB-D sensor limitations.
