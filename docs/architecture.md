# Architecture

```text
TUM RGB-D dataset
  rgb.txt / depth.txt / groundtruth.txt
        |
        v
Association scripts
  associations.txt
  rgbd_pose_associations.txt
        |
        v
RGB-D replay node
  publishes:
    /camera/color/image_raw
    /camera/depth/image_raw
    /camera/color/camera_info
    /tf  world -> camera_color_optical_frame
        |
        v
Depth-to-cloud node
  subscribes:
    RGB image, depth image, CameraInfo
  publishes:
    /rgbd/cloud_raw
        |
        v
Cloud filter node
  voxel/range filtering
  publishes:
    /rgbd/cloud_filtered
        |
        v
C++ / PCL map accumulator
  subscribes:
    /rgbd/cloud_filtered
    /tf
  publishes:
    /map/cloud
  exports:
    outputs/maps/final_map_cpp.pcd
    outputs/maps/final_map_cpp.ply
        |
        v
RViz
  visualizes raw cloud, filtered cloud, TF, and accumulated map
