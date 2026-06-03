# Resume Bullets

Project title:

ROS 2 RGB-D Mapping and 3D Reconstruction Pipeline | ROS 2, C++, Python, PCL, TF, OpenCV, RViz

Bullets:

- Built a ROS 2 Jazzy RGB-D mapping pipeline that replays TUM RGB-D sequences, publishes synchronized RGB/depth/camera-info topics, projects depth images into colored PointCloud2 clouds using the pinhole camera model, filters/downsamples clouds, and visualizes results in RViz.

- Implemented a C++/PCL map-accumulation node that subscribes to filtered point clouds, transforms them into a world frame using TUM ground-truth TF poses, publishes /map/cloud, voxel-downsamples the global map, and exports final reconstructions as PCD/PLY files.

- Measured ~10 Hz RGB-D replay/filtering, ~1.66 Hz map publishing, ~60-90 percent point-count reduction, and a ~17.8k-point final accumulated map on a TUM RGB-D indoor sequence.
