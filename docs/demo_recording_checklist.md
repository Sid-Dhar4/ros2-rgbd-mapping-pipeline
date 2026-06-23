# Demo Recording Checklist

This checklist defines the short visual demo for the ROS 2 RGB-D mapping pipeline.

Goal: show the full perception-to-map flow in 45-60 seconds.

## Demo structure

1. Show the repo title and CI badge.
2. Show the launch command.
3. Show RGB-D replay starting from the TUM RGB-D sequence.
4. Show raw colored PointCloud2 output.
5. Show filtered/downsampled point cloud.
6. Show accumulated `/map/cloud` in RViz.
7. Show exported PCD/PLY maps.
8. Show the metrics summary table.

## Visual overlays to add in the video

- Dataset: `rgbd_dataset_freiburg1_xyz`
- Raw cloud: approximately 10 Hz
- Filtered cloud: approximately 10 Hz
- Map cloud: approximately 1.66 Hz
- Point reduction: approximately 60-90 percent
- Final map: approximately 17.8k points

## Recording notes

- Keep the video under 60 seconds.
- Use large readable terminal text.
- Do not show private paths unrelated to the repo.
- Do not overclaim SLAM or autonomous navigation.
- Say that Version 1 uses TUM ground-truth poses for world-frame map accumulation.

## Planned media files

- `media/rgbd_mapping_demo.mp4`
- `media/rgbd_mapping_teaser.gif`
