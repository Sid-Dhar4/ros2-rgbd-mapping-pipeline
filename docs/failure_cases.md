# Failure Cases and Limitations

- Uses TUM ground-truth camera poses for map accumulation; this is not a full SLAM system.
- Map quality depends on depth validity, camera intrinsics, pose alignment, and voxel size.
- Dynamic objects are not explicitly removed.
- Invalid depth pixels are skipped, which can create holes in the map.
- The current pipeline is designed for reproducible offline evaluation, not live robot deployment.
- The RGB-D replay uses public dataset files and does not require a physical depth camera.
- Very dense settings can slow down Python point-cloud generation, so Version 1 uses pixel sampling and voxel filtering.
