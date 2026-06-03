"""
Camera projection utilities for RGB-D mapping.

This file holds math that is independent of ROS. Keeping the pinhole projection
separate makes it easier to test and easier to explain in interviews.
"""

from __future__ import annotations

import numpy as np


def depth_to_xyz(
    depth_raw: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    depth_scale: float = 5000.0,
    max_depth_m: float = 5.0,
    pixel_step: int = 4,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Convert a raw uint16 depth image into XYZ points in the camera optical frame.

    Args:
        depth_raw:
            HxW depth image. For TUM RGB-D, depth is uint16 and depth / 5000 = meters.
        fx, fy, cx, cy:
            Pinhole camera intrinsics from CameraInfo.K.
        depth_scale:
            Scale factor used to convert raw depth values to meters.
        max_depth_m:
            Ignore points farther than this distance.
        pixel_step:
            Use every Nth pixel to keep the first Python version lightweight.

    Returns:
        points_xyz:
            Nx3 float32 array of 3D points.
        valid_v:
            Row indices in the original image for each valid point.
        valid_u:
            Column indices in the original image for each valid point.
    """
    if depth_raw.ndim != 2:
        raise ValueError(f"Expected single-channel depth image, got shape {depth_raw.shape}")

    if pixel_step < 1:
        raise ValueError("pixel_step must be >= 1")

    depth_sampled = depth_raw[::pixel_step, ::pixel_step].astype(np.float32)
    z = depth_sampled / float(depth_scale)

    height, width = depth_raw.shape

    v_coords, u_coords = np.mgrid[0:height:pixel_step, 0:width:pixel_step]
    u_coords = u_coords.astype(np.float32)
    v_coords = v_coords.astype(np.float32)

    valid = np.isfinite(z) & (z > 0.0) & (z <= max_depth_m)

    z_valid = z[valid]
    u_valid = u_coords[valid]
    v_valid = v_coords[valid]

    x_valid = (u_valid - float(cx)) * z_valid / float(fx)
    y_valid = (v_valid - float(cy)) * z_valid / float(fy)

    points_xyz = np.stack((x_valid, y_valid, z_valid), axis=1).astype(np.float32)

    return points_xyz, v_valid.astype(np.int32), u_valid.astype(np.int32)
