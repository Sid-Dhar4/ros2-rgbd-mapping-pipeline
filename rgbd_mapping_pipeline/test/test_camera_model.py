import numpy as np

from rgbd_mapping_pipeline.camera_model import depth_to_xyz


def test_depth_to_xyz_center_pixel_projects_forward():
    depth = np.array([[5000]], dtype=np.uint16)

    points, v_idx, u_idx = depth_to_xyz(
        depth_raw=depth,
        fx=500.0,
        fy=500.0,
        cx=0.0,
        cy=0.0,
        depth_scale=5000.0,
        max_depth_m=5.0,
        pixel_step=1,
    )

    assert points.shape == (1, 3)
    np.testing.assert_allclose(points[0], [0.0, 0.0, 1.0], atol=1e-6)
    np.testing.assert_array_equal(v_idx, [0])
    np.testing.assert_array_equal(u_idx, [0])


def test_depth_to_xyz_filters_zero_and_far_depth():
    depth = np.array([[0, 5000, 30000]], dtype=np.uint16)

    points, _, _ = depth_to_xyz(
        depth_raw=depth,
        fx=500.0,
        fy=500.0,
        cx=0.0,
        cy=0.0,
        depth_scale=5000.0,
        max_depth_m=5.0,
        pixel_step=1,
    )

    assert points.shape[0] == 1
    np.testing.assert_allclose(points[0, 2], 1.0, atol=1e-6)
