import numpy as np

from grasp_server.geometry import (
    backproject_masked_depth,
    crop_workspace,
    invert_transform,
    resize_intrinsics,
    transform_points,
    workspace_mask,
)


def test_resize_intrinsics_scales_focal_and_principal_point():
    intrinsics = np.array([[100.0, 0.0, 50.0], [0.0, 120.0, 60.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    scaled = resize_intrinsics(intrinsics, (100, 200), (50, 100))
    np.testing.assert_allclose(scaled[0, 0], 50.0)
    np.testing.assert_allclose(scaled[1, 1], 60.0)
    np.testing.assert_allclose(scaled[0, 2], 25.0)
    np.testing.assert_allclose(scaled[1, 2], 30.0)


def test_backproject_masked_depth_returns_expected_points():
    depth = np.array([[1.0, 0.0], [2.0, 3.0]], dtype=np.float32)
    mask = np.array([[True, False], [True, True]])
    intrinsics = np.array([[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)

    points = backproject_masked_depth(depth, intrinsics, mask, min_depth_m=0.1, max_depth_m=5.0)
    expected = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 2.0],
            [1.5, 1.5, 3.0],
        ],
        dtype=np.float32,
    )
    np.testing.assert_allclose(points, expected)


def test_transform_and_crop_workspace():
    points = np.array([[0.0, 0.0, 0.1], [0.2, 0.0, 0.2], [2.0, 2.0, 2.0]], dtype=np.float32)
    transform = np.eye(4, dtype=np.float32)
    transform[0, 3] = 0.1

    transformed = transform_points(points, transform)
    np.testing.assert_allclose(transformed[0], [0.1, 0.0, 0.1])

    cropped = crop_workspace(transformed, (0.0, 0.5, -0.5, 0.5, 0.0, 0.5))
    assert cropped.shape == (2, 3)

    mask = workspace_mask(transformed, (0.0, 0.5, -0.5, 0.5, 0.0, 0.5))
    np.testing.assert_array_equal(mask, [True, True, False])


def test_invert_transform_accepts_4x4_and_3x4():
    transform = np.eye(4, dtype=np.float32)
    transform[:3, 3] = [0.1, -0.2, 0.3]

    inv_4x4 = invert_transform(transform)
    inv_3x4 = invert_transform(transform[:3, :4])

    np.testing.assert_allclose(inv_4x4 @ transform, np.eye(4), atol=1e-6)
    np.testing.assert_allclose(inv_3x4, inv_4x4, atol=1e-6)
