import numpy as np

from grasp_server.pipeline import (
    _backproject_pixel_to_base,
    _crop_pixel_to_image_pixel,
    _grasp_width_pixels_to_m,
    _image_angle_to_base_yaw,
    _square_crop_for_mask,
)


def test_square_crop_for_mask_keeps_mask_centered():
    mask = np.zeros((100, 120), dtype=bool)
    mask[40:50, 60:80] = True

    top, left, bottom, right = _square_crop_for_mask(mask, padding=2.0)

    assert bottom - top == right - left
    assert top <= 40
    assert left <= 60
    assert bottom >= 50
    assert right >= 80


def test_crop_pixel_to_image_pixel_maps_center():
    row, col = _crop_pixel_to_image_pixel(149.5, 149.5, (10, 20, 310, 320), 300)

    np.testing.assert_allclose([row, col], [159.5, 169.5])


def test_backproject_pixel_to_base_identity():
    intrinsics = np.array([[100.0, 0.0, 50.0], [0.0, 100.0, 60.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    transform = np.eye(4, dtype=np.float32)

    point = _backproject_pixel_to_base(60.0, 50.0, 0.5, intrinsics, transform)

    np.testing.assert_allclose(point, [0.0, 0.0, 0.5], atol=1e-6)


def test_image_angle_and_width_convert_in_base_frame():
    intrinsics = np.array([[100.0, 0.0, 50.0], [0.0, 100.0, 60.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    transform = np.eye(4, dtype=np.float32)

    yaw = _image_angle_to_base_yaw(60.0, 50.0, 0.5, 0.0, intrinsics, transform)
    width = _grasp_width_pixels_to_m(60.0, 50.0, 0.5, 0.0, 20.0, intrinsics, transform)

    np.testing.assert_allclose(yaw, 0.0, atol=1e-6)
    np.testing.assert_allclose(width, 0.1, atol=1e-6)
