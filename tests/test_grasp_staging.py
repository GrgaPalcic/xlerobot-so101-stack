import numpy as np

from so101_grasping.grasp_staging import (
    point_with_min_surface_clearance,
    signed_plane_distance,
    surface_relative_stages,
)


def test_surface_relative_close_does_not_use_fixed_base_z_floor():
    target = np.array([0.18, 0.25, -0.027], dtype=float)
    plane_point = np.array([0.0, 0.0, -0.030], dtype=float)
    plane_normal = np.array([0.0, 0.0, 1.0], dtype=float)

    stages = surface_relative_stages(
        target,
        plane_point,
        plane_normal,
        ready_clearance_m=0.12,
        pregrasp_clearance_m=0.065,
        close_clearance_m=0.0,
        close_surface_clearance_m=0.003,
    )

    np.testing.assert_allclose(stages.close, target)
    assert np.isclose(signed_plane_distance(stages.close, plane_point, plane_normal), 0.003)


def test_surface_relative_close_raises_below_plane_target_only_to_clearance():
    target = np.array([0.18, 0.25, -0.040], dtype=float)
    plane_point = np.array([0.0, 0.0, -0.030], dtype=float)
    plane_normal = np.array([0.0, 0.0, 1.0], dtype=float)

    close, clearance = point_with_min_surface_clearance(
        target,
        plane_point,
        plane_normal,
        min_clearance_m=0.003,
    )

    np.testing.assert_allclose(close, [0.18, 0.25, -0.027])
    assert np.isclose(clearance, 0.003)


def test_surface_relative_ready_and_pregrasp_follow_plane_normal():
    target = np.array([0.18, 0.25, 0.01], dtype=float)
    plane_point = np.zeros(3, dtype=float)
    plane_normal = np.array([1.0, 0.0, 1.0], dtype=float)

    stages = surface_relative_stages(
        target,
        plane_point,
        plane_normal,
        ready_clearance_m=0.12,
        pregrasp_clearance_m=0.065,
        close_clearance_m=0.0,
        close_surface_clearance_m=0.003,
    )
    normal = plane_normal / np.linalg.norm(plane_normal)

    np.testing.assert_allclose(stages.pregrasp - stages.close, normal * 0.065)
    np.testing.assert_allclose(stages.ready - stages.close, normal * 0.12)
