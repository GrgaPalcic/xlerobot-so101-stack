import numpy as np

from so101_grasping.wrist_views import upward_view_axis, wrist_view_candidates


def test_upward_view_axis_flips_downward_plane_normal():
    axis = upward_view_axis(np.array([0.0, 0.0, -1.0]))

    np.testing.assert_allclose(axis, [0.0, 0.0, 1.0])


def test_wrist_view_candidates_start_with_centered_view():
    candidates = wrist_view_candidates(
        target=np.array([0.20, 0.30, 0.0]),
        wrist_camera_xyz_in_ee=np.zeros(3),
        wrist_camera_quat_xyzw_in_ee=np.array([0.0, 0.0, 0.0, 1.0]),
        standoffs_m=[0.20],
        lateral_offsets_m=[0.0, 0.04],
        min_ee_z_m=0.05,
    )

    assert candidates[0].label == "s200mm_center"
    np.testing.assert_allclose(candidates[0].camera_position, [0.20, 0.30, 0.20])
    np.testing.assert_allclose(candidates[0].target_in_camera[:2], [0.0, 0.0], atol=1e-8)
    assert candidates[0].target_in_camera[2] > 0.19
    assert len(candidates) == 5


def test_wrist_view_candidates_clamp_end_effector_height():
    candidates = wrist_view_candidates(
        target=np.array([0.20, 0.30, -0.10]),
        wrist_camera_xyz_in_ee=np.zeros(3),
        wrist_camera_quat_xyzw_in_ee=np.array([0.0, 0.0, 0.0, 1.0]),
        standoffs_m=[0.05],
        lateral_offsets_m=[0.0],
        min_ee_z_m=0.02,
    )

    assert candidates[0].ee_position[2] == 0.02
