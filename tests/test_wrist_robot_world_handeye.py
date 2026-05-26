import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest


cv2 = pytest.importorskip("cv2")

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "solve_wrist_robot_world_handeye.py"


def load_module():
    spec = importlib.util.spec_from_file_location("solve_wrist_robot_world_handeye", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def rot_xyz(rx: float, ry: float, rz: float) -> np.ndarray:
    def axis_rot(vec):
        rot, _ = cv2.Rodrigues(np.asarray(vec, dtype=np.float64))
        return rot

    return axis_rot([0.0, 0.0, rz]) @ axis_rot([0.0, ry, 0.0]) @ axis_rot([rx, 0.0, 0.0])


def assert_transform_close(actual: np.ndarray, expected: np.ndarray, *, trans_tol: float = 1e-5, rot_tol_deg: float = 1e-4):
    module = load_module()
    delta = actual @ module.invert_transform(expected)
    assert np.linalg.norm(delta[:3, 3]) < trans_tol
    assert module.rotation_angle_deg(delta[:3, :3]) < rot_tol_deg


def make_sample(module, name: str, t_camera_world: np.ndarray, t_base_gripper: np.ndarray) -> dict:
    t_gripper_base = module.invert_transform(t_base_gripper)
    return {
        "name": name,
        "T_camera_world": module.transform_to_record("T_camera_world", "camera", "world", t_camera_world),
        "T_base_gripper": module.transform_to_record("T_base_gripper", "base", "gripper", t_base_gripper),
        "T_gripper_base": module.transform_to_record("T_gripper_base", "gripper", "base", t_gripper_base),
        "quality": {
            "detected_markers": 12,
            "mean_reprojection_error_px": 0.4,
        },
    }


def test_robot_world_handeye_recovers_synthetic_transforms():
    module = load_module()
    t_world_base = module.make_transform(rot_xyz(0.18, -0.08, 0.42), np.asarray([0.22, -0.31, 0.04]))
    t_gripper_camera = module.make_transform(rot_xyz(-0.28, 0.12, 0.36), np.asarray([0.035, -0.018, 0.052]))
    t_camera_gripper = module.invert_transform(t_gripper_camera)
    t_base_world = module.invert_transform(t_world_base)
    samples = []

    for idx in range(24):
        phase = idx / 23.0
        rot = rot_xyz(
            -0.55 + 1.10 * phase,
            0.25 * math.sin(idx * 0.73),
            -0.45 + 0.90 * ((idx * 5) % 23) / 22.0,
        )
        trans = np.asarray(
            [
                0.18 + 0.05 * math.sin(idx * 0.41),
                -0.04 + 0.07 * math.cos(idx * 0.37),
                0.16 + 0.04 * math.sin(idx * 0.29),
            ],
            dtype=np.float64,
        )
        t_base_gripper = module.make_transform(rot, trans)
        t_camera_world = t_camera_gripper @ module.invert_transform(t_base_gripper) @ t_base_world
        samples.append(make_sample(module, f"sample_{idx:03d}", t_camera_world, t_base_gripper))

    solved_world_base, solved_camera_gripper, parsed = module.solve_handeye(samples)
    residuals = module.residuals_for_solution(parsed, solved_world_base, solved_camera_gripper)

    assert_transform_close(solved_world_base, t_world_base)
    assert_transform_close(module.invert_transform(solved_camera_gripper), t_gripper_camera)
    assert max(row["translation_error_m"] for row in residuals) < 1e-7


def test_filter_samples_rejects_bad_reprojection_and_marker_count():
    module = load_module()
    base = {
        "name": "good",
        "quality": {"detected_markers": 10, "mean_reprojection_error_px": 0.6},
    }
    bad_error = {
        "name": "bad_error",
        "quality": {"detected_markers": 10, "mean_reprojection_error_px": 9.0},
    }
    bad_markers = {
        "name": "bad_markers",
        "quality": {"detected_markers": 2, "mean_reprojection_error_px": 0.5},
    }

    kept, rejected = module.filter_samples([base, bad_error, bad_markers], min_markers=8, max_reproj_px=2.5)

    assert [row["name"] for row in kept] == ["good"]
    assert {row["name"] for row in rejected} == {"bad_error", "bad_markers"}
