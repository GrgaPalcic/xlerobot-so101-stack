#!/usr/bin/env python3
"""Read-only SO-101 joint, TF, camera, and grasp snapshot audit.

This script never publishes commands. It only reads joint states, TF, optional
calibration YAML files, and optional saved grasp stack snapshots.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

try:
    import yaml
except ImportError:  # pragma: no cover - target ROS env should have python3-yaml.
    yaml = None


DEFAULT_POSES = (
    ("P0_neutral_reference", "Place the arm in the best physical neutral/upright reference pose.", ""),
    ("P1_shoulder_pan", "Move only shoulder_pan clearly left/right from P0.", "shoulder_pan"),
    ("P2_shoulder_lift", "Return near P0, then move only shoulder_lift clearly.", "shoulder_lift"),
    ("P3_elbow_flex", "Return near P0, then move only elbow_flex clearly.", "elbow_flex"),
    ("P4_wrist_flex", "Return near P0, then move only wrist_flex clearly.", "wrist_flex"),
    ("P5_wrist_roll", "Return near P0, then rotate only wrist_roll clearly.", "wrist_roll"),
)

LAYER_EXPLANATIONS = {
    "input_overhead.jpg": "raw overhead camera frame",
    "input_wrist.jpg": "raw wrist camera frame",
    "overhead_mask.png": "GroundedSAM mask from overhead image",
    "wrist_mask.png": "GroundedSAM mask from wrist image",
    "overhead_depth.png": "Depth Anything 3 visualization for overhead",
    "wrist_depth.png": "Depth Anything 3 visualization for wrist",
    "overhead_masked_depth.png": "overhead depth visualization clipped by mask",
    "wrist_masked_depth.png": "wrist depth visualization clipped by mask",
    "ggcnn_quality.png": "2D GG-CNN grasp quality map",
    "ggcnn_angle.png": "2D GG-CNN grasp angle map",
    "ggcnn_depth_input.png": "depth crop fed to GG-CNN",
    "ggcnn_overlay.png": "GG-CNN grasp overlay; not a reachability proof",
    "snapshot.json": "numeric grasp candidates, TF poses, service result",
    "summary.txt": "human-readable top-level snapshot summary",
    "clouds.npz": "metric point clouds in the configured base frame",
}


@dataclass
class TransformRecord:
    parent_frame: str
    child_frame: str
    translation_xyz: list[float]
    quaternion_xyzw: list[float]


class AuditNode(Node):
    def __init__(self, joint_states_topic: str) -> None:
        super().__init__("so101_calibration_audit")
        self.latest_joint_state: JointState | None = None
        self.create_subscription(
            JointState,
            joint_states_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

    def _on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def wait_joint_state(self, timeout_s: float) -> JointState:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_joint_state is not None:
                return self.latest_joint_state
        raise RuntimeError(f"no joint state received after {timeout_s:.1f}s")

    def lookup_transform(self, parent_frame: str, child_frame: str, timeout_s: float) -> TransformRecord:
        deadline = time.monotonic() + timeout_s
        last_error: Exception | None = None
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent_frame,
                    child_frame,
                    Time(),
                    timeout=Duration(seconds=0.15),
                )
                t = transform.transform.translation
                q = transform.transform.rotation
                return TransformRecord(
                    parent_frame=parent_frame,
                    child_frame=child_frame,
                    translation_xyz=[float(t.x), float(t.y), float(t.z)],
                    quaternion_xyzw=normalize_quat([float(q.x), float(q.y), float(q.z), float(q.w)]),
                )
            except TransformException as exc:
                last_error = exc
                rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f"no TF {parent_frame} <- {child_frame} after {timeout_s:.1f}s: {last_error}")


def normalize_quat(quat: list[float] | np.ndarray) -> list[float]:
    arr = np.asarray(quat, dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-12:
        raise ValueError("zero-length quaternion")
    return [float(v) for v in arr / norm]


def quat_angle_error_deg(a: list[float], b: list[float]) -> float:
    qa = np.asarray(normalize_quat(a), dtype=float)
    qb = np.asarray(normalize_quat(b), dtype=float)
    dot = float(abs(np.dot(qa, qb)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def joint_state_to_dict(msg: JointState) -> dict[str, float]:
    return {name: float(position) for name, position in zip(msg.name, msg.position, strict=False)}


def transform_to_plain(transform: TransformRecord) -> dict[str, Any]:
    return {
        "parent_frame": transform.parent_frame,
        "child_frame": transform.child_frame,
        "translation_xyz": transform.translation_xyz,
        "quaternion_xyzw": transform.quaternion_xyzw,
    }


def load_expected_transform(path: Path, key: str) -> TransformRecord:
    if yaml is None:
        raise RuntimeError("PyYAML is not installed; cannot read calibration YAML")
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    transform = data.get(key)
    if not isinstance(transform, dict):
        raise RuntimeError(f"{path} does not contain transform key {key!r}")
    return TransformRecord(
        parent_frame=str(transform["parent_frame"]),
        child_frame=str(transform["child_frame"]),
        translation_xyz=[float(v) for v in transform["translation_xyz"]],
        quaternion_xyzw=normalize_quat(transform["quaternion_xyzw"]),
    )


def compare_transform(live: TransformRecord, expected: TransformRecord) -> dict[str, Any]:
    live_t = np.asarray(live.translation_xyz, dtype=float)
    expected_t = np.asarray(expected.translation_xyz, dtype=float)
    return {
        "live": transform_to_plain(live),
        "expected": transform_to_plain(expected),
        "translation_error_m": float(np.linalg.norm(live_t - expected_t)),
        "rotation_error_deg": float(quat_angle_error_deg(live.quaternion_xyzw, expected.quaternion_xyzw)),
    }


def format_xyz(values: list[float] | np.ndarray) -> str:
    arr = np.asarray(values, dtype=float)
    return f"({arr[0]: .4f}, {arr[1]: .4f}, {arr[2]: .4f})"


def summarize_transform_compare(name: str, result: dict[str, Any], max_translation_m: float, max_rotation_deg: float) -> str:
    t_err = result["translation_error_m"]
    r_err = result["rotation_error_deg"]
    status = "PASS" if t_err <= max_translation_m and r_err <= max_rotation_deg else "FAIL"
    return (
        f"{status} {name}: translation_error={t_err:.4f} m, "
        f"rotation_error={r_err:.2f} deg"
    )


def sample_pose(
    node: AuditNode,
    args: argparse.Namespace,
    pose_name: str,
    expected_joint: str,
) -> dict[str, Any]:
    joint_samples: list[dict[str, float]] = []
    gripper_samples: list[TransformRecord] = []

    for _ in range(args.samples):
        joint_samples.append(joint_state_to_dict(node.wait_joint_state(args.timeout_s)))
        gripper_samples.append(node.lookup_transform(args.base_frame, args.gripper_frame, args.timeout_s))
        time.sleep(args.sample_period_s)

    names = sorted(joint_samples[-1])
    mean_joints = {
        name: float(np.mean([sample[name] for sample in joint_samples if name in sample]))
        for name in names
    }
    std_joints = {
        name: float(np.std([sample[name] for sample in joint_samples if name in sample]))
        for name in names
    }
    mean_gripper_xyz = np.mean([sample.translation_xyz for sample in gripper_samples], axis=0)
    std_gripper_xyz = np.std([sample.translation_xyz for sample in gripper_samples], axis=0)

    return {
        "name": pose_name,
        "expected_joint": expected_joint,
        "joint_mean_rad": mean_joints,
        "joint_std_rad": std_joints,
        "gripper_tf": {
            "parent_frame": args.base_frame,
            "child_frame": args.gripper_frame,
            "translation_xyz_mean": [float(v) for v in mean_gripper_xyz],
            "translation_xyz_std": [float(v) for v in std_gripper_xyz],
            "last_quaternion_xyzw": gripper_samples[-1].quaternion_xyzw,
        },
    }


def analyze_manual_poses(poses: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not poses:
        return []
    reference = poses[0]["joint_mean_rad"]
    analysis = []
    for pose in poses[1:]:
        current = pose["joint_mean_rad"]
        deltas = {
            name: float(current[name] - reference[name])
            for name in sorted(current)
            if name in reference
        }
        dominant_joint = max(deltas, key=lambda name: abs(deltas[name])) if deltas else ""
        expected_joint = pose["expected_joint"]
        analysis.append(
            {
                "pose": pose["name"],
                "expected_joint": expected_joint,
                "dominant_joint_from_P0": dominant_joint,
                "dominant_delta_rad": float(deltas.get(dominant_joint, 0.0)),
                "expected_delta_rad": float(deltas.get(expected_joint, 0.0)) if expected_joint else 0.0,
                "pass_expected_dominant": bool(expected_joint and dominant_joint == expected_joint),
                "all_deltas_rad": deltas,
            }
        )
    return analysis


def cloud_stats(points: np.ndarray) -> dict[str, Any]:
    if points.size == 0:
        return {
            "count": 0,
            "min_xyz": None,
            "max_xyz": None,
            "median_xyz": None,
            "centroid_xyz": None,
        }
    return {
        "count": int(points.shape[0]),
        "min_xyz": [float(v) for v in np.min(points, axis=0)],
        "max_xyz": [float(v) for v in np.max(points, axis=0)],
        "median_xyz": [float(v) for v in np.median(points, axis=0)],
        "centroid_xyz": [float(v) for v in np.mean(points, axis=0)],
    }


def summarize_snapshot(snapshot_dir: Path) -> dict[str, Any]:
    snapshot_json = snapshot_dir / "snapshot.json"
    clouds_npz = snapshot_dir / "clouds.npz"
    if not snapshot_json.exists():
        raise RuntimeError(f"missing {snapshot_json}")

    metadata = json.loads(snapshot_json.read_text(encoding="utf-8"))
    clouds: dict[str, Any] = {}
    if clouds_npz.exists():
        with np.load(clouds_npz) as loaded:
            for name in sorted(loaded.files):
                clouds[name] = cloud_stats(np.asarray(loaded[name], dtype=float))

    top_grasp = metadata.get("grasps", [{}])[0] if metadata.get("grasps") else {}
    gripper_pose = metadata.get("poses", {}).get("follower/gripper_frame_link", {})
    distances: dict[str, float] = {}
    if top_grasp.get("position") and gripper_pose.get("position"):
        grasp_xyz = np.asarray(top_grasp["position"], dtype=float)
        gripper_xyz = np.asarray(gripper_pose["position"], dtype=float)
        distances = {
            "gripper_to_top_grasp_xyz_m": float(np.linalg.norm(gripper_xyz - grasp_xyz)),
            "gripper_to_top_grasp_xy_m": float(np.linalg.norm(gripper_xyz[:2] - grasp_xyz[:2])),
            "gripper_minus_top_grasp_z_m": float(gripper_xyz[2] - grasp_xyz[2]),
        }

    present_layers = {
        path.name: LAYER_EXPLANATIONS.get(path.name, "stack artifact")
        for path in sorted(snapshot_dir.iterdir())
        if path.is_file()
    }

    warnings: list[str] = []
    wrist_count = clouds.get("wrist_object_cloud", {}).get("count")
    object_count = clouds.get("object_cloud", {}).get("count")
    if wrist_count == 0:
        warnings.append("wrist_object_cloud is empty; do not execute wrist-confirmed grasps")
    if object_count == 0:
        warnings.append("object_cloud is empty; metric grasp coordinates are not usable")
    if distances.get("gripper_to_top_grasp_xy_m", 0.0) > 0.08:
        warnings.append("top grasp is far from current gripper XY; check base/camera TF or plan stage")

    return {
        "snapshot_dir": str(snapshot_dir),
        "metadata": metadata,
        "cloud_stats": clouds,
        "top_grasp": top_grasp,
        "gripper_to_top_grasp": distances,
        "present_layers": present_layers,
        "warnings": warnings,
    }


def write_report(result: dict[str, Any], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "audit.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# SO-101 Calibration Audit",
        "",
        f"timestamp_utc: {result['timestamp_utc']}",
        "",
        "## Camera TF Checks",
    ]
    for name, check in result.get("camera_tf_checks", {}).items():
        if "error" in check:
            lines.append(f"- FAIL {name}: {check['error']}")
        else:
            lines.append(
                "- "
                + summarize_transform_compare(
                    name,
                    check,
                    result["thresholds"]["max_camera_translation_error_m"],
                    result["thresholds"]["max_camera_rotation_error_deg"],
                )
            )

    lines.extend(["", "## Manual Pose Samples"])
    for pose in result.get("manual_poses", []):
        xyz = pose["gripper_tf"]["translation_xyz_mean"]
        std = pose["gripper_tf"]["translation_xyz_std"]
        lines.append(f"- {pose['name']}: gripper_xyz={format_xyz(xyz)} std={format_xyz(std)}")

    analysis = result.get("manual_pose_analysis", [])
    if analysis:
        lines.extend(["", "## Joint Mapping From P0"])
        for item in analysis:
            status = "PASS" if item["pass_expected_dominant"] else "CHECK"
            lines.append(
                f"- {status} {item['pose']}: expected={item['expected_joint']} "
                f"dominant={item['dominant_joint_from_P0']} "
                f"dominant_delta={item['dominant_delta_rad']:.4f} rad"
            )

    snapshot = result.get("snapshot")
    if snapshot:
        lines.extend(["", "## Stack Snapshot"])
        meta = snapshot.get("metadata", {})
        lines.append(f"- prompt: {meta.get('prompt', '')}")
        lines.append(f"- success: {meta.get('success', '')}")
        lines.append(f"- num_grasps: {meta.get('num_grasps', '')}")
        for name, stats in snapshot.get("cloud_stats", {}).items():
            lines.append(
                f"- cloud {name}: count={stats['count']} "
                f"median={format_xyz(stats['median_xyz']) if stats['median_xyz'] else 'none'}"
            )
        for warning in snapshot.get("warnings", []):
            lines.append(f"- WARNING {warning}")
        lines.extend(["", "## Snapshot Layer Meanings"])
        for name, explanation in snapshot.get("present_layers", {}).items():
            lines.append(f"- {name}: {explanation}")

    (out_dir / "audit.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_camera_tf_checks(node: AuditNode, args: argparse.Namespace) -> dict[str, Any]:
    checks: dict[str, Any] = {}
    expected_specs = [
        (
            "overhead_base_to_optical",
            Path(args.overhead_extrinsics),
            "transform",
        ),
        (
            "wrist_gripper_to_optical",
            Path(args.wrist_extrinsics),
            "mount_transform",
        ),
    ]
    for name, path, key in expected_specs:
        try:
            expected = load_expected_transform(path, key)
            live = node.lookup_transform(expected.parent_frame, expected.child_frame, args.timeout_s)
            checks[name] = compare_transform(live, expected)
        except Exception as exc:  # noqa: BLE001 - report should preserve every failure.
            checks[name] = {"error": str(exc)}
    return checks


def run_manual_pose_audit(node: AuditNode, args: argparse.Namespace) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    poses: list[dict[str, Any]] = []
    for pose_name, instruction, expected_joint in DEFAULT_POSES:
        print(f"\n{pose_name}")
        print(instruction)
        print("Press Enter after the arm is stable. This script will only read state.")
        if not args.non_interactive:
            input("> ")
        pose = sample_pose(node, args, pose_name, expected_joint)
        poses.append(pose)
        joints = pose["joint_mean_rad"]
        print(f"gripper_xyz: {format_xyz(pose['gripper_tf']['translation_xyz_mean'])}")
        for name in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"):
            if name in joints:
                print(f"  {name}: {joints[name]: .4f} rad ({math.degrees(joints[name]): .1f} deg)")
    return poses, analyze_manual_poses(poses)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only SO-101 calibration audit. Does not command motors.",
    )
    parser.add_argument("--joint-states-topic", default="/follower/joint_states")
    parser.add_argument("--base-frame", default="follower/base_link")
    parser.add_argument("--gripper-frame", default="follower/gripper_frame_link")
    parser.add_argument(
        "--overhead-extrinsics",
        default="so101_bringup/config/cameras/extrinsics/cam_overhead_in_base_from_board.yaml",
    )
    parser.add_argument(
        "--wrist-extrinsics",
        default="so101_bringup/config/cameras/extrinsics/cam_wrist_eye_in_hand_from_board.yaml",
    )
    parser.add_argument("--snapshot-dir", default="", help="Optional captured stack snapshot to summarize")
    parser.add_argument("--out-dir", default="", help="Default: /tmp/so101_calibration_audit_<timestamp>")
    parser.add_argument("--timeout-s", type=float, default=3.0)
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--sample-period-s", type=float, default=0.05)
    parser.add_argument("--skip-manual-poses", action="store_true")
    parser.add_argument("--non-interactive", action="store_true")
    parser.add_argument("--max-camera-translation-error-m", type=float, default=0.02)
    parser.add_argument("--max-camera-rotation-error-deg", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = Path(args.out_dir) if args.out_dir else Path(f"/tmp/so101_calibration_audit_{timestamp}")

    rclpy.init()
    node = AuditNode(args.joint_states_topic)
    try:
        result: dict[str, Any] = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "thresholds": {
                "max_camera_translation_error_m": args.max_camera_translation_error_m,
                "max_camera_rotation_error_deg": args.max_camera_rotation_error_deg,
            },
            "camera_tf_checks": run_camera_tf_checks(node, args),
        }

        for name, check in result["camera_tf_checks"].items():
            if "error" in check:
                print(f"FAIL {name}: {check['error']}")
            else:
                print(
                    summarize_transform_compare(
                        name,
                        check,
                        args.max_camera_translation_error_m,
                        args.max_camera_rotation_error_deg,
                    )
                )

        if not args.skip_manual_poses:
            poses, analysis = run_manual_pose_audit(node, args)
            result["manual_poses"] = poses
            result["manual_pose_analysis"] = analysis
        else:
            result["manual_poses"] = []
            result["manual_pose_analysis"] = []

        if args.snapshot_dir:
            result["snapshot"] = summarize_snapshot(Path(args.snapshot_dir))

        write_report(result, out_dir)
        print(f"\nWrote {out_dir / 'audit.md'}")
        print(f"Wrote {out_dir / 'audit.json'}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
