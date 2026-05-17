#!/usr/bin/env python3
"""Record SO101 touch points and solve the calibration board pose in base frame."""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from datetime import datetime, timezone
from typing import Any

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML is installed in the target ROS env.
    yaml = None


POINT_LAYOUTS = {
    "tl_tr_bl_br": (
        (
            "top_left",
            "top-left OUTER checkerboard corner of the printed pattern, not the paper corner",
        ),
        (
            "top_right",
            "top-right OUTER checkerboard corner of the printed pattern",
        ),
        (
            "bottom_left",
            "bottom-left OUTER checkerboard corner of the printed pattern",
        ),
        (
            "bottom_right",
            "bottom-right OUTER checkerboard corner; this is a consistency check",
        ),
    ),
    "tl_bl_br": (
        (
            "top_left",
            "top-left OUTER checkerboard corner of the printed pattern, not the paper corner",
        ),
        (
            "bottom_left",
            "bottom-left OUTER checkerboard corner of the printed pattern",
        ),
        (
            "bottom_right",
            "bottom-right OUTER checkerboard corner of the printed pattern",
        ),
    ),
}


def parse_xyz(text: str) -> np.ndarray:
    values = [float(part) for part in text.replace(",", " ").split()]
    if len(values) != 3:
        raise argparse.ArgumentTypeError("expected three numbers, e.g. '0 0 0'")
    return np.array(values, dtype=float)


def quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0.0:
        raise ValueError("zero-length quaternion in TF")
    qx, qy, qz, qw = q / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
            ],
            [
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
            ],
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=float,
    )


def matrix_to_quat_xyzw(rot: np.ndarray) -> list[float]:
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rot[2, 1] - rot[1, 2]) / s
        qy = (rot[0, 2] - rot[2, 0]) / s
        qz = (rot[1, 0] - rot[0, 1]) / s
    else:
        diag = np.diag(rot)
        if diag[0] > diag[1] and diag[0] > diag[2]:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            qw = (rot[2, 1] - rot[1, 2]) / s
            qx = 0.25 * s
            qy = (rot[0, 1] + rot[1, 0]) / s
            qz = (rot[0, 2] + rot[2, 0]) / s
        elif diag[1] > diag[2]:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            qw = (rot[0, 2] - rot[2, 0]) / s
            qx = (rot[0, 1] + rot[1, 0]) / s
            qy = 0.25 * s
            qz = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            qw = (rot[1, 0] - rot[0, 1]) / s
            qx = (rot[0, 2] + rot[2, 0]) / s
            qy = (rot[1, 2] + rot[2, 1]) / s
            qz = 0.25 * s

    quat = np.array([qx, qy, qz, qw], dtype=float)
    quat /= np.linalg.norm(quat)
    return [float(v) for v in quat]


def wait_for_tool_point(
    tf_buffer: Buffer,
    base_frame: str,
    tool_frame: str,
    tool_offset: np.ndarray,
    timeout_s: float,
) -> np.ndarray:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            tf = tf_buffer.lookup_transform(base_frame, tool_frame, Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            rot = quat_to_matrix(q.x, q.y, q.z, q.w)
            return np.array([t.x, t.y, t.z], dtype=float) + rot @ tool_offset
        except (TransformException, ValueError) as exc:
            last_error = exc
            time.sleep(0.05)
    raise RuntimeError(
        f"no TF {base_frame} <- {tool_frame} after {timeout_s:.1f}s: {last_error}"
    )


def sample_tool_point(
    tf_buffer: Buffer,
    base_frame: str,
    tool_frame: str,
    tool_offset: np.ndarray,
    timeout_s: float,
    samples: int,
    sample_period_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    pts = []
    for _ in range(samples):
        pts.append(wait_for_tool_point(tf_buffer, base_frame, tool_frame, tool_offset, timeout_s))
        time.sleep(sample_period_s)
    arr = np.vstack(pts)
    return np.mean(arr, axis=0), np.std(arr, axis=0)


def normalize(vec: np.ndarray, name: str) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-6:
        raise RuntimeError(f"{name} vector is too small; repeated point?")
    return vec / norm


def solve_board_pose(
    points: dict[str, np.ndarray], width_m: float, height_m: float
) -> dict[str, Any]:
    origin = points["top_left"]
    y_raw = points["bottom_left"] - origin
    if "top_right" in points:
        x_raw = points["top_right"] - origin
        width_source = "top_left_to_top_right"
        check_point = points.get("bottom_right")
        check_name = "bottom_right"
    else:
        x_raw = points["bottom_right"] - points["bottom_left"]
        width_source = "bottom_left_to_bottom_right"
        check_point = None
        check_name = None

    x_axis = normalize(x_raw, "x")
    y_axis = y_raw - np.dot(y_raw, x_axis) * x_axis
    y_axis = normalize(y_axis, "y")
    z_axis = normalize(np.cross(x_axis, y_axis), "z")

    rot = np.column_stack((x_axis, y_axis, z_axis))
    if np.linalg.det(rot) < 0.0:
        raise RuntimeError("solved a left-handed board frame; point order is inconsistent")

    if check_point is not None:
        predicted_check = origin + width_m * x_axis + height_m * y_axis
        check_error = float(np.linalg.norm(check_point - predicted_check))
    else:
        predicted_check = None
        check_error = None
    measured_width = float(np.linalg.norm(x_raw))
    measured_height = float(np.linalg.norm(y_raw))
    raw_angle_deg = math.degrees(
        math.acos(float(np.clip(np.dot(normalize(x_raw, "x_raw"), normalize(y_raw, "y_raw")), -1.0, 1.0)))
    )

    return {
        "translation": origin,
        "rotation": rot,
        "quaternion_xyzw": matrix_to_quat_xyzw(rot),
        "measured_width_m": measured_width,
        "measured_height_m": measured_height,
        "width_source": width_source,
        "raw_xy_angle_deg": raw_angle_deg,
        "check_point_name": check_name,
        "check_point_predicted": predicted_check,
        "check_point_error_m": check_error,
    }


def to_plain(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return to_plain(value.tolist())
    if isinstance(value, dict):
        return {str(k): to_plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_plain(v) for v in value]
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.integer):
        return int(value)
    return value


def write_output(path: str, data: dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        if yaml is not None:
            yaml.safe_dump(to_plain(data), stream, sort_keys=False)
        else:
            import json

            json.dump(to_plain(data), stream, indent=2)
            stream.write("\n")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Touch board corners with the SO101 tool point and solve T_base_board."
    )
    parser.add_argument("--base-frame", default="follower/base_link")
    parser.add_argument("--tool-frame", default="follower/gripper_frame_link")
    parser.add_argument(
        "--tool-offset",
        type=parse_xyz,
        default=np.zeros(3, dtype=float),
        help="XYZ offset from tool_frame origin to the physical touch tip, expressed in tool_frame meters.",
    )
    parser.add_argument("--cols", type=int, default=11)
    parser.add_argument("--rows", type=int, default=8)
    parser.add_argument("--square-m", type=float, default=0.034)
    parser.add_argument("--marker-m", type=float, default=0.025)
    parser.add_argument("--start-id", type=int, default=2)
    parser.add_argument("--dictionary", default="DICT_5X5_100")
    parser.add_argument(
        "--corner-layout",
        choices=sorted(POINT_LAYOUTS.keys()),
        default="tl_tr_bl_br",
        help="Use tl_bl_br when top-right is unreachable; it solves x from the bottom edge.",
    )
    parser.add_argument("--timeout-s", type=float, default=5.0)
    parser.add_argument("--samples", type=int, default=7)
    parser.add_argument("--sample-period-s", type=float, default=0.04)
    parser.add_argument(
        "--min-point-separation-m",
        type=float,
        default=0.04,
        help="Reject a newly recorded point if it is closer than this to any earlier point.",
    )
    parser.add_argument(
        "--output",
        default="/home/dell/Documents/so101-ros-physical-ai/so101_bringup/config/cameras/extrinsics/board_in_base_touch.yaml",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    width_m = args.cols * args.square_m
    height_m = args.rows * args.square_m

    rclpy.init()
    node = Node("so101_board_touch_recorder")
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("")
    print("SO101 board touch recorder")
    print(f"TF: {args.base_frame} <- {args.tool_frame}")
    print(f"Tool offset in {args.tool_frame}: {args.tool_offset.tolist()} m")
    print(f"Board pattern: {width_m:.3f} m x {height_m:.3f} m ({args.cols} x {args.rows} squares)")
    print("")
    print("Use the OUTER checkerboard-pattern corners, not the A3 paper corners.")
    print("Keep the same physical touch point on the gripper for every sample.")
    print("Place the tool, hold it still, then press Enter in this terminal.")
    print("")

    points: dict[str, np.ndarray] = {}
    spreads: dict[str, np.ndarray] = {}
    try:
        wait_for_tool_point(tf_buffer, args.base_frame, args.tool_frame, args.tool_offset, args.timeout_s)
        for name, instruction in POINT_LAYOUTS[args.corner_layout]:
            while True:
                print(f"[{name}] {instruction}")
                input("Press Enter when the tool is touching that point...")
                point, spread = sample_tool_point(
                    tf_buffer=tf_buffer,
                    base_frame=args.base_frame,
                    tool_frame=args.tool_frame,
                    tool_offset=args.tool_offset,
                    timeout_s=args.timeout_s,
                    samples=args.samples,
                    sample_period_s=args.sample_period_s,
                )
                print(
                    f"  observed {point[0]: .4f} {point[1]: .4f} {point[2]: .4f} m "
                    f"(std mm: {(spread * 1000.0).round(2).tolist()})"
                )
                if not points:
                    break
                nearest_name, nearest_dist = min(
                    ((prev_name, float(np.linalg.norm(point - prev_point))) for prev_name, prev_point in points.items()),
                    key=lambda item: item[1],
                )
                if nearest_dist >= args.min_point_separation_m:
                    print(f"  distance from nearest previous point ({nearest_name}): {nearest_dist * 1000.0:.1f} mm")
                    break
                print(
                    f"  rejected: only {nearest_dist * 1000.0:.1f} mm from {nearest_name}. "
                    "Move the arm/tool to the requested corner and press Enter again."
                )
                print("")
            points[name] = point
            spreads[name] = spread
            print("  accepted")
            print("")

        solved = solve_board_pose(points, width_m, height_m)
        translation = solved["translation"]
        quat = solved["quaternion_xyzw"]

        output = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "transform": {
                "name": "T_base_board",
                "parent_frame": args.base_frame,
                "child_frame": "calibration_board",
                "translation_xyz": translation,
                "rotation_matrix": solved["rotation"],
                "quaternion_xyzw": quat,
            },
            "board": {
                "source": "caib.io ChArUco marker-board print",
                "dictionary": args.dictionary,
                "start_id": args.start_id,
                "cols": args.cols,
                "rows": args.rows,
                "square_m": args.square_m,
                "marker_m": args.marker_m,
                "width_m": width_m,
                "height_m": height_m,
                "corner_layout": args.corner_layout,
            },
            "capture": {
                "base_frame": args.base_frame,
                "tool_frame": args.tool_frame,
                "tool_offset_xyz_in_tool_frame": args.tool_offset,
                "samples_per_point": args.samples,
                "sample_period_s": args.sample_period_s,
            },
            "points_xyz_in_base": points,
            "point_std_xyz_m": spreads,
            "quality": {
                "measured_width_m": solved["measured_width_m"],
                "expected_width_m": width_m,
                "width_source": solved["width_source"],
                "measured_height_m": solved["measured_height_m"],
                "expected_height_m": height_m,
                "raw_xy_angle_deg": solved["raw_xy_angle_deg"],
                "check_point_name": solved["check_point_name"],
                "check_point_predicted_xyz": solved["check_point_predicted"],
                "check_point_error_m": solved["check_point_error_m"],
            },
        }
        write_output(args.output, output)

        print("Solved T_base_board")
        print(f"  translation xyz: {translation.tolist()}")
        print(f"  quaternion xyzw: {quat}")
        print(f"  measured width:  {solved['measured_width_m']:.4f} m, expected {width_m:.4f} m")
        print(f"  measured height: {solved['measured_height_m']:.4f} m, expected {height_m:.4f} m")
        print(f"  raw x/y angle:   {solved['raw_xy_angle_deg']:.2f} deg")
        if solved["check_point_error_m"] is not None:
            print(
                f"  {solved['check_point_name']} check error: "
                f"{solved['check_point_error_m'] * 1000.0:.1f} mm"
            )
        else:
            print("  independent corner check: skipped by selected layout")
        print(f"Saved: {args.output}")
        print("")
        print("Static TF command:")
        print(
            "ros2 run tf2_ros static_transform_publisher "
            f"--x {translation[0]:.9f} --y {translation[1]:.9f} --z {translation[2]:.9f} "
            f"--qx {quat[0]:.9f} --qy {quat[1]:.9f} --qz {quat[2]:.9f} --qw {quat[3]:.9f} "
            f"--frame-id {args.base_frame} --child-frame-id calibration_board"
        )
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; no complete transform written.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
