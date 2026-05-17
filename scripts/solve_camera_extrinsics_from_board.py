#!/usr/bin/env python3
"""Solve camera extrinsics from a caib.io marker board with known base pose."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import cv2.aruco as aruco
import numpy as np
import yaml


def _dictionary(name: str):
    attr = name if name.startswith("DICT_") else f"DICT_{name}"
    return aruco.getPredefinedDictionary(getattr(aruco, attr))


def _make_caib_marker_object_points(
    cols: int,
    rows: int,
    square_m: float,
    marker_m: float,
    start_id: int,
) -> dict[int, np.ndarray]:
    obj_by_id: dict[int, np.ndarray] = {}
    marker_id = start_id
    margin = 0.5 * (square_m - marker_m)
    for row in range(rows):
        marker_cols = range(0, cols, 2) if row % 2 == 0 else range(1, cols, 2)
        for col in marker_cols:
            x = col * square_m + margin
            y = row * square_m + margin
            obj_by_id[marker_id] = np.asarray(
                [
                    [x, y, 0.0],
                    [x + marker_m, y, 0.0],
                    [x + marker_m, y + marker_m, 0.0],
                    [x, y + marker_m, 0.0],
                ],
                dtype=np.float32,
            )
            marker_id += 1
    return obj_by_id


def _quat_to_matrix(q: list[float]) -> np.ndarray:
    qx, qy, qz, qw = np.asarray(q, dtype=np.float64)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError("zero-length quaternion")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.asarray(
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
        dtype=np.float64,
    )


def _matrix_to_quat_xyzw(rot: np.ndarray) -> list[float]:
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
    quat = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    quat /= np.linalg.norm(quat)
    return [float(v) for v in quat]


def _make_transform(rot: np.ndarray, translation: np.ndarray) -> np.ndarray:
    tf = np.eye(4, dtype=np.float64)
    tf[:3, :3] = rot
    tf[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return tf


def _invert_transform(tf: np.ndarray) -> np.ndarray:
    inv = np.eye(4, dtype=np.float64)
    rot = tf[:3, :3]
    inv[:3, :3] = rot.T
    inv[:3, 3] = -rot.T @ tf[:3, 3]
    return inv


def _to_plain(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return _to_plain(value.tolist())
    if isinstance(value, dict):
        return {str(k): _to_plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_plain(v) for v in value]
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    return value


def _load_camera_info(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    k = np.asarray(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    d = np.asarray(data["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1, 1)
    size = (int(data["image_width"]), int(data["image_height"]))
    return k, d, size


def _load_base_board(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    tf_data = data["transform"]
    if "rotation_matrix" in tf_data:
        rot = np.asarray(tf_data["rotation_matrix"], dtype=np.float64).reshape(3, 3)
    else:
        rot = _quat_to_matrix(tf_data["quaternion_xyzw"])
    trans = np.asarray(tf_data["translation_xyz"], dtype=np.float64)
    return _make_transform(rot, trans), data


def _lookup_ros_tf(parent_frame: str, child_frame: str, timeout_s: float) -> np.ndarray:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    node = Node("camera_extrinsics_tf_lookup")
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    deadline = time.monotonic() + timeout_s
    last_error = None
    try:
        while time.monotonic() < deadline:
            try:
                tf = tf_buffer.lookup_transform(parent_frame, child_frame, Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                return _make_transform(_quat_to_matrix([q.x, q.y, q.z, q.w]), np.asarray([t.x, t.y, t.z]))
            except (TransformException, ValueError) as exc:
                last_error = exc
                executor.spin_once(timeout_sec=0.05)
        raise RuntimeError(f"no TF {parent_frame} <- {child_frame} after {timeout_s:.1f}s: {last_error}")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def _capture_frame(args: argparse.Namespace) -> np.ndarray:
    if args.image:
        image = cv2.imread(str(args.image))
        if image is None:
            raise RuntimeError(f"Failed to read image: {args.image}")
        return image

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc[:4]))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open V4L2 device: {args.device}")
    try:
        frame = None
        deadline = time.monotonic() + args.capture_timeout_s
        while time.monotonic() < deadline:
            ok, frame = cap.read()
            if ok and frame is not None:
                break
            time.sleep(0.05)
        if frame is None:
            raise RuntimeError(f"No frame received from {args.device}")
        for _ in range(max(args.warmup_frames, 0)):
            ok, maybe = cap.read()
            if ok and maybe is not None:
                frame = maybe
        return frame
    finally:
        cap.release()


def _detect_board(
    image: np.ndarray,
    dictionary,
    obj_by_id: dict[int, np.ndarray],
    min_markers: int,
) -> tuple[np.ndarray, np.ndarray, list[np.ndarray], np.ndarray]:
    params = aruco.DetectorParameters_create()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        raise RuntimeError("No ArUco markers detected")

    object_points = []
    image_points = []
    kept_corners = []
    kept_ids = []
    for corner, marker_id in zip(corners, ids.ravel()):
        marker_id = int(marker_id)
        if marker_id not in obj_by_id:
            continue
        object_points.append(obj_by_id[marker_id])
        image_points.append(corner.reshape(4, 2).astype(np.float32))
        kept_corners.append(corner)
        kept_ids.append([marker_id])

    if len(kept_corners) < min_markers:
        raise RuntimeError(f"Need at least {min_markers} board markers, detected {len(kept_corners)}")

    return (
        np.concatenate(object_points, axis=0).astype(np.float32),
        np.concatenate(image_points, axis=0).astype(np.float32),
        kept_corners,
        np.asarray(kept_ids, dtype=np.int32),
    )


def _solve_pnp(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        iterationsCount=200,
        reprojectionError=4.0,
        confidence=0.999,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("solvePnPRansac failed")
    inlier_idx = inliers.reshape(-1) if inliers is not None else np.arange(len(object_points))
    if hasattr(cv2, "solvePnPRefineLM") and len(inlier_idx) >= 6:
        rvec, tvec = cv2.solvePnPRefineLM(
            object_points[inlier_idx],
            image_points[inlier_idx],
            camera_matrix,
            dist_coeffs,
            rvec,
            tvec,
        )
    return rvec, tvec, inlier_idx


def _reprojection_error(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> tuple[float, float]:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    diff = image_points - projected.reshape(-1, 2)
    errors = np.sqrt(np.sum(diff * diff, axis=1))
    return float(np.mean(errors)), float(np.max(errors))


def _write_overlay(
    path: Path,
    image: np.ndarray,
    corners: list[np.ndarray],
    ids: np.ndarray,
    object_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    label: str,
) -> None:
    overlay = image.copy()
    aruco.drawDetectedMarkers(overlay, corners, ids)
    axis = np.asarray([[0.0, 0.0, 0.0], [0.08, 0.0, 0.0], [0.0, 0.08, 0.0], [0.0, 0.0, -0.08]], dtype=np.float32)
    image_axis, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
    pts = image_axis.reshape(-1, 2).astype(int)
    cv2.line(overlay, tuple(pts[0]), tuple(pts[1]), (0, 0, 255), 3)
    cv2.line(overlay, tuple(pts[0]), tuple(pts[2]), (0, 255, 0), 3)
    cv2.line(overlay, tuple(pts[0]), tuple(pts[3]), (255, 0, 0), 3)
    cv2.putText(overlay, label, (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), overlay)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--image", type=Path)
    source.add_argument("--device", default="/dev/video2")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--warmup-frames", type=int, default=12)
    parser.add_argument("--capture-timeout-s", type=float, default=4.0)
    parser.add_argument("--camera-name", required=True)
    parser.add_argument("--camera-info", type=Path, required=True)
    parser.add_argument("--board-in-base", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--overlay-output", type=Path)
    parser.add_argument("--frame-output", type=Path)
    parser.add_argument("--parent-frame", default="follower/base_link")
    parser.add_argument(
        "--mount-parent-frame",
        help="Optional TF parent for an eye-in-hand/static mount transform, e.g. follower/base_link.",
    )
    parser.add_argument(
        "--mount-child-frame",
        help="Optional TF child for an eye-in-hand/static mount transform, e.g. follower/gripper_frame_link.",
    )
    parser.add_argument("--tf-timeout-s", type=float, default=5.0)
    parser.add_argument("--cols", type=int, default=11)
    parser.add_argument("--rows", type=int, default=8)
    parser.add_argument("--square-m", type=float, default=0.034)
    parser.add_argument("--marker-m", type=float, default=0.025)
    parser.add_argument("--start-id", type=int, default=2)
    parser.add_argument("--marker-count", type=int, default=44)
    parser.add_argument("--aruco-dict", default="DICT_5X5_100")
    parser.add_argument("--min-markers", type=int, default=8)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    camera_matrix, dist_coeffs, expected_size = _load_camera_info(args.camera_info)
    t_base_board, board_yaml = _load_base_board(args.board_in_base)
    obj_by_id = _make_caib_marker_object_points(
        args.cols,
        args.rows,
        args.square_m,
        args.marker_m,
        args.start_id,
    )

    image = _capture_frame(args)
    if args.frame_output:
        args.frame_output.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.frame_output), image)
    if (image.shape[1], image.shape[0]) != expected_size:
        print(
            f"WARN image size {image.shape[1]}x{image.shape[0]} differs from camera_info "
            f"{expected_size[0]}x{expected_size[1]}",
            flush=True,
        )

    dictionary = _dictionary(args.aruco_dict)
    object_points, image_points, kept_corners, kept_ids = _detect_board(
        image,
        dictionary,
        obj_by_id,
        args.min_markers,
    )
    rvec, tvec, inlier_idx = _solve_pnp(object_points, image_points, camera_matrix, dist_coeffs)
    mean_err, max_err = _reprojection_error(
        object_points[inlier_idx],
        image_points[inlier_idx],
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
    )
    r_cam_board, _ = cv2.Rodrigues(rvec)
    t_cam_board = _make_transform(r_cam_board, tvec.reshape(3))
    t_base_camera = t_base_board @ _invert_transform(t_cam_board)
    t_camera_base = _invert_transform(t_base_camera)
    quat_base_camera = _matrix_to_quat_xyzw(t_base_camera[:3, :3])
    mount_transform = None
    if bool(args.mount_parent_frame) != bool(args.mount_child_frame):
        raise RuntimeError("--mount-parent-frame and --mount-child-frame must be provided together")
    if args.mount_parent_frame and args.mount_child_frame:
        t_mount_parent_mount_child = _lookup_ros_tf(
            args.mount_parent_frame,
            args.mount_child_frame,
            args.tf_timeout_s,
        )
        t_mount_child_camera = _invert_transform(t_mount_parent_mount_child) @ t_base_camera
        mount_transform = {
            "name": "T_mount_camera",
            "parent_frame": args.mount_child_frame,
            "child_frame": args.camera_name,
            "translation_xyz": t_mount_child_camera[:3, 3],
            "rotation_matrix": t_mount_child_camera[:3, :3],
            "quaternion_xyzw": _matrix_to_quat_xyzw(t_mount_child_camera[:3, :3]),
            "source_tf": {
                "name": "T_mount_parent_mount_child",
                "parent_frame": args.mount_parent_frame,
                "child_frame": args.mount_child_frame,
                "translation_xyz": t_mount_parent_mount_child[:3, 3],
                "rotation_matrix": t_mount_parent_mount_child[:3, :3],
                "quaternion_xyzw": _matrix_to_quat_xyzw(t_mount_parent_mount_child[:3, :3]),
            },
        }

    overlay_output = args.overlay_output or args.output.with_suffix(".overlay.jpg")
    _write_overlay(
        overlay_output,
        image,
        kept_corners,
        kept_ids,
        object_points,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
        f"{args.camera_name}: {len(kept_corners)} markers, err {mean_err:.2f}px",
    )

    result = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "camera_name": args.camera_name,
        "transform": {
            "name": "T_base_camera",
            "parent_frame": args.parent_frame,
            "child_frame": args.camera_name,
            "translation_xyz": t_base_camera[:3, 3],
            "rotation_matrix": t_base_camera[:3, :3],
            "quaternion_xyzw": quat_base_camera,
        },
        "inverse_transform": {
            "name": "T_camera_base",
            "parent_frame": args.camera_name,
            "child_frame": args.parent_frame,
            "translation_xyz": t_camera_base[:3, 3],
            "rotation_matrix": t_camera_base[:3, :3],
            "quaternion_xyzw": _matrix_to_quat_xyzw(t_camera_base[:3, :3]),
        },
        "camera_to_board_pnp": {
            "translation_xyz": tvec.reshape(3),
            "rotation_matrix": r_cam_board,
            "quaternion_xyzw": _matrix_to_quat_xyzw(r_cam_board),
        },
        "mount_transform": mount_transform,
        "quality": {
            "detected_markers": int(len(kept_corners)),
            "inlier_points": int(len(inlier_idx)),
            "total_points": int(len(object_points)),
            "mean_reprojection_error_px": mean_err,
            "max_reprojection_error_px": max_err,
            "image_width": int(image.shape[1]),
            "image_height": int(image.shape[0]),
            "overlay_output": str(overlay_output),
            "frame_output": str(args.frame_output) if args.frame_output else None,
        },
        "inputs": {
            "camera_info": str(args.camera_info),
            "board_in_base": str(args.board_in_base),
            "board_touch_quality": board_yaml.get("quality", {}),
        },
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(_to_plain(result), sort_keys=False), encoding="utf-8")

    print(f"{args.camera_name}: markers={len(kept_corners)} inlier_points={len(inlier_idx)}/{len(object_points)}")
    print(f"  reprojection mean={mean_err:.3f}px max={max_err:.3f}px")
    print(f"  T_base_camera xyz={t_base_camera[:3, 3].tolist()}")
    print(f"  T_base_camera qxyzw={quat_base_camera}")
    print(f"  saved {args.output}")
    print(f"  overlay {overlay_output}")
    if mount_transform is not None:
        mount_q = mount_transform["quaternion_xyzw"]
        mount_t = mount_transform["translation_xyz"]
        print(f"  T_{args.mount_child_frame}_camera xyz={mount_t.tolist()}")
        print(f"  T_{args.mount_child_frame}_camera qxyzw={mount_q}")
        print("Mounted static TF command:")
        print(
            "ros2 run tf2_ros static_transform_publisher "
            f"--x {mount_t[0]:.9f} --y {mount_t[1]:.9f} --z {mount_t[2]:.9f} "
            f"--qx {mount_q[0]:.9f} --qy {mount_q[1]:.9f} "
            f"--qz {mount_q[2]:.9f} --qw {mount_q[3]:.9f} "
            f"--frame-id {args.mount_child_frame} --child-frame-id {args.camera_name}"
        )
    print("Static TF command:")
    print(
        "ros2 run tf2_ros static_transform_publisher "
        f"--x {t_base_camera[0, 3]:.9f} --y {t_base_camera[1, 3]:.9f} --z {t_base_camera[2, 3]:.9f} "
        f"--qx {quat_base_camera[0]:.9f} --qy {quat_base_camera[1]:.9f} "
        f"--qz {quat_base_camera[2]:.9f} --qw {quat_base_camera[3]:.9f} "
        f"--frame-id {args.parent_frame} --child-frame-id {args.camera_name}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
