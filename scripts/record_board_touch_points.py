#!/usr/bin/env python3
"""Record SO101 touch points and solve the calibration board pose in base frame."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

from so101_kinematics_msgs.srv import GoToPose

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

JOG_DELTAS = {
    "x+": np.array([1.0, 0.0, 0.0], dtype=float),
    "x-": np.array([-1.0, 0.0, 0.0], dtype=float),
    "y+": np.array([0.0, 1.0, 0.0], dtype=float),
    "y-": np.array([0.0, -1.0, 0.0], dtype=float),
    "z+": np.array([0.0, 0.0, 1.0], dtype=float),
    "z-": np.array([0.0, 0.0, -1.0], dtype=float),
}

ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

JOINT_JOG_ALIASES = {
    "pan": "shoulder_pan",
    "j1": "shoulder_pan",
    "lift": "shoulder_lift",
    "shoulder": "shoulder_lift",
    "j2": "shoulder_lift",
    "elbow": "elbow_flex",
    "j3": "elbow_flex",
    "wrist": "wrist_flex",
    "flex": "wrist_flex",
    "j4": "wrist_flex",
    "roll": "wrist_roll",
    "j5": "wrist_roll",
}


@dataclass(frozen=True)
class JogCommand:
    kind: str
    delta_axis: np.ndarray | None = None
    joint_name: str | None = None
    joint_sign: float | None = None
    value: float | None = None


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


def pose_stamped_from_matrix(T: np.ndarray, frame_id: str, node: Node) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(T[0, 3])
    msg.pose.position.y = float(T[1, 3])
    msg.pose.position.z = float(T[2, 3])
    qx, qy, qz, qw = matrix_to_quat_xyzw(T[:3, :3])
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def wait_for_tool_pose(
    tf_buffer: Buffer,
    base_frame: str,
    tool_frame: str,
    timeout_s: float,
) -> np.ndarray:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            tf = tf_buffer.lookup_transform(base_frame, tool_frame, Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            T = np.eye(4, dtype=float)
            T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
            T[:3, 3] = np.array([t.x, t.y, t.z], dtype=float)
            return T
        except (TransformException, ValueError) as exc:
            last_error = exc
            time.sleep(0.05)
    raise RuntimeError(
        f"no TF {base_frame} <- {tool_frame} after {timeout_s:.1f}s: {last_error}"
    )


def wait_for_tool_point(
    tf_buffer: Buffer,
    base_frame: str,
    tool_frame: str,
    tool_offset: np.ndarray,
    timeout_s: float,
) -> np.ndarray:
    T = wait_for_tool_pose(tf_buffer, base_frame, tool_frame, timeout_s)
    return T[:3, 3] + T[:3, :3] @ tool_offset


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


def parse_jog_command(text: str) -> JogCommand:
    command = text.strip().lower()
    if command in ("", "s", "sample", "record"):
        return JogCommand("sample")
    if command in ("q", "quit", "exit"):
        return JogCommand("quit")
    if command in ("h", "help", "?"):
        return JogCommand("help")
    if command == "pose":
        return JogCommand("pose")
    if command in JOG_DELTAS:
        return JogCommand("move", delta_axis=JOG_DELTAS[command])
    if len(command) > 1 and command[-1] in "+-":
        joint_key = command[:-1]
        if joint_key in JOINT_JOG_ALIASES:
            sign = 1.0 if command[-1] == "+" else -1.0
            return JogCommand(
                "joint_move",
                joint_name=JOINT_JOG_ALIASES[joint_key],
                joint_sign=sign,
            )

    parts = command.split()
    if len(parts) == 2 and parts[0] in JOG_DELTAS:
        try:
            value = float(parts[1])
        except ValueError as exc:
            raise ValueError("axis jog expects a numeric distance in meters") from exc
        if value <= 0.0:
            raise ValueError("axis jog distance must be positive")
        return JogCommand("move", delta_axis=JOG_DELTAS[parts[0]], value=value)
    if len(parts) == 2 and len(parts[0]) > 1 and parts[0][-1] in "+-":
        joint_key = parts[0][:-1]
        if joint_key in JOINT_JOG_ALIASES:
            try:
                value = float(parts[1])
            except ValueError as exc:
                raise ValueError("joint jog expects a numeric distance in radians") from exc
            if value <= 0.0:
                raise ValueError("joint jog distance must be positive")
            sign = 1.0 if parts[0][-1] == "+" else -1.0
            return JogCommand(
                "joint_move",
                joint_name=JOINT_JOG_ALIASES[joint_key],
                joint_sign=sign,
                value=value,
            )
    if len(parts) == 2 and parts[0] in ("step", "dur", "duration", "jstep", "joint-step"):
        try:
            value = float(parts[1])
        except ValueError as exc:
            raise ValueError(f"{parts[0]} expects a number") from exc
        if value <= 0.0:
            raise ValueError(f"{parts[0]} must be positive")
        if parts[0] in ("dur", "duration"):
            kind = "duration"
        elif parts[0] in ("jstep", "joint-step"):
            kind = "joint_step"
        else:
            kind = "step"
        return JogCommand(kind, value=value)

    raise ValueError(
        "unknown command; use x+/x-/y+/y-/z+/z-, step <m>, dur <s>, "
        "pan+/pan-/lift+/lift-/elbow+/elbow-/wrist+/wrist-/roll+/roll-, "
        "pose, sample, help, or q"
    )


def format_jog_help(step_m: float, duration_s: float, joint_step_rad: float | None) -> str:
    joint_help = ""
    if joint_step_rad is not None:
        joint_help = (
            "  pan+ pan- lift+ lift- elbow+ elbow- wrist+ wrist- roll+ roll-\n"
            f"                       move one joint by {math.degrees(joint_step_rad):.1f} deg\n"
            "  elbow- 0.05          move one joint by an explicit distance in radians\n"
            f"  jstep <radians>      change joint step (current {joint_step_rad:.4f} rad)\n"
        )
    return (
        "Jog commands:\n"
        f"  x+ x- y+ y- z+ z-   move {step_m * 1000.0:.1f} mm in the base frame\n"
        "  z+ 0.01              move one axis by an explicit distance in meters\n"
        f"{joint_help}"
        f"  step <meters>        change jog step (current {step_m:.4f} m)\n"
        f"  dur <seconds>        change motion duration (current {duration_s:.2f} s)\n"
        "  pose                 print current tool pose and touch point\n"
        "  sample / Enter       record this corner now\n"
        "  q                    abort without writing output"
    )


class JointJogSession:
    def __init__(
        self,
        *,
        node: Node,
        joint_states_topic: str,
        command_topic: str,
        joint_step_rad: float,
        duration_s: float,
        timeout_s: float,
    ) -> None:
        self.node = node
        self.joint_step_rad = joint_step_rad
        self.duration_s = duration_s
        self.timeout_s = timeout_s
        self._latest_positions: dict[str, float] = {}
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.node.create_subscription(
            JointState, joint_states_topic, self._on_joint_state, sensor_qos
        )
        self.publisher = node.create_publisher(Float64MultiArray, command_topic, 10)
        self.wait_for_joint_state()

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self._latest_positions[name] = float(position)

    def wait_for_joint_state(self) -> None:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            if all(name in self._latest_positions for name in ARM_JOINTS):
                return
            time.sleep(0.05)
        raise RuntimeError("no complete arm joint state received for joint jog")

    def move(self, joint_name: str, sign: float, step_rad: float | None = None) -> None:
        self.wait_for_joint_state()
        step = self.joint_step_rad if step_rad is None else step_rad
        q0 = np.array([self._latest_positions[name] for name in ARM_JOINTS], dtype=float)
        target = q0.copy()
        idx = ARM_JOINTS.index(joint_name)
        target[idx] += sign * step

        duration = max(0.05, self.duration_s)
        steps = max(2, int(duration * 50.0))
        for i in range(1, steps + 1):
            alpha = i / steps
            cmd = q0 + (target - q0) * alpha
            msg = Float64MultiArray()
            msg.data = [float(v) for v in cmd]
            self.publisher.publish(msg)
            time.sleep(duration / steps)

        time.sleep(0.15)
        self.wait_for_joint_state()
        q1 = np.array([self._latest_positions[name] for name in ARM_JOINTS], dtype=float)
        observed = q1[idx] - q0[idx]
        print(
            f"  commanded {joint_name}: "
            f"{math.degrees(sign * step): .1f} deg"
        )
        print(f"  observed {joint_name}:  {math.degrees(observed): .1f} deg")
        if abs(observed) < max(0.002, 0.25 * abs(step)):
            print("  WARNING: joint barely moved; check torque/controller or increase jstep")


class JogSession:
    def __init__(
        self,
        *,
        node: Node,
        tf_buffer: Buffer,
        base_frame: str,
        tool_frame: str,
        tool_offset: np.ndarray,
        service_name: str,
        step_m: float,
        duration_s: float,
        max_step_m: float,
        timeout_s: float,
        strategy: str,
        joint_jog: JointJogSession | None = None,
    ) -> None:
        self.node = node
        self.tf_buffer = tf_buffer
        self.base_frame = base_frame
        self.tool_frame = tool_frame
        self.tool_offset = tool_offset
        self.step_m = step_m
        self.duration_s = duration_s
        self.max_step_m = max_step_m
        self.timeout_s = timeout_s
        self.strategy = strategy
        self.joint_jog = joint_jog
        self.client = node.create_client(GoToPose, service_name)
        if not self.client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f"jog service {service_name} is not available")

    def run_corner_prompt(self, name: str, instruction: str) -> None:
        print(f"[{name}] {instruction}")
        print(format_jog_help(self.step_m, self.duration_s, self._joint_step_rad()))
        while True:
            raw = input(f"{name} jog> ")
            try:
                command = parse_jog_command(raw)
                if command.kind == "sample":
                    return
                if command.kind == "quit":
                    raise KeyboardInterrupt
                if command.kind == "help":
                    print(format_jog_help(self.step_m, self.duration_s, self._joint_step_rad()))
                elif command.kind == "pose":
                    self.print_pose()
                elif command.kind == "step":
                    assert command.value is not None
                    if command.value > self.max_step_m:
                        print(
                            f"  rejected: max jog step is "
                            f"{self.max_step_m * 1000.0:.1f} mm"
                        )
                    else:
                        self.step_m = command.value
                        print(f"  step set to {self.step_m * 1000.0:.1f} mm")
                elif command.kind == "duration":
                    assert command.value is not None
                    self.duration_s = command.value
                    print(f"  duration set to {self.duration_s:.2f} s")
                elif command.kind == "joint_step":
                    if self.joint_jog is None:
                        print("  joint jog is not enabled")
                    else:
                        assert command.value is not None
                        self.joint_jog.joint_step_rad = command.value
                        print(
                            f"  joint step set to "
                            f"{math.degrees(self.joint_jog.joint_step_rad):.1f} deg"
                        )
                elif command.kind == "joint_move":
                    if self.joint_jog is None:
                        print("  joint jog is not enabled")
                    else:
                        assert command.joint_name is not None
                        assert command.joint_sign is not None
                        self.joint_jog.move(
                            command.joint_name,
                            command.joint_sign,
                            step_rad=command.value,
                        )
                elif command.kind == "move":
                    assert command.delta_axis is not None
                    step_m = self.step_m if command.value is None else command.value
                    self.move(command.delta_axis * step_m)
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                print(f"  {exc}")

    def _joint_step_rad(self) -> float | None:
        if self.joint_jog is None:
            return None
        return self.joint_jog.joint_step_rad

    def print_pose(self) -> None:
        T = wait_for_tool_pose(
            self.tf_buffer, self.base_frame, self.tool_frame, self.timeout_s
        )
        point = T[:3, 3] + T[:3, :3] @ self.tool_offset
        quat = matrix_to_quat_xyzw(T[:3, :3])
        print(
            f"  {self.tool_frame} origin xyz: "
            f"{T[0, 3]: .4f} {T[1, 3]: .4f} {T[2, 3]: .4f} m"
        )
        print(
            f"  touch point xyz:        "
            f"{point[0]: .4f} {point[1]: .4f} {point[2]: .4f} m"
        )
        print(f"  tool quaternion xyzw:   {[round(v, 6) for v in quat]}")

    def move(self, delta_xyz: np.ndarray) -> None:
        T = wait_for_tool_pose(
            self.tf_buffer, self.base_frame, self.tool_frame, self.timeout_s
        )
        before_point = T[:3, 3] + T[:3, :3] @ self.tool_offset
        T[:3, 3] += delta_xyz

        request = GoToPose.Request()
        request.target = pose_stamped_from_matrix(T, self.base_frame, self.node)
        request.strategy = self.strategy
        request.duration = self.duration_s
        future = self.client.call_async(request)
        deadline = time.monotonic() + self.duration_s + self.timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            raise RuntimeError("jog service timed out")
        response = future.result()
        if response is None:
            raise RuntimeError("jog service returned no response")
        if not response.success:
            raise RuntimeError(f"jog failed: {response.message}")
        time.sleep(0.2)
        observed_T = wait_for_tool_pose(
            self.tf_buffer, self.base_frame, self.tool_frame, self.timeout_s
        )
        after_point = observed_T[:3, 3] + observed_T[:3, :3] @ self.tool_offset
        observed_delta = after_point - before_point
        print(
            f"  commanded delta xyz: "
            f"{delta_xyz[0] * 1000.0: .1f} "
            f"{delta_xyz[1] * 1000.0: .1f} "
            f"{delta_xyz[2] * 1000.0: .1f} mm"
        )
        print(
            f"  observed touch delta: "
            f"{observed_delta[0] * 1000.0: .1f} "
            f"{observed_delta[1] * 1000.0: .1f} "
            f"{observed_delta[2] * 1000.0: .1f} mm"
        )
        if np.linalg.norm(observed_delta) < max(0.0005, 0.25 * np.linalg.norm(delta_xyz)):
            print(
                "  WARNING: TF barely moved. The arm may not be executing commands, "
                "or the step is below backlash/visibility."
            )


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
    parser.add_argument(
        "--jog-service",
        default="",
        help="Optional GoToPose service for terminal x/y/z jogs, e.g. /left/go_to_pose.",
    )
    parser.add_argument(
        "--jog-step-m",
        type=float,
        default=0.002,
        help="Default jog step in meters when --jog-service is enabled.",
    )
    parser.add_argument(
        "--max-jog-step-m",
        type=float,
        default=0.02,
        help="Largest allowed per-command jog step in meters.",
    )
    parser.add_argument(
        "--jog-duration-sec",
        type=float,
        default=1.5,
        help="Duration for each jog motion.",
    )
    parser.add_argument(
        "--jog-timeout-s",
        type=float,
        default=8.0,
        help="Service wait/response timeout for jog moves.",
    )
    parser.add_argument(
        "--jog-strategy",
        choices=("joint_quintic", "cartesian"),
        default="joint_quintic",
        help="GoToPose planner strategy for each jog command.",
    )
    parser.add_argument(
        "--joint-jog-topic",
        default="",
        help="Optional Float64MultiArray arm command topic for direct joint jogs.",
    )
    parser.add_argument(
        "--joint-states-topic",
        default="",
        help="JointState topic for direct joint jog feedback.",
    )
    parser.add_argument(
        "--joint-step-rad",
        type=float,
        default=0.035,
        help="Default direct joint jog step in radians.",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.jog_step_m <= 0.0:
        print("ERROR: --jog-step-m must be positive", file=sys.stderr)
        return 2
    if args.max_jog_step_m <= 0.0:
        print("ERROR: --max-jog-step-m must be positive", file=sys.stderr)
        return 2
    if args.jog_step_m > args.max_jog_step_m:
        print("ERROR: --jog-step-m cannot exceed --max-jog-step-m", file=sys.stderr)
        return 2
    if args.jog_duration_sec <= 0.0:
        print("ERROR: --jog-duration-sec must be positive", file=sys.stderr)
        return 2
    if args.joint_step_rad <= 0.0:
        print("ERROR: --joint-step-rad must be positive", file=sys.stderr)
        return 2
    if bool(args.joint_jog_topic) != bool(args.joint_states_topic):
        print("ERROR: --joint-jog-topic and --joint-states-topic must be used together", file=sys.stderr)
        return 2

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

    joint_jog: JointJogSession | None = None
    if args.joint_jog_topic:
        joint_jog = JointJogSession(
            node=node,
            joint_states_topic=args.joint_states_topic,
            command_topic=args.joint_jog_topic,
            joint_step_rad=args.joint_step_rad,
            duration_s=args.jog_duration_sec,
            timeout_s=args.jog_timeout_s,
        )

    jog_session: JogSession | None = None
    if args.jog_service:
        jog_session = JogSession(
            node=node,
            tf_buffer=tf_buffer,
            base_frame=args.base_frame,
            tool_frame=args.tool_frame,
            tool_offset=args.tool_offset,
            service_name=args.jog_service,
            step_m=args.jog_step_m,
            duration_s=args.jog_duration_sec,
            max_step_m=args.max_jog_step_m,
            timeout_s=args.jog_timeout_s,
            strategy=args.jog_strategy,
            joint_jog=joint_jog,
        )

    print("")
    print("SO101 board touch recorder")
    print(f"TF: {args.base_frame} <- {args.tool_frame}")
    print(f"Tool offset in {args.tool_frame}: {args.tool_offset.tolist()} m")
    print(f"Board pattern: {width_m:.3f} m x {height_m:.3f} m ({args.cols} x {args.rows} squares)")
    print("")
    print("Use the OUTER checkerboard-pattern corners, not the A3 paper corners.")
    print("Keep the same physical touch point on the gripper for every sample.")
    if jog_session is None:
        print("Place the tool, hold it still, then press Enter in this terminal.")
    else:
        print("JOG MODE ENABLED: this terminal sends real arm motion through GoToPose.")
        print("Use one small jog command at a time, then press Enter/sample to record.")
    print("")

    points: dict[str, np.ndarray] = {}
    spreads: dict[str, np.ndarray] = {}
    try:
        wait_for_tool_point(tf_buffer, args.base_frame, args.tool_frame, args.tool_offset, args.timeout_s)
        for name, instruction in POINT_LAYOUTS[args.corner_layout]:
            while True:
                if jog_session is None:
                    print(f"[{name}] {instruction}")
                    input("Press Enter when the tool is touching that point...")
                else:
                    jog_session.run_corner_prompt(name, instruction)
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
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
