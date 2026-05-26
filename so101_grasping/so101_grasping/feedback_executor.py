"""Measured-joint feedback executor for SO-101 grasp primitives."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf_transformations import quaternion_matrix

from robokin.placo import PlacoConfig, PlacoKinematics
from robokin.robot_model import load_robot_description


@dataclass(frozen=True)
class FeedbackStageResult:
    name: str
    joint_names: list[str]
    positions: list[float]
    position_error_m: float | None
    joint_error_rad: float | None
    message: str


def normalize_angle(radians: float) -> float:
    return float((radians + math.pi) % (2.0 * math.pi) - math.pi)


def pose_to_matrix(pose_stamped: PoseStamped) -> np.ndarray:
    pose = pose_stamped.pose
    quat = [
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    ]
    matrix = quaternion_matrix(quat).astype(np.float64)
    matrix[0, 3] = float(pose.position.x)
    matrix[1, 3] = float(pose.position.y)
    matrix[2, 3] = float(pose.position.z)
    return matrix


def rotation_error_rad(actual: np.ndarray, desired: np.ndarray) -> float:
    delta = desired[:3, :3].T @ actual[:3, :3]
    trace = float(np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0))
    return float(math.acos(trace))


def quintic_ease(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return float(10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5)


class FeedbackArmExecutor:
    """Direct position-command executor with measured-state correction loops."""

    def __init__(
        self,
        node: Node,
        *,
        joint_states_topic: str,
        cmd_topic: str,
        base_frame: str,
        moveit_frame: str,
        ee_frame: str,
        arm_joint_names: list[str],
        gripper_joint_name: str,
    ) -> None:
        self._node = node
        self._base_frame = base_frame
        self._moveit_frame = moveit_frame
        self._ee_frame = ee_frame
        self._arm_joint_names = list(arm_joint_names)
        self._gripper_joint_name = gripper_joint_name

        model = load_robot_description("so_arm101_description")
        self._urdf_path = str(model.urdf_path)
        # Match the SO-101 grasp primitive: solve EE position first, then
        # stream the measured-joint correction with explicit timing.
        self._solver = self._make_solver(rot_weight=0.0)
        self._look_rot_weight = 0.35
        self._look_solver = self._make_solver(rot_weight=self._look_rot_weight)
        self._joint_names = list(self._solver.joint_names)
        if list(self._look_solver.joint_names) != self._joint_names:
            raise RuntimeError("feedback look IK solver joint order differs from position solver")
        self._arm_indices = [self._joint_names.index(name) for name in self._arm_joint_names]
        self._gripper_index = self._joint_names.index(self._gripper_joint_name)
        self._wrist_roll_index = (
            self._joint_names.index("wrist_roll") if "wrist_roll" in self._joint_names else None
        )

        self._lock = threading.Lock()
        self._measured_q: np.ndarray | None = None
        self._last_command_q: np.ndarray | None = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._node.create_subscription(JointState, joint_states_topic, self._on_joint_state, sensor_qos)
        self._cmd_pub = self._node.create_publisher(Float64MultiArray, cmd_topic, 10)

        self.rate_hz = 50.0
        self.position_tolerance_m = 0.015
        self.ik_position_tolerance_m = 0.060
        self.joint_tolerance_rad = 0.18
        self.max_correction_iters = 4
        self.settle_s = 0.35
        self.min_joint_delta_rad = 0.035
        self.max_joint_speed_rad_s = 0.45
        self.min_motion_duration_s = 0.80
        self.joint_state_timeout_s = 3.0
        self.correction_command_gain = 1.45
        self.max_overcommand_rad = 0.12
        self.look_rotation_tolerance_rad = math.radians(20.0)

    def _make_solver(self, *, rot_weight: float) -> PlacoKinematics:
        return PlacoKinematics(
            urdf_path=self._urdf_path,
            ee_frame=self._ee_frame,
            cfg=PlacoConfig(
                dt=1.0 / 50.0,
                rot_weight=float(rot_weight),
                enable_velocity_limits=False,
                enable_self_collisions=False,
            ),
        )

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    @property
    def arm_joint_names(self) -> list[str]:
        return list(self._arm_joint_names)

    def configure(
        self,
        *,
        rate_hz: float,
        position_tolerance_m: float,
        ik_position_tolerance_m: float,
        joint_tolerance_rad: float,
        max_correction_iters: int,
        settle_s: float,
        min_joint_delta_rad: float,
        max_joint_speed_rad_s: float,
        min_motion_duration_s: float,
        joint_state_timeout_s: float,
        correction_command_gain: float,
        max_overcommand_rad: float,
        look_rot_weight: float,
        look_rotation_tolerance_rad: float,
    ) -> None:
        self.rate_hz = max(5.0, float(rate_hz))
        self.position_tolerance_m = max(0.001, float(position_tolerance_m))
        self.ik_position_tolerance_m = max(self.position_tolerance_m, float(ik_position_tolerance_m))
        self.joint_tolerance_rad = max(0.01, float(joint_tolerance_rad))
        self.max_correction_iters = max(0, int(max_correction_iters))
        self.settle_s = max(0.0, float(settle_s))
        self.min_joint_delta_rad = max(0.0, float(min_joint_delta_rad))
        self.max_joint_speed_rad_s = max(0.05, float(max_joint_speed_rad_s))
        self.min_motion_duration_s = max(0.05, float(min_motion_duration_s))
        self.joint_state_timeout_s = max(0.2, float(joint_state_timeout_s))
        self.correction_command_gain = max(1.0, float(correction_command_gain))
        self.max_overcommand_rad = max(0.0, float(max_overcommand_rad))
        look_rot_weight = max(0.001, float(look_rot_weight))
        if not math.isclose(look_rot_weight, self._look_rot_weight, rel_tol=1e-6, abs_tol=1e-9):
            self._look_rot_weight = look_rot_weight
            self._look_solver = self._make_solver(rot_weight=self._look_rot_weight)
        self.look_rotation_tolerance_rad = max(math.radians(2.0), float(look_rotation_tolerance_rad))

    def _on_joint_state(self, msg: JointState) -> None:
        values = self._measured_q.copy() if self._measured_q is not None else np.zeros(len(self._joint_names))
        by_name = dict(zip(msg.name, msg.position, strict=False))
        for index, name in enumerate(self._joint_names):
            if name in by_name:
                values[index] = float(by_name[name])
        with self._lock:
            self._measured_q = values
            if self._last_command_q is None:
                self._last_command_q = values.copy()

    def measured_q(self, *, timeout_s: float | None = None) -> np.ndarray | None:
        timeout = self.joint_state_timeout_s if timeout_s is None else max(0.0, float(timeout_s))
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() <= deadline:
            with self._lock:
                if self._measured_q is not None:
                    return self._measured_q.copy()
            time.sleep(0.01)
        return None

    def current_wrist_roll(self) -> float | None:
        if self._wrist_roll_index is None:
            return None
        measured = self.measured_q(timeout_s=0.1)
        if measured is None:
            return None
        return float(measured[self._wrist_roll_index])

    def current_arm_positions(self) -> np.ndarray:
        measured = self.measured_q()
        if measured is None:
            raise RuntimeError("no joint_states received for feedback executor")
        return measured[self._arm_indices].astype(np.float64)

    def fk(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64)
        return np.asarray(self._solver.fk(q), dtype=np.float64)

    def pose_position_error(self, q: np.ndarray, target: PoseStamped) -> float:
        return float(np.linalg.norm(self.pose_position_delta(q, target)))

    def pose_position_delta(self, q: np.ndarray, target: PoseStamped) -> np.ndarray:
        actual = self.fk(q)[:3, 3]
        desired = pose_to_matrix(target)[:3, 3]
        return np.asarray(actual - desired, dtype=np.float64)

    def pose_rotation_error(self, q: np.ndarray, target: PoseStamped) -> float:
        return rotation_error_rad(self.fk(q), pose_to_matrix(target))

    def solve_sequence(
        self,
        stages: list[Any],
        *,
        reference_wrist_roll: float | None,
        max_wrist_roll_delta_rad: float,
    ) -> tuple[list[FeedbackStageResult], str, float | None]:
        measured = self.measured_q()
        if measured is None:
            return [], "no joint_states received for feedback planning", None
        q_seed = measured.copy()
        results: list[FeedbackStageResult] = []
        messages: list[str] = []
        max_wrist_delta = 0.0
        saw_wrist_delta = False
        for stage in stages:
            result, q_goal = self.solve_stage(stage, q_seed)
            if result is None:
                return [], f"{stage.name}: feedback IK failed", None
            if q_goal is None:
                return [], f"{stage.name}: {result.message}", None
            wrist_delta = self._wrist_roll_delta(q_goal, reference_wrist_roll)
            if wrist_delta is not None:
                saw_wrist_delta = True
                max_wrist_delta = max(max_wrist_delta, wrist_delta)
                if max_wrist_roll_delta_rad > 0.0 and wrist_delta > max_wrist_roll_delta_rad:
                    return (
                        [],
                        f"{stage.name}: wrist_roll_delta={wrist_delta:.3f}rad exceeds "
                        f"max_wrist_roll_delta_rad={max_wrist_roll_delta_rad:.3f}",
                        wrist_delta,
                    )
            results.append(result)
            messages.append(f"{stage.name}: {result.message}")
            q_seed = q_goal
        return results, " -> ".join(messages), max_wrist_delta if saw_wrist_delta else None

    def solve_stage(self, stage: Any, q_seed: np.ndarray) -> tuple[FeedbackStageResult | None, np.ndarray | None]:
        if getattr(stage, "configuration_name", None) is not None:
            return self._solve_named_stage(stage.name, str(stage.configuration_name), q_seed)
        if getattr(stage, "pose", None) is not None:
            return self._solve_pose_stage(stage.name, stage.pose, q_seed)
        return None, None

    def _stage_uses_pose_orientation(self, stage_name: str) -> bool:
        return stage_name == "wrist_camera_look"

    def _pose_stage_reached(self, stage_name: str, q: np.ndarray, pose: PoseStamped) -> tuple[bool, float, float | None]:
        pos_error = self.pose_position_error(q, pose)
        if not self._stage_uses_pose_orientation(stage_name):
            return pos_error <= self.position_tolerance_m, pos_error, None
        rot_error = self.pose_rotation_error(q, pose)
        return (
            pos_error <= self.position_tolerance_m
            and rot_error <= self.look_rotation_tolerance_rad,
            pos_error,
            rot_error,
        )

    def execute_stage(
        self,
        stage: Any,
        *,
        reference_wrist_roll: float | None,
        max_wrist_roll_delta_rad: float,
    ) -> tuple[bool, str]:
        for attempt in range(self.max_correction_iters + 1):
            measured = self.measured_q()
            if measured is None:
                return False, f"{stage.name}: no joint_states received"

            if getattr(stage, "configuration_name", None) is not None:
                result, q_goal = self._solve_named_stage(stage.name, str(stage.configuration_name), measured)
                if result is None or q_goal is None:
                    return False, f"{stage.name}: unknown named target {stage.configuration_name!r}"
                actual_error = self._arm_joint_error(measured, q_goal)
                if actual_error <= self.joint_tolerance_rad:
                    return True, f"{stage.name}: reached named target joint_error={actual_error:.3f}rad"
                metric = actual_error
                tolerance = self.joint_tolerance_rad
                metric_name = "joint_error"
            elif getattr(stage, "pose", None) is not None:
                reached, actual_error, rot_error = self._pose_stage_reached(stage.name, measured, stage.pose)
                if reached:
                    message = f"{stage.name}: reached pose pos_error={actual_error * 1000.0:.1f}mm"
                    if rot_error is not None:
                        message += f" rot_error={math.degrees(rot_error):.1f}deg"
                    return True, message
                result, q_goal = self._solve_pose_stage(stage.name, stage.pose, measured)
                if result is not None and q_goal is None:
                    return False, f"{stage.name}: {result.message}"
                if result is None or q_goal is None:
                    return False, f"{stage.name}: feedback IK failed"
                if rot_error is None:
                    metric = actual_error
                    tolerance = self.position_tolerance_m
                    metric_name = "pos_error"
                else:
                    metric = max(
                        actual_error / self.position_tolerance_m,
                        rot_error / self.look_rotation_tolerance_rad,
                    )
                    tolerance = 1.0
                    metric_name = "pose_error_ratio"
            else:
                return False, f"{stage.name}: no pose or named target"

            wrist_delta = self._wrist_roll_delta(q_goal, reference_wrist_roll)
            if wrist_delta is not None and max_wrist_roll_delta_rad > 0.0 and wrist_delta > max_wrist_roll_delta_rad:
                return (
                    False,
                    f"{stage.name}: wrist_roll_delta={wrist_delta:.3f}rad exceeds "
                    f"max_wrist_roll_delta_rad={max_wrist_roll_delta_rad:.3f}",
                )

            delta = self._max_arm_delta(measured, q_goal)
            if delta < self.min_joint_delta_rad:
                if metric <= tolerance:
                    return True, f"{stage.name}: reached within tolerance"
                return (
                    False,
                    f"{stage.name}: correction below servo deadband "
                    f"(max_delta={delta:.3f}rad < {self.min_joint_delta_rad:.3f}rad, "
                    f"{metric_name}={metric:.4f})",
                )

            command_goal = q_goal
            if attempt > 0 and getattr(stage, "pose", None) is not None:
                command_goal = self._correction_command_goal(measured, q_goal)
                command_delta = self._max_arm_delta(q_goal, command_goal)
                if command_delta > 0.0:
                    self._node.get_logger().info(
                        f"{stage.name}: applying capped feedback over-command "
                        f"max_extra={command_delta:.3f}rad"
                    )

            self._stream_joint_goal(measured, command_goal)
            if self.settle_s > 0.0:
                time.sleep(self.settle_s)

            after = self.measured_q(timeout_s=0.2)
            if after is None:
                continue
            if getattr(stage, "pose", None) is not None:
                after_error = self.pose_position_error(after, stage.pose)
                after_delta = self.pose_position_delta(after, stage.pose)
                rot_error = (
                    self.pose_rotation_error(after, stage.pose)
                    if self._stage_uses_pose_orientation(stage.name)
                    else None
                )
                rot_text = ""
                if rot_error is not None:
                    rot_text = f" rot_error={math.degrees(rot_error):.1f}deg"
                self._node.get_logger().info(
                    f"{stage.name}: feedback attempt {attempt + 1}/"
                    f"{self.max_correction_iters + 1} pos_error={after_error * 1000.0:.1f}mm "
                    f"delta_xyz_mm=({after_delta[0] * 1000.0:.1f},"
                    f"{after_delta[1] * 1000.0:.1f},{after_delta[2] * 1000.0:.1f})"
                    f"{rot_text}"
                )
            else:
                after_error = self._arm_joint_error(after, q_goal)
                self._node.get_logger().info(
                    f"{stage.name}: feedback attempt {attempt + 1}/"
                    f"{self.max_correction_iters + 1} joint_error={after_error:.3f}rad"
                )

        measured = self.measured_q(timeout_s=0.2)
        if measured is not None and getattr(stage, "pose", None) is not None:
            delta = self.pose_position_delta(measured, stage.pose)
            rot_error = (
                self.pose_rotation_error(measured, stage.pose)
                if self._stage_uses_pose_orientation(stage.name)
                else None
            )
            rot_text = ""
            if rot_error is not None:
                rot_text = f", rot_error={math.degrees(rot_error):.1f}deg"
            return (
                False,
                f"{stage.name}: feedback max corrections reached "
                f"(pos_error={self.pose_position_error(measured, stage.pose) * 1000.0:.1f}mm, "
                f"delta_xyz_mm=({delta[0] * 1000.0:.1f},{delta[1] * 1000.0:.1f},"
                f"{delta[2] * 1000.0:.1f}){rot_text})",
            )
        if measured is not None and getattr(stage, "configuration_name", None) is not None:
            _, q_goal = self._solve_named_stage(stage.name, str(stage.configuration_name), measured)
            if q_goal is not None:
                return (
                    False,
                    f"{stage.name}: feedback max corrections reached "
                    f"(joint_error={self._arm_joint_error(measured, q_goal):.3f}rad)",
                )
        return False, f"{stage.name}: feedback max corrections reached"

    def _solve_named_stage(
        self,
        stage_name: str,
        configuration_name: str,
        q_seed: np.ndarray,
    ) -> tuple[FeedbackStageResult | None, np.ndarray | None]:
        name = configuration_name.strip().lower()
        q_goal = np.asarray(q_seed, dtype=np.float64).copy()
        if name == "zero":
            values = {
                "shoulder_pan": 0.0,
                "shoulder_lift": 0.0,
                "elbow_flex": 0.0,
                "wrist_flex": 0.0,
                "wrist_roll": 0.0,
            }
        elif name == "rest":
            values = {
                "shoulder_pan": 0.0,
                "shoulder_lift": -math.pi / 2.0,
                "elbow_flex": math.pi / 2.0,
                "wrist_flex": math.radians(42.97),
                "wrist_roll": 0.0,
            }
        else:
            return None, None
        for joint, value in values.items():
            if joint in self._joint_names:
                q_goal[self._joint_names.index(joint)] = float(value)
        q_goal[self._gripper_index] = q_seed[self._gripper_index]
        joint_error = self._arm_joint_error(q_seed, q_goal)
        result = FeedbackStageResult(
            name=stage_name,
            joint_names=self.arm_joint_names,
            positions=[float(q_goal[index]) for index in self._arm_indices],
            position_error_m=None,
            joint_error_rad=joint_error,
            message=f"feedback named target '{configuration_name}' solved joint_error={joint_error:.3f}rad",
        )
        return result, q_goal

    def _solve_pose_stage(
        self,
        stage_name: str,
        pose: PoseStamped,
        q_seed: np.ndarray,
    ) -> tuple[FeedbackStageResult | None, np.ndarray | None]:
        frame = pose.header.frame_id.strip()
        if frame and frame not in {self._base_frame, self._moveit_frame}:
            result = FeedbackStageResult(
                name=stage_name,
                joint_names=self.arm_joint_names,
                positions=[],
                position_error_m=None,
                joint_error_rad=None,
                message=f"feedback pose frame {frame!r} is not {self._base_frame!r} or {self._moveit_frame!r}",
            )
            return result, None
        target = pose_to_matrix(pose)
        uses_orientation = self._stage_uses_pose_orientation(stage_name)
        solver = self._look_solver if uses_orientation else self._solver
        n_iters = 200 if uses_orientation else 100
        try:
            q_goal = np.asarray(solver.solve_goal(q_seed, target, n_iters=n_iters), dtype=np.float64)
        except Exception as exc:  # noqa: BLE001 - surfaced to ROS service response.
            result = FeedbackStageResult(
                name=stage_name,
                joint_names=self.arm_joint_names,
                positions=[],
                position_error_m=None,
                joint_error_rad=None,
                message=f"feedback IK exception: {exc}",
            )
            return result, None
        if q_goal.shape != q_seed.shape or not np.isfinite(q_goal).all():
            return None, None
        q_goal[self._gripper_index] = q_seed[self._gripper_index]
        pos_error = self.pose_position_error(q_goal, pose)
        rot_error = self.pose_rotation_error(q_goal, pose) if uses_orientation else None
        if pos_error > self.ik_position_tolerance_m:
            result = FeedbackStageResult(
                name=stage_name,
                joint_names=self.arm_joint_names,
                positions=[float(q_goal[index]) for index in self._arm_indices],
                position_error_m=pos_error,
                joint_error_rad=None,
                message=(
                    f"feedback IK residual {pos_error * 1000.0:.1f}mm exceeds "
                    f"{self.ik_position_tolerance_m * 1000.0:.1f}mm"
                ),
            )
            return result, None
        rot_text = ""
        if rot_error is not None:
            rot_text = f" rot_error={math.degrees(rot_error):.1f}deg look_rot_weight={self._look_rot_weight:.3f}"
        result = FeedbackStageResult(
            name=stage_name,
            joint_names=self.arm_joint_names,
            positions=[float(q_goal[index]) for index in self._arm_indices],
            position_error_m=pos_error,
            joint_error_rad=None,
            message=f"feedback IK solved pos_error={pos_error * 1000.0:.1f}mm{rot_text}",
        )
        return result, q_goal

    def _wrist_roll_delta(self, q_goal: np.ndarray, reference_wrist_roll: float | None) -> float | None:
        if reference_wrist_roll is None or self._wrist_roll_index is None:
            return None
        return abs(normalize_angle(float(q_goal[self._wrist_roll_index]) - float(reference_wrist_roll)))

    def _arm_joint_error(self, q_actual: np.ndarray, q_goal: np.ndarray) -> float:
        return float(np.max(np.abs(q_actual[self._arm_indices] - q_goal[self._arm_indices])))

    def _max_arm_delta(self, q_start: np.ndarray, q_goal: np.ndarray) -> float:
        return float(np.max(np.abs(q_goal[self._arm_indices] - q_start[self._arm_indices])))

    def _correction_command_goal(self, q_start: np.ndarray, q_goal: np.ndarray) -> np.ndarray:
        command_goal = np.asarray(q_goal, dtype=np.float64).copy()
        if self.correction_command_gain <= 1.0 or self.max_overcommand_rad <= 0.0:
            return command_goal
        extra = (command_goal - q_start) * (self.correction_command_gain - 1.0)
        extra[self._gripper_index] = 0.0
        extra[self._arm_indices] = np.clip(
            extra[self._arm_indices],
            -self.max_overcommand_rad,
            self.max_overcommand_rad,
        )
        command_goal[self._arm_indices] = command_goal[self._arm_indices] + extra[self._arm_indices]
        return command_goal

    def _stream_joint_goal(self, q_start: np.ndarray, q_goal: np.ndarray) -> None:
        delta = q_goal - q_start
        max_delta = self._max_arm_delta(q_start, q_goal)
        duration = max(
            self.min_motion_duration_s,
            (15.0 / 8.0) * max_delta / self.max_joint_speed_rad_s,
        )
        dt = 1.0 / self.rate_hz
        steps = max(2, int(math.ceil(duration / dt)))
        for step in range(steps + 1):
            if not rclpy.ok():
                break
            alpha = quintic_ease(step / steps)
            command = q_start + delta * alpha
            command[self._gripper_index] = q_start[self._gripper_index]
            self._publish(command)
            time.sleep(dt)
        self._publish(q_goal)

    def _publish(self, q_command: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [float(q_command[index]) for index in self._arm_indices]
        self._cmd_pub.publish(msg)
        with self._lock:
            self._last_command_q = q_command.copy()
