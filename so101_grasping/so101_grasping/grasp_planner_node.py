"""MoveIt-backed grasp planning service for SO-101.

The node plans to a pregrasp pose derived from GraspNet candidates. Execution
is gated by an explicit parameter so perception tests cannot move the real arm.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
import rclpy
from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from moveit_msgs.msg import CollisionObject, DisplayTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_matrix
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from so101_grasping.grasp_staging import (
    SurfaceRelativeStages,
    normalize_vector,
    signed_plane_distance,
    surface_relative_stages,
)
from so101_grasping.wrist_views import WristViewCandidate, wrist_view_candidates
from so101_grasp_msgs.msg import GraspCandidate
from so101_grasp_msgs.srv import DetectGrasps, PlanGrasp


BASE_FRAME = "follower/base_link"
MOVEIT_FRAME = "base_link"
PLANNING_GROUP = "manipulator"
EE_FRAME = "gripper_frame_link"
ARM_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER_JOINT_NAME = "gripper"


@dataclass(frozen=True)
class PlannedStage:
    name: str
    pose: PoseStamped | None
    result: Any
    configuration_name: str | None = None


@dataclass(frozen=True)
class PrimitiveStage:
    name: str
    pose: PoseStamped | None = None
    configuration_name: str | None = None


@dataclass(frozen=True)
class PlanSelection:
    candidate_index: int
    grasp: GraspCandidate
    planned_stages: list[PlannedStage]
    primitive_stages: list[PrimitiveStage]
    strategy_label: str
    message: str
    wrist_roll_delta_rad: float | None = None


def _point(values: np.ndarray) -> Point:
    point = Point()
    point.x = float(values[0])
    point.y = float(values[1])
    point.z = float(values[2])
    return point


def _pose_to_numpy(pose) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    position = np.asarray([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
    quat = np.asarray([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], dtype=np.float64)
    matrix = quaternion_matrix(quat)
    return position, quat, matrix[:3, :3]


def _pose_matrix(position: np.ndarray, quat_xyzw: np.ndarray) -> np.ndarray:
    matrix = quaternion_matrix(quat_xyzw)
    matrix[:3, 3] = position
    return matrix


def _pose_from_matrix(matrix: np.ndarray) -> Pose:
    pose = Pose()
    pose.position = _point(np.asarray(matrix[:3, 3], dtype=np.float64))
    quat = np.asarray(quaternion_from_matrix(matrix), dtype=np.float64)
    quat /= max(float(np.linalg.norm(quat)), 1e-9)
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


def _copy_grasp(grasp: GraspCandidate) -> GraspCandidate:
    copied = GraspCandidate()
    copied.header = grasp.header
    copied.pose = grasp.pose
    copied.score = grasp.score
    copied.width = grasp.width
    copied.source = grasp.source
    return copied


class GraspPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_planner_node")

        self.declare_parameter("detect_service", "/detect_grasps")
        self.declare_parameter("plan_service", "/plan_grasp")
        self.declare_parameter("allow_execution", False)
        self.declare_parameter("execution_backend", "moveit")
        self.declare_parameter("moveit_node_name", "so101_grasp_moveit_py")
        self.declare_parameter("grasp_frame", BASE_FRAME)
        self.declare_parameter("arm_base_frame", BASE_FRAME)
        self.declare_parameter("moveit_frame", MOVEIT_FRAME)
        self.declare_parameter("ee_frame", EE_FRAME)
        self.declare_parameter("joint_states_topic", "/follower/joint_states")
        self.declare_parameter("feedback_cmd_topic", "/follower/arm_forward_controller/commands")
        self.declare_parameter("feedback_rate_hz", 50.0)
        self.declare_parameter("feedback_position_tolerance_m", 0.015)
        self.declare_parameter("feedback_ik_position_tolerance_m", 0.060)
        self.declare_parameter("feedback_joint_tolerance_rad", 0.18)
        self.declare_parameter("feedback_max_correction_iters", 4)
        self.declare_parameter("feedback_settle_s", 0.35)
        self.declare_parameter("feedback_min_joint_delta_rad", 0.035)
        self.declare_parameter("feedback_max_joint_speed_rad_s", 0.45)
        self.declare_parameter("feedback_min_motion_duration_s", 0.80)
        self.declare_parameter("feedback_joint_state_timeout_s", 3.0)
        self.declare_parameter("feedback_correction_command_gain", 1.45)
        self.declare_parameter("feedback_max_overcommand_rad", 0.12)
        self.declare_parameter("feedback_look_rot_weight", 0.35)
        self.declare_parameter("feedback_look_rotation_tolerance_deg", 20.0)
        self.declare_parameter("object_cloud_topic", "/so101_grasping/object_cloud")
        self.declare_parameter("wrist_object_cloud_topic", "/so101_grasping/wrist_object_cloud")
        self.declare_parameter("display_topic", "/so101_grasping/display_planned_path")
        self.declare_parameter("planned_markers_topic", "/so101_grasping/planned_path_markers")
        self.declare_parameter("default_prompt", "pink cube")
        self.declare_parameter("default_top_k", 8)
        self.declare_parameter("detect_timeout_s", 60.0)
        self.declare_parameter("pregrasp_offset_m", 0.10)
        self.declare_parameter("max_grasp_radius_m", 0.75)
        self.declare_parameter("ik_timeout_s", 0.75)
        self.declare_parameter("planner_parameter_set", "ompl_rrtc")
        self.declare_parameter("add_table_collision", False)
        self.declare_parameter("table_size_xyz", [1.20, 0.80, 0.035])
        self.declare_parameter("table_center_xyz", [0.45, 0.0, -0.025])
        self.declare_parameter("add_object_bbox_collision", False)
        self.declare_parameter("object_bbox_padding_m", 0.025)
        self.declare_parameter("ready_clearance_m", 0.12)
        self.declare_parameter("pregrasp_clearance_m", 0.065)
        self.declare_parameter("close_clearance_m", 0.000)
        self.declare_parameter("min_ready_z_m", 0.130)
        self.declare_parameter("min_pregrasp_z_m", 0.075)
        self.declare_parameter("min_close_z_m", 0.032)
        self.declare_parameter("use_support_plane_staging", False)
        self.declare_parameter("support_plane_frame", "world")
        self.declare_parameter("support_plane_point_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("support_plane_normal_xyz", [0.0, 0.0, 1.0])
        self.declare_parameter("close_surface_clearance_m", 0.003)
        self.declare_parameter("gripper_action", "/follower/gripper_controller/gripper_cmd")
        self.declare_parameter("gripper_open_position", 0.45)
        self.declare_parameter("gripper_closed_position", -0.50)
        self.declare_parameter("gripper_open_allow_stall", True)
        self.declare_parameter("gripper_max_effort", 15.0)
        self.declare_parameter("gripper_action_timeout_s", 4.0)
        self.declare_parameter("arm_execution_joint_tolerance_rad", 0.35)
        self.declare_parameter("post_stage_settle_s", 0.75)
        self.declare_parameter("ready_configuration_name", "zero")
        self.declare_parameter("default_plan_budget_s", 20.0)
        self.declare_parameter("max_plan_candidates", 3)
        self.declare_parameter("max_primitive_options_per_candidate", 16)
        self.declare_parameter("prefer_low_wrist_roll", True)
        self.declare_parameter("preferred_wrist_roll_delta_rad", 0.35)
        self.declare_parameter("max_wrist_roll_delta_rad", 0.90)
        self.declare_parameter("use_pose_goal_fallback", True)
        self.declare_parameter("grasp_pitch_options_rad", [2.20, 2.55, 2.90, 3.141592653589793])
        self.declare_parameter("cloud_target_z_percentiles", [35.0, 55.0, 75.0])
        self.declare_parameter("wrist_confirmation_before_descent", True)
        self.declare_parameter("require_wrist_confirmation_for_execution", False)
        self.declare_parameter("require_wrist_cloud_for_execution", False)
        self.declare_parameter("min_wrist_cloud_points_for_execution", 1)
        self.declare_parameter("max_wrist_cloud_age_s", 5.0)
        self.declare_parameter("wrist_refine_before_grasp", True)
        self.declare_parameter("wrist_refine_top_k", 8)
        self.declare_parameter("wrist_refine_plan_budget_s", 20.0)
        self.declare_parameter("wrist_refine_settle_s", 0.75)
        self.declare_parameter("wrist_refine_camera_standoff_m", 0.22)
        self.declare_parameter("wrist_refine_view_standoffs_m", [0.18, 0.22, 0.26])
        self.declare_parameter("wrist_refine_view_lateral_offsets_m", [0.0, 0.035, 0.070])
        self.declare_parameter("wrist_refine_max_view_attempts", 3)
        self.declare_parameter("wrist_refine_min_ee_z_m", 0.12)
        self.declare_parameter("wrist_refine_require_wrist_cloud", True)
        self.declare_parameter("wrist_refine_min_wrist_cloud_points", 64)
        self.declare_parameter("wrist_refine_fallback_to_initial", False)
        self.declare_parameter("wrist_refine_max_xy_shift_m", 0.08)
        self.declare_parameter("wrist_refine_max_z_shift_m", 0.10)
        self.declare_parameter("post_grasp_lift_m", 0.0)
        self.declare_parameter("wrist_camera_xyz_in_ee", [0.002344943, 0.072594056, -0.119362094])
        self.declare_parameter(
            "wrist_camera_quat_xyzw_in_ee",
            [0.001371523, -0.196624502, 0.979365428, 0.046693506],
        )

        self._allow_execution = bool(self.get_parameter("allow_execution").value)
        self._execution_backend = self._normalized_execution_backend()
        self._moveit_node_name = str(self.get_parameter("moveit_node_name").value)
        self._grasp_frame = str(self.get_parameter("grasp_frame").value)
        self._arm_base_frame = str(self.get_parameter("arm_base_frame").value)
        self._moveit_frame = str(self.get_parameter("moveit_frame").value)
        self._ee_frame = str(self.get_parameter("ee_frame").value)
        self._joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self._feedback_cmd_topic = str(self.get_parameter("feedback_cmd_topic").value)
        self._default_prompt = str(self.get_parameter("default_prompt").value)
        self._default_top_k = int(self.get_parameter("default_top_k").value)
        self._detect_timeout_s = max(1.0, float(self.get_parameter("detect_timeout_s").value))
        self._default_pregrasp_offset_m = float(self.get_parameter("pregrasp_offset_m").value)
        self._max_grasp_radius_m = float(self.get_parameter("max_grasp_radius_m").value)
        self._ik_timeout_s = float(self.get_parameter("ik_timeout_s").value)
        self._planner_parameter_set = str(self.get_parameter("planner_parameter_set").value)
        self._add_table_collision = bool(self.get_parameter("add_table_collision").value)
        self._add_object_bbox_collision = bool(self.get_parameter("add_object_bbox_collision").value)
        self._object_bbox_padding_m = float(self.get_parameter("object_bbox_padding_m").value)
        self._ready_clearance_m = float(self.get_parameter("ready_clearance_m").value)
        self._pregrasp_clearance_m = float(self.get_parameter("pregrasp_clearance_m").value)
        self._close_clearance_m = float(self.get_parameter("close_clearance_m").value)
        self._min_ready_z_m = float(self.get_parameter("min_ready_z_m").value)
        self._min_pregrasp_z_m = float(self.get_parameter("min_pregrasp_z_m").value)
        self._min_close_z_m = float(self.get_parameter("min_close_z_m").value)
        self._use_support_plane_staging = bool(self.get_parameter("use_support_plane_staging").value)
        self._support_plane_frame = str(self.get_parameter("support_plane_frame").value)
        self._support_plane_point_xyz = np.asarray(
            self.get_parameter("support_plane_point_xyz").value,
            dtype=np.float64,
        )
        self._support_plane_normal_xyz = normalize_vector(
            np.asarray(self.get_parameter("support_plane_normal_xyz").value, dtype=np.float64)
        )
        self._close_surface_clearance_m = max(
            0.0,
            float(self.get_parameter("close_surface_clearance_m").value),
        )
        self._gripper_open_position = float(self.get_parameter("gripper_open_position").value)
        self._gripper_closed_position = float(self.get_parameter("gripper_closed_position").value)
        self._gripper_open_allow_stall = bool(self.get_parameter("gripper_open_allow_stall").value)
        self._gripper_max_effort = float(self.get_parameter("gripper_max_effort").value)
        self._gripper_action_timeout_s = float(self.get_parameter("gripper_action_timeout_s").value)
        self._arm_execution_joint_tolerance_rad = float(
            self.get_parameter("arm_execution_joint_tolerance_rad").value
        )
        self._post_stage_settle_s = max(0.0, float(self.get_parameter("post_stage_settle_s").value))
        self._ready_configuration_name = str(self.get_parameter("ready_configuration_name").value)
        self._default_plan_budget_s = float(self.get_parameter("default_plan_budget_s").value)
        self._max_plan_candidates = max(1, int(self.get_parameter("max_plan_candidates").value))
        self._max_primitive_options_per_candidate = max(
            1,
            int(self.get_parameter("max_primitive_options_per_candidate").value),
        )
        self._prefer_low_wrist_roll = bool(self.get_parameter("prefer_low_wrist_roll").value)
        self._preferred_wrist_roll_delta_rad = max(
            0.0,
            float(self.get_parameter("preferred_wrist_roll_delta_rad").value),
        )
        self._max_wrist_roll_delta_rad = float(self.get_parameter("max_wrist_roll_delta_rad").value)
        self._use_pose_goal_fallback = bool(self.get_parameter("use_pose_goal_fallback").value)
        self._grasp_pitch_options_rad = self._float_parameter_list(
            "grasp_pitch_options_rad",
            [2.20, 2.55, 2.90, float(np.pi)],
        )
        self._cloud_target_z_percentiles = self._float_parameter_list(
            "cloud_target_z_percentiles",
            [35.0, 55.0, 75.0],
        )
        self._require_wrist_cloud_for_execution = bool(
            self.get_parameter("require_wrist_cloud_for_execution").value
        )
        self._min_wrist_cloud_points_for_execution = max(
            1,
            int(self.get_parameter("min_wrist_cloud_points_for_execution").value),
        )
        self._max_wrist_cloud_age_s = float(self.get_parameter("max_wrist_cloud_age_s").value)
        self._wrist_refine_before_grasp = bool(self.get_parameter("wrist_refine_before_grasp").value)
        self._wrist_refine_top_k = max(1, int(self.get_parameter("wrist_refine_top_k").value))
        self._wrist_refine_plan_budget_s = max(
            1.0,
            float(self.get_parameter("wrist_refine_plan_budget_s").value),
        )
        self._wrist_refine_settle_s = max(0.0, float(self.get_parameter("wrist_refine_settle_s").value))
        self._wrist_refine_camera_standoff_m = max(
            0.08,
            float(self.get_parameter("wrist_refine_camera_standoff_m").value),
        )
        self._wrist_refine_view_standoffs_m = self._float_parameter_list(
            "wrist_refine_view_standoffs_m",
            [0.18, 0.22, 0.26],
        )
        self._wrist_refine_view_lateral_offsets_m = self._float_parameter_list(
            "wrist_refine_view_lateral_offsets_m",
            [0.0, 0.035, 0.070],
        )
        self._wrist_refine_max_view_attempts = max(
            1,
            int(self.get_parameter("wrist_refine_max_view_attempts").value),
        )
        self._wrist_refine_min_ee_z_m = float(self.get_parameter("wrist_refine_min_ee_z_m").value)
        self._wrist_refine_require_wrist_cloud = bool(
            self.get_parameter("wrist_refine_require_wrist_cloud").value
        )
        self._wrist_refine_min_wrist_cloud_points = max(
            1,
            int(self.get_parameter("wrist_refine_min_wrist_cloud_points").value),
        )
        self._wrist_refine_fallback_to_initial = bool(
            self.get_parameter("wrist_refine_fallback_to_initial").value
        )
        self._wrist_refine_max_xy_shift_m = max(
            0.0,
            float(self.get_parameter("wrist_refine_max_xy_shift_m").value),
        )
        self._wrist_refine_max_z_shift_m = max(
            0.0,
            float(self.get_parameter("wrist_refine_max_z_shift_m").value),
        )
        self._post_grasp_lift_m = max(0.0, float(self.get_parameter("post_grasp_lift_m").value))
        self._wrist_camera_xyz_in_ee = np.asarray(
            self.get_parameter("wrist_camera_xyz_in_ee").value,
            dtype=np.float64,
        )
        self._wrist_camera_quat_xyzw_in_ee = np.asarray(
            self.get_parameter("wrist_camera_quat_xyzw_in_ee").value,
            dtype=np.float64,
        )

        cb_group = ReentrantCallbackGroup()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._detect_client = self.create_client(
            DetectGrasps,
            str(self.get_parameter("detect_service").value),
            callback_group=cb_group,
        )
        self.create_service(
            PlanGrasp,
            str(self.get_parameter("plan_service").value),
            self._on_plan_grasp,
            callback_group=cb_group,
        )
        self._display_pub = self.create_publisher(
            DisplayTrajectory,
            str(self.get_parameter("display_topic").value),
            1,
        )
        self._markers_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("planned_markers_topic").value),
            1,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("object_cloud_topic").value),
            self._on_object_cloud,
            1,
            callback_group=cb_group,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("wrist_object_cloud_topic").value),
            self._on_wrist_object_cloud,
            1,
            callback_group=cb_group,
        )
        self._gripper_client = ActionClient(
            self,
            ParallelGripperCommand,
            str(self.get_parameter("gripper_action").value),
            callback_group=cb_group,
        )

        self._latest_object_cloud: np.ndarray | None = None
        self._latest_wrist_cloud_count = 0
        self._latest_wrist_cloud_stamp_s = 0.0
        self._cloud_lock = threading.Lock()
        self._plan_lock = threading.Lock()

        self._moveit: MoveItPy | None = None
        self._arm = None
        self._plan_params = None
        self._feedback_executor = None
        self._moveit_joint_names = list(ARM_JOINT_NAMES)
        if self._execution_backend == "feedback":
            self._init_feedback_executor()
        else:
            self._init_moveit()
        self._wrist_roll_joint_name = self._find_joint_name("wrist_roll")
        self.get_logger().info(
            f"grasp_planner_node ready: /plan_grasp, arm_base={self._arm_base_frame}, "
            f"moveit_frame={self._moveit_frame}, execution_backend={self._execution_backend}, "
            f"allow_execution={self._allow_execution}"
        )

    def _normalized_execution_backend(self) -> str:
        value = str(self.get_parameter("execution_backend").value).strip().lower()
        if value in {"feedback", "closed_loop", "closed-loop"}:
            return "feedback"
        return "moveit"

    def _init_moveit(self) -> None:
        self.get_logger().info("Initializing MoveItPy grasp planner")
        self._moveit = MoveItPy(
            node_name=self._moveit_node_name,
            remappings={"joint_states": self._joint_states_topic},
        )
        joint_model_group = self._moveit.get_robot_model().get_joint_model_group(PLANNING_GROUP)
        self._moveit_joint_names = list(getattr(joint_model_group, "active_joint_model_names", []))
        if not self._moveit_joint_names:
            self._moveit_joint_names = list(ARM_JOINT_NAMES)
        self._arm = self._moveit.get_planning_component(PLANNING_GROUP)
        self._plan_params = MultiPipelinePlanRequestParameters(self._moveit, [self._planner_parameter_set])

    def _init_feedback_executor(self) -> None:
        self.get_logger().info("Initializing measured-joint feedback grasp executor")
        from so101_grasping.feedback_executor import FeedbackArmExecutor

        self._feedback_executor = FeedbackArmExecutor(
            self,
            joint_states_topic=self._joint_states_topic,
            cmd_topic=self._feedback_cmd_topic,
            base_frame=self._arm_base_frame,
            moveit_frame=self._moveit_frame,
            ee_frame=self._ee_frame,
            arm_joint_names=list(ARM_JOINT_NAMES),
            gripper_joint_name=GRIPPER_JOINT_NAME,
        )
        self._refresh_feedback_settings()

    def _refresh_feedback_settings(self) -> None:
        if self._feedback_executor is None:
            return
        self._feedback_executor.configure(
            rate_hz=float(self.get_parameter("feedback_rate_hz").value),
            position_tolerance_m=float(self.get_parameter("feedback_position_tolerance_m").value),
            ik_position_tolerance_m=float(self.get_parameter("feedback_ik_position_tolerance_m").value),
            joint_tolerance_rad=float(self.get_parameter("feedback_joint_tolerance_rad").value),
            max_correction_iters=int(self.get_parameter("feedback_max_correction_iters").value),
            settle_s=float(self.get_parameter("feedback_settle_s").value),
            min_joint_delta_rad=float(self.get_parameter("feedback_min_joint_delta_rad").value),
            max_joint_speed_rad_s=float(self.get_parameter("feedback_max_joint_speed_rad_s").value),
            min_motion_duration_s=float(self.get_parameter("feedback_min_motion_duration_s").value),
            joint_state_timeout_s=float(self.get_parameter("feedback_joint_state_timeout_s").value),
            correction_command_gain=float(self.get_parameter("feedback_correction_command_gain").value),
            max_overcommand_rad=float(self.get_parameter("feedback_max_overcommand_rad").value),
            look_rot_weight=float(self.get_parameter("feedback_look_rot_weight").value),
            look_rotation_tolerance_rad=np.deg2rad(
                float(self.get_parameter("feedback_look_rotation_tolerance_deg").value)
            ),
        )

    def _float_parameter_list(self, name: str, fallback: list[float]) -> list[float]:
        values = self.get_parameter(name).value
        if values is None:
            return list(fallback)
        try:
            parsed = [float(value) for value in values]
        except TypeError:
            parsed = [float(values)]
        return parsed or list(fallback)

    def _on_object_cloud(self, msg: PointCloud2) -> None:
        points = [
            [float(point[0]), float(point[1]), float(point[2])]
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ]
        array = np.asarray(points, dtype=np.float64)
        source_frame = msg.header.frame_id.strip() or self._grasp_frame
        if len(array):
            try:
                array = self._points_to_arm_base(array, source_frame)
            except Exception as exc:  # noqa: BLE001 - planner can continue with grasp poses.
                self.get_logger().warning(f"Could not transform object cloud from {source_frame}: {exc}")
        with self._cloud_lock:
            self._latest_object_cloud = array

    def _on_wrist_object_cloud(self, msg: PointCloud2) -> None:
        count = 0
        for _ in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            count += 1
        with self._cloud_lock:
            self._latest_wrist_cloud_count = count
            self._latest_wrist_cloud_stamp_s = time.monotonic()

    def _on_plan_grasp(self, request: PlanGrasp.Request, response: PlanGrasp.Response):
        if not self._plan_lock.acquire(blocking=False):
            response.success = False
            response.message = "Another grasp plan is already running"
            return response
        try:
            return self._plan_grasp_locked(request, response)
        except Exception as exc:  # noqa: BLE001 - service callbacks must not kill the planner node.
            self.get_logger().exception(f"plan_grasp failed: {exc}")
            response.success = False
            response.planned = False
            response.executed = False
            response.message = f"plan_grasp failed: {exc}"
            return response
        finally:
            self._plan_lock.release()

    def _plan_grasp_locked(self, request: PlanGrasp.Request, response: PlanGrasp.Response):
        self._execution_backend = self._normalized_execution_backend()
        if self._execution_backend == "feedback":
            if self._feedback_executor is None:
                self._init_feedback_executor()
            self._refresh_feedback_settings()
        elif self._moveit is None:
            self._init_moveit()

        prompt = request.prompt.strip() or self._default_prompt
        top_k = int(request.top_k) if int(request.top_k) > 0 else self._default_top_k
        pregrasp_offset = (
            float(request.pregrasp_offset_m)
            if float(request.pregrasp_offset_m) > 0.0
            else self._default_pregrasp_offset_m
        )
        plan_budget_s = (
            float(request.plan_time_s)
            if float(request.plan_time_s) > 0.0
            else self._default_plan_budget_s
        )
        plan_deadline = time.monotonic() + max(1.0, plan_budget_s)

        detect_timeout_s = min(self._detect_timeout_s, max(1.0, plan_deadline - time.monotonic()))
        detect_response = self._detect_grasps(prompt, top_k, timeout_s=detect_timeout_s)
        if not detect_response.success:
            response.success = False
            response.message = f"detect_grasps failed: {detect_response.message}"
            return response
        if not detect_response.grasps:
            response.success = False
            response.message = "detect_grasps returned no candidates"
            return response
        try:
            planning_grasps = [self._grasp_to_arm_base(grasp) for grasp in detect_response.grasps]
        except Exception as exc:
            response.success = False
            response.message = f"failed to transform grasp candidates into {self._arm_base_frame}: {exc}"
            return response

        if self._execution_backend == "moveit":
            self._sync_collision_scene()
        selection, failures = self._select_plan_for_backend(
            planning_grasps,
            requested_index=int(request.grasp_index),
            pregrasp_offset=pregrasp_offset,
            deadline=plan_deadline,
        )
        if selection is not None:
            response.success = True
            response.planned = True
            response.executed = False
            response.message = (
                f"Planned candidate {selection.candidate_index} with SO-101 primitive "
                f"{selection.strategy_label}: {selection.message}"
            )
            self._fill_plan_response(response, selection)

            if bool(request.execute):
                if not bool(self.get_parameter("allow_execution").value):
                    response.message += "; execution refused because allow_execution=false"
                elif bool(self.get_parameter("wrist_refine_before_grasp").value):
                    executed, execute_message, refined_selection = self._execute_wrist_refined_grasp(
                        selection,
                        prompt=prompt,
                        top_k=top_k,
                        pregrasp_offset=pregrasp_offset,
                        deadline=plan_deadline,
                    )
                    if refined_selection is not None:
                        self._fill_plan_response(response, refined_selection)
                    response.executed = executed
                    response.message += f"; {execute_message}"
                else:
                    confirmation_target, _, _ = _pose_to_numpy(selection.grasp.pose)
                    executed, execute_message = self._execute_primitive(
                        selection.primitive_stages,
                        prompt=prompt,
                        top_k=top_k,
                        confirmation_target=confirmation_target,
                        planned_stages=selection.planned_stages,
                    )
                    response.executed = executed
                    response.message += f"; {execute_message}"
            return response

        response.success = False
        response.planned = False
        response.executed = False
        response.message = f"No candidate produced a {self._execution_backend} plan. " + " | ".join(failures[:10])
        return response

    def _fill_plan_response(self, response: PlanGrasp.Response, selection: PlanSelection) -> None:
        response.selected_grasp = _copy_grasp(selection.grasp)
        response.pregrasp_pose = self._pregrasp_pose_from_stages(selection.planned_stages)
        if self._selection_uses_feedback(selection):
            response.trajectory = self._feedback_trajectory_from_selection(selection)
        else:
            response.trajectory = self._joint_trajectory_from_plan(selection.planned_stages[-1].result)
            self._publish_display_trajectories([
                self._robot_trajectory_msg_from_plan(stage.result) for stage in selection.planned_stages
            ])
        self._publish_plan_markers(selection.grasp, selection.planned_stages)

    def _selection_uses_feedback(self, selection: PlanSelection) -> bool:
        return any(hasattr(stage.result, "joint_names") and hasattr(stage.result, "positions") for stage in selection.planned_stages)

    def _feedback_trajectory_from_selection(self, selection: PlanSelection) -> JointTrajectory:
        trajectory = JointTrajectory()
        if self._feedback_executor is not None:
            trajectory.joint_names = self._feedback_executor.arm_joint_names
        else:
            trajectory.joint_names = list(ARM_JOINT_NAMES)
        stamp_ns = 0
        for stage in selection.planned_stages:
            result = stage.result
            if not hasattr(result, "positions"):
                continue
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in result.positions]
            stamp_ns += 1_000_000_000
            point.time_from_start.sec = stamp_ns // 1_000_000_000
            point.time_from_start.nanosec = stamp_ns % 1_000_000_000
            trajectory.points.append(point)
        return trajectory

    def _frame_transform(self, target_frame: str, source_frame: str) -> np.ndarray:
        if target_frame == source_frame:
            return np.eye(4, dtype=np.float64)
        transform = self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=1.0),
        )
        quat = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
        matrix = quaternion_matrix(quat).astype(np.float64)
        matrix[0, 3] = float(transform.transform.translation.x)
        matrix[1, 3] = float(transform.transform.translation.y)
        matrix[2, 3] = float(transform.transform.translation.z)
        return matrix

    def _points_to_arm_base(self, points: np.ndarray, source_frame: str) -> np.ndarray:
        transform = self._frame_transform(self._arm_base_frame, source_frame)
        hom = np.c_[np.asarray(points, dtype=np.float64), np.ones((len(points), 1), dtype=np.float64)]
        return (transform @ hom.T).T[:, :3]

    def _grasp_to_arm_base(self, grasp: GraspCandidate) -> GraspCandidate:
        source_frame = grasp.header.frame_id.strip() or self._grasp_frame
        planned = _copy_grasp(grasp)
        if source_frame == self._arm_base_frame or source_frame == self._moveit_frame:
            planned.header.frame_id = self._arm_base_frame
            return planned

        target_from_source = self._frame_transform(self._arm_base_frame, source_frame)
        position, quat, _ = _pose_to_numpy(grasp.pose)
        planned.pose = _pose_from_matrix(target_from_source @ _pose_matrix(position, quat))
        planned.header.frame_id = self._arm_base_frame
        return planned

    def _select_plan_for_backend(
        self,
        grasps,
        *,
        requested_index: int,
        pregrasp_offset: float,
        deadline: float,
        strip_initial_ready: bool = False,
    ) -> tuple[PlanSelection | None, list[str]]:
        if self._execution_backend == "feedback":
            return self._select_feedback_plan_from_grasps(
                grasps,
                requested_index=requested_index,
                pregrasp_offset=pregrasp_offset,
                deadline=deadline,
                strip_initial_ready=strip_initial_ready,
            )
        return self._select_plan_from_grasps(
            grasps,
            requested_index=requested_index,
            pregrasp_offset=pregrasp_offset,
            deadline=deadline,
            strip_initial_ready=strip_initial_ready,
        )

    def _select_feedback_plan_from_grasps(
        self,
        grasps,
        *,
        requested_index: int,
        pregrasp_offset: float,
        deadline: float,
        strip_initial_ready: bool = False,
    ) -> tuple[PlanSelection | None, list[str]]:
        if self._feedback_executor is None:
            return None, ["feedback executor is not initialized"]
        ordered_grasps = list(grasps)
        if requested_index > 0 and requested_index < len(ordered_grasps):
            ordered_grasps = [ordered_grasps[requested_index]] + [
                grasp for idx, grasp in enumerate(ordered_grasps) if idx != requested_index
            ]
        ordered_grasps = ordered_grasps[: self._max_plan_candidates]

        failures: list[str] = []
        best_selection: PlanSelection | None = None
        best_wrist_delta = float("inf")
        reference_wrist_roll = self._current_wrist_roll()
        for idx, grasp in enumerate(ordered_grasps):
            if time.monotonic() >= deadline:
                failures.append(f"planning budget exhausted after {idx} candidate(s)")
                break
            reach_filter_reason = self._reach_filter_reason(grasp)
            if reach_filter_reason is not None:
                failures.append(f"candidate {idx}: {reach_filter_reason}")
                continue

            attempted = False
            options = self._make_so101_primitive_options(grasp, pregrasp_offset)[
                : self._max_primitive_options_per_candidate
            ]
            self.get_logger().info(
                f"candidate {idx}: generated {len(options)} feedback primitive option(s) "
                f"after cap={self._max_primitive_options_per_candidate}"
            )
            for option_label, pose_stages in options:
                if time.monotonic() >= deadline:
                    failures.append(f"candidate {idx}: planning budget exhausted")
                    break
                attempted = True
                if strip_initial_ready:
                    pose_stages = self._strip_initial_ready_stage(pose_stages)
                    if not pose_stages:
                        failures.append(f"candidate {idx} {option_label}: no stages after ready strip")
                        continue
                    option_label = f"wrist_refined_{option_label}"
                results, message, wrist_delta = self._feedback_executor.solve_sequence(
                    pose_stages,
                    reference_wrist_roll=reference_wrist_roll,
                    max_wrist_roll_delta_rad=self._max_wrist_roll_delta(),
                )
                if results:
                    planned_stages = [
                        PlannedStage(stage.name, stage.pose, result, stage.configuration_name)
                        for stage, result in zip(pose_stages, results, strict=False)
                    ]
                    wrist_note = self._wrist_roll_note(wrist_delta)
                    selection = PlanSelection(
                        candidate_index=idx,
                        grasp=grasp,
                        planned_stages=planned_stages,
                        primitive_stages=pose_stages,
                        strategy_label=option_label,
                        message=message + self._close_target_note(pose_stages) + wrist_note,
                        wrist_roll_delta_rad=wrist_delta,
                    )
                    if not self._prefer_low_wrist_roll_enabled() or wrist_delta is None:
                        return selection, failures
                    preferred_delta = self._preferred_wrist_roll_delta()
                    if wrist_delta <= preferred_delta:
                        return selection, failures
                    if wrist_delta < best_wrist_delta:
                        best_selection = selection
                        best_wrist_delta = wrist_delta
                    failures.append(
                        f"candidate {idx} {option_label}: feedback feasible but wrist_roll_delta="
                        f"{wrist_delta:.3f}rad; searching for <= {preferred_delta:.3f}rad"
                    )
                    continue
                final_pose_stage = next((stage for stage in reversed(pose_stages) if stage.pose is not None), None)
                final_pose = final_pose_stage.pose.pose.position if final_pose_stage is not None else Point()
                failures.append(
                    f"candidate {idx} {option_label}: {message} "
                    f"close_target=({final_pose.x:.3f},{final_pose.y:.3f},{final_pose.z:.3f})"
                )
            if not attempted:
                failures.append(f"candidate {idx}: no SO-101 primitive options generated")
        if best_selection is not None:
            return best_selection, failures
        return None, failures

    def _select_plan_from_grasps(
        self,
        grasps,
        *,
        requested_index: int,
        pregrasp_offset: float,
        deadline: float,
        strip_initial_ready: bool = False,
    ) -> tuple[PlanSelection | None, list[str]]:
        ordered_grasps = list(grasps)
        if requested_index > 0 and requested_index < len(ordered_grasps):
            ordered_grasps = [ordered_grasps[requested_index]] + [
                grasp for idx, grasp in enumerate(ordered_grasps) if idx != requested_index
            ]
        ordered_grasps = ordered_grasps[: self._max_plan_candidates]

        failures: list[str] = []
        best_selection: PlanSelection | None = None
        best_wrist_delta = float("inf")
        reference_wrist_roll = self._current_wrist_roll()
        for idx, grasp in enumerate(ordered_grasps):
            if time.monotonic() >= deadline:
                failures.append(f"planning budget exhausted after {idx} candidate(s)")
                break
            reach_filter_reason = self._reach_filter_reason(grasp)
            if reach_filter_reason is not None:
                failures.append(f"candidate {idx}: {reach_filter_reason}")
                continue

            attempted = False
            options = self._make_so101_primitive_options(grasp, pregrasp_offset)[
                : self._max_primitive_options_per_candidate
            ]
            self.get_logger().info(
                f"candidate {idx}: generated {len(options)} SO-101 primitive option(s) "
                f"after cap={self._max_primitive_options_per_candidate}"
            )
            for option_label, pose_stages in options:
                if time.monotonic() >= deadline:
                    failures.append(f"candidate {idx}: planning budget exhausted")
                    break
                attempted = True
                if strip_initial_ready:
                    pose_stages = self._strip_initial_ready_stage(pose_stages)
                    if not pose_stages:
                        failures.append(f"candidate {idx} {option_label}: no stages after ready strip")
                        continue
                    option_label = f"wrist_refined_{option_label}"
                self.get_logger().info(f"Trying candidate {idx} SO-101 primitive {option_label}")
                planned_stages, message = self._plan_pose_sequence(pose_stages, deadline=deadline)
                if planned_stages:
                    wrist_delta = self._planned_stages_wrist_roll_delta(
                        planned_stages,
                        reference_wrist_roll,
                    )
                    wrist_reject_reason = self._wrist_roll_reject_reason(wrist_delta)
                    if wrist_reject_reason is not None:
                        failures.append(f"candidate {idx} {option_label}: {wrist_reject_reason}")
                        continue

                    wrist_note = self._wrist_roll_note(wrist_delta)
                    selection = PlanSelection(
                        candidate_index=idx,
                        grasp=grasp,
                        planned_stages=planned_stages,
                        primitive_stages=pose_stages,
                        strategy_label=option_label,
                        message=message + self._close_target_note(pose_stages) + wrist_note,
                        wrist_roll_delta_rad=wrist_delta,
                    )
                    if not self._prefer_low_wrist_roll_enabled() or wrist_delta is None:
                        return selection, failures
                    preferred_delta = self._preferred_wrist_roll_delta()
                    if wrist_delta <= preferred_delta:
                        return selection, failures
                    if wrist_delta < best_wrist_delta:
                        best_selection = selection
                        best_wrist_delta = wrist_delta
                    failures.append(
                        f"candidate {idx} {option_label}: feasible but wrist_roll_delta="
                        f"{wrist_delta:.3f}rad; searching for <= {preferred_delta:.3f}rad"
                    )
                    continue

                final_pose_stage = next((stage for stage in reversed(pose_stages) if stage.pose is not None), None)
                final_pose = final_pose_stage.pose.pose.position if final_pose_stage is not None else Point()
                failures.append(
                    f"candidate {idx} {option_label}: {message} "
                    f"close_target=({final_pose.x:.3f},{final_pose.y:.3f},{final_pose.z:.3f})"
                )
            if not attempted:
                failures.append(f"candidate {idx}: no SO-101 primitive options generated")
        if best_selection is not None:
            return best_selection, failures
        return None, failures

    def _strip_initial_ready_stage(self, stages: list[PrimitiveStage]) -> list[PrimitiveStage]:
        stripped = list(stages)
        while stripped and stripped[0].configuration_name is not None:
            stripped = stripped[1:]
        return stripped

    def _detect_grasps(self, prompt: str, top_k: int, timeout_s: float | None = None):
        if not self._detect_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/detect_grasps service is not available")
        request = DetectGrasps.Request()
        request.prompt = prompt
        request.top_k = top_k
        future = self._detect_client.call_async(request)
        timeout_s = self._detect_timeout_s if timeout_s is None else max(1.0, float(timeout_s))
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            raise TimeoutError(f"/detect_grasps timed out after {timeout_s:.1f}s")
        return future.result()

    def _wrist_refine_target(self, grasp: GraspCandidate) -> np.ndarray:
        options = self._prefer_cloud_targets(self._target_position_options(grasp))
        if options:
            return np.asarray(options[0][1], dtype=np.float64)
        position, _, _ = _pose_to_numpy(grasp.pose)
        return position

    def _wrist_refine_view_normal(self) -> np.ndarray | None:
        if not self._support_plane_staging_enabled():
            return None
        try:
            _, normal = self._support_plane_in_arm_base()
        except Exception as exc:  # noqa: BLE001 - fall back to base +Z view generation.
            self.get_logger().warning(f"wrist-refine could not read support-plane normal: {exc}")
            return None
        return normal

    def _make_wrist_camera_look_stage(self, grasp: GraspCandidate) -> PrimitiveStage:
        options = self._wrist_refine_view_stage_options_for_target(self._wrist_refine_target(grasp))
        if not options:
            raise RuntimeError("no wrist camera view candidates generated")
        _, stages = options[0]
        return stages[-1]

    def _wrist_refine_view_stage_options_for_target(
        self,
        target: np.ndarray,
    ) -> list[tuple[str, list[PrimitiveStage]]]:
        self._wrist_refine_view_standoffs_m = self._float_parameter_list(
            "wrist_refine_view_standoffs_m",
            [0.18, 0.22, 0.26],
        )
        self._wrist_refine_view_lateral_offsets_m = self._float_parameter_list(
            "wrist_refine_view_lateral_offsets_m",
            [0.0, 0.035, 0.070],
        )
        self._wrist_refine_min_ee_z_m = float(self.get_parameter("wrist_refine_min_ee_z_m").value)
        candidates = wrist_view_candidates(
            target=np.asarray(target, dtype=np.float64),
            wrist_camera_xyz_in_ee=self._wrist_camera_xyz_in_ee,
            wrist_camera_quat_xyzw_in_ee=self._wrist_camera_quat_xyzw_in_ee,
            standoffs_m=self._wrist_refine_view_standoffs_m,
            lateral_offsets_m=self._wrist_refine_view_lateral_offsets_m,
            min_ee_z_m=self._wrist_refine_min_ee_z_m,
            plane_normal=self._wrist_refine_view_normal(),
        )
        options: list[tuple[str, list[PrimitiveStage]]] = []
        for candidate in candidates:
            options.append((candidate.label, [*self._initial_ready_stages(), self._view_stage_from_candidate(target, candidate)]))
        return options

    def _view_stage_from_candidate(self, target: np.ndarray, candidate: WristViewCandidate) -> PrimitiveStage:
        self.get_logger().info(
            "wrist_camera_look: "
            f"view={candidate.label} "
            f"target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}) "
            f"camera=({candidate.camera_position[0]:.3f},{candidate.camera_position[1]:.3f},"
            f"{candidate.camera_position[2]:.3f}) "
            f"target_in_camera=({candidate.target_in_camera[0]:.3f},{candidate.target_in_camera[1]:.3f},"
            f"{candidate.target_in_camera[2]:.3f}) "
            f"ee=({candidate.ee_position[0]:.3f},{candidate.ee_position[1]:.3f},"
            f"{candidate.ee_position[2]:.3f})"
        )
        return PrimitiveStage("wrist_camera_look", pose=self._make_pose(candidate.ee_position, candidate.ee_quat_xyzw))

    def _wrist_refine_view_stages(self, selection: PlanSelection) -> list[PrimitiveStage]:
        options = self._wrist_refine_view_stage_options(selection)
        return options[0][1] if options else []

    def _wrist_refine_view_stage_options(self, selection: PlanSelection) -> list[tuple[str, list[PrimitiveStage]]]:
        return self._wrist_refine_view_stage_options_for_target(self._wrist_refine_target(selection.grasp))

    def _initial_ready_stages(self) -> list[PrimitiveStage]:
        name = str(self.get_parameter("ready_configuration_name").value).strip()
        if name.lower() in {"", "none", "off", "false", "skip"}:
            return []
        self._ready_configuration_name = name
        return [PrimitiveStage("ready_up", configuration_name=name)]

    def _filter_wrist_refine_candidates(
        self,
        grasps: list[GraspCandidate],
        initial_target: np.ndarray,
    ) -> tuple[list[GraspCandidate], list[str]]:
        self._wrist_refine_max_xy_shift_m = max(
            0.0,
            float(self.get_parameter("wrist_refine_max_xy_shift_m").value),
        )
        self._wrist_refine_max_z_shift_m = max(
            0.0,
            float(self.get_parameter("wrist_refine_max_z_shift_m").value),
        )
        kept: list[GraspCandidate] = []
        rejected: list[str] = []
        for idx, grasp in enumerate(grasps):
            position, _, _ = _pose_to_numpy(grasp.pose)
            xy_shift = float(np.linalg.norm(position[:2] - initial_target[:2]))
            z_shift = abs(float(position[2] - initial_target[2]))
            if xy_shift > self._wrist_refine_max_xy_shift_m or z_shift > self._wrist_refine_max_z_shift_m:
                rejected.append(
                    f"{idx}:shift_xy={xy_shift:.3f}m z={z_shift:.3f}m "
                    f"pos=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f})"
                )
                continue
            kept.append(grasp)
        return kept, rejected

    def _support_plane_staging_enabled(self) -> bool:
        self._use_support_plane_staging = bool(self.get_parameter("use_support_plane_staging").value)
        return self._use_support_plane_staging

    def _support_plane_in_arm_base(self) -> tuple[np.ndarray, np.ndarray]:
        self._support_plane_frame = str(self.get_parameter("support_plane_frame").value)
        self._support_plane_point_xyz = np.asarray(
            self.get_parameter("support_plane_point_xyz").value,
            dtype=np.float64,
        )
        self._support_plane_normal_xyz = normalize_vector(
            np.asarray(self.get_parameter("support_plane_normal_xyz").value, dtype=np.float64)
        )
        target_from_plane = self._frame_transform(self._arm_base_frame, self._support_plane_frame)
        point_hom = np.r_[self._support_plane_point_xyz, 1.0]
        point = (target_from_plane @ point_hom)[:3]
        normal = normalize_vector(target_from_plane[:3, :3] @ self._support_plane_normal_xyz)
        return point.astype(np.float64), normal.astype(np.float64)

    def _surface_relative_stages(
        self,
        target: np.ndarray,
        *,
        ready_clearance_m: float,
        pregrasp_clearance_m: float,
    ) -> SurfaceRelativeStages | None:
        if not self._support_plane_staging_enabled():
            return None
        plane_point, plane_normal = self._support_plane_in_arm_base()
        self._close_surface_clearance_m = max(
            0.0,
            float(self.get_parameter("close_surface_clearance_m").value),
        )
        return surface_relative_stages(
            target,
            plane_point,
            plane_normal,
            ready_clearance_m=ready_clearance_m,
            pregrasp_clearance_m=pregrasp_clearance_m,
            close_clearance_m=self._close_clearance_m,
            close_surface_clearance_m=self._close_surface_clearance_m,
        )

    def _legacy_pregrasp_close(self, target: np.ndarray, *, pregrasp_clearance_m: float) -> tuple[np.ndarray, np.ndarray]:
        pregrasp = target.copy()
        close = target.copy()
        pregrasp[2] = max(float(target[2] + pregrasp_clearance_m), self._min_pregrasp_z_m)
        close[2] = max(float(target[2] + self._close_clearance_m), self._min_close_z_m)
        return pregrasp, close

    def _close_target_note(self, stages: list[PrimitiveStage]) -> str:
        close_stage = next(
            (stage for stage in reversed(stages) if stage.pose is not None and stage.name.endswith("descent_close")),
            None,
        )
        if close_stage is None or close_stage.pose is None:
            return ""
        position = np.asarray(
            [
                close_stage.pose.pose.position.x,
                close_stage.pose.pose.position.y,
                close_stage.pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        note = f"; close_target=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f})"
        if self._support_plane_staging_enabled():
            try:
                plane_point, plane_normal = self._support_plane_in_arm_base()
                clearance = signed_plane_distance(position, plane_point, plane_normal)
                note += f"; surface_clearance={clearance:.3f}m"
            except Exception as exc:  # noqa: BLE001 - planning already succeeded; keep the response usable.
                note += f"; surface_clearance=unavailable({exc})"
        return note

    def _target_position_options(self, grasp: GraspCandidate) -> list[tuple[str, np.ndarray]]:
        grasp_position, _, _ = _pose_to_numpy(grasp.pose)
        source_label = str(grasp.source).strip().lower() or "grasp"
        options = [(f"{source_label}_center", grasp_position)]

        with self._cloud_lock:
            cloud = None if self._latest_object_cloud is None else self._latest_object_cloud.copy()
        if cloud is None or len(cloud) < 1:
            return options

        finite = cloud[np.isfinite(cloud).all(axis=1)]
        if len(finite) < 1:
            return options
        centroid_xy = np.median(finite[:, :2], axis=0).astype(np.float64)
        if np.linalg.norm(centroid_xy - grasp_position[:2]) > 0.30:
            return options

        for percentile in self._cloud_target_z_percentiles:
            target = np.asarray(
                [
                    float(centroid_xy[0]),
                    float(centroid_xy[1]),
                    float(np.percentile(finite[:, 2], percentile)),
                ],
                dtype=np.float64,
            )
            if any(np.linalg.norm(target - existing) < 0.012 for _, existing in options):
                continue
            options.append((f"cloud_z{int(round(percentile))}", target))
        return options

    def _grasp_yaw_options(self, rotation: np.ndarray) -> list[tuple[str, float]]:
        candidates: list[float] = []
        for axis_index in (1, 0):
            axis_xy = np.asarray(rotation[:2, axis_index], dtype=np.float64)
            if float(np.linalg.norm(axis_xy)) > 1e-6:
                candidates.append(float(np.arctan2(axis_xy[1], axis_xy[0])))
        if not candidates:
            candidates.append(0.0)

        yaw_options: list[tuple[str, float]] = []
        seen: list[float] = []
        for base_yaw in candidates:
            for delta in (0.0, np.pi / 2.0, -np.pi / 2.0, np.pi):
                yaw = self._normalize_angle(base_yaw + delta)
                if any(abs(self._normalize_angle(yaw - previous)) < 0.17 for previous in seen):
                    continue
                seen.append(yaw)
                yaw_options.append((f"yaw_{int(round(np.degrees(yaw)))}deg", yaw))
                if len(yaw_options) >= 6:
                    return yaw_options
        return yaw_options

    def _grasp_orientation_options(self, rotation: np.ndarray, *, max_yaws: int = 4) -> list[tuple[str, np.ndarray]]:
        """Return sampled soft orientation hints for a 5-DOF arm.

        The MoveIt kinematics plugin is configured position-first. These
        orientations still influence approximate IK seeds when possible, but
        planning must remain feasible when exact roll/pitch/yaw is impossible.
        """
        yaw_options = self._grasp_yaw_options(rotation)[:max(1, max_yaws)]
        if not yaw_options:
            yaw_options = [("yaw_0deg", 0.0)]

        options: list[tuple[str, np.ndarray]] = []
        seen: list[np.ndarray] = []
        for pitch in self._grasp_pitch_options_rad:
            pitch = float(np.clip(float(pitch), -np.pi, np.pi))
            pitch_label = f"pitch_{int(round(np.degrees(pitch)))}deg"
            for yaw_label, yaw in yaw_options:
                quat = np.asarray(quaternion_from_euler(0.0, pitch, yaw), dtype=np.float64)
                quat = quat / max(float(np.linalg.norm(quat)), 1e-9)
                if any(float(np.linalg.norm(quat - previous)) < 1e-4 for previous in seen):
                    continue
                seen.append(quat)
                options.append((f"{pitch_label}_{yaw_label}", quat))
        return options

    def _prefer_cloud_targets(self, target_options: list[tuple[str, np.ndarray]]) -> list[tuple[str, np.ndarray]]:
        return sorted(
            target_options,
            key=lambda item: (0 if item[0].startswith("cloud_") else 1, item[0]),
        )

    def _make_so101_primitive_options(
        self,
        grasp: GraspCandidate,
        offset_m: float,
    ) -> list[tuple[str, list[PrimitiveStage]]]:
        if str(grasp.source).strip().lower() == "ggcnn":
            return self._make_ggcnn_planar_primitive_options(grasp, offset_m)

        _, _, rotation = _pose_to_numpy(grasp.pose)
        pregrasp_clearance = min(max(float(offset_m), 0.045), 0.12)

        options: list[tuple[str, list[PrimitiveStage]]] = []
        target_options = self._prefer_cloud_targets(self._target_position_options(grasp))
        orientation_options = self._grasp_orientation_options(rotation)
        for target_label, target in target_options:
            for orientation_label, quat in orientation_options:
                surface_stages = self._surface_relative_stages(
                    target,
                    ready_clearance_m=self._ready_clearance_m,
                    pregrasp_clearance_m=pregrasp_clearance,
                )
                if surface_stages is None:
                    pregrasp, close = self._legacy_pregrasp_close(
                        target,
                        pregrasp_clearance_m=pregrasp_clearance,
                    )
                    label = f"{target_label}_{orientation_label}"
                else:
                    pregrasp = surface_stages.pregrasp
                    close = surface_stages.close
                    label = f"{target_label}_{orientation_label}_surface"
                stages = [
                    *self._initial_ready_stages(),
                    PrimitiveStage("pregrasp_align", pose=self._make_pose(pregrasp, quat)),
                    PrimitiveStage("descent_close", pose=self._make_pose(close, quat)),
                ]
                options.append((label, stages))
        return options

    def _make_ggcnn_planar_primitive_options(
        self,
        grasp: GraspCandidate,
        offset_m: float,
    ) -> list[tuple[str, list[PrimitiveStage]]]:
        _, _, rotation = _pose_to_numpy(grasp.pose)
        pregrasp_clearance = min(max(float(offset_m), 0.055), 0.12)

        orientation_options = self._grasp_orientation_options(rotation, max_yaws=4)

        options: list[tuple[str, list[PrimitiveStage]]] = []
        # GG-CNN already returns a grasp peak in image space. The object-cloud
        # median is useful as a fallback, but on small objects it can shift the
        # target by centimeters if the mask includes table, shadow, or handle
        # pixels.
        target_options = self._target_position_options(grasp)
        for target_label, target in target_options:
            for orientation_label, quat in orientation_options:
                surface_stages = self._surface_relative_stages(
                    target,
                    ready_clearance_m=self._ready_clearance_m,
                    pregrasp_clearance_m=pregrasp_clearance,
                )
                if surface_stages is None:
                    ready = target.copy()
                    ready[2] = max(float(target[2] + self._ready_clearance_m), self._min_ready_z_m)
                    pregrasp, close = self._legacy_pregrasp_close(
                        target,
                        pregrasp_clearance_m=pregrasp_clearance,
                    )
                    label = f"ggcnn_planar_{target_label}_{orientation_label}"
                else:
                    ready = surface_stages.ready
                    pregrasp = surface_stages.pregrasp
                    close = surface_stages.close
                    label = f"ggcnn_planar_{target_label}_{orientation_label}_surface"
                stages = [
                    *self._initial_ready_stages(),
                    PrimitiveStage("ggcnn_ready_over_object", pose=self._make_pose(ready, quat)),
                    PrimitiveStage("ggcnn_pregrasp_align", pose=self._make_pose(pregrasp, quat)),
                    PrimitiveStage("ggcnn_descent_close", pose=self._make_pose(close, quat)),
                ]
                options.append((label, stages))
        return options

    def _normalize_angle(self, radians: float) -> float:
        return float((radians + np.pi) % (2.0 * np.pi) - np.pi)

    def _find_joint_name(self, suffix: str) -> str | None:
        for name in self._moveit_joint_names or ARM_JOINT_NAMES:
            if name == suffix or name.endswith(f"/{suffix}"):
                return name
        return None

    def _make_pose(self, position: np.ndarray, quat: np.ndarray) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._moveit_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = _point(position)
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        return pose

    def _pose_summary(self, pose_stamped: PoseStamped) -> str:
        pose = pose_stamped.pose
        return (
            f"pos=({pose.position.x:.3f},{pose.position.y:.3f},{pose.position.z:.3f}) "
            f"quat=({pose.orientation.x:.3f},{pose.orientation.y:.3f},"
            f"{pose.orientation.z:.3f},{pose.orientation.w:.3f})"
        )

    def _current_ee_quaternion(self) -> np.ndarray | None:
        robot_model = self._moveit.get_robot_model()
        robot_state = RobotState(robot_model)
        monitor = self._moveit.get_planning_scene_monitor()
        with monitor.read_only() as scene:
            current = scene.current_state
            robot_state.set_joint_group_positions(
                PLANNING_GROUP,
                current.get_joint_group_positions(PLANNING_GROUP),
            )
        robot_state.update()
        try:
            transform = np.asarray(robot_state.get_global_link_transform(self._ee_frame), dtype=np.float64)
        except Exception as exc:  # noqa: BLE001 - fallback planning can continue without it
            self.get_logger().warning(f"Could not read current EE transform for orientation fallback: {exc}")
            return None
        if transform.shape != (4, 4):
            self.get_logger().warning(f"Unexpected current EE transform shape: {transform.shape}")
            return None
        return np.asarray(quaternion_from_matrix(transform), dtype=np.float64)

    def _reach_filter_reason(self, grasp: GraspCandidate) -> str | None:
        position, _, _ = _pose_to_numpy(grasp.pose)
        radius = float(np.linalg.norm(position[:2]))
        if radius > self._max_grasp_radius_m:
            return (
                f"outside radius filter "
                f"(radius={radius:.3f}m > {self._max_grasp_radius_m:.3f}m, "
                f"pos=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}))"
            )
        if self._support_plane_staging_enabled():
            return None
        if position[2] < -0.08:
            return (
                f"below z filter "
                f"(z={position[2]:.3f}m < -0.080m, "
                f"pos=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}))"
            )
        if position[2] > 0.80:
            return (
                f"above z filter "
                f"(z={position[2]:.3f}m > 0.800m, "
                f"pos=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}))"
            )
        return None

    def _current_robot_state(self) -> RobotState:
        robot_model = self._moveit.get_robot_model()
        robot_state = RobotState(robot_model)
        monitor = self._moveit.get_planning_scene_monitor()
        with monitor.read_only() as scene:
            current = scene.current_state
            robot_state.set_joint_group_positions(
                PLANNING_GROUP,
                current.get_joint_group_positions(PLANNING_GROUP),
            )
        robot_state.update()
        return robot_state

    def _copy_robot_state(self, source: RobotState) -> RobotState:
        robot_state = RobotState(self._moveit.get_robot_model())
        robot_state.set_joint_group_positions(
            PLANNING_GROUP,
            source.get_joint_group_positions(PLANNING_GROUP),
        )
        robot_state.update()
        return robot_state

    def _robot_trajectory_msg_from_plan(self, plan_result: Any) -> Any:
        trajectory = getattr(plan_result, "trajectory", plan_result)
        if hasattr(trajectory, "get_robot_trajectory_msg"):
            return trajectory.get_robot_trajectory_msg()
        return trajectory

    def _joint_trajectory_from_plan(self, plan_result: Any) -> Any:
        trajectory_msg = self._robot_trajectory_msg_from_plan(plan_result)
        joint_trajectory = getattr(trajectory_msg, "joint_trajectory", None)
        if joint_trajectory is None:
            raise AttributeError(f"Plan trajectory has no joint_trajectory: {type(trajectory_msg)!r}")
        return joint_trajectory

    def _state_joint_position(self, state: RobotState, joint_name: str | None) -> float | None:
        if joint_name is None:
            return None
        joint_names = self._moveit_joint_names or ARM_JOINT_NAMES
        if joint_name not in joint_names:
            return None
        positions = list(state.get_joint_group_positions(PLANNING_GROUP))
        index = joint_names.index(joint_name)
        if index >= len(positions):
            return None
        return float(positions[index])

    def _current_wrist_roll(self) -> float | None:
        if self._execution_backend == "feedback" and self._feedback_executor is not None:
            return self._feedback_executor.current_wrist_roll()
        try:
            return self._state_joint_position(self._current_robot_state(), self._wrist_roll_joint_name)
        except Exception as exc:  # noqa: BLE001 - planning can continue without the roll heuristic.
            self.get_logger().warning(f"Could not read current wrist_roll for grasp scoring: {exc}")
            return None

    def _trajectory_wrist_roll_delta(self, plan_result: Any, reference_roll: float | None) -> float | None:
        if reference_roll is None or self._wrist_roll_joint_name is None:
            return None
        trajectory = self._joint_trajectory_from_plan(plan_result)
        joint_names = list(trajectory.joint_names)
        if self._wrist_roll_joint_name not in joint_names:
            return None
        index = joint_names.index(self._wrist_roll_joint_name)
        max_delta = 0.0
        found = False
        for point in trajectory.points:
            positions = list(point.positions)
            if index >= len(positions):
                continue
            found = True
            delta = abs(self._normalize_angle(float(positions[index]) - reference_roll))
            max_delta = max(max_delta, delta)
        return max_delta if found else None

    def _planned_stages_wrist_roll_delta(
        self,
        planned_stages: list[PlannedStage],
        reference_roll: float | None,
    ) -> float | None:
        deltas = [
            delta
            for stage in planned_stages
            if (delta := self._trajectory_wrist_roll_delta(stage.result, reference_roll)) is not None
        ]
        return max(deltas) if deltas else None

    def _prefer_low_wrist_roll_enabled(self) -> bool:
        self._prefer_low_wrist_roll = bool(self.get_parameter("prefer_low_wrist_roll").value)
        return self._prefer_low_wrist_roll

    def _preferred_wrist_roll_delta(self) -> float:
        self._preferred_wrist_roll_delta_rad = max(
            0.0,
            float(self.get_parameter("preferred_wrist_roll_delta_rad").value),
        )
        return self._preferred_wrist_roll_delta_rad

    def _max_wrist_roll_delta(self) -> float:
        self._max_wrist_roll_delta_rad = float(self.get_parameter("max_wrist_roll_delta_rad").value)
        return self._max_wrist_roll_delta_rad

    def _wrist_roll_reject_reason(self, wrist_delta: float | None) -> str | None:
        limit = self._max_wrist_roll_delta()
        if wrist_delta is None or limit <= 0.0:
            return None
        if wrist_delta > limit:
            return f"wrist_roll_delta={wrist_delta:.3f}rad exceeds max_wrist_roll_delta_rad={limit:.3f}"
        return None

    def _wrist_roll_note(self, wrist_delta: float | None) -> str:
        if wrist_delta is None:
            return ""
        return f"; wrist_roll_delta={wrist_delta:.3f}rad"

    def _state_from_plan_end(self, plan_result: Any, fallback: RobotState) -> RobotState:
        trajectory = self._joint_trajectory_from_plan(plan_result)
        if not trajectory.points:
            return self._copy_robot_state(fallback)

        joint_names = list(trajectory.joint_names)
        positions = list(trajectory.points[-1].positions)
        if len(joint_names) != len(positions):
            return self._copy_robot_state(fallback)

        by_name = dict(zip(joint_names, positions, strict=False))
        if all(name in by_name for name in self._moveit_joint_names):
            ordered = [float(by_name[name]) for name in self._moveit_joint_names]
        elif len(positions) == len(self._moveit_joint_names):
            ordered = [float(value) for value in positions]
        else:
            return self._copy_robot_state(fallback)

        robot_state = RobotState(self._moveit.get_robot_model())
        robot_state.set_joint_group_positions(PLANNING_GROUP, ordered)
        robot_state.update()
        return robot_state

    def _plan_pose_sequence(
        self,
        pose_stages: list[PrimitiveStage],
        *,
        deadline: float | None = None,
    ) -> tuple[list[PlannedStage], str]:
        start_state = self._current_robot_state()
        planned: list[PlannedStage] = []
        messages: list[str] = []
        for stage in pose_stages:
            if deadline is not None and time.monotonic() >= deadline:
                return [], f"{stage.name}: planning budget exhausted"
            if stage.configuration_name is not None:
                plan_result, next_state, message = self._plan_to_configuration(
                    stage.configuration_name,
                    start_state=start_state,
                )
            elif stage.pose is not None:
                plan_result, next_state, message = self._plan_to_pose(stage.pose, start_state=start_state)
            else:
                return [], f"{stage.name}: no pose or named configuration"
            if not plan_result:
                return [], f"{stage.name}: {message}"
            planned.append(PlannedStage(stage.name, stage.pose, plan_result, stage.configuration_name))
            messages.append(f"{stage.name}: {message}")
            start_state = next_state if next_state is not None else self._state_from_plan_end(plan_result, start_state)
        return planned, " -> ".join(messages)

    def _pregrasp_pose_from_stages(self, planned_stages: list[PlannedStage]) -> PoseStamped:
        for stage in planned_stages:
            if stage.pose is not None and "pregrasp" in stage.name:
                return stage.pose
        for stage in planned_stages:
            if stage.pose is not None:
                return stage.pose
        return PoseStamped()

    def _plan_to_configuration(self, configuration_name: str, start_state: RobotState | None = None) -> tuple[Any | None, RobotState | None, str]:
        start_state = self._current_robot_state() if start_state is None else self._copy_robot_state(start_state)
        self._arm.set_start_state(robot_state=start_state)
        self._arm.set_goal_state(configuration_name=configuration_name)
        plan_result = self._arm.plan(multi_plan_parameters=self._plan_params)
        if not plan_result:
            return None, None, f"MoveIt named target '{configuration_name}' planning failed"
        return plan_result, self._state_from_plan_end(plan_result, start_state), f"MoveIt named target '{configuration_name}' plan found"

    def _plan_to_pose(self, pose_stamped: PoseStamped, start_state: RobotState | None = None) -> tuple[Any | None, RobotState | None, str]:
        start_state = self._current_robot_state() if start_state is None else self._copy_robot_state(start_state)
        robot_state = self._copy_robot_state(start_state)

        if not robot_state.set_from_ik(PLANNING_GROUP, pose_stamped.pose, self._ee_frame, self._ik_timeout_s):
            self._use_pose_goal_fallback = bool(self.get_parameter("use_pose_goal_fallback").value)
            if not self._use_pose_goal_fallback:
                return None, None, f"IK failed; pose-goal fallback disabled; {self._pose_summary(pose_stamped)}"
            self._arm.set_start_state(robot_state=start_state)
            self._arm.set_goal_state(pose_stamped_msg=pose_stamped, pose_link=self._ee_frame)
            plan_result = self._arm.plan(multi_plan_parameters=self._plan_params)
            if plan_result:
                return plan_result, self._state_from_plan_end(plan_result, start_state), "MoveIt pose-goal plan found after direct IK failed"
            return None, None, f"IK failed; pose-goal planning failed; {self._pose_summary(pose_stamped)}"
        robot_state.update()

        self._arm.set_start_state(robot_state=start_state)
        self._arm.set_goal_state(robot_state=robot_state)
        plan_result = self._arm.plan(multi_plan_parameters=self._plan_params)
        if not plan_result:
            return None, None, f"OMPL/collision planning failed; {self._pose_summary(pose_stamped)}"
        return plan_result, robot_state, "MoveIt IK + collision plan found"

    def _sync_collision_scene(self) -> None:
        objects: list[CollisionObject] = []
        if self._add_table_collision:
            objects.append(self._make_box_collision_object(
                object_id="so101_table",
                center=np.asarray(self.get_parameter("table_center_xyz").value, dtype=np.float64),
                size=np.asarray(self.get_parameter("table_size_xyz").value, dtype=np.float64),
            ))

        if self._add_object_bbox_collision:
            object_box = self._make_object_bbox_collision()
            if object_box is not None:
                objects.append(object_box)

        if not objects:
            return
        monitor = self._moveit.get_planning_scene_monitor()
        with monitor.read_write() as scene:
            for collision_object in objects:
                scene.apply_collision_object(collision_object)

    def _make_object_bbox_collision(self) -> CollisionObject | None:
        with self._cloud_lock:
            cloud = None if self._latest_object_cloud is None else self._latest_object_cloud.copy()
        if cloud is None or len(cloud) < 1:
            return None
        finite = cloud[np.isfinite(cloud).all(axis=1)]
        if len(finite) < 1:
            return None
        mins = np.percentile(finite, 2.5, axis=0) - self._object_bbox_padding_m
        maxs = np.percentile(finite, 97.5, axis=0) + self._object_bbox_padding_m
        size = np.maximum(maxs - mins, 0.02)
        center = 0.5 * (mins + maxs)
        return self._make_box_collision_object("segmented_object_bbox", center, size)

    def _make_box_collision_object(self, object_id: str, center: np.ndarray, size: np.ndarray) -> CollisionObject:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]

        collision_object = CollisionObject()
        collision_object.header.frame_id = self._moveit_frame
        collision_object.id = object_id
        collision_object.primitives = [primitive]
        pose = PoseStamped()
        pose.pose.position = _point(center)
        pose.pose.orientation.w = 1.0
        collision_object.primitive_poses = [pose.pose]
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def _wait_for_future(self, future, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return bool(future.done())

    def _send_gripper_goal(self, position: float, *, allow_stall: bool) -> tuple[bool, str]:
        timeout_s = max(0.5, self._gripper_action_timeout_s)
        if not self._gripper_client.wait_for_server(timeout_sec=timeout_s):
            return False, "gripper action server not available"

        goal = ParallelGripperCommand.Goal()
        goal.command.name = [GRIPPER_JOINT_NAME]
        goal.command.position = [float(position)]
        goal.command.effort = [float(self._gripper_max_effort)]
        send_future = self._gripper_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, timeout_s):
            return False, "gripper goal send timed out"
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, "gripper goal rejected"

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_s):
            return False, "gripper result timed out"
        result = result_future.result().result
        reached_goal = bool(getattr(result, "reached_goal", False))
        stalled = bool(getattr(result, "stalled", False))
        if reached_goal:
            return True, "gripper reached goal"
        if allow_stall and stalled:
            return True, "gripper stalled before exact target"
        return False, f"gripper did not reach goal (stalled={stalled})"

    def _current_arm_positions(self) -> np.ndarray:
        if self._execution_backend == "feedback" and self._feedback_executor is not None:
            return self._feedback_executor.current_arm_positions()
        state = self._current_robot_state()
        return np.asarray(state.get_joint_group_positions(PLANNING_GROUP), dtype=np.float64)

    def _state_arm_positions(self, state: RobotState) -> np.ndarray:
        return np.asarray(state.get_joint_group_positions(PLANNING_GROUP), dtype=np.float64)

    def _verify_arm_state(self, expected_state: RobotState, stage_name: str) -> tuple[bool, str]:
        expected = self._state_arm_positions(expected_state)
        actual = self._current_arm_positions()
        if expected.shape != actual.shape:
            return False, f"{stage_name}: state shape mismatch expected={expected.shape} actual={actual.shape}"
        error = np.abs(actual - expected)
        worst_index = int(np.argmax(error))
        worst_error = float(error[worst_index])
        joint_names = self._moveit_joint_names or ARM_JOINT_NAMES
        worst_name = joint_names[worst_index] if worst_index < len(joint_names) else f"joint[{worst_index}]"
        if worst_error > self._arm_execution_joint_tolerance_rad:
            return (
                False,
                f"{stage_name}: hardware did not reach planned target "
                f"(worst {worst_name} error={worst_error:.3f} rad "
                f"> {self._arm_execution_joint_tolerance_rad:.3f})",
            )
        return True, f"{stage_name}: reached target (worst_error={worst_error:.3f} rad)"

    def _plan_primitive_stage_from_current(self, stage: PrimitiveStage) -> tuple[Any | None, RobotState | None, str]:
        if stage.configuration_name is not None:
            return self._plan_to_configuration(stage.configuration_name)
        if stage.pose is not None:
            return self._plan_to_pose(stage.pose)
        return None, None, f"{stage.name}: no pose or named configuration"

    def _primitive_from_planned_stage(self, stage: PlannedStage) -> PrimitiveStage:
        return PrimitiveStage(stage.name, pose=stage.pose, configuration_name=stage.configuration_name)

    def _execution_wrist_cloud_check(self) -> tuple[bool, str]:
        self._require_wrist_cloud_for_execution = bool(
            self.get_parameter("require_wrist_cloud_for_execution").value
        )
        if not self._require_wrist_cloud_for_execution:
            return True, "wrist-cloud execution gate disabled"

        with self._cloud_lock:
            count = int(self._latest_wrist_cloud_count)
            stamp_s = float(self._latest_wrist_cloud_stamp_s)
        age_s = time.monotonic() - stamp_s if stamp_s > 0.0 else float("inf")
        if count < self._min_wrist_cloud_points_for_execution:
            return (
                False,
                f"wrist object cloud has {count} point(s), "
                f"need >= {self._min_wrist_cloud_points_for_execution}",
            )
        if age_s > self._max_wrist_cloud_age_s:
            return (
                False,
                f"wrist object cloud is stale ({age_s:.1f}s > {self._max_wrist_cloud_age_s:.1f}s)",
            )
        return True, f"wrist object cloud ok ({count} points, age={age_s:.1f}s)"

    def _wrist_refine_cloud_check(self) -> tuple[bool, str]:
        self._wrist_refine_require_wrist_cloud = bool(
            self.get_parameter("wrist_refine_require_wrist_cloud").value
        )
        if not self._wrist_refine_require_wrist_cloud:
            return True, "wrist-refine cloud gate disabled"

        with self._cloud_lock:
            count = int(self._latest_wrist_cloud_count)
            stamp_s = float(self._latest_wrist_cloud_stamp_s)
        age_s = time.monotonic() - stamp_s if stamp_s > 0.0 else float("inf")
        if count < self._wrist_refine_min_wrist_cloud_points:
            return (
                False,
                f"wrist-refine needs wrist cloud >= {self._wrist_refine_min_wrist_cloud_points} "
                f"point(s), got {count}",
            )
        if age_s > self._max_wrist_cloud_age_s:
            return (
                False,
                f"wrist-refine wrist cloud is stale ({age_s:.1f}s > {self._max_wrist_cloud_age_s:.1f}s)",
            )
        return True, f"wrist-refine wrist cloud ok ({count} points, age={age_s:.1f}s)"

    def _execute_feedback_arm_stages(
        self,
        stages: list[PrimitiveStage],
        *,
        prompt: str,
        top_k: int,
        executed_messages: list[str],
        refresh_before_descent: bool,
        confirmation_target: np.ndarray | None = None,
    ) -> tuple[bool, str]:
        if self._feedback_executor is None:
            return False, "feedback executor is not initialized"
        self._refresh_feedback_settings()
        reference_wrist_roll = self._current_wrist_roll()
        for stage in stages:
            if stage.name.endswith("descent_close"):
                if refresh_before_descent:
                    refresh_ok, refresh_message = self._refresh_wrist_confirmation(
                        prompt,
                        top_k,
                        confirmation_target=confirmation_target,
                    )
                else:
                    refresh_ok = True
                    refresh_message = "wrist confirmation already refreshed before final plan"
                wrist_ok, wrist_message = self._execution_wrist_cloud_check()
                executed_messages.append(refresh_message)
                if not refresh_ok:
                    return (
                        False,
                        "feedback execution stopped before descent/close: "
                        + " -> ".join(executed_messages + [wrist_message]),
                    )
                if not wrist_ok:
                    return (
                        False,
                        "feedback execution stopped before descent/close: "
                        + " -> ".join(executed_messages + [wrist_message]),
                    )

            self.get_logger().info(f"Executing feedback SO-101 grasp stage: {stage.name}")
            ok, message = self._feedback_executor.execute_stage(
                stage,
                reference_wrist_roll=reference_wrist_roll,
                max_wrist_roll_delta_rad=self._max_wrist_roll_delta(),
            )
            executed_messages.append(message)
            if not ok:
                return False, "feedback execution stopped after arm motion: " + " -> ".join(executed_messages)
        return True, " -> ".join(executed_messages)

    def _execute_arm_stages(
        self,
        stages: list[PrimitiveStage],
        *,
        prompt: str,
        top_k: int,
        executed_messages: list[str],
        refresh_before_descent: bool,
        confirmation_target: np.ndarray | None = None,
    ) -> tuple[bool, str]:
        reference_wrist_roll = self._current_wrist_roll()
        for stage in stages:
            if stage.name.endswith("descent_close"):
                if refresh_before_descent:
                    refresh_ok, refresh_message = self._refresh_wrist_confirmation(
                        prompt,
                        top_k,
                        confirmation_target=confirmation_target,
                    )
                else:
                    refresh_ok = True
                    refresh_message = "wrist confirmation already refreshed before final plan"
                wrist_ok, wrist_message = self._execution_wrist_cloud_check()
                executed_messages.append(refresh_message)
                if not refresh_ok:
                    return (
                        False,
                        "execution stopped before descent/close: "
                        + " -> ".join(executed_messages + [wrist_message]),
                    )
                if not wrist_ok:
                    return (
                        False,
                        "execution stopped before descent/close: "
                        + " -> ".join(executed_messages + [wrist_message]),
                    )

            self.get_logger().info(f"Replanning SO-101 grasp stage from live state: {stage.name}")
            plan_result, expected_state, plan_message = self._plan_primitive_stage_from_current(stage)
            if not plan_result or expected_state is None:
                return False, f"execution stopped at {stage.name}: {plan_message}"
            wrist_delta = self._trajectory_wrist_roll_delta(plan_result, reference_wrist_roll)
            wrist_reject_reason = self._wrist_roll_reject_reason(wrist_delta)
            if wrist_reject_reason is not None:
                return False, f"execution stopped at {stage.name}: {wrist_reject_reason}"
            wrist_note = self._wrist_roll_note(wrist_delta)
            plan_message += wrist_note
            self.get_logger().info(f"{stage.name}: {plan_message}")

            self.get_logger().info(f"Executing SO-101 grasp stage: {stage.name}")
            execute_result = self._moveit.execute(plan_result.trajectory, controllers=[])
            if execute_result is False:
                return False, f"execution stopped at {stage.name}: MoveIt execute returned false"

            if self._post_stage_settle_s > 0.0:
                time.sleep(self._post_stage_settle_s)
            verify_ok, verify_message = self._verify_arm_state(expected_state, stage.name)
            verify_message += wrist_note
            executed_messages.append(verify_message)
            if not verify_ok:
                return False, "execution stopped after arm motion: " + " -> ".join(executed_messages)
        return True, " -> ".join(executed_messages)

    def _execute_planned_stages(
        self,
        planned_stages: list[PlannedStage],
        *,
        prompt: str,
        top_k: int,
        executed_messages: list[str],
        refresh_before_descent: bool,
        confirmation_target: np.ndarray | None = None,
    ) -> tuple[bool, str]:
        reference_wrist_roll = self._current_wrist_roll()
        for index, stage in enumerate(planned_stages):
            if stage.name.endswith("descent_close"):
                if refresh_before_descent:
                    refresh_ok, refresh_message = self._refresh_wrist_confirmation(
                        prompt,
                        top_k,
                        confirmation_target=confirmation_target,
                    )
                else:
                    refresh_ok = True
                    refresh_message = "wrist confirmation already refreshed before final plan"
                wrist_ok, wrist_message = self._execution_wrist_cloud_check()
                executed_messages.append(refresh_message)
                if not refresh_ok:
                    return (
                        False,
                        "execution stopped before descent/close: "
                        + " -> ".join(executed_messages + [wrist_message]),
                    )
                if not wrist_ok:
                    return (
                        False,
                        "execution stopped before descent/close: "
                        + " -> ".join(executed_messages + [wrist_message]),
                    )

            if index == 0:
                plan_result = stage.result
                expected_state = self._state_from_plan_end(plan_result, self._current_robot_state())
                plan_source = "preplanned"
                plan_message = ""
            else:
                primitive_stage = self._primitive_from_planned_stage(stage)
                self.get_logger().info(f"Replanning SO-101 grasp stage from live state: {stage.name}")
                plan_result, expected_state, plan_message = self._plan_primitive_stage_from_current(primitive_stage)
                if not plan_result or expected_state is None:
                    return False, f"execution stopped at {stage.name}: live replan failed: {plan_message}"
                plan_source = "live-replanned"
            wrist_delta = self._trajectory_wrist_roll_delta(plan_result, reference_wrist_roll)
            wrist_reject_reason = self._wrist_roll_reject_reason(wrist_delta)
            if wrist_reject_reason is not None:
                if index > 0 and plan_source == "live-replanned":
                    fallback_delta = self._trajectory_wrist_roll_delta(stage.result, reference_wrist_roll)
                    fallback_reject_reason = self._wrist_roll_reject_reason(fallback_delta)
                    if fallback_reject_reason is None:
                        self.get_logger().warning(
                            f"{stage.name}: live replan rejected ({wrist_reject_reason}); "
                            "falling back to selected preplanned stage"
                            f"{self._wrist_roll_note(fallback_delta)}"
                        )
                        plan_result = stage.result
                        expected_state = self._state_from_plan_end(plan_result, self._current_robot_state())
                        plan_source = "preplanned-fallback"
                        plan_message = ""
                        wrist_delta = fallback_delta
                        wrist_reject_reason = None
                    else:
                        return (
                            False,
                            f"execution stopped at {stage.name}: live-replanned {wrist_reject_reason}; "
                            f"preplanned fallback also rejected: {fallback_reject_reason}",
                        )
                if wrist_reject_reason is not None:
                    return False, f"execution stopped at {stage.name}: {plan_source} {wrist_reject_reason}"
            wrist_note = self._wrist_roll_note(wrist_delta)
            if plan_message:
                self.get_logger().info(f"{stage.name}: {plan_message}{wrist_note}")
            self.get_logger().info(f"Executing {plan_source} SO-101 grasp stage: {stage.name}{wrist_note}")
            execute_result = self._moveit.execute(plan_result.trajectory, controllers=[])
            if execute_result is False:
                return False, f"execution stopped at {stage.name}: MoveIt execute returned false"

            if self._post_stage_settle_s > 0.0:
                time.sleep(self._post_stage_settle_s)
            verify_ok, verify_message = self._verify_arm_state(expected_state, stage.name)
            verify_message += wrist_note
            executed_messages.append(verify_message)
            if not verify_ok:
                return False, "execution stopped after arm motion: " + " -> ".join(executed_messages)
        return True, " -> ".join(executed_messages)

    def _post_grasp_lift_axis(self) -> np.ndarray:
        axis = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
        if not self._support_plane_staging_enabled():
            return axis
        try:
            _, normal = self._support_plane_in_arm_base()
        except Exception as exc:  # noqa: BLE001 - fall back to base +Z lift.
            self.get_logger().warning(f"post-grasp lift could not read support-plane normal: {exc}")
            return axis
        if float(normal @ axis) < 0.0:
            normal = -normal
        if abs(float(normal @ axis)) < 0.20:
            return axis
        return normalize_vector(normal)

    def _post_grasp_lift_stage(self, primitive_stages: list[PrimitiveStage]) -> PrimitiveStage | None:
        self._post_grasp_lift_m = max(0.0, float(self.get_parameter("post_grasp_lift_m").value))
        if self._post_grasp_lift_m <= 1e-6:
            return None
        base_stage = next((stage for stage in reversed(primitive_stages) if stage.pose is not None), None)
        if base_stage is None or base_stage.pose is None:
            return None
        position, quat, _ = _pose_to_numpy(base_stage.pose.pose)
        lifted = position + self._post_grasp_lift_axis() * self._post_grasp_lift_m
        return PrimitiveStage("post_grasp_lift", pose=self._make_pose(lifted, quat))

    def _execute_primitive(
        self,
        primitive_stages: list[PrimitiveStage],
        *,
        prompt: str,
        top_k: int,
        open_first: bool = True,
        confirmation_target: np.ndarray | None = None,
        planned_stages: list[PlannedStage] | None = None,
        refresh_before_descent: bool = True,
    ) -> tuple[bool, str]:
        executed_messages: list[str] = []
        if open_first:
            open_ok, open_message = self._send_gripper_goal(
                self._gripper_open_position,
                allow_stall=self._gripper_open_allow_stall,
            )
            if not open_ok:
                return False, f"execution stopped before arm motion: {open_message}"
            executed_messages.append(f"preopen: {open_message}")

        if self._execution_backend == "feedback":
            arm_ok, arm_message = self._execute_feedback_arm_stages(
                primitive_stages,
                prompt=prompt,
                top_k=top_k,
                executed_messages=executed_messages,
                refresh_before_descent=refresh_before_descent,
                confirmation_target=confirmation_target,
            )
        elif planned_stages is None:
            arm_ok, arm_message = self._execute_arm_stages(
                primitive_stages,
                prompt=prompt,
                top_k=top_k,
                executed_messages=executed_messages,
                refresh_before_descent=refresh_before_descent,
                confirmation_target=confirmation_target,
            )
        else:
            arm_ok, arm_message = self._execute_planned_stages(
                planned_stages,
                prompt=prompt,
                top_k=top_k,
                executed_messages=executed_messages,
                refresh_before_descent=refresh_before_descent,
                confirmation_target=confirmation_target,
            )
        if not arm_ok:
            return False, arm_message

        close_ok, close_message = self._send_gripper_goal(self._gripper_closed_position, allow_stall=True)
        if not close_ok:
            return False, f"arm stages executed; close failed: {close_message}"
        executed_messages.append(f"close: {close_message}")

        lift_stage = self._post_grasp_lift_stage(primitive_stages)
        if lift_stage is not None:
            lift_messages: list[str] = []
            if self._execution_backend == "feedback":
                lift_ok, lift_message = self._execute_feedback_arm_stages(
                    [lift_stage],
                    prompt=prompt,
                    top_k=top_k,
                    executed_messages=lift_messages,
                    refresh_before_descent=False,
                )
            else:
                lift_ok, lift_message = self._execute_arm_stages(
                    [lift_stage],
                    prompt=prompt,
                    top_k=top_k,
                    executed_messages=lift_messages,
                    refresh_before_descent=False,
                )
            if not lift_ok:
                return False, "closed gripper; post-grasp lift failed: " + lift_message
            executed_messages.append(f"lift: {lift_message}")

        return True, "executed ready/pregrasp/descent/close/lift: " + " -> ".join(executed_messages)

    def _execute_wrist_refined_grasp(
        self,
        initial_selection: PlanSelection,
        *,
        prompt: str,
        top_k: int,
        pregrasp_offset: float,
        deadline: float,
    ) -> tuple[bool, str, PlanSelection | None]:
        open_ok, open_message = self._send_gripper_goal(
            self._gripper_open_position,
            allow_stall=self._gripper_open_allow_stall,
        )
        if not open_ok:
            return False, f"wrist-refine stopped before arm motion: {open_message}", None

        initial_target = self._wrist_refine_target(initial_selection.grasp).copy()
        base_messages = [f"preopen: {open_message}"]
        view_options = self._wrist_refine_view_stage_options(initial_selection)
        if not view_options:
            return False, "wrist-refine generated no camera view candidates", None

        max_attempts = max(1, int(self.get_parameter("wrist_refine_max_view_attempts").value))
        failures: list[str] = []
        for attempt_index, (view_label, view_stages) in enumerate(view_options[:max_attempts], start=1):
            executed_messages = [*base_messages, f"view {attempt_index}/{max_attempts}: {view_label}"]
            if self._execution_backend == "feedback":
                view_ok, view_message = self._execute_feedback_arm_stages(
                    view_stages,
                    prompt=prompt,
                    top_k=top_k,
                    executed_messages=executed_messages,
                    refresh_before_descent=False,
                )
            else:
                view_ok, view_message = self._execute_arm_stages(
                    view_stages,
                    prompt=prompt,
                    top_k=top_k,
                    executed_messages=executed_messages,
                    refresh_before_descent=False,
                )
            if not view_ok:
                failures.append(f"{view_label}: view move failed: {view_message}")
                continue

            if self._wrist_refine_settle_s > 0.0:
                time.sleep(self._wrist_refine_settle_s)

            ok, message, selection, retryable = self._execute_refined_grasp_from_current_view(
                initial_selection,
                initial_target=initial_target,
                prompt=prompt,
                top_k=top_k,
                pregrasp_offset=pregrasp_offset,
                deadline=deadline,
                executed_messages=executed_messages,
            )
            if ok or not retryable:
                return ok, message, selection
            failures.append(f"{view_label}: {message}")

        return (
            False,
            "wrist-refine failed all view attempts: " + " | ".join(failures[:8]),
            None,
        )

    def _execute_refined_grasp_from_current_view(
        self,
        initial_selection: PlanSelection,
        *,
        initial_target: np.ndarray,
        prompt: str,
        top_k: int,
        pregrasp_offset: float,
        deadline: float,
        executed_messages: list[str],
    ) -> tuple[bool, str, PlanSelection | None, bool]:
        refine_top_k = max(int(top_k), self._wrist_refine_top_k)
        try:
            detect_response = self._detect_grasps(prompt, refine_top_k)
        except Exception as exc:  # noqa: BLE001 - keep hardware run result explicit.
            if self._wrist_refine_fallback_to_initial:
                fallback_ok, fallback_message = self._execute_primitive(
                    initial_selection.primitive_stages,
                    prompt=prompt,
                    top_k=top_k,
                    open_first=False,
                )
                return (
                    fallback_ok,
                    "wrist-refine detection failed; fell back to initial plan: "
                    + f"{exc}; {fallback_message}",
                    initial_selection,
                    False,
                )
            return False, f"wrist-refine detection failed after view move: {exc}", None, True

        if not detect_response.success or not detect_response.grasps:
            reason = detect_response.message if detect_response.message else "no refreshed candidates"
            if self._wrist_refine_fallback_to_initial:
                fallback_ok, fallback_message = self._execute_primitive(
                    initial_selection.primitive_stages,
                    prompt=prompt,
                    top_k=top_k,
                    open_first=False,
                )
                return (
                    fallback_ok,
                    "wrist-refine produced no usable candidates; fell back to initial plan: "
                    + f"{reason}; {fallback_message}",
                    initial_selection,
                    False,
                )
            return False, f"wrist-refine produced no usable candidates after view move: {reason}", None, True

        try:
            refreshed_grasps = [self._grasp_to_arm_base(grasp) for grasp in detect_response.grasps]
        except Exception as exc:  # noqa: BLE001 - keep hardware run result explicit.
            if self._wrist_refine_fallback_to_initial:
                fallback_ok, fallback_message = self._execute_primitive(
                    initial_selection.primitive_stages,
                    prompt=prompt,
                    top_k=top_k,
                    open_first=False,
                )
                return (
                    fallback_ok,
                    "wrist-refine transform failed; fell back to initial plan: "
                    + f"{exc}; {fallback_message}",
                    initial_selection,
                    False,
                )
            return False, f"wrist-refine failed to transform refreshed candidates: {exc}", None, True

        refreshed_grasps, rejected = self._filter_wrist_refine_candidates(refreshed_grasps, initial_target)
        if not refreshed_grasps:
            reason = "; ".join(rejected[:6]) if rejected else "no candidates after same-object gate"
            if self._wrist_refine_fallback_to_initial:
                fallback_ok, fallback_message = self._execute_primitive(
                    initial_selection.primitive_stages,
                    prompt=prompt,
                    top_k=top_k,
                    open_first=False,
                )
                return (
                    fallback_ok,
                    "wrist-refine rejected refreshed candidates; fell back to initial plan: "
                    + f"{reason}; {fallback_message}",
                    initial_selection,
                    False,
                )
            return False, f"wrist-refine rejected refreshed candidates too far from initial target: {reason}", None, True

        wrist_ok, wrist_message = self._wrist_refine_cloud_check()
        if not wrist_ok:
            if self._wrist_refine_fallback_to_initial:
                fallback_ok, fallback_message = self._execute_primitive(
                    initial_selection.primitive_stages,
                    prompt=prompt,
                    top_k=top_k,
                    open_first=False,
                )
                return (
                    fallback_ok,
                    "wrist-refine wrist cloud check failed; fell back to initial plan: "
                    + f"{wrist_message}; {fallback_message}",
                    initial_selection,
                    False,
                )
            return False, "wrist-refine refused overhead-only replan: " + wrist_message, None, True

        refined_selection, failures = self._select_plan_for_backend(
            refreshed_grasps,
            requested_index=0,
            pregrasp_offset=pregrasp_offset,
            deadline=max(deadline, time.monotonic() + self._wrist_refine_plan_budget_s),
            strip_initial_ready=True,
        )
        if refined_selection is None:
            message = " | ".join(failures[:6])
            if self._wrist_refine_fallback_to_initial:
                fallback_ok, fallback_message = self._execute_primitive(
                    initial_selection.primitive_stages,
                    prompt=prompt,
                    top_k=top_k,
                    open_first=False,
                )
                return (
                    fallback_ok,
                    "wrist-refine replanning failed; fell back to initial plan: "
                    + f"{message}; {fallback_message}",
                    initial_selection,
                    False,
                )
            return False, "wrist-refine replanning failed after view move: " + message, None, True

        final_ok, final_message = self._execute_primitive(
            refined_selection.primitive_stages,
            prompt=prompt,
            top_k=refine_top_k,
            open_first=False,
            planned_stages=refined_selection.planned_stages,
            refresh_before_descent=False,
        )
        prefix = (
            "wrist-refined grasp: "
            + " -> ".join(executed_messages)
            + f"; {wrist_message}; refreshed plan candidate {refined_selection.candidate_index} "
            + f"{refined_selection.strategy_label}: {refined_selection.message}; "
        )
        return final_ok, prefix + final_message, refined_selection, False

    def _refresh_wrist_confirmation(
        self,
        prompt: str,
        top_k: int,
        *,
        confirmation_target: np.ndarray | None,
    ) -> tuple[bool, str]:
        if not bool(self.get_parameter("wrist_confirmation_before_descent").value):
            return True, "wrist confirmation skipped; disabled"
        require_confirmation = bool(self.get_parameter("require_wrist_confirmation_for_execution").value)
        required_cloud = bool(self.get_parameter("require_wrist_cloud_for_execution").value)
        gate_note = "required" if require_confirmation or required_cloud else "advisory"
        try:
            response = self._detect_grasps(prompt, top_k)
        except Exception as exc:  # noqa: BLE001 - return as execution audit detail.
            message = f"wrist confirmation {gate_note} refresh failed: {exc}"
            return (False, message) if require_confirmation else (True, message)
        if not bool(response.success):
            message = f"wrist confirmation {gate_note} refresh failed: {response.message}"
            return (False, message) if require_confirmation else (True, message)
        if confirmation_target is None:
            return True, f"wrist confirmation {gate_note}: refreshed {len(response.grasps)} candidate(s)"
        try:
            refreshed_grasps = [self._grasp_to_arm_base(grasp) for grasp in response.grasps]
        except Exception as exc:  # noqa: BLE001 - report the concrete TF issue.
            message = f"wrist confirmation {gate_note} transform failed: {exc}"
            return (False, message) if require_confirmation else (True, message)
        matched, rejected = self._filter_wrist_refine_candidates(refreshed_grasps, confirmation_target)
        if not matched:
            reason = "; ".join(rejected[:6]) if rejected else "no candidates after same-object gate"
            message = (
                f"wrist confirmation {gate_note}: found {len(response.grasps)} candidate(s), "
                f"but none matched initial target: {reason}"
            )
            if require_confirmation:
                return False, message
            return (
                True,
                message + "; continuing on overhead-primary plan",
            )
        return (
            True,
            f"wrist confirmation {gate_note}: matched {len(matched)}/{len(response.grasps)} candidate(s) "
            "near initial target",
        )

    def _publish_display_trajectories(self, trajectories: list[Any]) -> None:
        msg = DisplayTrajectory()
        msg.model_id = "so101_arm"
        msg.trajectory = list(trajectories)
        self._display_pub.publish(msg)

    def _publish_plan_markers(self, grasp: GraspCandidate, planned_stages: list[PlannedStage]) -> None:
        markers = MarkerArray()
        start = np.asarray([0.0, 0.0, 0.0], dtype=np.float64)
        position, _, _ = _pose_to_numpy(grasp.pose)
        stage_points = []
        pose_stage_pairs: list[tuple[PlannedStage, np.ndarray]] = []
        for stage in planned_stages:
            if stage.pose is None:
                continue
            point = np.asarray(
                [
                    stage.pose.pose.position.x,
                    stage.pose.pose.position.y,
                    stage.pose.pose.position.z,
                ],
                dtype=np.float64,
            )
            stage_points.append(point)
            pose_stage_pairs.append((stage, point))

        line = Marker()
        line.header.frame_id = self._arm_base_frame
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = "moveit_grasp_plan"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.01
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.2
        line.color.a = 0.9
        line.points = [_point(start)] + [_point(point) for point in stage_points] + [_point(position)]
        markers.markers.append(line)

        stage_markers: list[tuple[int, str, np.ndarray, tuple[float, float, float]]] = []
        for offset, (stage, point) in enumerate(pose_stage_pairs, start=1):
            is_close = stage.name.endswith("descent_close") or stage.name == "descent_close"
            color = (1.0, 0.8, 0.0) if not is_close else (1.0, 0.35, 0.0)
            stage_markers.append((offset, stage.name, point, color))
        source_name = str(grasp.source).strip() or "grasp"
        stage_markers.append((len(stage_markers) + 1, f"{source_name}_grasp", position, (0.0, 1.0, 0.2)))

        for marker_id, name, point, color in stage_markers:
            marker = Marker()
            marker.header = line.header
            marker.ns = f"moveit_grasp_{name}"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = _point(point)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.035
            marker.scale.y = 0.035
            marker.scale.z = 0.035
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.9
            markers.markers.append(marker)
        self._markers_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspPlannerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
