"""Calibrated XLeRobot grasp runtime for one selected arm side."""

from __future__ import annotations

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _load_transform_args(path: Path, *, child_override: str | None = None) -> list[str]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    transform = data.get("transform", {})
    translation = transform.get("translation_xyz", [0.0, 0.0, 0.0])
    quat = transform.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
    child_frame = child_override or transform["child_frame"]
    return [
        "--x",
        f"{float(translation[0]):.9f}",
        "--y",
        f"{float(translation[1]):.9f}",
        "--z",
        f"{float(translation[2]):.9f}",
        "--qx",
        f"{float(quat[0]):.9f}",
        "--qy",
        f"{float(quat[1]):.9f}",
        "--qz",
        f"{float(quat[2]):.9f}",
        "--qw",
        f"{float(quat[3]):.9f}",
        "--frame-id",
        transform["parent_frame"],
        "--child-frame-id",
        child_frame,
    ]


def _write_moveit_py_params(path: Path, *, node_name: str, params: dict) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump({f"/{node_name}": {"ros__parameters": params}}, sort_keys=False),
        encoding="utf-8",
    )
    return str(path)


def _runtime_nodes(context):
    side = LaunchConfiguration("side").perform(context)
    out_dir = Path(LaunchConfiguration("out_dir").perform(context))
    grasp_server_address = LaunchConfiguration("grasp_server_address").perform(context)
    allow_execution = LaunchConfiguration("allow_execution").perform(context)
    allow_execution_bool = allow_execution.strip().lower() in {"1", "true", "yes", "on"}
    use_rviz = LaunchConfiguration("use_rviz").perform(context)

    if side not in {"left", "right"}:
        raise RuntimeError(f"side must be left or right, got {side!r}")

    config_dir = out_dir / "config"
    extrinsics_dir = out_dir / "extrinsics"

    xacro_path = os.path.join(
        get_package_share_directory("so101_description"),
        "urdf",
        "so101_arm.urdf.xacro",
    )
    moveit_config = (
        MoveItConfigsBuilder("so101_arm", package_name="so101_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={"variant": "follower", "use_ros2_control": "false"},
        )
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .joint_limits()
        .trajectory_execution(
            file_path=str(config_dir / f"{side}_moveit_controllers.yaml"),
            moveit_manage_controllers=False,
        )
        .moveit_cpp(file_path=str(config_dir / f"{side}_moveit_py_config.yaml"))
        .to_moveit_configs()
    )

    static_tf_specs = [
        (f"world_to_{side}_base", extrinsics_dir / f"world_to_{side}_base.yaml", None),
        (f"{side}_wrist_camera", extrinsics_dir / f"{side}_wrist_camera_in_gripper.yaml", None),
        ("center_gopro", extrinsics_dir / "center_gopro_in_world.yaml", None),
        ("moveit_world_to_base", extrinsics_dir / f"world_to_{side}_base.yaml", "base_link"),
    ]
    static_tfs = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"xlerobot_{name}",
            arguments=_load_transform_args(path, child_override=child_override),
            output="log",
        )
        for name, path, child_override in static_tf_specs
    ]

    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("so101_bringup"), "launch", "follower_split.launch.py")
        ),
        launch_arguments={
            "namespace": side,
            "frame_prefix": f"{side}/",
            "hardware_type": LaunchConfiguration("hardware_type"),
            "usb_port": LaunchConfiguration(f"{side}_port"),
            "joint_config_file": str(config_dir / f"{side}_joints_from_lerobot.yaml"),
            "controller_config_file": str(config_dir / f"{side}_split_controllers.yaml"),
            "arm_controller": "arm_trajectory_controller",
            "use_rviz": use_rviz,
        }.items(),
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("so101_bringup"), "launch", "cameras.launch.py")
        ),
        launch_arguments={
            "cameras_config": str(config_dir / f"{side}_grasp_cameras.yaml"),
        }.items(),
    )

    grasp_runtime_params = str(config_dir / f"{side}_grasp_runtime.yaml")
    moveit_node_name = f"{side}_grasp_moveit_py"
    moveit_dict = moveit_config.to_dict()
    moveit_py_params = _write_moveit_py_params(
        config_dir / f"{side}_moveit_py_runtime_params.yaml",
        node_name=moveit_node_name,
        params=moveit_dict,
    )
    request_node = Node(
        package="so101_grasping",
        executable="grasp_request_node",
        namespace=f"{side}_grasp",
        name="grasp_request_node",
        parameters=[grasp_runtime_params, {"server_address": grasp_server_address}],
        output="screen",
    )
    planner_node = Node(
        package="so101_grasping",
        executable="grasp_planner_node",
        namespace=f"{side}_grasp",
        name="grasp_planner_node",
        parameters=[
            moveit_py_params,
            moveit_dict,
            grasp_runtime_params,
            {"allow_execution": allow_execution_bool, "moveit_node_name": moveit_node_name},
        ],
        output="screen",
    )

    return [*static_tfs, arm, cameras, request_node, planner_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("side", default_value="left"),
            DeclareLaunchArgument("out_dir"),
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument("left_port", default_value=""),
            DeclareLaunchArgument("right_port", default_value=""),
            DeclareLaunchArgument("grasp_server_address", default_value="127.0.0.1:8091"),
            DeclareLaunchArgument("allow_execution", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            OpaqueFunction(function=_runtime_nodes),
        ]
    )
