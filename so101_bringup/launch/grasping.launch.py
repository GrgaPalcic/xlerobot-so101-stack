"""Follower arm + dual cameras + remote grasp perception client."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    hardware_type = LaunchConfiguration("hardware_type")
    follower_ns = LaunchConfiguration("follower_namespace")
    follower_frame_prefix = LaunchConfiguration("follower_frame_prefix")
    follower_usb = LaunchConfiguration("follower_usb_port")
    follower_joint_cfg = LaunchConfiguration("follower_joint_config_file")
    follower_ctrl_cfg = LaunchConfiguration("follower_controller_config_file")
    arm_controller = LaunchConfiguration("arm_controller")
    gripper_controller = LaunchConfiguration("gripper_controller")
    cameras_config_file = LaunchConfiguration("cameras_config_file")
    grasping_config_file = LaunchConfiguration("grasping_config_file")
    grasp_server_address = LaunchConfiguration("grasp_server_address")
    use_rviz = LaunchConfiguration("use_rviz")
    use_moveit_planner = LaunchConfiguration("use_moveit_planner")
    allow_grasp_execution = LaunchConfiguration("allow_grasp_execution")
    use_calibrated_camera_tf = LaunchConfiguration("use_calibrated_camera_tf")
    use_urdf_camera_tf = LaunchConfiguration("use_urdf_camera_tf")

    cam_static_xyz = LaunchConfiguration("cam_static_xyz")
    cam_static_rpy = LaunchConfiguration("cam_static_rpy")
    cam_wrist_xyz = LaunchConfiguration("cam_wrist_xyz")
    cam_wrist_rpy = LaunchConfiguration("cam_wrist_rpy")

    xacro_path = os.path.join(
        get_package_share_directory("so101_description"),
        "urdf",
        "so101_arm.urdf.xacro",
    )
    moveit_config = (
        MoveItConfigsBuilder("so101_arm", package_name="so101_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={
                "variant": "follower",
                "use_ros2_control": "false",
            },
        )
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .joint_limits()
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("so101_moveit_config"),
                "config",
                "moveit_py_config.yaml",
            )
        )
        .to_moveit_configs()
    )

    follower_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_bringup"), "launch", "follower_vision.launch.py"]
            )
        ),
        launch_arguments={
            "hardware_type": hardware_type,
            "follower_namespace": follower_ns,
            "follower_frame_prefix": follower_frame_prefix,
            "follower_usb_port": follower_usb,
            "follower_joint_config_file": follower_joint_cfg,
            "follower_controller_config_file": follower_ctrl_cfg,
            "arm_controller": arm_controller,
            "gripper_controller": gripper_controller,
            "cameras_config_file": cameras_config_file,
            "use_calibrated_camera_tf": use_calibrated_camera_tf,
            "use_urdf_camera_tf": use_urdf_camera_tf,
            "cam_static_xyz": cam_static_xyz,
            "cam_static_rpy": cam_static_rpy,
            "cam_wrist_xyz": cam_wrist_xyz,
            "cam_wrist_rpy": cam_wrist_rpy,
            "use_rviz": use_rviz,
        }.items(),
    )

    grasp_request_node = Node(
        package="so101_grasping",
        executable="grasp_request_node",
        name="grasp_request_node",
        parameters=[
            grasping_config_file,
            {"server_address": grasp_server_address},
        ],
        output="screen",
    )

    grasp_planner_node = Node(
        package="so101_grasping",
        executable="grasp_planner_node",
        name="grasp_planner_node",
        parameters=[
            moveit_config.to_dict(),
            {
                "allow_execution": allow_grasp_execution,
                "detect_service": "/detect_grasps",
                "plan_service": "/plan_grasp",
            },
        ],
        remappings=[("joint_states", "/follower/joint_states")],
        output="screen",
        condition=IfCondition(use_moveit_planner),
    )

    static_world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="so101_world_to_base",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        output="log",
        condition=IfCondition(use_moveit_planner),
    )

    static_base_to_follower_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="so101_base_to_follower_base",
        arguments=["--frame-id", "base_link", "--child-frame-id", "follower/base_link"],
        output="log",
        condition=IfCondition(use_moveit_planner),
    )

    default_follower_ctrl_cfg = PathJoinSubstitution(
        [
            FindPackageShare("so101_bringup"),
            "config",
            "ros2_control",
            "follower_controllers.yaml",
        ]
    )
    default_cameras_cfg = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "cameras", "so101_cameras.yaml"]
    )
    default_grasping_cfg = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "grasping", "grasping.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument("follower_namespace", default_value="follower"),
            DeclareLaunchArgument("follower_frame_prefix", default_value="follower/"),
            DeclareLaunchArgument("follower_usb_port", default_value="/dev/so101_follower"),
            DeclareLaunchArgument("follower_joint_config_file", default_value=""),
            DeclareLaunchArgument("follower_controller_config_file", default_value=default_follower_ctrl_cfg),
            DeclareLaunchArgument("arm_controller", default_value="forward_controller"),
            DeclareLaunchArgument("gripper_controller", default_value="gripper_controller"),
            DeclareLaunchArgument("cameras_config_file", default_value=default_cameras_cfg),
            DeclareLaunchArgument("grasping_config_file", default_value=default_grasping_cfg),
            DeclareLaunchArgument("grasp_server_address", default_value="127.0.0.1:8091"),
            DeclareLaunchArgument("use_moveit_planner", default_value="false"),
            DeclareLaunchArgument("allow_grasp_execution", default_value="false"),
            DeclareLaunchArgument(
                "use_calibrated_camera_tf",
                default_value="true",
                description="Use calibrated optical-frame camera TFs instead of stale URDF defaults",
            ),
            DeclareLaunchArgument(
                "use_urdf_camera_tf",
                default_value="false",
                description="Enable URDF camera frames for custom uncalibrated camera TF overrides",
            ),
            DeclareLaunchArgument(
                "cam_static_xyz",
                default_value="0.2 0.0 0.60",
                description="Static camera position relative to base_link (x y z meters)",
            ),
            DeclareLaunchArgument(
                "cam_static_rpy",
                default_value="0.0 1.5708 0.0",
                description="Static camera orientation relative to base_link (roll pitch yaw radians)",
            ),
            DeclareLaunchArgument(
                "cam_wrist_xyz",
                default_value="0.0 0.0 -0.02",
                description="Wrist camera position relative to end-effector link (x y z meters)",
            ),
            DeclareLaunchArgument(
                "cam_wrist_rpy",
                default_value="-1.5708 0.0 -1.5708",
                description="Wrist camera orientation relative to end-effector link (roll pitch yaw radians)",
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            follower_vision,
            static_world_to_base,
            static_base_to_follower_base,
            grasp_request_node,
            grasp_planner_node,
        ]
    )
