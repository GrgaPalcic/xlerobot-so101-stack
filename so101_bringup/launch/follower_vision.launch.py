"""Follower arm + cameras with camera TF frames enabled."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Launch arguments ---
    hardware_type = LaunchConfiguration("hardware_type")
    follower_ns = LaunchConfiguration("follower_namespace")
    follower_frame_prefix = LaunchConfiguration("follower_frame_prefix")
    follower_usb = LaunchConfiguration("follower_usb_port")
    follower_joint_cfg = LaunchConfiguration("follower_joint_config_file")
    follower_ctrl_cfg = LaunchConfiguration("follower_controller_config_file")
    arm_controller = LaunchConfiguration("arm_controller")
    gripper_controller = LaunchConfiguration("gripper_controller")

    cameras_config_file = LaunchConfiguration("cameras_config_file")
    use_calibrated_camera_tf = LaunchConfiguration("use_calibrated_camera_tf")
    use_urdf_camera_tf = LaunchConfiguration("use_urdf_camera_tf")

    # Camera TF overrides
    cam_static_xyz = LaunchConfiguration("cam_static_xyz")
    cam_static_rpy = LaunchConfiguration("cam_static_rpy")
    cam_wrist_xyz = LaunchConfiguration("cam_wrist_xyz")
    cam_wrist_rpy = LaunchConfiguration("cam_wrist_rpy")

    use_rviz = LaunchConfiguration("use_rviz")

    # --- Include follower bringup (with cameras enabled) ---
    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_bringup"), "launch", "follower.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": follower_ns,
            "hardware_type": hardware_type,
            "usb_port": follower_usb,
            "frame_prefix": follower_frame_prefix,
            "joint_config_file": follower_joint_cfg,
            "controller_config_file": follower_ctrl_cfg,
            "arm_controller": arm_controller,
            "gripper_controller": gripper_controller,
            "use_rviz": use_rviz,
            "enable_static_cam": use_urdf_camera_tf,
            "enable_wrist_cam": use_urdf_camera_tf,
            "cam_static_xyz": cam_static_xyz,
            "cam_static_rpy": cam_static_rpy,
            "cam_wrist_xyz": cam_wrist_xyz,
            "cam_wrist_rpy": cam_wrist_rpy,
        }.items(),
    )

    calibrated_camera_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_bringup"), "launch", "camera_tf.launch.py"]
            )
        ),
        condition=IfCondition(use_calibrated_camera_tf),
    )

    # --- Include cameras launch (driver nodes) ---
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_bringup"), "launch", "cameras.launch.py"]
            )
        ),
        launch_arguments={"cameras_config": cameras_config_file}.items(),
    )

    # --- Defaults ---
    default_follower_joint_cfg = ""
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument("follower_namespace", default_value="follower"),
            DeclareLaunchArgument("follower_frame_prefix", default_value="follower/"),
            DeclareLaunchArgument("follower_usb_port", default_value="/dev/so101_follower"),
            DeclareLaunchArgument("follower_joint_config_file", default_value=default_follower_joint_cfg),
            DeclareLaunchArgument("follower_controller_config_file", default_value=default_follower_ctrl_cfg),
            DeclareLaunchArgument("arm_controller", default_value="forward_controller"),
            DeclareLaunchArgument("gripper_controller", default_value=""),
            DeclareLaunchArgument("cameras_config_file", default_value=default_cameras_cfg),
            DeclareLaunchArgument(
                "use_calibrated_camera_tf",
                default_value="true",
                description="Publish calibrated optical-frame camera TFs from camera_tf.launch.py",
            ),
            DeclareLaunchArgument(
                "use_urdf_camera_tf",
                default_value="false",
                description="Publish camera TFs from URDF camera args instead of the calibrated static TF launch",
            ),
            # Camera TF overrides
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
            follower_launch,
            calibrated_camera_tf_launch,
            cameras_launch,
        ]
    )
