"""MoveIt grasp planner only.

Use this when follower hardware, cameras, camera TF, and grasp_request_node are
already running. It avoids relaunching camera frames from the URDF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    allow_grasp_execution = LaunchConfiguration("allow_grasp_execution")
    detect_service = LaunchConfiguration("detect_service")
    plan_service = LaunchConfiguration("plan_service")
    publish_world_to_base = LaunchConfiguration("publish_world_to_base")
    publish_base_to_follower_base = LaunchConfiguration("publish_base_to_follower_base")

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

    grasp_planner_node = Node(
        package="so101_grasping",
        executable="grasp_planner_node",
        name="grasp_planner_node",
        parameters=[
            moveit_config.to_dict(),
            {
                "allow_execution": allow_grasp_execution,
                "detect_service": detect_service,
                "plan_service": plan_service,
            },
        ],
        remappings=[("joint_states", "/follower/joint_states")],
        output="screen",
    )

    static_world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="so101_world_to_base",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        output="log",
        condition=IfCondition(publish_world_to_base),
    )

    static_base_to_follower_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="so101_base_to_follower_base",
        arguments=["--frame-id", "base_link", "--child-frame-id", "follower/base_link"],
        output="log",
        condition=IfCondition(publish_base_to_follower_base),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("allow_grasp_execution", default_value="false"),
            DeclareLaunchArgument("detect_service", default_value="/detect_grasps"),
            DeclareLaunchArgument("plan_service", default_value="/plan_grasp"),
            DeclareLaunchArgument("publish_world_to_base", default_value="true"),
            DeclareLaunchArgument("publish_base_to_follower_base", default_value="true"),
            static_world_to_base,
            static_base_to_follower_base,
            grasp_planner_node,
        ]
    )
