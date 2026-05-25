from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


def _launch_nodes(context, *args, **kwargs):
    arm = LaunchConfiguration("arm").perform(context).strip("/")
    robot_description = LaunchConfiguration("robot_description").perform(context)

    return [
        Node(
            package="so101_kinematics",
            executable="cartesian_motion_node",
            namespace=arm,
            name="cartesian_motion_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "joints_topic": f"/{arm}/joint_states",
                    "cmd_topic": f"/{arm}/arm_forward_controller/commands",
                    "base_frame": f"{arm}/base_link",
                    "robot_description": robot_description,
                    "command_joint_names": ARM_JOINTS,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm",
                default_value="left",
                description="Arm namespace to command: left or right.",
            ),
            DeclareLaunchArgument(
                "robot_description",
                default_value="so_arm101_description",
                description="robokin robot description name.",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
