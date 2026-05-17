from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Calibrated 2026-05-09 from caib.io board touch + GoPro PnP solve.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_overhead_cam",
                arguments=[
                    "--x", "0.004521045",
                    "--y", "-0.115300473",
                    "--z", "0.277194991",
                    "--qx", "-0.626906101",
                    "--qy", "0.646509608",
                    "--qz", "-0.308160835",
                    "--qw", "0.306677303",
                    "--frame-id", "follower/base_link",
                    "--child-frame-id", "follower/static_camera_optical_frame",
                ],
            ),

            # Calibrated 2026-05-09 from caib.io board touch + wrist PnP solve.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_wrist_cam",
                arguments=[
                    "--x", "0.002344943",
                    "--y", "0.072594056",
                    "--z", "-0.119362094",
                    "--qx", "0.001371523",
                    "--qy", "-0.196624502",
                    "--qz", "0.979365428",
                    "--qw", "0.046693506",
                    "--frame-id", "follower/gripper_frame_link",
                    "--child-frame-id", "follower/wrist_camera_optical_frame",
                ],
            ),
        ]
    )
