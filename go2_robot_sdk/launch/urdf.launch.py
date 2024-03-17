import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory("go2_robot_sdk"), "urdf", "go2.urdf"
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": open(urdf_file, "r").read()}],
            )
        ]
    )
