import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("hik_camera")
    params_file = os.path.join(pkg_share, "config", "hik_camera.yaml")

    return LaunchDescription([
        Node(
            package="hik_camera",
            executable="hik_camera_node",
            name="hik_camera",
            output="screen",
            parameters=[params_file],
        ),
    ])
