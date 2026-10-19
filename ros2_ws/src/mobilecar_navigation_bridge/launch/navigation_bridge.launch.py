from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    parameters = str(
        Path(get_package_share_directory("mobilecar_navigation_bridge"))
        / "config"
        / "navigation_bridge.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="mobilecar_navigation_bridge",
                executable="navigation_bridge",
                name="mobilecar_navigation_bridge",
                output="screen",
                parameters=[parameters],
            )
        ]
    )
