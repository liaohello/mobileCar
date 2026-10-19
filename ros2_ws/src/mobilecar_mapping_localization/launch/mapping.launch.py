from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("mobilecar_mapping_localization"))
    params = str(share / "config" / "mapper_params_mapping.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[params, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="mobilecar_mapping_localization",
                executable="localization_monitor",
                name="mobilecar_localization_monitor",
                output="screen",
                parameters=[{"mode": "MAPPING", "use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ]
    )
