from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("mobilecar_goal_supervisor"))
    params = str(share / "config" / "g3b_supervisor.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_id", default_value="map_unconfigured"),
            DeclareLaunchArgument("map_manifest", default_value=""),
            DeclareLaunchArgument("runtime_config", default_value=""),
            DeclareLaunchArgument("socket_path", default_value="../../var/ipc/g3b_navigation.sock"),
            Node(
                package="mobilecar_goal_supervisor",
                executable="g3b_supervisor",
                name="mobilecar_navigation_bridge",
                output="screen",
                parameters=[
                    params,
                    {
                        "map_id": LaunchConfiguration("map_id"),
                        "map_manifest": LaunchConfiguration("map_manifest"),
                        "runtime_config": LaunchConfiguration("runtime_config"),
                        "socket_path": LaunchConfiguration("socket_path"),
                        "use_sim_time": True,
                    },
                ],
            ),
        ]
    )
