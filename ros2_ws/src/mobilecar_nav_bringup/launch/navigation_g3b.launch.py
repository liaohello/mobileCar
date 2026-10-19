from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("mobilecar_nav_bringup"))
    params = str(share / "config" / "nav2_g3b.yaml")
    bt_xml = str(share / "behavior_trees" / "navigate_to_pose_g3b.xml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params,
            root_key="nav2",
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )
    common = {
        "output": "screen",
        "parameters": [configured_params],
    }
    nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            namespace="nav2",
            remappings=[("cmd_vel", "/mobilecar/nav2/cmd_vel_raw")],
            **common,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            namespace="nav2",
            **common,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            namespace="nav2",
            remappings=[("cmd_vel", "/mobilecar/nav2/cmd_vel_raw")],
            **common,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            namespace="nav2",
            parameters=[
                configured_params,
                {
                    "default_nav_to_pose_bt_xml": bt_xml,
                },
            ],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            namespace="nav2",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "bond_timeout": 0.0,
                    "node_names": [
                        "controller_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                    ],
                }
            ],
        ),
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            *nodes,
        ]
    )
