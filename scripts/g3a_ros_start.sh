#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID="${G3A_ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${G3A_ROS_RMW:-rmw_fastrtps_cpp}"

exec ros2 launch mobilecar_navigation_bridge navigation_bridge.launch.py
