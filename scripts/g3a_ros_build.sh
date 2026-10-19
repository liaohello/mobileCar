#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${G3A_ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${G3A_ROS_RMW:-rmw_fastrtps_cpp}"

colcon build \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --symlink-install \
  --event-handlers console_direct+
