# mobileCar G3A ROS 2 Humble bridge

This workspace contains only the independent bridge process. It never creates a
MuJoCo model or data object and never steps physics.

## Build on the server

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths ros2_ws/src --build-base ros2_ws/build \
  --install-base ros2_ws/install --symlink-install
```

## Start

The mobileCar application must already expose the configured Unix socket.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch mobilecar_navigation_bridge navigation_bridge.launch.py
```

Published interfaces:

- `/clock`
- `/scan`
- `/imu/data_raw`
- `/joint_states`
- `/odom`
- `/tf` and `/tf_static`
- `/mobilecar/navigation_health`
- subscribes `/mobilecar/cmd_vel_nav_test`

The bridge uses capture simulation time in every measurement header. Its command
watchdog uses wall `time.monotonic()` and never waits for simulated time.
`/clock` is produced from capture stamps; `use_sim_time` may be enabled for
downstream consumers without coupling the watchdog to simulation progress.

No `map -> odom` transform is published. Static transforms have one source:
`/tf_static`. `odom -> base_link` and the active arm chain are published only by
this bridge from estimate and model-joint feedback.
