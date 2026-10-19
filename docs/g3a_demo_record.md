# G3A 演示记录

日期：2026-10-17。执行位置：服务器。演示不是浏览器 UI 重跑，也不包含 SLAM 或 Nav2。

## 物理演示序列

服务器 job：`g3a-frozen-physical-1`，release
`g3a-20261017T194117Z-aa6a6e987115`。

1. 启动真实 `robot_navigation_diagnostic.xml`。机器人保持自由麦轮底盘、动态五轴臂、
   link4 相机；场景没有 `target_cube`。
2. 静止 30s，记录 1498 个 50Hz 导航 capture、1498 个 odom 和 300 个 scan。
   平面真值对齐漂移 `0.000017m`，yaw 误差 `0.0156deg`。
3. 经 IPC `ros_navigation_test` 路径执行前进、横移、组合和双向转动，同时给独立 arm
   目标。base 和 arm 同时运动；单次初始对齐后终点平面误差 `0.00198m`、yaw
   `0.0646deg`。
4. 人工 base 接管成功后继续发布旧 nav 命令，底盘保持停止，旧 session 未复活；
   arm owner 保持不变。
5. 扫描非零 yaw 房间：267 个 HIT 与独立场景 box raycast 全部一致，最大差值约
   `2.44e-15m`；89 个方向为机器人自遮挡，4 个方向无回波。
6. 在 base 与 arm 并行活动时持续采集 scan；自遮挡保持显式，非 HIT beam 均无伪造
   finite range。
7. 依次注入 scan 全无效、IMU 缺失和 wheel 无效。health 分别为 STALE/FAILED/FAILED，
   都有 `stop_required=true`，底盘停车。故障清除后未自动恢复旧导航 session。
8. Reset 后 epoch 从 6 增至 7；新 Episode、新 clock、odom 原点和旧命令拒绝均通过。

原始 run：

- `run_3abe7c1ea1794407aa7b8707af030c4b`（物理六案例）
- `run_7e92d9a79dda4cdabfb9afc90dd9c863`（ROS 端到端）

## ROS 演示

服务器 job：`g3a-frozen-ros-1`，`ROS_DOMAIN_ID=73`。

- 一个 MuJoCo 应用和一个独立 bridge；bridge/估计器没有第二个 Simulation。
- probe 实际收到 clock 659、scan 44、IMU 218、joint 218、odom 218、TF 440、
  health 220。
- `/scan` 同测得到 finite/NaN/+Inf 三种 range，映射符合冻结状态语义。
- `/imu/data_raw` 的 orientation covariance 首项为 `-1`。
- TF 包含 `odom->base_link`、`base_link->laser_frame/imu_link` 和完整活动臂链，
  不含 `map->odom`。
- probe 从 ROS `/mobilecar/cmd_vel_nav_test` 发送命令，底盘产生 `0.0169m` 实际位移；
  输入停止后 bridge 在 0.25s watchdog 后发送 release，health 记录 timeout。

## 精简证据

- [物理报告](evidence/g3a_acceptance_report.json)
- [ROS 报告](evidence/g3a_ros_acceptance_report.json)
- [ROS probe](evidence/g3a_ros_probe.json)
- [artifact index](evidence/g3a_server_artifact_index.json)

原始 20.6 MiB IPC NDJSON、完整 MuJoCo/ROS 日志和 Episode 留在服务器。
