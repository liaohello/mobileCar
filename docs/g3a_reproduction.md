# G3A 服务器复现与启动说明

所有安装、构建、pytest、MuJoCo、ROS 和指标计算在服务器项目根目录执行。本地不运行
这些命令。连接信息由 `tools/server/remote_job.py` 从未跟踪的 `remote.md` 读取，不
上传或输出凭据。

## 1. 冻结 release

验收 release：

```text
releases/g3a-20261017T194117Z-aa6a6e987115
```

aggregate：

```text
aa6a6e9871159310cf3d2499847e181a7e962f7a8c552c9d5304461806848859
```

进入 release 后先核对：

```bash
sha256sum -c SHA256SUMS
python3 -c 'import json; p=json.load(open("SOURCE_MANIFEST.json")); print(p["snapshot_id"], p["aggregate_sha256"], p["file_count"])'
```

## 2. 后端定向回归

```bash
mkdir -p ../../var/reports/g3a-repro-regression
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONPATH=src \
  ../../envs/g2a-py310-mujoco316/bin/python -m pytest \
  tests/contracts tests/distribution tests/estimation \
  tests/integration/test_app_mock.py tests/integration/test_navigation_control.py \
  tests/backend tests/data tests/export -q \
  --junitxml=../../var/reports/g3a-repro-regression/core.xml
```

## 3. ROS Humble 构建和单测

```bash
bash scripts/g3a_ros_build.sh
source /opt/ros/humble/setup.bash
PYTHONPATH=ros2_ws/src/mobilecar_navigation_bridge:src:$PYTHONPATH \
  python3 -m pytest tests/bridge -q \
  --junitxml=../../var/reports/g3a-repro-regression/bridge.xml
```

构建输出在 release 内 `ros2_ws/build`、`ros2_ws/install`、`ros2_ws/log`，这些生成树
不进入源码 release。

## 4. 物理有限验收

脚本顺序保持一个 MuJoCo 实例和一个物理推进者：

```bash
mkdir -p ../../var/reports/g3a-repro-physical
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONPATH=src \
  ../../envs/g2a-py310-mujoco316/bin/python tools/g3a/run_acceptance.py \
  --config configs/g3a_server.yaml \
  --profile configs/profiles/g3a_navigation_v1.yaml \
  --report-root ../../var/reports/g3a-repro-physical \
  --job-id g3a-repro-physical
```

脚本顺序执行静止 30s、平移/组合/臂/人工接管、旋转与独立 scan 几何、臂活动、
scan/IMU/wheel 故障和 Reset。退出码 0 只在所有 gate 通过时产生；始终读取
`g3a_acceptance_report.json`，不能用退出码替代各 gate。

## 5. ROS 2 Humble 端到端

在同一 release：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
mkdir -p ../../var/reports/g3a-repro-ros
PYTHONPATH=src:$PYTHONPATH \
  ../../envs/g2a-py310-mujoco316/bin/python tools/g3a/run_ros_acceptance.py \
  --config configs/g3a_server.yaml \
  --report-root ../../var/reports/g3a-repro-ros \
  --job-id g3a-repro-ros --domain-id 73
```

该脚本启动一个 MuJoCo 应用、一个 Episode 和一个独立 ROS bridge，随后由
`scripts/g3a_ros_probe.py` 从 ROS 发布速度并验证话题、QoS、帧、TF、NaN/+Inf、
orientation unavailable 和输入超时 release。

## 6. 手动启动与检查

终端 A，应用：

```bash
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONPATH=src \
  ../../envs/g2a-py310-mujoco316/bin/python -m mobilecar \
  --config configs/g3a_server.yaml --host 127.0.0.1 --port 8793
```

IPC 默认：

```text
<server-project>/var/ipc/g3a_navigation.sock
```

终端 B，bridge：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
bash scripts/g3a_ros_start.sh
```

话题/TF 检查：

```bash
ros2 topic list | grep -E 'clock|scan|imu/data_raw|joint_states|odom|navigation_health|cmd_vel_nav_test'
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo --once /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo base_link imu_link
```

## 7. 停车与进程清理

- 通过应用 Stop/EStop 锁存停车，不按进程名批量 kill。
- bridge 输入超时会发送 zero release；关闭 bridge 时也发送 release。
- 只向本任务记录的进程组发送 SIGINT/SIGTERM。
- 验收脚本自身超时保留日志、最后停车状态和退出原因。

## 8. 证据

生成 artifact index：

```bash
PYTHONPATH=src ../../envs/g2a-py310-mujoco316/bin/python \
  tools/g3a/build_artifact_index.py --release-id <release> \
  --physical-report <physical-report> --ros-report <ros-report> \
  --regression-root <regression-root> --output <index.json>
```

本地精简副本：

- [server artifact index](evidence/g3a_server_artifact_index.json)
- [physical report](evidence/g3a_acceptance_report.json)
- [ROS report](evidence/g3a_ros_acceptance_report.json)
- [pytest core](evidence/pytest-g3a-core.xml)
- [pytest bridge](evidence/pytest-g3a-bridge.xml)

原始 20.6 MiB IPC trace、完整 server/ROS 日志和 `var/runs` 留在服务器。
