# G3B 服务器复现说明

日期：2026-10-18。所有命令均在服务器项目根的冻结 release 目录内执行。

## 环境

- ROS 2 Humble overlay：`envs/g3b-humble-overlay`。
- MuJoCo/Python：`envs/g2a-py310-mujoco316`。
- Humble 包：slam_toolbox 2.6.10、Nav2 1.1.20、RViz 1.14.x。
- 同一时刻只启动一个物理 MuJoCo；每个案例使用独立 `ROS_DOMAIN_ID`。
- overlay 的 `source` 会打印一条无害的缺少 `local_setup.bash` 提示。

以下变量与最终 `SOURCE_MANIFEST.json` 一致：

```bash
export G3B_RELEASE=g3b-20261018T103442Z-a4081b0147bd
export G3B_AGGREGATE=a4081b0147bd871a5161aa2b574f534dec44fa66cf6a47969f94aab63a51d9a5
cd "$HOME"/Desktop/mobileCar/releases/"$G3B_RELEASE"
source ../../envs/g3b-humble-overlay/overlay_env.sh
export LD_LIBRARY_PATH=../../envs/g3b-humble-overlay/usr/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 构建和测试

```bash
sha256sum -c SHA256SUMS
npm ci --no-audit --no-fund
npm run build:ui

colcon build \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --symlink-install \
  --event-handlers console_direct+
source ros2_ws/install/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
mkdir -p ../../var/reports/g3b-final-pytest-a408
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONPATH=src:$PYTHONPATH \
  ../../envs/g2a-py310-mujoco316/bin/python -m pytest \
  --junitxml=../../var/reports/g3b-final-pytest-a408/junit.xml \
  --ignore=tests/integration/test_provenance.py
```

最终服务器结果为 `123 passed`、0 failure、0 error。`test_provenance` 对历史
`contacts.py` hash 的既有失败单独保留，不把该文件排除解释为测试通过。ROS setup 必须在
启用 `set -u` 前 source；否则 Humble 的 `AMENT_TRACE_SETUP_FILES` 会被 shell 判为
未定义。

## 地图准备

每个命令均来自实际 MuJoCo scan 和 G3A 独立轮/IMU 里程计；不读取场景几何生成占据图。

```bash
source ros2_ws/install/setup.bash
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONPATH=src:$PYTHONPATH \
  ../../envs/g2a-py310-mujoco316/bin/python tools/g3b/run_mapping.py \
  --config configs/g3b_server_room_a.yaml \
  --map-id map_room_a_g3b_release_v1 \
  --scene-id navigation_diagnostic \
  --report-root ../../var/reports/g3b-final-map-room-a \
  --map-root ../../var/maps/map_room_a_g3b_release_v1 \
  --domain-id 30

MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONPATH=src:$PYTHONPATH \
  ../../envs/g2a-py310-mujoco316/bin/python tools/g3b/run_mapping.py \
  --config configs/g3b_server_room_b.yaml \
  --map-id map_room_b_g3b_release_v1 \
  --scene-id g3b_l_shape_wide_door \
  --report-root ../../var/reports/g3b-final-map-room-b \
  --map-root ../../var/maps/map_room_b_g3b_release_v1 \
  --domain-id 31
```

每条 map package 应包含 `map.pgm`、`map.yaml`、`posegraph.posegraph`、
`posegraph.data`、`manifest.json` 和 `manifest.sha256`。YAML 必须为 trinary，
并从 fresh app/Episode 加载 posegraph 重启定位。

## 正式 10 例

```bash
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONPATH=src:$PYTHONPATH \
  ../../envs/g2a-py310-mujoco316/bin/python \
  tools/g3b/run_formal_acceptance.py \
  --cases configs/g3b/cases_v1.yaml \
  --report-root ../../var/reports/g3b-final-formal-10-v8 \
  --domain-base 40 \
  --case-timeout-s 360
```

runner 顺序执行 A01、A02、B01、B02、A03、B03、A04、A05、B04、A06。
每例独立启动 app、mapping-required check、localization、Nav2 和 supervisor；
每例结束后关闭本任务进程组。

## 必需子检查与 GUI

```bash
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONPATH=src:$PYTHONPATH \
  ../../envs/g2a-py310-mujoco316/bin/python tools/g3b/run_subchecks.py \
  --cases configs/g3b/cases_v1.yaml \
  --formal-report ../../var/reports/g3b-final-formal-10-v8/g3b_formal_report.json \
  --report-root ../../var/reports/g3b-final-subchecks-v8 \
  --domain-base 140
```

子检查包括 odom frame 拒绝、地图外拒绝、unknown cell 拒绝、篡改 scene 兼容字段的
运行时拒绝、重复 `map->odom` 同 stamp 冲突检查、NaN/+Inf 消费者策略、全臂 footprint、
backend rejection receipt 和 RViz 自动鼠标设点。

单独复跑 RViz：

```bash
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONPATH=src:$PYTHONPATH \
  ../../envs/g2a-py310-mujoco316/bin/python tools/g3b/run_rviz_gui.py \
  --config configs/g3b_server_room_a.yaml \
  --map-root ../../var/maps/map_room_a_g3b_release_v1 \
  --map-id map_room_a_g3b_release_v1 \
  --report-root ../../var/reports/g3b-final-rviz \
  --domain-id 150 \
  --display-number 96
```

最终子检查使用 domain 140–144。任务结束后正常停止对应 ros2cli daemon，并对本 release
做只读进程检查：

```bash
for domain_id in 140 141 142 143 144; do
  ROS_DOMAIN_ID="$domain_id" ros2 daemon stop || true
done
```

最终 completion、资源和回归 job 分别为 `g3b-final-completion-record`、
`g3b-final-resource-summary`、`g3b-final-audit-a408-regression-2`；独立进程审计 job
为 `g3b-final-process-audit`。

## 结果边界

- Action 成功、物理评分和数据完整性分别报告。
- `format_valid`/`data_complete` 只描述 Episode 记录；G3B 导航 Episode 不是人类抓取
  示教，`training_qualified=false`。
- 地图大文件、原始 Episode、完整日志和截图留在服务器；本地仅保存摘要、hash、配置和
  少量关键截图。
- RViz 关闭时可能返回 Qt/rclpy teardown 崩溃码，已知限制见失败记录；Action、物理和
  截图门仍分别核验。
