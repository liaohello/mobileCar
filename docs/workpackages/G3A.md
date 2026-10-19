# G3A 工作包与文件所有权

日期：2026-10-17。状态：实现与有限服务器验收完成。runtime/acceptance release
`g3a-20261017T194117Z-aa6a6e987115`，aggregate
`aa6a6e9871159310cf3d2499847e181a7e962f7a8c552c9d5304461806848859`；
同 release 物理 6/6、ROS 端到端、pytest 88+3 全部通过。最终报告和证据索引见
`docs/g3a_report.md`。公共合同在开发期冻结；以下所有权在扩大并行前生效。

## 全局边界

- 本地只编辑/读取源码、配置、资产和文档；安装、构建、pytest、lint、MuJoCo、ROS
  与指标计算只在服务器 release 中执行。
- 同时最多一个 MuJoCo 物理实例、一个 `mj_step` 所有者。ROS 节点、估计器和
  Recorder 都不得创建第二个 `mujoco.MjModel/MjData` 仿真。
- G1/G2A/G2B 的报告、runs、数据集、环境、原始记录不得覆盖、移动或重标。
- 每文件唯一写入者；公共接口、应用装配、依赖和最终验收仅由主负责人修改。
- agent 的可核对交付必须出现在共享工作树；空返回、仅等待或无文件时不重复派发，
  由主负责人记录并接管。

## 主负责人独占

- `AGENTS.md`
- `src/mobilecar/contracts/**`
- `src/mobilecar/app.py`
- `src/mobilecar/cli.py`
- `src/mobilecar/config.py`
- `src/mobilecar/backends/common.py`
- `src/mobilecar/distribution/**`（主负责人新增）
- `src/mobilecar/ros_ipc/**`（主负责人新增，与 C 的桥接包分离）
- `configs/default.yaml`
- `configs/profiles/**`
- `configs/examples/**`
- `pyproject.toml`、`requirements*.lock`
- `tools/server/**`
- `tools/g3a/**`
- `tests/contracts/**`
- `tests/integration/**`
- `tests/distribution/**`
- 公共 `docs/**`、最终报告、复现、验收索引

职责：冻结 profile/合同；backend 原始分发与估计器/ROS 消费接口装配；应用和
CLI；依赖/服务器队列；唯一 release；跨包集成；物理/ROS 有限验收和最终结果。

## A：后端观测与场景

只写：

- `src/mobilecar/backends/mujoco/backend.py`
- `src/mobilecar/backends/mujoco/navigation_sensors.py`（可新增）
- `src/mobilecar/backends/mock/backend.py`
- `src/mobilecar/control/base_authority.py`（可新增）
- `src/mobilecar/control/wheel.py`
- `src/mobilecar/sensors/**`
- `assets/landerpi/scenes/navigation_diagnostic.xml`
- `assets/landerpi/scenes/navigation_diagnostic_scene.xml`（若真实加载器需要）
- `tests/backend/**`
- `docs/workpackages/A/**`

只读：`src/mobilecar/contracts/**`、`configs/profiles/g3a_navigation_v1.yaml`。

职责：同 capture laser/轮/IMU/joint 原始生成；laser frame 与真实自遮挡；
可选 `target_cube`；无目标 Oracle `NOT_APPLICABLE`；导航与人工 base 控制组、
arm 独立性、失败停车和 Reset 清理；backend 内多消费者非阻塞分发；定向测试。
不得编辑 app/config/contracts/estimation/ROS 包。

## B：独立平面估计与健康

只写：

- `src/mobilecar/estimation/**`
- `tests/estimation/**`
- `docs/workpackages/B/**`

只读：冻结观测/估计/health 合同、G3A profile 和 wheel 参数。

职责：只消费测量白名单的麦轮/gyro 平面融合；covariance、来源、输入年龄、
health 状态、Reset/时间回跳和 stationarity；纯模块定向测试与 fixture。
不得读取模型 `MjData`、真实世界 pose、backend 内部状态或写公共合同。

## C：ROS2 Humble 桥接

只写：

- `ros2_ws/**`
- `tests/bridge/**`
- `scripts/g3a_ros_*`（`g3a_ros_` 前缀仅 C）
- `docs/workpackages/C/**`

只读：冻结 envelope/payload 合同、G3A profile、主负责人 IPC 路径定义。

职责：独立 Humble 工作区/进程；`/clock`、`/scan`、`/imu/data_raw`、
`/joint_states`、`/odom`、唯一来源 TF、health、受控 cmd_vel；capture stamp、
QoS、NaN/+Inf、orientation unavailable、输入租约与失联停车；桥接定向验证。
不得创建/步进 MuJoCo、修改 backend/app/estimation/依赖锁或启动 SLAM/Nav2。

## 冻结接口

- 观测：`NavigationObservation`，`g3a-1.0`
- 估计：`EstimatedBaseState`，来源 `wheel_gyro_planar_fusion`
- 健康：`NavigationHealth`
- IPC：`NavigationStreamEnvelope`、`NavigationControlMessage`
- 控制来源：仅 `ros_navigation_test`
- profile：`configs/profiles/g3a_navigation_v1.yaml`
- 接口说明：`docs/g3a_interface_v1.md`

A/B/C 不得修改这些公共文件；发现不一致时提交明确文件/行/建议，由主负责人
决定是否生成新冻结版本。已冻结的 G2A `g2a-1.0` 语义保持兼容。

## 验收与回执

先由 fixture agent 产生实际 LaserScan/self-occlusion 样例和测试文件，主负责人
核验后才允许 A/B/C 扩展。每个 agent 回执需记录：实际文件、未解决项、仅本地编辑
声明、请求的服务器定向测试与期望产物。所有测试请求统一由主负责人映射到带
`snapshot_id` 的服务器 job。正式案例与门见
`docs/goals/G3A_navigation_foundation.md`，结果必须与开发探针、修复复验分列。
