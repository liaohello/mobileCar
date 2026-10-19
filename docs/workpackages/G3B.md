# G3B 工作包与文件所有权

日期：2026-10-17。状态：执行中。公共接口先冻结；物理作业由主负责人统一排队。

## 全局约束

- 所有安装、colcon/前端构建、pytest、MuJoCo、ROS、RViz、GUI 和指标计算在服务器。
- 同时最多一个 MuJoCo 实例和一个 `mj_step` 所有者；SLAM/Nav2/supervisor 不得创建仿真。
- 保留 G1/G2A/G2B/G3A 报告、runs、地图、环境及用户文件；不提交或推送 Git。
- 每文件唯一写入者；公共合同、backend/IPC/bridge/app/launch/依赖由主负责人集成。
- 先验证一个窄 agent 任务有实际文件；无产物记录并接管，不循环空派发。

## 主负责人独占

- `src/mobilecar/contracts/**`
- `src/mobilecar/app.py`、`cli.py`、`config.py`
- `src/mobilecar/backends/**`、`src/mobilecar/ros_ipc/**`
- `src/mobilecar/distribution/**`、`src/mobilecar/estimation/**`
- `ros2_ws/src/mobilecar_navigation_bridge/**`
- `ros2_ws/src/mobilecar_goal_supervisor/**`（最终集成所有权）
- `ros2_ws/src/mobilecar_mapping_localization/**`（最终集成所有权）
- `ros2_ws/src/mobilecar_nav_bringup/**`
- `configs/g3b*.yaml`、`configs/profiles/g3b_*`、`configs/g3b/**`
- `tests/contracts/**`、`tests/integration/**`、`tests/bridge/**`
- `tools/server/**`、`tools/g3b/**`、`scripts/g3b_*`
- 所有最终公共 docs/报告/地图 manifest 与证据索引

职责：版本合同/profile；共享授权与 receipt；时钟、最终速度门和 backend 接线；
唯一 launch/lifecycle/TF owner；服务器依赖与作业；端到端集成、正式 10 例和最终 release。

## A：建图与定位

只写：

- `ros2_ws/src/mobilecar_mapping_localization/**`
- `configs/g3b/slam/**`
- `tests/mapping/**`
- `docs/workpackages/A/**`

职责：slam_toolbox Humble mapping/localization 包装、地图/posegraph 保存加载、定位
match/TF health 信号和定向测试。不得修改 backend/bridge/supervisor/Nav2 参数。

## B：目标监督与导航控制

只写：

- `ros2_ws/src/mobilecar_goal_supervisor/**`
- `configs/g3b/nav2/**`
- `configs/g3b/bt/**`
- `tests/navigation/**`
- `docs/workpackages/B/**`

职责：goal Action/BT、授权 session、cancel/takeover、迟到速度屏障、最终 governor、
acceptance receipt 关联、Nav2/DWB 全向参数和 RViz 配置。不得修改 backend/IPC/bridge。

## C：场景、包络和评分

只写：

- `assets/landerpi/scenes/g3b_*`
- `assets/landerpi/model/robot_g3b_*`
- `tests/g3b_cases/**`
- `docs/workpackages/C/**`

职责：两个场景、宽门洞/L 通路、完整臂包络工具、case manifest、隔离物理评分和证据工具。
不得自行启动物理运行、修改公共 launch 或生成导航地图。

## 冻结接口

- `src/mobilecar/contracts/g3b.py`
- `docs/g3b_interface_v1.md`
- `configs/profiles/g3b_navigation_v1.yaml`

A/B/C 发现接口缺口时报告文件/行/建议，不能直接改公共合同。G3A
`g3a-1.0` 合同和 profile 保持只读兼容。

## 委派门

第一个小任务只允许创建一个边界明确的 map/goal fixture 和回执。主负责人核验实际文件后
才允许 A/B/C 开工。所有 agent 测试请求由主负责人映射到带 `snapshot_id`、独立
ROS_DOMAIN_ID 和输出目录的服务器 job。
