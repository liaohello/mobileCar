# mobileCar
An open-source robot car control framework that integrates Vision-and-Language Navigation (VLN) and Vision-Language-Action (VLA), enabling an end-to-end closed loop from natural language instructions to real-world autonomous navigation.

## 当前状态

G1、G2A、G2B、G3A、G3B、G4A-1均已完成。G2A已在服务器验证5Hz采集、时间/变换和浏览器可靠性；G2B已完成官方LeRobot开发出口与因果追溯，训练资格仍为false。G3A已通过导航观测、独立平面里程计、健康/TF、ROS 2 Humble桥接、受控速度、人工接管、失效停车和Reset有限验收。G3B已在服务器完成两场景建图与重载定位、RViz设点、Nav2全向到点、人工接管、有限臂活动、失效停车和正式10例隔离验收。G4A-1已在服务器完成固定底盘受限 link4 RGB-D/本体视图、六维动作接口、中央/偏移抓取、Depth/接管/Reset 中止和采集就绪报告。见[G2A报告](docs/g2a_server_report.md)、[G2B报告](docs/g2b_report.md)、[G3A报告](docs/g3a_report.md)、[G3B报告](docs/g3b_report.md)和[G4A-1报告](docs/g4a1_report.md)。

G3B最终 release 为 `g3b-20261018T103442Z-a4081b0147bd`，aggregate 为
`a4081b0147bd871a5161aa2b574f534dec44fa66cf6a47969f94aab63a51d9a5`。服务器
完整受影响回归为 `123 passed`，正式10例及地图、子检查、RViz GUI、数据完整性和
最终进程审计均绑定该 release。导航 Episode 的 `training_qualified=false`；本轮结束
后不自动进入训练或实车。

G4A-1最终 release 为 `g4a1-20261018T125226Z-134032d390f8`，aggregate 为
`134032d390f83a4875e8bd936b9dead4fec9fbcd55e6c251cfde3ff5c7da4747`。
六例符合预期，受影响回归 `130 passed`；parent split和正式出口尚未评估，训练资格仍为false。

G4A-2已完成H0采集工具准备，真人试点仍待用户实际操作和审计。现有采集页有图片闪烁、停止反馈和操作模式入口不清的问题；对应前端改进已整理在本版本中，尚未部署到正式采集服务。用户本人观察并控制，agent准备界面、记录、检查与导出。

8792 第三人称页面的底盘驾驶已在服务器完成排查与最小修复：根因是软件停止锁存与强制录制两道既有门控同时在浏览器内静默拦截，驾驶区没有持久原因说明。修复后驾驶区显示门控状态、一键处置按钮与"目标/受理/实际"三列速度，后端拒绝原因保留在驾驶区和事件记录中；保护逻辑未放宽。release 为 `drvfix-20261019T044549Z-0a80415b2577`，aggregate 为 `0a80415b25778927158b53a48d4af8e4ea1530934554b71ae3db763003d76369`，回归 `129 passed`（另有 `contacts.py` 历史 provenance 例外），57 项驾驶链路检查全部通过。详见[排查与修复报告](docs/drive_path_diagnosis.md)。

页面入口：`/` 是第三人称整车遥操作页面，W/S前进后退、A/D左右平移、Q/E转向；`/g4a2` 是固定底盘近场抓取采集页。后续按[执行计划](docs/post_g4a1_execution_plan.md)和[G4A-2 Goal](docs/goals/G4A2_human_demonstrations_export.md)继续，本阶段不训练、不进入实车。

## G1 状态

G1 已实现独立的 LanderPi 单场景移动操作遥操作与原始 Episode 平台：

- 自由四麦轮底盘、五轴动态机械臂、独立夹爪和固定于 `link4` 的 RGB-D 相机。
- 浏览器三维整车视图、键盘驾驶、五轴/夹爪滑块、TCP 预览/执行、停止、Reset 和只读回放。
- `mobilecar-1.0` 统一命令、状态、事件和传感器合同。
- 原生多频 Episode 录制、validator、隔离物理 Oracle 和数据资格报告。
- 真实 MuJoCo 移动抓取成功 Episode，以及 audit 完整、物理结果为 failure 的异常 Episode。

以下安装与启动命令在**服务器**执行；本地保留源码和相关文件，不运行测试、仿真或构建。后续以[服务器执行规范](docs/server_execution_policy.md)为准。

快速启动（完整步骤见 [使用说明](docs/usage.md)）：

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install -e '.[test]'
MUJOCO_GL=egl .venv/bin/mobilecar --backend mujoco --require-recording
```

打开 `http://127.0.0.1:8787`。

## 交付与证据

- [使用与启动说明](docs/usage.md)
- [G1 验收报告](docs/acceptance_report.md)
- [实际演示记录](docs/demo_record.md)
- [非作者交叉复核记录](docs/cross_review.md)
- [失败与修复记录](docs/failures_and_fixes.md)
- [成功和异常 Episode 证据](docs/evidence/g1_success_and_failure.json)
- [原始来源与哈希清单](provenance/source_manifest.json)
- [多 agent 实施计划](docs/implementation_plan.md)
- [最小接口与原始数据契约](docs/interface_contract_v1.md)
- [运行时 API](docs/runtime_api_v1.md)

本次交付完成 G1 后停止，不包含导航、训练、LeRobot 正式导出或实车运动。

## 后续执行

- [G4A-1之后的执行计划（当前）](docs/post_g4a1_execution_plan.md)
- [G4A-2：真人示教与数据出口 Goal（待启动）](docs/goals/G4A2_human_demonstrations_export.md)
- [G3B之后的执行计划（历史）](docs/post_g3b_execution_plan.md)
- [G4A-1：近场抓取与示教采集准备 Goal（已完成）](docs/goals/G4A1_nearfield_readiness.md)
- [G4A-1服务器报告（已完成）](docs/g4a1_report.md)
- [G4A-1精简证据索引](docs/evidence/g4a1_server_artifact_index.json)
- [G3A之后的执行计划（历史）](docs/post_g3a_execution_plan.md)
- [G2B之后的执行计划](docs/post_g2b_execution_plan.md)
- [G2A之后的执行计划（历史）](docs/post_g2a_execution_plan.md)
- [G1之后的服务器执行计划（历史）](docs/post_g1_execution_plan.md)
- [服务器执行与本地源码管理规范](docs/server_execution_policy.md)
- [G2A：服务器观测与遥操作加固 Goal（已完成）](docs/goals/G2A_server_quality.md)
- [G2B：服务器离线 LeRobot 导出 Goal（已完成）](docs/goals/G2B_lerobot_export.md)
- [G3A：导航观测、里程计与ROS桥接 Goal（已完成）](docs/goals/G3A_navigation_foundation.md)
- [G3A服务器报告（已完成）](docs/g3a_report.md)
- [G3B：建图、重载定位与到点 Goal（已完成）](docs/goals/G3B_mapping_navigation.md)
- [G3B服务器验收报告（已完成）](docs/g3b_report.md)
- [G3B精简证据索引](docs/evidence/g3b_server_artifact_index.json)

G3B已完成并停止；大地图、原始 Episode、bags 和完整日志保留在服务器。后续示教、学习、
Frontier、目标搜索或实车任务需另行启动新Goal。
