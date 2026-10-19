# mobileCar G2B 离线数据出口报告

日期：2026-10-17。状态：**开发出口完成**。所有实际数据解析、H.264 编码、官方 LeRobot
写入、loader 读取和 hash 复算均在服务器执行；没有新增采集，也没有运行 MuJoCo、renderer、
浏览器、导航或训练。

最终运行时 release：`g2a-20261017T181443Z-ab5d023c98d3`，aggregate
`ab5d023c98d3d45d6c0c9b530ce91423cdcb4453d7a0fd0a9a750850e76566bc`。最终 job：
`g2b-final-unit-3`（45 passed）和 `g2b-final-export-3`。最终数据位于服务器
`var/datasets/g2b_final_3/`。

## 1. 三种结论

| 结论 | 结果 | 说明 |
|---|---|---|
| `format_valid` | **true** | 官方 `lerobot==0.4.4` 创建、finalize，并能通过官方 `LeRobotDataset` 读取全部样本和 batch |
| `export_valid` | **true** | 固定 5 Hz profile 的 D0/D2 逐行来源、动作层、因果到达、新鲜度、单位和媒体检查通过 |
| `train_qualified` | **false** | 当前只有 `agent_ui`、`third_person_3d`、`task_result=failure` 的单父 Episode，不满足训练或人类示教资格 |

`format_valid` 和 `export_valid` 不升级为任务成功、策略效果、人类示教或训练资格。

## 2. 冻结版本与 profile

- LeRobot package：`lerobot==0.4.4`
- release tag commit：`8fff0fde7c79f23a93d845d1a50e985de01f8b8a`
- wheel SHA-256：`e53800ead8216861540ad3aebaf12e3cf87a399b3c1f234eeead33716c9c24fd`
- dataset codebase：`v3.0`
- Python：3.12.14；独立环境 `envs/g2b-lerobot-0.4.4`
- PyTorch/torchvision：`2.7.1+cpu` / `0.22.1+cpu`
- 视频：RGB，H.264/PyAV，640x480，5 fps
- profile：`configs/export/g2b_export_v1.yaml`
- 完整依赖锁：`requirements-g2b-lerobot.lock`

固定 grid 为源 `recording_start.sim_time_s + 0.13 s`，随后每 0.2 s 一个决策点。+
时钟映射只使用源中显式配对的 sim/backend-monotonic 时刻；persist 只证明可靠写入，
不作为在线可用时间。图像 age 门 0.4 s、本体 age 门 0.2 s、图像/本体 capture 偏差门 0.2 s。

## 3. 输入

| 用途 | run / Episode | 原结论 |
|---|---|---|
| Motion D0/D2 | `run_81ccba839bec4e1991278b913fa2f1b3` / `episode_c20c70aa2ada4e4fbd4c1a9ec5672090` | agent_ui, third_person_3d, task failure |
| Static 拒绝对照 | 同 run / `episode_32975d5ce7eb4b10a58588858de103d7` | quality PASS, no demonstration action |
| Fault 拒绝对照 | `run_d669e1a5aa2149cfac566d9990c75703` / `episode_b08d13faa97341088a8c21c4136b4d2e` | recorder failure, audit INCOMPLETE |

Motion 源 Episode 有 931 个文件和约 384.8 MiB 原始资产。导出前后 source-tree SHA-256
均为 `761f70e969ec2bdcad105bc2b670e33aa389ac13e94045a8178fb6c8eb9ea568`。

## 4. D0 开发出口

- 行数：14，单段连续 Episode，官方 info 显示 `splits={"dev":"0:1"}`。
- state：4 个轮速度 + 3 个 IMU acceleration + 3 个 IMU gyro，共 10 维。
- action：实际 `actuator_reference.base_twist` 的 `[vx, vy, wz]`，base_link，m/s、m/s、rad/s。
- 未把 `base_twist_body_m_s`、Lidar、world_sim 或反馈底盘速度写入特征。
- 每条 action 都保留实际 dispatch command、request/approval、control tick 和生效区间。
- 官方 loader：14/14 样本逐帧读取；single shape state `[10]`、action `[3]`、image `[3,480,640]`；
  batch shape `[4,10]`、`[4,3]`、`[4,3,480,640]`。

## 5. D2 开发出口

- 行数：272，单段连续 Episode，官方 info 显示 `splits={"dev":"0:1"}`。
- observation：实际五轴 user angle rad + 实际夹爪宽度 m；图像为 link4 RGB。
- action：`actuator_reference` 的模型五轴参考转换为 user angle，加米制夹爪目标，共 6 维。
- 零位/方向、模型/配置/arm_spec hash 随每行 sidecar 保存。
- Depth/Mask 不写入策略 feature，原始引用与 SHA-256 保存在 server sidecar。
- 官方 loader：272/272 样本逐帧读取；single shape state `[6]`、action `[6]`、image `[3,480,640]`；
  batch shape `[4,6]`、`[4,6]`、`[4,3,480,640]`。

首/中/末 loader 样本都能反查到父 Episode、recording segment、image sample/control tick、
joint feedback sample 和 actuator-reference control tick。报告内保存了三个完整 sidecar trace。

## 6. 拒绝与负例

- Motion 的 305 个网格槽中，D0 接受 14，D2 接受 272；拒绝原因按
  `d0_no_effective_dispatch`、`d0_post_zero_hold_ended`、`d2_no_effective_dispatch`、
  `duplicate_image_for_new_grid`、`feedback_age_exceeded`、`image_not_arrived_by_decision`
  分别计数，没有补零、未来插值或复用旧图像。
- Static 的 303 个槽没有真实底盘/臂/夹爪演示命令，D0/D2 都是 0 行；不生成空占位示范。
- Fault Episode 的 audit 为 INCOMPLETE，直接拒绝，`grid_slots=0`。
- Fixture 覆盖 future arrival、错时钟、跨 Episode、缺媒体、长缺口、未知动作、
  preview/rejected、world_sim truth 和父 Episode train/validation 泄漏。

## 7. 来源与 split

每个输出行 sidecar 包含：

- run/episode/recording segment、operator/view source、原始 task result；
- grid sim 与 decision backend-monotonic；
- RGB/Depth/Mask sample 和 capture/publish/arrival/persist；
- encoder/IMU/feedback sample、capture tick；
- request/approved/dispatch/control tick、动作层、单位和生效区间；
- age/skew、profile/rule hash、include/exclude 原因。

只有一条真实父 Episode，因此官方数据和 sidecar 都只声明 dev；train/validation 为空。
没有把同一父 Episode 的子段分到不同 split。

## 8. 证据

- 验收 JSON：[g2b_export_report.json](evidence/g2b_export_report.json)
- 定向 JUnit：[pytest-g2b.xml](evidence/pytest-g2b.xml)
- 依赖检查：[g2b_pip_check.txt](evidence/g2b_pip_check.txt)
- D0 官方 info：[g2b_d0_info.json](evidence/g2b_d0_info.json)
- D2 官方 info：[g2b_d2_info.json](evidence/g2b_d2_info.json)
- D0 output manifest：[g2b_d0_dataset_manifest.json](evidence/g2b_d0_dataset_manifest.json)
- D2 output manifest：[g2b_d2_dataset_manifest.json](evidence/g2b_d2_dataset_manifest.json)
- 远端数据/报告/sidecar 索引：[g2b_server_artifact_index.json](evidence/g2b_server_artifact_index.json)

复现命令见 [G2B 服务器复现](g2b_reproduction.md)，逐段记录见
[G2B 演示记录](g2b_demo_record.md)，开发失败与修复见
[G2B 失败与修复](g2b_failures_and_fixes.md)。

## 9. 已知限制

- 只有一个 motion 父 Episode；D0 只有 14 个有效开发样本，覆盖不足以训练导航 BC。
- D2 为 `agent_ui`、第三人称视图、任务 failure；link4 目标可见性未做正式人工评价。
- H.264 有损视频不等于原始 PNG 字节；原媒体和 Depth/Mask 保留在服务器。
- 当前 action 为执行参考型开发出口，不是已验证的部署策略控制适配器。
- 不包含 D1/D3、LeRobot Hub 上传、训练、评估、导航或实车。
