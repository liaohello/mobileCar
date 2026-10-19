# G1/G2A/G2B/G3A/G3B/G4A-1 证据索引

G1 内容保持历史原状。G2A 最终小证据见
[`g2a_server_artifact_index.json`](g2a_server_artifact_index.json)；其 release、远端原始
Episode、资产计数和复算命令与本地小证据逐项绑定。

## G2A

- `g2a_server_acceptance.json`：同 release 的 5/5 gate 验收汇总。
- `pytest-g2a.xml`：同 release 的 55 tests JUnit。
- `g2a_renderer_probe.json`：MuJoCo EGL 与 full Chromium Vulkan/ANGLE 实际 renderer。
- `g2a_browser_full_chromium.png`：30 s 页面验收截图。
- `g2a_stationary_quality.json` / `g2a_stationary_eligibility.json`：静止质量 PASS，
  D0/D2/D3=false。
- `g2a_motion_quality.json` / `g2a_motion_eligibility.json`：运动质量 PASS，
  声明 profile 内 D0/D2=true、D3=false。

## G2B

- `g2b_export_report.json`：format/export/train 三结论、逐行 trace、loader 和拒绝报告。
- `pytest-g2b.xml`：export/data/contract 定向 JUnit。
- `g2b_pip_check.txt`：独立 LeRobot 环境依赖一致性。
- `g2b_d0_info.json` / `g2b_d2_info.json`：官方 v3.0 metadata、feature shape 和 dev split。
- `g2b_d0_dataset_manifest.json` / `g2b_d2_dataset_manifest.json`：输出文件大小和 SHA-256。
- `g2b_server_artifact_index.json`：服务器 dataset/sidecar/report 路径、输入源 hash 和复算命令。

## G3B

- `g3b_final_formal_report.json`：同一最终 release 的正式10例、Action 结果、隔离物理评分、
  backend receipt、gate 和 Episode 完整性。
- `g3b_final_subchecks_report.json`：四个负向目标、NaN/+Inf、footprint 和 RViz GUI。
- `g3b_duplicate_tf_source_decision.json`：保留原始严格 TF 检查失败，并说明同一
  slam_toolbox 源的同 stamp 微修正分类。
- `g3b_rviz_goal_interaction.png`：自动鼠标设位置和朝向后的最终 RViz 截图。
- `g3b_map_room_a_manifest.json` / `g3b_map_room_b_manifest.json`：两份地图包 manifest
  和 posegraph 资产摘要。
- `pytest-g3b-final.xml`：最终 release 完整受影响回归，123 tests / 0 failures / 0 errors。
- `g3b_completion_audit.json`：服务器完成审计、逐例状态、数据/训练资格、地图摘要、
  JUnit hash 和最终进程检查。
- `g3b_server_artifact_index.json`：最终 release、原始报告、地图、截图及服务器路径。

## G3A

- `g3a_acceptance_report.json`：同一 release 的六例物理结果、频率、年龄、漂移、
  终点误差、scan 几何、故障停车与 Reset gate。
- `g3a_ros_acceptance_report.json`：实际 MuJoCo 应用加独立 Humble bridge 的端到端结果。
- `g3a_ros_probe.json`：ROS 话题计数、消息样本、TF 表面和 NaN/+Inf 映射。
- `pytest-g3a-core.xml`：合同/backend/估计/分发/data/export 相关回归，88 passed。
- `pytest-g3a-bridge.xml`：ROS bridge 纯逻辑测试，3 passed。
- `pytest-g3a-physical.xml`：六例物理验收 JUnit。
- `g3a_server_artifact_index.json`：冻结 release、原始 run、完整日志和 trace 的服务器路径、
  hash、大小与复算命令。

## G4A-1

- `g4a1_formal_report.json`：最终 release 的六例、物理任务/安全结果和 Episode 指标。
- `g4a1_readiness.json`：format/data/task/observability/action-causality/source 分项；
  `parent_split=NOT_EVALUATED`、`training_qualified=false`、`export_valid=NOT_EVALUATED`。
- `pytest-g4a1-final.xml`：最终受影响回归 130 tests / 0 failures / 0 errors，其中新增
  G4A-1 合同测试 7 tests。
- `g4a1_center_lift.png`：真实服务器浏览器受限视图操作后的中央抬升截图。
- `g4a1_completion_audit.json`：六例、数据/因果/来源门、回归结果和服务器进程审计摘要。
- `g4a1_server_artifact_index.json`：最终 release 的运行报告、就绪报告、JUnit 和截图。

## G1

- `g1_success_and_failure.json`：API 读取的实际 Episode 元数据、validator、Oracle、资格、
  三处 TCP 到达、原始 RGB 响应检查和截图索引。
- `pytest-g1.xml`：最终 JUnit，40 tests / 0 failures / 0 errors。
- `g1_initial_recording.png`：正式录制起点。
- `g1_tcp_above_cube.png`：方块上方 TCP 计划与实测。
- `g1_mujoco_lift_hold.png`：真实抓取抬升保持。
- `g1_replay_success.png`：只读回放与完整性状态。
- `g1_replay_oracle_success.png`：隔离 Oracle 成功结论。

正式原生 Episode：

```text
runs/run_b2228ee6630e4080ab6a7c813505dcd5/
  episodes/episode_4414a5ec8ae645d29d8e7df7826160cb/   # success
  episodes/episode_4cb1700ce2e04a8f9c5276b623ddb43f/   # failure
```

成功后，验证器在移除 backend incoming 中转副本的情况下仍为 `COMPLETE/passed`；
Episode `assets/` 中的已提交原始文件足以独立校验和回放。开发过程中其他无关运行目录已移至
`/tmp/mobilecar_g1_dev_runs_archive.0VMybX/`，本交付索引不使用这些目录。
