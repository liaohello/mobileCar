# G3B 最终恢复点

日期：2026-10-18。状态：**已恢复并完成最终验收**。

## 最终冻结 release

- release：`g3b-20261018T103442Z-a4081b0147bd`
- aggregate：`a4081b0147bd871a5161aa2b574f534dec44fa66cf6a47969f94aab63a51d9a5`
- 本地 provenance：
  `provenance/releases/g3b-20261018T103442Z-a4081b0147bd.json`
- 服务器 295 个冻结文件 hash 全部通过；前端构建成功。
- 本地源码也已逐文件核对；235 个非交付文档现有文件与 release manifest 一致，包括
  恢复后的 `tools/g3b/run_single_goal.py`。
- 同 release 完整受影响回归：`123 passed`，JUnit 0 failure / 0 error。

已经定向验证、但不替代最终矩阵的结果：

- Room A 目标 `(0,-0.25)`。
- Room B 目标 `(1.00,0.00)`。
- A02 导航成功后臂活动＋同目标纠偏 Action 均成功，最终约 `0.0218m / 9.82°`。
- B02 目标 `(1.00,0.00)` 分段运行成功，臂变化约 `0.040rad`。
- A04 取消、非零人工接管、松手不恢复、臂目标保持、显式新目标启动/终止已通过。
- A06 旧进程退出、新 Episode/epoch、旧 receipt 拒绝、新目标到达已通过。
- A05/B04 故障停止逻辑已修复；最终 gate 零速以发布点 steady-clock 记录。
- RViz 自动点击/拖动目标、物理到达和截图曾在开发 release 成功。

## 恢复结果

服务器 SSH 已恢复。恢复后的 preflight 确认 Ubuntu 22.04.5、ROS 2 Humble 环境可用，
没有使用本地测试替代服务器执行。

最终 release 上已经完成：

1. Room A/B 地图重新生成和 hash 校验；两份 manifest 均含 YAML、PGM、posegraph、
   posegraph data 和 digest。
2. 正式10例全部成功，Action 状态依次为
   `4,4,4,4,4,4,5,6,6,4`；A04/A05/B04 的预期取消、中止和故障停止均通过。
3. 负向目标4/4、NaN/+Inf、footprint、TF 来源补充审计和 RViz 自动 GUI 通过。
4. 10个主 Episode 全部 `COMPLETE`、`format_valid=true`、`data_complete=true`；
   `training_qualified=false`。
5. `ros2cli` daemon 正常停止，独立审计确认没有遗留物理或 ROS 任务进程。

最终报告见 [g3b_report.md](g3b_report.md)，复现步骤见
[g3b_reproduction.md](g3b_reproduction.md)，服务器证据索引见
[g3b_server_artifact_index.json](evidence/g3b_server_artifact_index.json)。
