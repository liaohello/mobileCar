# G3B 演示记录

日期：2026-10-18。状态：**最终服务器 release 已绑定并通过验收**。

- release：`g3b-20261018T103442Z-a4081b0147bd`
- aggregate：`a4081b0147bd871a5161aa2b574f534dec44fa66cf6a47969f94aab63a51d9a5`

## 开发串通

- Room A 横移：目标 `(0,-0.40,0)`，Nav2 `SUCCEEDED`；实际命令含非零 `vy`。
- Room B 穿宽门：目标 `(1.20,0.00)`，开发成功；Room B 障碍不足 clearance 的原目标失败
  已保留。
- 保存地图：Room A/B 均由实际 scan 保存占据图、posegraph 和 data；fresh process
  加载后完成定位和新目标。
- 故障：scan 注入后约 0.22s governed 零速、约 0.39s 物理停稳；定位匹配注入时
  scan/odom 保持新鲜，零速约 0.12s 后、物理停稳约 0.22s 后。
- Reset：Episode 和 navigation epoch 切换，旧 ROS 栈退出后重启，旧 receipt 拒绝，
  新目标到达。
- RViz：自动点击 `Nav2 Goal`、拖动朝向，goal `(0.236,0.314,yaw 1.571)`；
  Action 成功，物理误差 `0.0293m/3.63°`，截图包含地图、扫描和路径。

## 最终 release 演示

- Room A 正式目标冻结为 `(0,-0.25,0)`；Room B 正式目标冻结为 `(1.00,0.00,0)`，
  后者真实穿过宽门。
- 正式10例全部达到各自预期：7例到达为 Nav2 Action `SUCCEEDED`；A04 为预期取消后的
  人工接管和重新授权；A05/B04 为预期 Action 中止与故障停车。正式报告记录
  `case_success_count=10`、`action_success_count=7`。
- 普通有限臂例在导航成功后执行有限臂活动并发送同目标纠偏 Action；两个 Action 均成功。
- 两场景均从实际 scan 建图并保存占据图、posegraph 和 data，重载后再定位并到达。
- RViz 自动鼠标设位置和朝向通过；最终截图、
  [正式报告](evidence/g3b_final_formal_report.json)和
  [子检查报告](evidence/g3b_final_subchecks_report.json)均绑定同一 release。
- 最终服务器完整受影响回归为 `123 passed`；完成审计为
  [g3b_completion_audit.json](evidence/g3b_completion_audit.json)。

逐例、地图、任务回执和服务器远端路径由
[g3b_server_artifact_index.json](evidence/g3b_server_artifact_index.json) 指向。开发
目标或开发截图不作为正式分数。
