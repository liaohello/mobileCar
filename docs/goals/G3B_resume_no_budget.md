# G3B续跑：不设置人为token预算

日期：2026-10-17；状态于2026-10-18更新：**G3B已完成，此续跑提示词仅保留历史，不再次执行**。后续使用[后G3B计划](../post_g3b_execution_plan.md)及[G4A-1](G4A1_nearfield_readiness.md)。原Goal的旧预算元数据保留，新Goal单独省略token_budget；不要为了新阶段恢复已complete的G3B。

原Goal错误设置1000000 token而进入budget_limited，见[诊断](../g3b_budget_diagnosis.md)。先在原客户端调整/清除该错误Goal；不要通过update_goal伪造完成，不直接修改Codex内部数据库。

---

继续mobileCar现有G3B，用户明确不需要人为token预算。

1. 先get_goal检查原执行会话状态。若已有同范围且无预算的active Goal则继续；若原错误budget_limited状态尚未解除，准确报告tokenBudget/status和所需客户端控制，不重复尝试create_goal或绕过工具限制。用户已清除错误Goal后，调用create_goal时只传objective，**完全省略token_budget字段**。不传0、1、1000000或任何自拟额度。创建后立即get_goal，确认tokenBudget=null，再开展工作；remainingTokens=null不是零余额。

2. 复用现有源码、地图包、环境与运行记录。阅读AGENTS.md、docs/server_execution_policy.md、docs/goals/G3B_mapping_navigation.md、docs/g3b_interface_v1.md、工作包及本次恢复记录。完整G3B合同和正式10例及必需子门保持，不重开G1–G3A，不从零重做已保存实现。

3. 先诊断g3b-single-goal-room-b-1失败报告和nav2.log，确定是目标/costmap、定位、路径跟踪、控制权限、碰撞或超时问题，再做最小修复。该案例目前属于开发串通；确需调整开发目标时先解释原目标为何不适用并保留失败。正式目标仍在开发闭环后冻结，冻结后不通过换简单目标、缩小footprint或放宽判据制造通过。

4. 已有恢复点来自用户交接，实际使用前核对远端存在性与源码hash：
   - 临时release：g3a-20261018T033929Z-2a58686333eb；
   - ROS overlay：/home/liaozhengyan/Desktop/mobileCar/envs/g3b-humble-overlay；
   - Room A单目标曾SUCCEEDED，位置约0.0237m、yaw约2.845°；这是开发证据，不替代最终同release验收；
   - 两场景地图包已生成；保留并核对兼容manifest，不无故重建；
   - 定向测试曾11 passed，ROS包曾构建；不能写成新release已通过；
   - 原A/B/C lanes无可核对源码产物，追加真实接管回执，不再循环空派发。

5. 补齐正式runner、目标/地图manifest、剩余故障/接管/Reset/GUI及拒绝检查。先定向复验，稳定后生成统一g3b-... release，逐文件核对aggregate；在同一release完成正式10例、必需子门和受影响回归。保留真实失败及修复记录，不拼接不同release的成功。

6. 所有安装、构建、测试、MuJoCo、ROS/RViz、自动GUI和指标计算仍在服务器。单一物理仿真实例，由主负责人统一队列；本地只维护源码、配置和精简证据。保留作业超时、租约和停止门；“不设token预算”不代表取消这些安全限制。

7. 无需为了消耗估计或单轮结束而结束整个G3B。遇到真实平台/账户限制时如实记录工具状态并留下恢复点；不声称已突破限制。每个阶段维护简短恢复记录，减少重复读取/安装/建图和无变化的全量复测。

8. 完成G3B原合同的全部必需门，交付统一release/aggregate、接口、配置锁、逐例结果、复现/演示/失败记录、地图包清单、GUI证据、agent回执和服务器证据索引后，才标complete并停止。不进入Frontier、自动抓取、VLN/VLA、训练或实车，不自动提交或推送。
