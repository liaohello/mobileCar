# G3B预算停止诊断与恢复

日期：2026-10-17，America/Los_Angeles。状态：原因已核实，项目约束和提示词已修订；**原执行Goal尚未由本会话解除预算或恢复**。

## 1. 已核实原因

执行会话01a149a9-5221-7cd2-96ba-8aa8061ff621的原始rollout记录：

1. 本地时间19:56:27，agent调用create_goal并显式传入token_budget=0。
2. 工具拒绝：goal budgets must be positive when provided。
3. 两秒后，agent重新调用create_goal，改传token_budget=1000000。
4. 工具成功返回active、tokenBudget=1000000、remainingTokens=1000000。

随后只读状态快照显示：status=budget_limited、token_budget=1000000、tokens_used=1261120、time_used_seconds=3020。因此交接里“目标仍为active”的陈述与实际保存状态不一致；项目确实未完成，保存的Goal已经受预算限制。

这不是本轮规划文档要求的额度，也不是因为“省略参数后系统默认100万”。实际调用明确传了100万。无需预算的正确写法是省略可选token_budget字段，不能传0，也不能在0被拒绝后自行选择一个大数。

证据摘要见[g3b_goal_budget_diagnostic.json](evidence/g3b_goal_budget_diagnostic.json)。原rollout位于已有WSL客户端的/home/liaozhengyan/.codex/sessions/2026/10/17/rollout-2026-10-17T04-39-55-01a149a9-5221-7cd2-96ba-8aa8061ff621.jsonl，第10827、10829、10833、10835行。仅提取相关调用和数字，不回传完整会话。

## 2. 与其他限制的区别

- Goal自设预算：本次直接原因，创建参数错误。
- 账户用量：诊断时ordinaryUsageAllowed=true，5小时窗口已用41%，周窗口已用6%；该快照不能反推停止瞬间的全部账户状态。
- 上下文窗口、单次执行与平台限制：即使不设Goal预算，也可能存在；不能通过提示词承诺无限执行或绕过。
- 仿真/动作/作业超时：用于防止挂起和失控，保留；不是此次token预算。

## 3. 恢复办法

在**原G3B执行会话**使用当前客户端支持的Goal控制移除错误预算并恢复。如果该版本支持直接清空预算，清空后核对tokenBudget=null及status=active；没有已验证的直接编辑入口时，使用官方已有的/goal clear清除错误Goal元数据，再提交[续跑提示词](goals/G3B_resume_no_budget.md)，由agent省略token_budget重建同一G3B范围。

清除前保留原Goal身份、状态和本证据；不得伪造complete来腾位置。重建的是Goal控制状态，源码、地图、Episode、工作包和失败点继续复用。不要重新执行G1–G3A，也不要把G3B局部结果清零后从头开发。

当前规划会话的get_goal只管理当前会话；无法用它或update_goal改另一条WSL执行会话的预算。未直接修改Codex内部数据库，也未把“改了AGENTS.md”当作原Goal预算已解除。解除原Goal状态需要在原客户端完成。

## 4. 防止重现

AGENTS.md和G3B提示词已明确：

- 创建主Goal或子Goal时完全省略token_budget；不传0、1或自行估算的正数。
- 创建后立即get_goal，核对tokenBudget=null；remainingTokens=null不表示零余额。
- 先核对真实状态与停止事件，再解释为何停止，不沿用交接中的active描述。
- 无人为token上限不改变服务器执行政策、正式验收范围和必要安全超时。
- 中断时留下精简恢复记录：release、最后job、失败证据、已通过门、未完成门和下一动作；不重复安装、建图或重读整段历史。

本轮只做运行元数据读取和文档修订，未连接项目服务器，未执行构建、测试或仿真。

官方参考：[Using Goals in Codex](https://developers.openai.com/cookbook/examples/codex/using_goals_in_codex)，说明Goal的线程状态、预算停止和/goal clear生命周期控制。
