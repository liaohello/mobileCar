# G3B agent 回执与接管记录

日期：2026-10-17。

## 小型 fixture agent

- 任务：只创建 `tests/g3b/fixtures/map_goal_binding_v1.json`。
- 实际结果：agent 只加载约束并返回“请发送具体任务”，没有新增或修改任何文件。
- 判定：无有效产物；按 G3B 委派门记录并接管，不重复空派发。
- 接管：主负责人创建完整 map package、authorize/cmd_vel receipt 和 stale epoch/旧
  goal generation 拒绝 fixture。
- 约束：fixture 明确标记 `NOT_SERVER_EVIDENCE`，不能作为地图、定位或导航通过证据。
- 后续服务器请求：用 G3B 合同解析 fixture，并验证两个 accepted 与一个 expected
  rejection。

## A/B/C

小型 fixture 的服务器合同检查已通过，但随后创建的三条执行 lane 均未提交可核对源码：

- A（建图定位）：会话只返回通用就绪状态，未新增或修改
  `ros2_ws/src/mobilecar_mapping_localization/**`、`tests/mapping/**`。
- B（导航控制）：会话只返回通用就绪状态，未新增或修改
  `ros2_ws/src/mobilecar_goal_supervisor/**`、`tests/navigation/**`。
- C（场景评分）：会话只返回通用就绪状态，未新增或修改
  `tests/g3b_cases/**` 或场景资产。
- 判定：三路均为无有效产物；按续跑合同记录并由主负责人接管，不循环创建同类空任务。
- 已存在地图、Room A 横移、Room B `(1.20, 0.00)` 最小到点结果为当前会话开发证据，
  但在统一最终 release 重跑前不构成 G3B 正式通过。
- 后续只在主负责人完成定向实现后请求针对明确文件/行的独立审查；不能把本次接管或
  作者自检描述成独立 agent 实现。
