# G4A-1 演示记录

日期：2026-10-18。状态：最终服务器 release 已通过。

- 中央单例：先失败于观测 `timing` 字段读取、fast-sensor qpos 索引和动作新鲜度竞态；
  修复后在 `g4a1-20261018T125226Z-134032d390f8` 通过。
- 中央抓取抬升峰值约 `0.0961 m`，双侧保持约 `7.0 s`，Episode 完整。
- 小横向/纵向偏移均重新观察后抓取；最终六例全部符合各自预期。
- M04 通过 Depth 失效撤销和恢复；M05 通过接管/旧 session 拒绝；M06 通过 Reset
  新 Episode/epoch 隔离。
- 受限浏览器视图实际显示 link4 RGB、有效率和中心深度；六维动作通过页面按钮下发，
  记录来源为 `agent_ui`，教师候选来源为 `scripted`。

开发失败的每个中央串通版本保留在 服务器 `var/reports/g4a1-dev-center-*`，不与正式
六例成功混合。开发失败与修复见[失败记录](g4a1_failures_and_fixes.md)。
