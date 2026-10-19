# G2A 执行 agent 回执

日期：2026-10-17。三个执行 agent 已创建并分配明确文件所有权，但均未产生可核对的代码、
测试或服务器 job。主负责人按 `docs/goals/G2A_server_quality.md` 的无有效交付规则停止重复
派发并接管，不把这些 agent 描述成实现者或独立审查者。

## A：后端、时钟与规划

- 状态：无有效交付。
- 实际修改文件：无。
- 最终实现：主负责人完成；见 `docs/g2a_failures_and_fixes.md`。
- 验证：同最终 release 的 backend 定向测试和 55 项回归，均在服务器执行。

## B：三维界面可靠性

- 状态：无有效交付。
- 实际修改文件：无。
- 最终实现：主负责人完成按需渲染、像素比/阴影策略、优先安全输入、失焦/断连/重连链和
  full Chromium 测试。
- 验证：服务器 full Chromium 153 + NVIDIA Vulkan/ANGLE。

## C：数据质量与资格

- 状态：无有效交付。
- 实际修改文件：无。
- 最终实现：主负责人完成 sample-id 唯一性、三窗口报告、截断/因果时域、policy allowlist、
  observation/action profile 绑定和 STOP-only 资格反例。
- 验证：服务器数据定向测试及最终 S4 质量/资格文件。

## 主负责人

- 完成公共合同、app/CLI/config 集成、release 同步、专用环境、服务器构建、回归、MuJoCo、
  full Chromium、故障注入、证据回传和报告。
- 最终 release：`g2a-20261017T163244Z-5cbe582fa6b2`。
