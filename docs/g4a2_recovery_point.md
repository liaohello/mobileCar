# G4A-2 当前恢复点

日期：2026-10-18。状态：**H0 工具准备完成，等待用户本人 H1 三条试点**。

- 承接 G4A-1：`g4a1-20261018T125226Z-134032d390f8`
- 采集运行时/UI 修复 release：`g4a2-20261018T171942Z-030bf47cef6f`
- 采集运行时/UI 修复 aggregate：`030bf47cef6f6084720227f26e2874fecd0671e28191c6bd11d9866a4e33ddd6`
- 离线工具 release：`g4a2-20261018T162926Z-4938bfe1e2d2`
- 离线工具 aggregate：`4938bfe1e2d2d0e1b23781924529a4d9712e797dbc6e3dd4fcc912281f93cb4a`
- 服务器采集页：`127.0.0.1:8792/g4a2`，仅通过 SSH tunnel 访问
- 当前物理实例：一个 MuJoCo app，端口 8792，recording 尚未由用户开始

## 已完成

- G4A-2 human session、human intent、普通六维动作合同。
- 独立 `/g4a2` 受限 RGB-D 页面，不加载第三人称画布。
- H0 合同/static UI 测试 `6 passed`，离线 split/reader/合同测试 `8 passed`，现有 export smoke
  `17 passed`。
- 锁定 LeRobot 0.4.4 synthetic 2-Episode RGB-D writer/finalize/official loader 读回通过。
- G4A-2 human action reader/qualification 增加 human source、intent/action link、
  display sample、普通六维路径和 parent/family split 负例测试。
- H1 自动发现工具已扫描服务器 runs：当前 `count=0`，状态
  `WAITING_FOR_3_HUMAN_EPISODES`。
- UI 修复已通过服务器状态机 smoke：正常停止保存、立即结束反馈、停止失败、
  破图保留旧帧/过期标记、实际显示 frame 与动作 sample 绑定、重新开始录制。
- 服务器页面 smoke：HTTP 200，无 `three-container`/snapshot 调试节点，截图索引已回传。

## 未完成/下一动作

1. 用户本人完成两条成功抓取和一条取消/失败，来源必须为真实 human session。
2. 检查三条 H1 的显示 sample→human intent→server receive→control decision→dispatch→
   actual tick 因果链及 RGB-D/physical gates。
3. H1 通过后冻结 12 family、16/4/4 split 和正式 cohort，再开始默认 24 条成功采集。
4. 最后执行官方 LeRobot RGB-D 全样本、batch、逐行来源、parent/family 防泄漏和源
   不可变检查。

真人未操作、数量不足、出口或资格未通过时不得标记 Goal complete。

## 2026-10-19（UTC）部署核查与线上验收

用户在修复前 bundle 上截图反馈问题（停止后 NO RGB-D 覆盖录制状态、旧图残留、停止反馈只在日志）。
本轮服务器只读核查确认：

- 线上 8792 服务（PID 3824559）cwd 为 `releases/g4a2-20261018T171942Z-030bf47cef6f`，
  由作业 `var/jobs/g4a2-ui-page-restart` 于 2026-10-18T17:20Z 启动；用户截图早于该重启，
  截图中的旧行为来自修复前 bundle。
- 线上所服务 5 个静态资源（g4a2.js/g4a2.html/app.js/style.css/index.html）哈希与该 release
  逐一匹配；本地工作树与该 release SOURCE_MANIFEST 370 个文件中 367 一致，3 个差异均为
  同步后更新的文档，源码零差异。故本轮不重启同一 release（避免物理实例无意义 churn）。
- 证据索引：`docs/evidence/g4a2_live_deploy_acceptance_20261019.json`。

线上验收（全部在服务器执行，automation 来源，不计真人示教）：

1. 慢网/破图（对真实部署 bundle，API 全拦截）：`check_preview_ui.py` 4/4 通过
   （慢图单飞且布局稳定、破图保留上一帧、无隐式真人确认、显示样本与加载时间先于 intent）。
2. 正常停止/停止失败/重新开始录制（真实服务+真实 API+真实浏览器）：
   `live_acceptance.py` 9/9 通过（新 Episode 启动、立即"正在结束"、"已结束+记录已保存+Episode ID"
   与 health 一致、拦截一次 stop 失败后"结束失败+原因"可见且重试后保存、可重新开始录制、
   无真人会话、无页面错误）。
3. 数据完整性：验收前后 `discover_h1` 发现集逐字节一致（count=2），自动验收 6 条
   automation Episode 均不含 human_session_confirmed，不计入 H1。

H1 现状更正：`var/runs` 下现有 2 条满足 human_teleop + link4_rgbd_proprio +
human_session_confirmed 的 Episode，均为 task_result=failure（用户修复前试用留下）：
`episode_ff3423e1578b4aa58e8ea50d5393dcc2`（run_8302fbd17a6b4d9b86e7297fcde441f5）、
`episode_f2f3b92cc6624146b2e2338adaa1b19e`（run_eca082ac6da7412eb78b5ef3ab890aff）。
是否计入"取消/失败"试点由 H1 审计时按因果链判定，本轮不下结论。

下一动作不变：用户本人完成三条试点（两条成功抓取 + 一条取消/失败）后告知，再做只读审计。
