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
