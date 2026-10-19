# G2B 工作包与协作

日期：2026-10-17。状态：开发出口完成。最终 release
`g2a-20261017T181443Z-ab5d023c98d3`；format_valid=true、export_valid=true、
train_qualified=false。

## 主负责人

冻结 profile/公共类型/CLI/依赖锁；只读服务器 G2A 数据；运行完整导出、官方 loader、
源 hash 前后校验、定向回归和最终报告。主负责人独占 `configs/export/**`、
`tools/export/**`、`scripts/g2b_*`、项目依赖锁和公共合同。

## A：只读 Reader 与因果 Aligner

只写 `src/mobilecar/data/export/reader.py`、`aligner.py` 和
`tests/export/test_reader_aligner.py`。负责源事件/观测只读索引、时钟域、片段和
5 Hz 在线可用性选择。不得回写源文件。

## B：LeRobot Adapter 与 sidecar

只写 `src/mobilecar/data/export/lerobot_adapter.py`、`sidecar.py` 和
`tests/export/test_lerobot_adapter.py`。负责锁定版本官方 writer/feature/video 映射和
逐行来源 sidecar；不得改用非官方手写格式替代最终格式验证。

## Fixture agent

仅创建 `tests/export/fixtures/causal_input_v1.json`。先验证该小任务能产生可核对文件，
再决定是否扩大到 A/B；无有效文件则记录并接管，不重复空派发。

## 验收边界

不运行 MuJoCo、renderer、浏览器、G2A 的 55 项物理/UI矩阵、采集或训练。只做导出模块、
相关 data/contract 定向回归和一次稳定 release 的 G2B 验收。

## 结果

- Fixture agent 产生实际 fixture 后才扩大协作。
- A 交付 Reader/Aligner；B 交付 adapter/sidecar/tests；主负责人完成 exact feedback、
  profile、资格、CLI、pipeline、环境和正式验收。
- D0：14 行，dev Episode 1；官方 loader 全 14 样本读取。
- D2：272 行，dev Episode 1；官方 loader 全 272 样本读取。
- Static/fault 拒绝；源 hash 不变；训练资格 false。
