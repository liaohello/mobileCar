# G2B 执行 agent 回执

日期：2026-10-17。

## Fixture 小任务

- 产物：`tests/export/fixtures/causal_input_v1.json`
- 状态：有效交付；JSON 可解析，包含有效因果场景和 10 个负例。
- 该交付后才扩大到两个执行 agent。

## A：Reader / Aligner

- 实际文件：`reader.py`、`aligner.py`
- 有效内容：只读解析、5Hz grid、时钟 anchor、动作 provenance、D0/D2 对齐。
- 缺口：无测试；初版 D2 使用 10Hz joint_feedback observation，无法匹配图像 exact tick。
- 主负责人补 event-feedback、start 前 anchor 过滤、phase offset、容差、零速窗口和测试。

## B：LeRobot Adapter / Sidecar

- 实际文件：`lerobot_adapter.py`、`sidecar.py`、`test_lerobot_adapter.py`
- 有效内容：官方 writer/loader、feature、sidecar、source hash、split 防泄漏。
- 主负责人补 CHW shape、官方 dev split 写入、trace parent identity 和集成验收。

## 主负责人

完成 profile、公共模型/pipeline、资格、CLI、独立 Python3.12 环境、依赖锁、服务器导出、
D0/D2 正式数据、官方 loader 全样本读取、源 hash 前后核对、总报告和证据索引。
