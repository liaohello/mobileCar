# G2B 失败、修复与协作记录

日期：2026-10-17。

## 协作

先派发 fixture 小任务，确认实际生成
`tests/export/fixtures/causal_input_v1.json` 后才扩大到两个执行 agent。

- A 交付只读 Reader、5Hz Aligner 及源码；测试和 exact-tick feedback 仍缺失。
- B 交付官方 LeRobot adapter、sidecar 和测试。
- 主负责人停止继续空转，补 event-feedback、D0/D2 窗口、资格、CLI、pipeline、
  服务器环境、正式导出和验收。

未把 agent 产物冒充独立非作者审查。

## 环境

- 系统 ffmpeg 不可用，但 LeRobot 0.4.4 使用 PyAV 内置 H.264，不需要系统 ffmpeg。
- 首次 socket 查询显示只有系统 Python 3.10；进一步检查发现用户级 Python 3.12 和
  virtualenv。创建独立 `envs/g2b-lerobot-0.4.4`，没有修改 G2A 环境。
- 一次 PyTorch 依赖解析卡在慢索引的 numpy 下载；终止该自建作业后，用 CPU wheel
  `--no-deps` 加独立镜像依赖安装成功。`pip check` 无冲突。

## 时钟和窗口

- G2A stationary 的首个持久 image capture 早于 `recording_start`；时钟 anchor 过滤掉
  start 前的 capture，避免 backend anchor 非递增。
- 5Hz 固定相位在部分网格点产生约 0.20–0.21 s 图像 age；服务器扫描固定相位后，
  选择 `recording_start + 0.13 s`，在 0.4/0.2/0.2 s 门槛下最大化连续覆盖。
- Join 不采信 persist 作为在线 arrival；persist 只作可靠写入证据。
- Stop 边界有浮点微小越界，clock mapper 增加 1e-9 容差并 clamp，不改变时间值。

## 本体反馈

G2A 的 joint_feedback observation 只每 10 tick发布，而 image feedback sample id
指向每个 capture 的具体 tick。改为从 100Hz `feedback` event 按 control tick 取快照：
`feedback_time_monotonic` 为 capture，event backend time 为 arrival，
`arm_model_qpos_rad` 经零位/方向转 user angle。没有用未来插值。

## Writer 与 split

- LeRobot 0.4.4 writer 默认把唯一 Episode 标成 `train`。使用官方 `write_info`
  在 finalize 后改成 `dev: 0:1`，sidecar 显式保留 train/validation 为空。
- video feature 从初版 HWC 声明修正为标准 CHW `[3,480,640]`，帧输入仍为 HWC RGB，
  由官方 writer 转换。
- 首轮 loader trace 中 parent episode 为空，因为 ImageSample 未赋 run/episode；
  修复后首/中/末均能反查父 Episode。
- 首轮报告未记录 `write_attempted=True`，导致实际成功的 D0/D2 被总 gate 误判空；
  补字段后重新正式导出。
- Fault 与 motion 位于不同 run；pipeline 初版误共用 run_id。改为显式 fault_run_id 后
  fault 正确读取为 INCOMPLETE 并拒绝。

## 最终结果

最终候选导出为 D0 14 行、D2 272 行，两套官方 loader 和 batch 均通过；
static/fault 均拒绝，源文件 hash 前后不变。`train_qualified=false` 是预期和正确结论。
