# C 数据线实现报告

日期：2026-10-17。合同：`mobilecar-1.0`。

## 已交付

- `Recorder`/`EpisodeRecorder`：实现 EventSink 的 `publish`、`publish_observation`、`recording_gate`，并提供装配层所需的 `status()`、`start()`、`stop()`、`validate()`、`close()`。
- 原生布局：`<root>/<run_id>/manifest.json`、`episodes/<episode_id>/episode.json`、`events.jsonl`、`commands.jsonl`、`control.jsonl`、`observations/*.jsonl`、`transforms.jsonl`、`assets/{rgb,depth,masks}`、`assessments/`、`quality_report.json`、`episode_complete.json`。
- 有界非阻塞队列；控制事件队列溢出会显式进入 FAILED，不静默丢弃。Recorder 写盘故障置 FAILED、关闭录制 gate，并可通过 `set_failure_callback(code, message)` 通知装配层执行受控停车。
- 100 Hz 控制/反馈压力下使用长生命周期文件句柄、64 KiB 缓冲、50 ms 周期 flush 和停止时逐文件 `fsync`，避免每事件重复 open/close 导致控制事件积压。
- 起始事件、初始快照和 Episode 元数据完成 `fsync` 后才进入 RECORDING；raw asset 先完成临时文件写入、`fsync`、原子替换，再发布引用。
- stop 先进入 DRAINING，等待队列排空，写 `episode_terminal`、文件校验和与 `episode_complete.json`。缺尾部、校验和不符、FAILED/INCOMPLETE 尾状态均不能通过 validator。
- 同一 Episode 停止后重开产生新 `recording_segment_id`，显式记录 `previous_recording_segment_id`。
- `ReplayReader` 只读取文件，不持有后端，不调用动作或物理推进；索引、事件、观测、媒体、校验/任务/资格报告分列。
- `validate_episode` 检查合同/run/episode 身份、事件序号、命令关联、资产存在/空文件/哈希、RGB 重复、逐流测量因果、实际频率/最大间隔/覆盖、真值隔离和完整尾部。
- `assess_grasp_oracle` 使用显式隔离 world-sim 样本检查 cube 抬升、双侧接触与连续保持，不读取默认策略观测。
- 默认 `feedback_initial`/`recording_stop` 快照会剥离 `world_sim` 位姿；Oracle 样本单独写 `assessments/oracle_samples.jsonl`。Backend 可发布 `oracle_sample`/`world_state_sample` 事件进入该隔离路径。
- `evaluate_eligibility` 分离 audit、physical task result 与 D0–D3 资格；`agent_ui` 明确保留为开发来源。
- CLI：`python -m mobilecar.data {list,info,validate,eligibility,assess}`。

## 验证

```text
PYTHONPATH=src python3 -m pytest tests/data -q
11 passed

PYTHONPATH=src python3 -m pytest tests/contracts tests/data tests/ui/test_static_ui.py -q
19 passed
```

数据测试覆盖完整 Episode/资产/回放、backend incoming 资产复制、缺资产、重复帧、深度米制/掩码、注入写盘故障、缺尾部、正常 close 排空、segment 重开、truth 泄漏、Oracle 样本隔离路径、抬升/保持与资格来源分离。测试中的图像字节是合成 fixture，不作为物理或界面证据。

## 装配注意事项

- `Runtime.start_episode` 必须先为新 Episode 取得新身份并完成后端 reset/边界，再把新快照交给 Recorder；只调用 `recorder.start` 会让 backend episode_id 不匹配。
- `Runtime.stop_episode` 应在后端停止/停车反馈完成后调用 `recorder.stop(reason, final_snapshot)`。
- 写盘故障回调应连接后端受控停车；Recorder 本身不直接写 MuJoCo 状态。
- 后端发布 raw observation 时可在 metadata 传 `asset_data_b64` 和 `valid_mask_data_b64`，或直接传相对 `asset_uri`；C 对前者先可靠落盘，对后者 validator 核验。
- `quality_report.json` 是 validator 报告；任务结果由 Oracle/Assessment 单独保存，不通过 UI 标签替代。

## 最终真实 Episode

- 成功：`episode_4414a5ec8ae645d29d8e7df7826160cb`。validator `COMPLETE`/passed，0 errors、0 warnings；Oracle `success`，抬升 0.09597 m、双侧接触连续保持 8.65 s；D0/D2/D3 可用，来源仍标 `agent_ui`。
- 异常：`episode_4cb1700ce2e04a8f9c5276b623ddb43f`。validator `COMPLETE`/passed；Oracle `failure`；终止原因 `BROWSER_CLICK_STALL`。
- 证据汇总：`docs/evidence/g1_success_and_failure.json`；JUnit：`docs/evidence/pytest-g1.xml`。
