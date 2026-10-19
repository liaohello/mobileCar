# G2B 离线出口接口 v1

日期：2026-10-17。状态：E0 冻结，待服务器实现与验证。

## 数据流

```text
只读 Episode Reader
  -> 5 Hz causal Aligner
  -> D0/D2 profile Exporter
  -> locked official LeRobot Writer
  -> official LeRobotDataset loader
```

Reader、Aligner、Writer 均不得写回源 Episode，不得启动后端、MuJoCo、浏览器或训练。

## 文件所有权

| 所有者 | 文件 |
|---|---|
| 主负责人 | `configs/export/**`、profile/资格、CLI、依赖锁、服务器 release/job、最终验收 |
| A | `src/mobilecar/data/export/reader.py`、`aligner.py`、`tests/export/test_reader_aligner.py` |
| B | `src/mobilecar/data/export/lerobot_adapter.py`、`sidecar.py`、`tests/export/test_lerobot_adapter.py` |
| fixture agent | `tests/export/fixtures/causal_input_v1.json` |

公共类型或现有运行时代码变更必须由主负责人集成。

## 冻结 profile

- grid：5 Hz，源 `recording_start.sim_time_s + 0.13 s` 固定相位，保存绝对 sim 与
  backend monotonic 映射。相位由服务器现有 G2A 图像抖动扫描预先选定，最大化 0.2 s
  因果新鲜度门内的连续覆盖。
- image age：<= 0.4 s；proprio age：<= 0.2 s；image/proprio capture 偏差：<= 0.2 s。
- 仅选择 `arrival <= decision_time` 的样本；persist 只证明可靠保存。
- 禁止未来插值、复制旧图像、跨 Episode/Reset 拼接；超时/长缺口拆段或拒绝。
- D0 action：实际生效 body twist `[vx,vy,wz]`，base_link，SI。
- D2 action：实际 `actuator_reference` 的模型五轴参考映射为用户角，加夹爪宽度米制目标。
- D2 observation：五轴实际用户角、夹爪实际宽度、link4 RGB；Depth/Mask 仅 sidecar。
- Policy features 不含 `world_sim`、Oracle、整对象 diagnostics 或第三人称手柄状态。

## 逐行 sidecar

每行至少记录：

- run/episode/recording segment；
- grid、decision sim/monotonic；
- RGB/Depth/Mask sample、capture/arrival/persist；
- joint feedback sample/control tick/capture/arrival；
- command、dispatch、actuator control tick 和生效区间；
- action profile/layer、值和单位；
- selection age/bias、规则 hash、segment、include/exclude 原因。

## 三结论

| 结论 | 含义 |
|---|---|
| `format_valid` | 官方锁定版本可以创建、finalize、逐样本和小 batch 读取 |
| `export_valid` | 指定 D0/D2 profile 的逐行来源、动作、时钟、新鲜度和缺口门通过 |
| `train_qualified` | 指定训练用途还满足来源、任务结果、可观测性和覆盖 |

三者独立。当前 G2A `agent_ui`/`third_person_3d`/failure 来源必须保留，预期
`train_qualified=false`，不阻塞开发出口的 format/export 验收。
