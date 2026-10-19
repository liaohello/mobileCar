# G2A 工作包与文件所有权

日期：2026-10-17。状态：G2A 完成；最终服务器 release
`g2a-20261017T163244Z-5cbe582fa6b2`，S4 PASS，55 tests passed。

## 全局约束

- 所有安装、构建、测试、MuJoCo、渲染、Playwright、重算和导出只在服务器 release 中执行。
- 本地只改源码/配置/文档、生成 allowlist/hash、保存 ≤20 MB 精简证据和远端索引。
- 不在本地回退测试；服务器失败保留原因和未完成项。
- 不修改 G1 runs/报告；G2A 新报告另名、另目录并绑定输入 hash。
- 每文件唯一写入者；公共接口变更只由主负责人完成。

## 主负责人独占

- `AGENTS.md`
- `src/mobilecar/contracts/**`
- `src/mobilecar/app.py`
- `src/mobilecar/cli.py`
- `src/mobilecar/config.py`
- `configs/default.yaml`
- `configs/profiles/**`
- `pyproject.toml`
- `requirements*.lock`
- `package.json` / `package-lock.json`
- `tools/server/**`
- `tests/contracts/**`
- `tests/integration/**`
- 公共 `docs/**`（A/B/C 独立 workpackage 文档除外）

主负责人同时负责服务器 release、专用环境、单一测试/仿真队列和最终验收。

## A：后端观测与时钟

- `src/mobilecar/backends/**`
- `src/mobilecar/planning/**`
- `src/mobilecar/control/**`
- `src/mobilecar/sensors/**`
- `tests/backend/**`
- `docs/workpackages/A/**`

职责：capture 状态与多段时间、deadline 采样、分段性能、base 外参、图像时刻 FK、离锁规划与版本复核、G1 功能回归。不得修改公共 contracts/app/config。

## B：三维界面可靠性

- `src/mobilecar/ui/**`
- `tests/ui/**`
- `docs/workpackages/B/**`

职责：渲染负载、自适应质量、键盘/滑块/TCP/失焦/断连可靠性、服务器 Chromium 证据。不得修改依赖锁和后端。

## C：数据质量与资格

- `src/mobilecar/data/**`
- `tools/data/**`
- `tests/data/**`
- `docs/workpackages/C/**`

职责：计划覆盖率、时间/对齐/时效、persist 观察、profile 资格、policy allowlist、旧记录兼容只读报告。不得修改公共 contracts/app。

## 服务器执行门

执行 agent 不得在本地运行 pytest/npm/MuJoCo/Playwright。回执必须写明本地只编辑、测试请求由主负责人通过带 `snapshot_id` 的服务器 job 执行；报告绑定 release hash、环境 lock、模型/配置 hash 和服务器身份。

## 冻结 profile

`configs/profiles/g2a_rgbd_5hz.yaml`：

- 原分辨率 640×480、5 Hz 名义 RGB-D。
- 60 s Episode；0-5 s 初始化和 55-60 s 收尾窗口保留并报告，不删除。
- 5-55 s 分析窗口计划 250 样本。
- 唯一持久 RGB-D 平均 wall 频率 ≥4.5 Hz，p99 相邻间隔 ≤0.4 s。
- 至少 225 个窗口内唯一且持久化 RGB-D；RGB/Depth/Mask 成对。
- 快流目标发布 10 Hz，实际值单列。

## 里程碑

S0 远端基线与 release hash -> S1 时钟/外参/离锁规划 -> S2 服务器渲染与页面可靠性 -> S3 质量/资格/traceability -> S4 静止 60 s、运动/臂 60 s、30 s 浏览器、故障反例和一次相关回归。

## 最终冻结哈希

| 文件 | SHA-256 |
|---|---|
| `docs/g2a_interface_v1.md` | `ba35308475536819be0c4c840e70a98345a304470dc24a4fad67b1b3eb0e54a4` |
| `configs/profiles/g2a_rgbd_5hz.yaml` | `9473171d7460ba968813b40ad1cae2f392bbc168e85ef4524eced6f38b5e3042` |
| `configs/profiles/g2a_action_teleop_v1.yaml` | `4b037ff671fb766452630b234d5680383381a2e6597b5855a5d92131a5c9f7dc` |
| `src/mobilecar/contracts/g2a.py` | `ee3a38e36eed5d9eb6e6bd213e69e5731481606e80bd2893a92135f815f3fac5` |
| `src/mobilecar/contracts/models.py` | `504dcf744ad1e9a764fb26e817a4c41c2b90ba11fe06490dddae9e23e1e44d40` |
| `src/mobilecar/contracts/protocols.py` | `b7d46b7ec6c5675f12d8d7fbb44229f601ca0c79047a11862f69ee42b99f01e1` |
| `docs/runtime_api_v1.md` | `237a5e78c6f13badc89914f70efa3a48d9e4a296d49e8e9093224d59f8b3e298` |

公共文件冻结后由主负责人集成适配。A/B/C 未产生有效代码交付，主负责人按规则接管；
详见 `G2A_agent_receipts.md`。
