# mobileCar G1 运行时 API v1

状态：M0 冻结。ContractVersion：`mobilecar-1.0`。

本文件冻结主负责人装配的浏览器与数据工具边界。A/B/C 的包内私有接口可以演进，但不得绕过这里定义的公共路径和消息。字段调整仍由主负责人在 `src/mobilecar/contracts/` 统一维护。

## 1. 应用端点

| 方法 | 路径 | 请求/响应 | 所有权 |
|---|---|---|---|
| GET | `/api/v1/health` | `{"status":"ok","run_id":"...","backend_generation":1}` | 主负责人 |
| GET | `/api/v1/capabilities` | `Capabilities` | A 提供，主负责人暴露 |
| GET | `/api/v1/scene` | `SceneManifest` | A 提供同源模型，主负责人暴露 |
| GET | `/api/v1/scene/assets/{asset_id}` | 不可变网格字节；响应带 SHA-256 | A 资产，主负责人暴露 |
| GET | `/api/v1/snapshot` | `RobotSnapshot` | A 状态，主负责人叠加 C 的录制状态 |
| POST | `/api/v1/commands` | `CommandRequest` -> `CommandResponse` | 主负责人调用 A |
| POST | `/api/v1/lease/renew` | `{"session_id":"..."}` -> `{accepted,session_id}` | 仅续接收端租约，不占用动作序号 |
| POST | `/api/v1/episodes/start` | `EpisodeStartRequest` -> `EpisodeControlResponse` | 主负责人调用 C |
| POST | `/api/v1/episodes/stop` | `{"reason":"..."}` -> `EpisodeControlResponse` | 主负责人协调停车与 C |
| POST | `/api/v1/episodes/reset` | `CommandRequest(command_type=reset)` -> `CommandResponse` | 主负责人调用 A |
| GET | `/api/v1/episodes` | 只读 Episode 索引 | C 提供，主负责人暴露 |
| GET | `/api/v1/episodes/{episode_id}` | 只读元数据、事件和观测索引 | C 提供，主负责人暴露 |
| GET | `/api/v1/episodes/{episode_id}/assets/{asset_path}` | 原始只读资产 | C 提供，主负责人暴露 |
| POST | `/api/v1/episodes/{episode_id}/validate` | `ValidationReport` | C |
| GET | `/api/v1/episodes/{episode_id}/assessment` | `TaskAssessment` 或未生成状态 | C/Oracle |
| GET | `/api/v1/episodes/{episode_id}/eligibility` | `DataEligibility` | C |
| GET | `/api/v1/episodes/{episode_id}/g2a-quality` | `G2AQualityReport`；profile/时钟/间隔/延迟/gate 只读报告 | 主负责人集成 C |
| GET | `/api/v1/episodes/{episode_id}/g2a-eligibility` | profile 绑定的 G2A `DataEligibility`；不覆盖 G1 资格文件 | 主负责人集成 C |

浏览器轮询 `/api/v1/snapshot`，目标刷新率来自配置，但 UI 快照不冒充控制反馈。回放端点只读，绝不调用 `submit`、`reset`、`cancel`、`stop` 或物理推进。

Episode 完成后，G2A 报告写入配置的 `quality.report_root/g2a/<run_id>/`：
`<episode_id>.g2a_quality.json` 和 `<episode_id>.g2a_eligibility.json`。这些文件
分别绑定输入内容 hash 和 G2A profile hash；G1 的
`assessments/eligibility.json` 保持原结论，D2 历史布尔值不自动升级。

## 2. 场景描述

`SceneManifest` 由 A 从实际编译的 MuJoCo 模型导出，包含：

- 世界静态 body/geom 和机器人动态 body/geom 的局部 `parent_from_child` 变换；
- 以 xyzw 表示的合法四元数；
- 每个 link 的 joint 名称、类型和轴；
- 网格资产 ID、URI 和 SHA-256；
- `camera_ids` 与 `tcp_site`。

`SceneGeom.size_m` 对 box 使用半边长，对 sphere 使用半径，对 cylinder/capsule/ellipsoid 使用 MuJoCo 的半尺寸语义；对 mesh，若非空则依次表示模型 `mesh_scale_xyz`。该约定避免 UI 自己猜测单位。

B 可据此直接用 Three.js 绘制，不读取 MuJoCo `MjData`。同一 `model_sha256` 必须同时出现在 `Capabilities` 与 `SceneManifest` 中。界面状态到 link 变换的映射只使用公开关节/夹爪字段。

## 3. 录制生命周期

1. `POST /api/v1/episodes/start` 由主负责人取得当前不可变快照后交给 C。
2. C 只有在 manifest、Episode 起点和初始反馈可靠落盘后才返回 `accepted=true`，并把状态置为 `RECORDING`。
3. 若请求的 `require_recording=true`，A 的后端运动门在 `recording_gate(episode_id)` 为真前拒绝运动。
4. `POST /api/v1/episodes/stop` 先拒绝新运动，再由主负责人触发 A 的有界停车/保持，继续收集停车反馈，确认停稳或超时后交给 C 排空。
5. UI 显示的 `RecordingState` 来自 C 的确认状态，不由浏览器自行推断。

## 4. 命令与事件

- 所有运动只通过 `POST /api/v1/commands` 的 `CommandRequest`，回复的 `CommandResponse` 只用合同状态值。
- A 按 `requested -> approved/rejected -> dispatch/expired -> actuator_reference -> feedback` 发布 `EventRecord` 和 `SensorObservation`。
- C 不得为缺失的控制事件填零或从反馈反推动作标签。
- 拒绝与过期请求同样保存，并通过 `command_id`、`command_seq`、`control_tick_id` 关联。
- UI 不得直接写 MuJoCo 状态、启动物理线程或把回放请求转为动作。

## 5. 共享样例

- `configs/examples/command_set_base_twist.json`
- `configs/examples/capabilities_fixture.json`
- `configs/examples/snapshot_fixture.json`
- `configs/examples/scene_fixture.json`

这些文件仅证明接口可解析，不代表物理或验收结果。A/B/C 的单元测试必须使用同一版本样例或由这些样例派生的明确标注合成夹具。
