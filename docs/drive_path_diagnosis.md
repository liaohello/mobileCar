# 8792 底盘驾驶链排查与最小修复

日期：2026-10-19（UTC）。范围：仅 `http://127.0.0.1:8792/` 第三人称页面的实体键盘与屏幕 WASD/QE 驾驶。
未执行 mujoco_learning 的 E5D，未训练、未上传、未实车、未提交 Git。

## 1. 结论

底盘"按了没反应"不是单一按钮样式问题，而是两道既有的安全门在**浏览器里把输入丢掉、且驾驶区没有任何持久说明**：

| 顺序 | 门 | 触发状态 | 实测回执 | 结果 |
|---|---|---|---|---|
| 1 | 软件停止锁存 | `safety_latched=true`（旧进程自 2026-10-18T17:20Z 起一直锁存） | `set_base_twist` → `REJECTED · software stop is latched` | 浏览器在"输入事件→目标速度"就返回，连 HTTP 都不发 |
| 2 | 强制录制 | `recording.required_for_motion=true`，录制为 `COMPLETE/IDLE` | 解锁后再发 → `REJECTED · formal recording is required and command capture is not ready` | 解锁也仍然不动 |

**第一个断点在浏览器**：`pressDriveKey()` / `movement()` 见到 `safety_latched` 直接返回，只弹 3 秒 toast，按键仍显示按下态，读数仍是 `0.000`，所以看起来"键盘和屏幕键都坏了"。**第二个断点在后端**：录制门同样只以瞬时 toast 出现。两者叠加，用户无法从界面判断原因。

修复保留停止、录制、近场互斥与租约四类保护，只把"是否可驾驶、为什么不可驾驶、该点哪里"变成驾驶区里的持久状态，并把速度拆成**目标 / 受理 / 实际**三列。

## 2. 只读核对（修复前，线上 8792）

进程与 release：

- PID 3824559，cwd `releases/g4a2-20261018T171942Z-030bf47cef6f`，命令
  `python -m mobilecar --config configs/g4a2_server.yaml --host 127.0.0.1 --port 8792`。
- `/api/v1/health`：`status=ok`，`recording_status=COMPLETE`，`interface_profile=g4a1-1.0`。
- `/api/v1/capabilities`：`backend_kind=mujoco`，`base_linear_limit_m_s=0.05`，`base_yaw_limit_rad_s=0.30`。

状态面：

| 项 | 值 | 含义 |
|---|---|---|
| `safety_latched` / `base.stop_latched` | `true` | 软件停止锁存，底盘不可动 |
| `recording.status` | `COMPLETE` | 上一段录制已结束，`command_capture_ready=false` |
| `base_owner_session_id` / `arm_owner_session_id` | `null` / `null` | 无租约占用，不是"被别人抢走" |
| `active_session_id` | `null` | 无活动会话 |
| `/api/v1/g4a1/status` | `permission_active=false`，`nearfield_epoch=14` | 近场权限未激活，排除近场互斥 |
| `contact_summary.unexpected_contact` | `false` | 锁存不是接触违规引起，而是人工/结束录制路径 |

回执（命令均为零速，不产生位移）：

```text
POST /api/v1/commands set_base_twist(vx=0,vy=0,wz=0)
  → REJECTED  "software stop is latched"      tick_003905729
解锁（stop latch=false）→ DISPATCHED
再发 set_base_twist(0,0,0)
  → REJECTED  "formal recording is required and command capture is not ready"
位移 [9.4e-16, 1.8e-16, 0.0] m
```

原始记录：`docs/evidence/drive_gate_probe_8792.json`（部署后同一路径复测）、服务器
`var/reports/drive_diag_20261019/chain_probe_live.json`（修复前）。

## 3. 断点定位

链路与实测：

| 阶段 | 修复前 | 证据 |
|---|---|---|
| 输入事件 | 键盘 `keydown`、屏幕键 `pointerdown` 都到达 `pressDriveKey` | 按钮按下态会变化；浏览器内 `keys` 始终为空 |
| 目标速度 | **返回 null（断点）**：`!connected \|\| snapshot.safety_latched` | 线上 bundle 含同一守卫；`safety_latched=true` |
| HTTP 请求 | 未发出 | 命令计数不增长；后端无请求事件 |
| 后端接纳/拒绝 | 若绕过前端：`REJECTED software stop is latched` | 上面的回执 |
| 控制 tick | `_stop_latched` 时 `_base_request=0` | `tick_003905729` |
| 轮子指令/位移 | 零速、零位移 | `displacement [0,0,0]` |

解锁后第二道门把链路再次截断在"后端拒绝"，且拒绝原因只进 toast（驾驶命令以 `quiet:true` 发送）。

## 4. 最小修复

后端（只增可观测性，不改门控判定）：

- `backends/mujoco/backend.py`、`backends/mock/backend.py`：快照 `diagnostics.base_drive_gate`
  输出 `{blocked, code, reason, action, recording_required, owner_session_id, lease_remaining_s}`，
  顺序与 `_set_base_twist` 的三项检查一致（锁存 → 录制 → 近场），命令仍逐项复检。
- `app.py`：`/api/v1/health` 增加 `recording_required_for_motion`，让界面能如实说明服务端强制策略。

前端（`index.html` / `main.js` / `style.css`）：

- 驾驶区新增持久门控条：标题（如"软件停止已锁存"/"需要先开始录制"）、原因、当前录制状态，
  以及一键动作按钮（解除停止 / 开始录制 / 撤销近场）。
- 门控生效时 `[data-drive-key]` 变灰并 `aria-disabled=true`；按下被拦截时显示"已拦截 W 键"。
- 读数改为三列：**目标**（按键意图）、**受理**（后端最近接纳的 `requested_*`）、**实际**（轮子反馈）。
- 后端拒绝底盘指令时，驾驶区保留红色横幅 `后端拒绝底盘指令：<reason>`，并写入事件记录（同因去重，避免 150 ms 刷屏）。
- "未确认录制不允许运动"勾选框改为反映真实策略：服务端强制时勾选且禁用，并注明"该勾选只写元数据"的旧行为已不存在。

保护未改动：`STOP` 锁存、录制就绪门、`_g4a1_permission` 互斥、base 租约 0.5 s、接触违规锁存、失焦/断连释放。

## 5. 验证（全部在服务器执行，单物理实例）

- release `drvfix-20261019T044549Z-0a80415b2577`，aggregate `0a80415b...3d76369`，
  `app.js` `ada45e4d...3e16be0`，模型 `3ca3d6d0...`。
- 环境 `envs/g2a-py310-mujoco316`（Python 3.10 / MuJoCo 3.1.6），无头 Chromium 1243，
  配置 `configs/g4a2_drive_validation.yaml`（继承 `g4a2_server.yaml`，独立 runs/reports，端口 8795）。
- 输入为真实 Chromium 键盘/指针事件；`operator_kind=scripted`、无 human session，**不是**真人示教，也不算 H1 数据。

用例（快照位移/转角，`tests` 57 项全部通过）：

| 用例 | 按住 | Δx (m) | Δy (m) | Δyaw (rad) | 界面 目标/受理/实际 vx |
|---|---|---|---|---|---|
| W 前进 | 1.0 s | **+0.0508** | +0.0000 | +0.0000 | 0.050 / 0.050 / 0.078 |
| S 后退 | 0.75 s | **−0.0339** | +0.0000 | −0.0000 | −0.050 / −0.050 / −0.028 |
| A 左移 | 0.75 s | +0.0010 | **+0.0418** | −0.0596 | 0.000 / 0.000 / −0.002 |
| D 右移 | 0.75 s | −0.0012 | **−0.0334** | −0.0002 | 0.000 / 0.000 / −0.001 |
| Q 左转 | 0.75 s | −0.0003 | −0.0000 | **+0.1197** | 0.000 / 0.000 / −0.002 |
| E 右转 | 0.75 s | −0.0007 | +0.0001 | **−0.2392** | 0.000 / 0.000 / 0.001 |

轮子关节角同步变化（如 W 四轮均 +0.52 rad），证明"控制 tick→轮子指令"不是空转。

停车与失效：

| 场景 | 零速请求延迟 | 静止延迟 | 备注 |
|---|---|---|---|
| 松键 | 0.034 s | 0.412 s | 松键即发 `zero_base` |
| 失焦 blur | 0.029 s | 0.310 s | `keys=0`，不恢复旧输入 |
| 断连（关页） | — | — | 关页瞬间 −0.056 m/s，1.2 s 后速度≈0、`base_owner=null`、后续漂移 1.5e-6 m |

其它核查：

- 屏幕 W 按钮：+0.0500 m，按住速度 0.079 m/s。
- 锁存时按 W：`commandSeq` 82→82（未发 HTTP）、`keys=0`、横幅"已拦截 W 键 · 软件停止已锁存…"。
- 绕过门控直接 `sendCommand` 的拒绝在驾驶区留痕：`后端拒绝底盘指令：software stop is latched（REJECTED）`。
- 恢复路径（界面按钮驱动）：结束录制 → 门控"软件停止已锁存" → 点"解除软件停止" → "需要先开始录制" →
  点"开始录制" → `READY` → W 前进 +0.0256 m，新 Episode `episode_1ca0147a...`。
- 回归：`pytest -q --ignore=tests/bridge --ignore=tests/navigation` → **129 passed, 1 failed**。
  唯一失败为 `tests/integration/test_provenance.py` 的 `contacts.py` 历史 provenance 例外，
  在上一版 `g4a2-20261018T171942Z` 上同样失败且哈希相同（AGENTS.md 已记录），非本次引入。
  `tests/bridge`、`tests/navigation` 需 Humble overlay，本次未改 ROS 代码，未运行。

## 6. 部署后状态（线上 8792）

- PID 941842，cwd `releases/drvfix-20261019T044549Z-0a80415b2577`，
  配置 `configs/g4a2_server.yaml`，日志 `var/jobs/drive-fix-8792-restart/server.log`。
- 静态资产 served==release：`index.html 8d9d4769…`、`style.css 7b211a80…`、`app.js ada45e4d…`、`g4a2.*` 一致。
- `/api/v1/health`：`recording_required_for_motion=true`，`recording_status=IDLE`。
- 当前门控：`RECORDING_REQUIRED`（未锁存、未录制），驾驶区显示"需要先开始录制"+按钮，符合安全前提。
  要驾驶：点"开始录制"（或先录制再驾驶）；结束后点"解除软件停止"。
- 线上同一路径复测两道门控：`REJECTED software stop is latched` 与
  `REJECTED formal recording is required and command capture is not ready`，位移均为 0。

## 7. 复现

```bash
# 服务器、单实例
cd ~/Desktop/mobileCar/releases/drvfix-20261019T044549Z-0a80415b2577
export PATH=$HOME/.nvm/versions/node/v20.20.2/bin:$PATH
export ESBUILD_BINARY_PATH=$PWD/node_modules/esbuild/lib/downloaded-@esbuild-linux-x64-esbuild
npm run build:ui
../../envs/g2a-py310-mujoco316/bin/python -m pytest -q --ignore=tests/bridge --ignore=tests/navigation

MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONPATH=src \
  ../../envs/g2a-py310-mujoco316/bin/python -m mobilecar \
  --config configs/g4a2_drive_validation.yaml --host 127.0.0.1 --port 8795 &
../../envs/g2a-py310-mujoco316/bin/python tools/drive/validate_drive_chain.py \
  --base http://127.0.0.1:8795 --out ~/Desktop/mobileCar/var/reports/drive_fix/recheck.json
```

## 8. 证据

| 文件 | 内容 |
|---|---|
| `docs/evidence/drive_gate_probe_8792.json` | 部署后 8792 两道门控的原始回执与快照 |
| `docs/evidence/drive_validation_summary.json` | 57 项检查、六向用例、停车/断连/恢复流程摘要 |
| `docs/evidence/drive_validation_final.png` | 验证页面：驾驶就绪 + 目标/受理/实际 + 拒绝留痕 |
| `docs/evidence/drive_gate_live_8792.png` | 线上 8792 当前门控"需要先开始录制" |
| `docs/evidence/pytest-drive-fix.xml` | release 6 回归 JUnit（129 passed / 1 known failure） |
| 服务器 `var/reports/drive_fix/drive_validation_final.json` | 完整逐次快照、轮角、时间戳、控制 tick |

## 9. 边界

- 未执行 E5D，未修改 `mujoco_learning`，未训练/导出/上传/实车。
- 本页 8792 的运动仍受录制前置约束；这是原有策略，未被本次修复放宽。
- 只验证了该场景与 `0.05 m/s / 0.30 rad/s` 上限内的短时驾驶；不代表任意地图、任意速度或实车。
- 自动化用例不冒充真人示教，`training_qualified=false`。
