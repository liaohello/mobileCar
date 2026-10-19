# mobileCar G1 分层验收报告

日期：2026-10-17。合同：`mobilecar-1.0`。结论：G1 范围内完成。

## 1. 验收环境

- Python 3.10.12；MuJoCo 3.1.6；NumPy 2.2.4；FastAPI 0.136.3；Pydantic 2.13.5。
- Node.js 20.20.2；Three.js 0.180.0；Playwright 1.63.0。
- 唯一真实 MuJoCo 实例：1 ms 物理步长、10 ms 控制周期，本机 EGL 渲染。
- 旧来源仓只读 HEAD：`dcae2c81eb30c1bdb3b6d92a1116f1345d2a2fca`；选定文件逐项哈希见
  `provenance/source_manifest.json`。

实际关键命令与退出状态：

```text
$ npm run build:ui
exit 0

$ .venv/bin/python -m pytest tests/contracts tests/backend tests/data tests/integration tests/ui -q \
    --junitxml=docs/evidence/pytest-g1.xml
40 passed, 1 third-party deprecation warning
exit 0

$ MUJOCO_GL=egl .venv/bin/mobilecar --backend mujoco --no-realtime --print-bootstrap
mujoco landerpi_mecanum_dynamic_5dof_3ca3d6d0a92a ...
exit 0

$ .venv/bin/python scripts/collect_g1_evidence.py
success: COMPLETE / success / lift 0.0959706942 m / hold 8.65 s
failure: COMPLETE / failure / BROWSER_CLICK_STALL
exit 0

$ .venv/bin/python scripts/g1_acceptance.py --url http://127.0.0.1:8787
agent_ui real MuJoCo Episode recorded; recording COMPLETE
exit 0
```

另从 `/tmp` 使用已安装 `.venv` 实例运行 MuJoCo，并加入 Python audit hook 禁止任何
`mujoco_learning` 路径访问；后端加载成功且 forbidden access 计数为 0，exit 0。

## 2. 代码与合同证据

```text
$ .venv/bin/python -m pytest tests/contracts tests/backend tests/data tests/integration tests/ui -q
40 passed
```

覆盖：

- `mobilecar-1.0` 命令/快照/场景/传感器严格字段与样例 round-trip。
- 命令争用、重复/乱序、跨 Episode/代数、TTL、停止锁存、Reset、租约。
- 四麦轮前后/左右/双向旋转/合成输入/释放；五轴小幅正负与夹爪开合。
- 不可达、超出用户限位、终点合法但路径碰撞、执行中取消、手动接管、过期计划。
- 相机位于 `link4` 分支且不随 q5/开口变化；TCP 固定 `guide_rod_link` 并随 q5 变化。
- 规划副本不修改运行态；RGB/米制 Depth/掩码、激光、IMU、编码器、关节反馈。
- Mock 应用 API → Recorder → validator → 只读回放闭环。
- 实际 Chromium 中的三维视图、键盘、焦点隔离、TCP 预览/执行、录制与回放交互。

## 3. 真实 MuJoCo 动力学

定向测试确认自由底盘四方向、正反转向、合成 twist、释放减速、五轴有限力运动和夹爪有限力开合。
模型身份：

- 模型：`landerpi_mecanum_dynamic_5dof_3ca3d6d0a92a`
- 文件 SHA-256：`3ca3d6d0a92a48bbfbce5ac7a04e37a0f7cf7a73297045ae327bf461ad50e280`
- `Capabilities.model_sha256 == SceneManifest.model_sha256`

成功 Episode 中三个真实 TCP 目标均通过预览、同轨迹检查和执行：

| 目标用途 | 实测位置误差 | 姿态残差 |
|---|---:|---:|
| 方块上方 `(0.4, 0, 0.2)` | 0.231 mm | 0.00676 rad |
| 方块抓取位 `(0.4, 0, 0.127)` | 0.055 mm | 0.00161 rad |
| 抬升位 `z≈0.227` | 0.631 mm | 0.00487 rad |

全部低于 5 mm 首轮位置容差。

## 4. 浏览器与操作者来源

- 成功 Episode：`runs/run_b2228ee6630e4080ab6a7c813505dcd5/episodes/episode_4414a5ec8ae645d29d8e7df7826160cb`
- 异常 Episode：`runs/run_b2228ee6630e4080ab6a7c813505dcd5/episodes/episode_4cb1700ce2e04a8f9c5276b623ddb43f`
- 操作者：`agent_ui_acceptance`；`operator_kind=agent_ui`；`view_source=third_person_3d`。
- 该记录是开发 agent 经三维界面操作证据，明确不等同于 `human_teleop`，也不具备仅腕部 RGB VLA 资格。

同源浏览器实际打开三维场景、开始正式录制并显示接近/TCP/抓取/抬升状态；受控 `agent_ui`
通过版本化应用 API 完成连续动作，结束后在同一浏览器中只读回放。键盘、五轴/夹爪滑块、
TCP 数值/拖动与失焦停车由独立真实 Chromium 交互测试覆盖。截图见 [演示记录](demo_record.md)。

## 5. 数据完整性与隔离物理评分

成功 Episode：

- `audit_status=COMPLETE`，validator passed，0 errors，0 warnings。
- `task_result=success`；隔离 Oracle 最大抬升 `0.095971 m`，双侧有效接触连续保持 `8.650 s`。
- 最终数据资格：D0=true、D2=true、D3=true、D1=false、`wrist_rgb_only_eligible=false`。
- 11705 events、105 command records、11196 control records、2398 observations、87 transform records、
  261 referenced assets；87 组 RGB/Depth/Mask 严格成对，所有媒体引用与完整尾部校验通过。
- 实际频率：IMU/编码器/关节/激光约 9.56 Hz，RGB-D 约 1.488 Hz；实际频次按记录报告，不复制帧补齐。

异常 Episode：

- `audit_status=COMPLETE`，终止原因 `BROWSER_CLICK_STALL`。
- 隔离 Oracle `task_result=failure`，抬升 `0 m`，无双侧抓取保持。
- 证明异常记录、任务结果与审计状态分开。

证据 JSON：[g1_success_and_failure.json](evidence/g1_success_and_failure.json)。

开发期间的失败、诊断与修复见 [失败与修复记录](failures_and_fixes.md)。

## 6. 五类证据分列

| 类别 | 证据 | 结论 |
|---|---|---|
| 源码存在 | `src/`、`assets/`、`provenance/`、`docs/` | 已交付 |
| 合同/逻辑/几何测试 | 40 passed；JUnit 见 `docs/evidence/pytest-g1.xml` | 通过 |
| 真实动力学 | 后端定向测试 + 三个真实 TCP 目标 | 通过 |
| 浏览器交互 | Playwright UI 测试 + 实际 MuJoCo 截图/Episode | 通过 |
| 人类复核 | 本次仅 agent_ui，无正式 human_teleop | 未提供，不阻塞 G1 |

## 7. 未覆盖/不作声明

- 不包含 ROS2/SLAM、自主导航、Frontier、自动视觉抓取、自动全身协同、训练和实车运动。
- 不声明六维末端严格到位；姿态是软残差。
- 不声明实车电机力矩、线束边界、相机外参或通信/硬件急停已验证。
- 不声明 LeRobot 正式导出。
- 旧来源仓无顶层许可证文件，资产再分发许可仍需权利人确认。
