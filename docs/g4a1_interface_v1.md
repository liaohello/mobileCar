# G4A-1 近场观测、权限与六维动作接口 v1

日期：2026-10-18。状态：**G4A-1 有限验收通过，接口冻结**。wire 兼容基线仍为 `mobilecar-1.0`；
本阶段新增语义 profile `g4a1-1.0`，不改变 G1–G3B 历史字段含义。

## 1. 任务与权限

- 场景：固定底盘、单个 20 mm 方块、接近/闭爪/抬升/保持。
- 停车来源固定为 `park_source=prepared_pose`；不代表 Nav2 到近场交接已验收。
- 相机固定 `link4`；joint5 旋转夹爪，独立驱动两指开合。禁止改挂 TCP。
- 底盘保持自由动力学。初始 base/cube 位姿只在 Episode 开始前设置，运行中禁止用
  qpos、真值或 Oracle 搬动底盘/物体。

`NearfieldPermissionV1` 至少绑定：

- `run_id`、`episode_id`、`backend_generation`、`nearfield_epoch`；
- `permission_id`、`session_id`、操作来源；
- observation/action profile ID、version、SHA-256；
- 请求、批准和过期 backend-monotonic 时刻；
- 允许阶段、取消代数与唯一 arm owner。

只有底盘停止门成立、无活动导航授权且录制 gate 就绪时才可授予。权限活动时，任何
非零底盘请求必须在 backend 拒绝。停止、取消、接管、测量失效或 Reset 立即撤销权限、
停止新轨迹并保持最后安全关节/夹爪目标；不得自动张开夹爪。

## 2. 受限观测

`G4A1ObservationV1` 只允许：

- 当前 run/Episode/generation/epoch、sample/stream/capture tick；
- link4 RGB、米制 Depth、Depth 有效 mask 的受控 URI 和 SHA-256；
- capture sim/backend-monotonic、render、publish、consumer arrival；
- 同 capture 五轴实测用户角、夹爪实测宽度与 FK 引用；
- 相机内参、`T_base_camera`、`T_base_link4`、`T_base_tcp`；
- profile/模型/scene/标定身份和有效性摘要。

明确排除：`world_sim` 物体/底盘位姿真值、Oracle 接触、第三人称操作状态、实例分割
真值、完整 diagnostics、来自场景几何生成的路线或目标中心。第三人称画布仅可作为
显式 `GT_DEBUG` 调试入口，其 Episode 不能充当受限视图示范。

图像/反馈年龄沿用 G2A 在线语义。正式门暂冻结为：

- 图像 arrival 到 decision ≤ `0.4 s`；
- proprio 到 decision ≤ `0.2 s`；
- 图像与 proprio capture skew ≤ `0.2 s`；
- RGB、Depth、valid mask 必须同 sample/capture；Depth 单位米，无效值为 NaN。

## 3. 六维动作

冻结 action profile：`g4a1-sixdim-absolute-goal-v1`。

```text
action_t = [
  q1_target_user_rad, q2_target_user_rad, q3_target_user_rad,
  q4_target_user_rad, q5_target_user_rad, gripper_width_target_m
]
```

- 五点角度为绝对 user 角；夹爪宽度为绝对米制开口。
- joint order 固定 `joint1..joint5`，wire 单位为 rad/m。
- 不是实测反馈、TCP 宏目标或 G2B 历史 actuator_reference。
- 决策目标频率 5 Hz；低层继续使用现有有限力、速度/加速度限幅和 PD 执行。
- arm 与 gripper 必须同 `action_id`、observation、权限和 tick 一致应用或整体拒绝。

`G4A1ActionV1` 必须携带 action frame ID、seq、`observation_sample_id`、decision/issue/
expiry backend-monotonic 时刻、permission/session、profile hash、来源和六维目标。
backend 拒绝声明不匹配、维度不符、非有限、越界、过期、旧 seq、旧 Episode/generation/
epoch/permission 或 observation 尚未 arrival 的帧。

处置策略冻结为：拒绝旧帧，停止新轨迹并在有限力下保持最后安全关节/夹爪目标；无新
命令不等于松爪。显式新权限和新的安全目标才可继续。

## 4. 因果链

每次动作需能反查：

```text
obs capture
  -> consumer arrival
  -> decision
  -> requested_action
  -> approved_target
  -> dispatch
  -> actual control tick / actuator reference
  -> measured feedback
```

记录 `requested_action`、`approved_target`、`dispatch`、`actuator_reference` 和反馈为
不同层。写入时间只证明持久化，不等于在线 arrival 或决策时刻。

## 5. 阶段与来源

阶段固定为 `OBSERVE / READY / APPROACH / CLOSE / LIFT / HOLD / DONE / ABORTED`。
阶段标签不替代物理成功评分。

来源保持：

- 真人实际操作：`human_teleop`；
- agent 经真实受限界面操作：`agent_ui`；
- 按声明输入运行的代码控制器：`scripted`；
- 自动化故障/旧帧探针：`test_injection`。

本 Goal 的开发数据全部 `purpose=dev`、`training_qualified=false`。`agent_ui`、
`scripted` 或 `test_injection` 不升级为人类示教。

## 6. 文件所有权

| 所有者 | 独占路径 |
|---|---|
| 主负责人 | `src/mobilecar/contracts/**`、`src/mobilecar/app.py`、`src/mobilecar/config.py`、`src/mobilecar/backends/**`、`src/mobilecar/ui/routes.py`、`src/mobilecar/ui/static/index.html`、`src/mobilecar/ui/web/main.js`、`src/mobilecar/ui/web/api.js`、`configs/g4a1*.yaml`、`configs/profiles/g4a1_*`、`configs/g4a1/**`、`tests/contracts/**`、`tests/integration/**`、`tools/g4a1/run_formal.py`、最终报告/证据 |
| A 视图/几何 | `src/mobilecar/nearfield/observation.py`、`src/mobilecar/ui/web/g4a1-view.js`、`tests/g4a1/test_observation.py`、`docs/workpackages/A_G4A1/**` |
| B 动作适配 | `src/mobilecar/nearfield/action.py`、`tests/g4a1/test_action_adapter.py`、`docs/workpackages/B_G4A1/**` |
| C 数据/验收 | `src/mobilecar/data/g4a1_readiness.py`、`tools/g4a1/score_case.py`、`tests/g4a1/test_readiness.py`、`tests/g4a1/fixtures/**`、`docs/workpackages/C_G4A1/**` |

A/B/C 不得修改 backend、app、公共 contracts、公共 UI 装配、profile 或启动第二个
物理实例。公共文件只能由主负责人集成。

## 7. 完成边界

六例及全部子门必须通过后 G4A-1 才可 complete。正式 LeRobot 导出、编码、loader、
Hub、训练、ACT/VLA、自动目标搜索、联合移动抓取和实车均不在本接口范围。
