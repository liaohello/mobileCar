# G2A 观测质量与兼容接口 v1

日期：2026-10-17。wire ContractVersion：`mobilecar-1.0`；新增语义 profile：`g2a-1.0`。

## 兼容策略

G2A 不修改 G1 命令和身份字段的 wire 含义，旧 `mobilecar-1.0` Episode 仍可读取。新观测必须声明：

- `interface_profile=g2a-1.0`
- `profile_id=g2a-rgbd-5hz-original`
- `profile_version=1.0`
- profile YAML 的 SHA-256

旧记录缺少 capture/persist 时间时显示 `UNKNOWN`，不得反推、补造或把 G1 报告重标为 G2A。

数据资格还必须声明独立的 action profile：

- `action_profile_id=g2a-teleop-action-1.0`
- `action_profile_version=1.0`
- `configs/profiles/g2a_action_teleop_v1.yaml` 的 SHA-256

动作资格依据 requested/approved/dispatch/actuator_reference 关联与明确动作层，
不依据原始传感器是否包含 Depth、Lidar 或 IMU。静止观测质量通过但无动作记录时，
不得获得 D0/D2；RGB-only D2 由声明的 observation profile 和本体/动作门决定。

## 图像时间

每个 RGB-D capture 必须在同一物理快照上绑定：

1. `capture_sim_time_s`
2. `capture_backend_monotonic_s`
3. `capture_control_tick_id`
4. 同一快照的关节/夹爪反馈样本 ID 与时间
5. `render_start_backend_monotonic_s`
6. `render_complete_backend_monotonic_s`
7. `publish_backend_monotonic_s`
8. `arrival_backend_monotonic_s`
9. Recorder 可靠写入后的 `persist_backend_monotonic_s`

不同 clock domain 不做隐式相减。`measurement_time_s` 表示图像 capture 对应的 backend monotonic 时刻，`capture_sim_time_s` 单独保存。

## 外参方向

`T_parent_child` 把 child 坐标变换到 parent。对任意底盘世界位姿：

```text
T_base_link4 = inverse(T_world_base) @ T_world_link4
T_base_camera = inverse(T_world_base) @ T_world_camera
T_base_tcp = inverse(T_world_base) @ T_world_tcp
T_TCP_camera = inverse(T_base_tcp) @ T_base_camera
```

相机仍安装于 `link4` 分支；joint5 和开口不改变相机。TCP 固定于 `guide_rod_link`；开口不改变 TCP，q5 改变 TCP 与相机相对姿态。

## 规划离锁

TCP 预览/长路径检查：

1. 仅在短锁内复制不可变模型、状态和版本字段。
2. 在锁外求解、碰撞检查和时间参数化。
3. 回锁或通过版本比较复核 run/episode/generation、target revision、cancel epoch、模型/配置 hash、实际起始状态与 lease。
4. 状态已变化时丢弃结果并返回显式拒绝原因。

物理线程不得等待 IK、碰撞采样或渲染。

## PolicyObservation 白名单

允许：RGB/Depth/Mask 引用、时间/坐标身份、base body twist、五轴用户角、夹爪开口、传感器可用状态、来源与质量。

禁止：`world_sim` 物体/底盘真值、`diagnostics` 整对象、Oracle、map 伪估计、第三人称交互手柄状态。GT_DEBUG UI 数据保持显式标签，不作为策略输入。

## 质量报告

`ValidationReport` 可携带 `profile_id/version/sha256`、`timing` 和 `gates`。coverage 按冻结计划分别报告：

- planned
- generated
- delivered
- persisted
- unique
- skipped 与原因

旧 G1 的 D2 布尔值不自动升级。G2A 资格必须绑定 observation/action profile
ID、版本与 hash，并且只对声明窗口和输入内容有效。
