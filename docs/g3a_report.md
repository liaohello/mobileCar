# G3A 导航基础服务器验收报告

日期：2026-10-17。状态：**完成**。本报告的安装、构建、pytest、MuJoCo 和 ROS
运行均在服务器执行；本地只保留源码、配置、文档和约 136 KiB 精简证据。

## 1. 结论

G3A 的有限范围已通过实际服务器验收：

- 360-beam、10Hz、`laser_frame` 激光观测，唯一平面 wheel/gyro 里程计和健康；
- Recorder 与估计器、ROS bridge 独立有界分发，物理线程保持唯一 `mj_step` 所有者；
- ROS 2 Humble `/clock`、`/scan`、`/imu/data_raw`、`/joint_states`、`/odom`、
  `/tf`、`/tf_static`、health 和受控 `/cmd_vel` 实际运行；
- 导航来源速度与独立 arm 目标并行，人工接管撤销旧导航权限；
- wheel、IMU、scan 无效/过期与 ROS 输入超时均停车，恢复后不自动重放旧命令；
- Reset 新 Episode/epoch、原点和消息边界正确；
- 无 `target_cube` 的不对称诊断房间真实加载，Oracle 返回 `NOT_APPLICABLE`；
- G1/G2A/G2B 报告、原始 runs、数据集和环境未覆盖或重标。

不包含且未启动：SLAM、Nav2、建图、到点、Frontier、自动抓取、训练和实车。

## 2. 冻结身份

| 项目 | 值 |
|---|---|
| 最终 release | `g3a-20261017T194117Z-aa6a6e987115` |
| source aggregate | `aa6a6e9871159310cf3d2499847e181a7e962f7a8c552c9d5304461806848859` |
| 文件数 | 227 |
| interface profile | `g3a-1.0` / `g3a-nav-50hz-laser10-v1` |
| wire contract | `mobilecar-1.0` |
| navigation profile SHA-256 | `c9de2351cee65908ed646b434278fa9eee3f0c6b90910146815d46241c6e51e1` |
| navigation contract SHA-256 | `6aa93a726d2aa6ce7ba054e4e0c7fd721b067e5f4c33f8e95d0dc2818e527e13` |
| diagnostic model SHA-256 | `62bea46ce1468baf227a44e2ff71d509010e173337f1c875d2bd3c0399ab4241` |
| scene SHA-256 | `d2d63d7e94adbc36fb7e6163a0bb754fbc0fb51db6d949d7acb54164e5cc6427` |

## 3. 实现边界

### 观测与分发

`NavigationObservation` 在同一 capture 绑定 wheel、IMU、五轴模型角和可选 scan。
backend 控制循环为 100Hz，导航使用仿真时间 deadline 调度到 50Hz，scan 为 10Hz。
scan 为单 capture 瞬时 360-beam 环形视场，`[-pi, pi-1deg]`，不重复端点；
`time_increment_s=0`。

激光原点是 base 前 `0.07302117405202284m`、上方 `0.08m` 的实际水平扫描面。
每条射线同时考虑世界、机器人碰撞与视觉几何；机器人命中为 `SELF_OCCLUDED`，
无回波为 `NO_RETURN`，无效采样为 `INVALID`。只有 `HIT` 存有限 range。

Recorder、估计器、telemetry 和 ROS bridge 各自有独立有界队列。慢消费者丢最旧项并
计数，不阻塞物理；Recorder 未丢样本。正式 Episode 同时保留事件和控制生命周期。

### 估计与健康

估计来源固定 `wheel_gyro_planar_fusion`，采用可解释的平面互补融合，不称为 EKF。
轮序为 `front_left, rear_left, front_right, rear_right`，麦轮常数 `k=0.141m`；
gyro 显式转换到 base z。输出 `odom` pose 与 base body twist，pose/twist 各 36 维
开发 covariance。估计器只接收测量白名单，不读取世界底盘 pose、真实 twist 或 Oracle。

输入年龄门：wheel/IMU 0.2s，scan 0.3s。全无效 scan 不算可用 scan。失效时停止底盘，
保留独立 arm 控制组；恢复不自动恢复旧导航 session。

### 控制与 Reset

可信 ROS 来源固定 `ros_navigation_test`。`cmd_vel` 更新不取消 arm；人工 base 接管会
撤销导航 session、递增导航 cancel epoch 并保持 arm。软件 Stop、记录门、lease、
stale/身份不匹配和 health stop 优先。Reset 递增 `navigation_epoch`，清空估计、
reference ring、故障和旧权限。

## 4. 逐例结果

正式物理 job：`g3a-frozen-physical-1`。完整结果见
[g3a_acceptance_report.json](evidence/g3a_acceptance_report.json)。

| 案例 | 关键结果 | 判定 |
|---|---|---|
| 静止 30s | wheel/IMU/odom `48.32/48.32/48.32Hz`，scan `9.68Hz`；输入最大年龄 `0.0048/0.0048/0.0886s`；平面漂移 `0.000017m`，yaw `0.0156deg`，health HEALTHY | PASS |
| 平移/组合/臂/接管 | arm 与 base 独立运动；单次初始对齐终点平面误差 `0.00198m`、yaw 误差 `0.0646deg`；人工接管成功，旧 nav 命令未恢复 | PASS |
| 双向旋转/scan 几何 | 最终正 yaw `0.23753rad`；267 个 HIT 全部与独立 box raycast 符合，最大误差 `2.44e-15m`；89 SELF_OCCLUDED、4 NO_RETURN | PASS |
| 有限臂活动/遮挡 | 五轴位移实际发生；scan 持续 360 beams；87 个自遮挡方向；非 HIT 伪造 finite range 数为 0 | PASS |
| scan/IMU/wheel/接管 | scan -> STALE、IMU -> FAILED、wheel -> FAILED，均 `stop_required=true` 且停车；恢复后无需新命令仍保持停；旧 Episode 拒绝 | PASS |
| Reset | epoch `6 -> 7`，新 Episode/clock、odom 原点、旧命令拒绝 | PASS |

## 5. ROS 2 Humble 实测

正式 job：`g3a-frozen-ros-1`，`ROS_DOMAIN_ID=73`，
`rmw_fastrtps_cpp`。Probe 收到 clock 659、scan 44、IMU 218、joint 218、odom 218、
TF 440、health 220 条消息。ROS `/cmd_vel` 实际产生 `0.0169m` 平面位移。

- scan：`finite=11746`、`NaN=3916`、`+Inf=178`；
- TF 唯一表面：`odom->base_link`、`base_link->laser_frame/imu_link/arm_base`、
  `arm_base->link1->...->gripper_link`，并含 link4 相机链；
- 没有 `map->odom`；
- IMU orientation unavailable，`orientation_covariance[0]=-1`；
- cmd_vel 停发超过 0.25s 后 bridge 发送 zero release，health 记录
  `cmd_vel_input_timeout`。

完整证据见 [ROS 报告](evidence/g3a_ros_acceptance_report.json) 与
[ROS probe](evidence/g3a_ros_probe.json)。

## 6. 回归与数据兼容

最终同 release 回归：

- G1/G2 相关合同、distribution、estimation、backend、data、export 与 app/navigation
  integration：`88 passed`；
- Humble bridge 纯逻辑与消息映射：`3 passed`；
- 物理验收 JUnit：6 cases，0 failures。

没有重跑 G2A 浏览器/5Hz 长序列或 G2B LeRobot 导出/loader。G1 的
`provenance/source_manifest.json` 保持历史原值，因此其旧 hash 不匹配本轮主动修改的
源码是预期限制，不被误报为 G3A 回归通过。

## 7. 已知限制

- covariance 是明确标记的开发值，不是硬件标定精度。
- IMU 使用现有 MuJoCo gyro/accelerometer site，以 `imu_link` 暴露 identity 轴向；
  acceleration 只作诊断，不用于姿态或横移融合。
- 本轮只验证受控速度，不是 Nav2 到点，也不证明 SLAM 可用性。
- 一帧 scan 是瞬时仿真快照，不含真实 D500 逐束时延、噪声或标定。
- 非 Recorder 消费者在压力下允许有界丢旧消息；丢包计数保留，Recorder 为 0 丢包。
- G2A 质量 profile 仅适用于原 G2A 模型；诊断 G3A 房间不复用该资格计算。

## 8. 证据与停止边界

服务器完整 artifact、原始 run 和复算路径见
[server artifact index](evidence/g3a_server_artifact_index.json)。raw run 根目录为
`var/runs`，本任务两个 run 为 `run_3abe7c1ea1794407aa7b8707af030c4b` 和
`run_7e92d9a79dda4cdabfb9afc90dd9c863`。

启动和复现见 [G3A reproduction](g3a_reproduction.md)，故障与修复见
[failures and fixes](g3a_failures_and_fixes.md)，演示记录见
[demo record](g3a_demo_record.md)。

G3A 到此停止，不自动进入 G3B、SLAM/Nav2、训练或实车。
