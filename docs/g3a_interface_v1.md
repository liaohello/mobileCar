# G3A 导航接口与冻结 profile v1

日期：2026-10-17。状态：公共接口冻结；`interface_profile=g3a-1.0`，
wire `contract_version=mobilecar-1.0`。本文件不代表已通过服务器验收。

## 1. 身份、时钟与 Reset

所有导航消息携带 `run_id`、`episode_id`、`backend_generation`、
`navigation_epoch`、`control_tick_id`、capture 仿真时间与 backend monotonic
时间。时间戳取同一控制 tick 的 capture，不使用 publish、arrival、队列或 UI
收样时间。

Reset 生成新 Episode，并递增 `navigation_epoch`。新 epoch 必须清空估计器、
观测/命令队列、cmd_vel 租约和旧控制权限。旧 clock、旧命令或旧 scan 不得续接
新 Episode。时间回跳与 Reset 同等清理。

`backend_monotonic_s` 只允许在明确声明的同一 SSH/主机时钟域内解释。ROS
跨进程消息不把本地接收 monotonic 与 backend monotonic 隐式相减；ROS header
使用 capture sim time，接收端 watchdog 使用节点自身 steady clock。

## 2. 测量合同

`NavigationObservation` 同一 capture 绑定 wheel、IMU、arm joints 与可选 scan。
估计器白名单只含：

- 四轮关节角、轮速、固定半径和符号；
- IMU gyro/acceleration 的明确 validity、质量与坐标转换；
- capture sim/monotonic/tick、sample ID、epoch；
- 用于 health 的年龄和接收状态。

估计器禁止读取 `base_twist_body_m_s`、世界底盘 pose/qvel、Oracle 或动态
`world_sim` 真值。传感器的 MuJoCo 内部实现可使用 `MjData` 生成测量，这不等于
允许估计器消费特权状态。

IMU 采样失败时 `gyro_valid=false` 且 payload 为 `null`，不得补零。合法静止
gyro 为零则保留有限数值和 `valid=true`。`orientation_available=false` 始终
成立；G3A 不声称 IMU 提供姿态。

## 3. LaserScan

冻结 profile：

- frame：`laser_frame`；安装 body：`lidar_link`；
- 360 beams，`[-pi, pi - 1deg]`，不重复端点；
- 增量 `1deg`，range `[0.10, 8.00] m`，目标 10 Hz；
- 单 capture 瞬时扫描，`time_increment_s=0`；
- `T_base_laser=[0.07302117405202284, 0, 0.08] m` 表示实际水平扫描面，
  与静态配置/模型同源，方向在 laser frame 表达；
- 机器人真实碰撞 geom 保留，机械臂遮挡不能被过滤穿透。

每个 beam 状态为 `HIT`、`NO_RETURN`、`INVALID` 或 `SELF_OCCLUDED`。只有
`HIT` 携带有限 range；其余状态 range 必须为 `null`。ROS LaserScan 映射冻结为
`HIT -> finite`、`NO_RETURN -> +Inf`、`INVALID -> NaN`、
`SELF_OCCLUDED -> NaN`，同时发布状态计数诊断。无效/遮挡 beam 不得被当作清除射线。

## 4. 独立平面估计

`EstimatedBaseState` 来源固定为 `wheel_gyro_planar_fusion`。首版采用可解释的
平面互补融合：

- 轮速经已声明麦轮正运动学得到 base body `vx/vy/wz_wheel`；
- IMU gyro 绕 IMU z 轴转换到 base z，用于 yaw rate；
- 用显式配置系数融合 wheel yaw rate 与 gyro z；
- 一阶积分 pose，无逐 tick 世界 pose 纠偏；
- 输出 `odom -> base_link`，`twist` 在 base body frame；
- pose 与 twist 各输出 36 维 ROS covariance；零维度和开发参数明确标记，
  不把全零矩阵解释为已标定完美精度。

每条连续轨迹只允许一次显式零原点或评分初始对齐。静止、输入年龄、无效和
Reset 触发健康转换。`STALE`/`FAILED`/`STOPPED` 均携带显式原因；过期输入不重发
旧 pose 配新 stamp。

## 5. 原始分发

backend 采样侧向 Recorder、导航估计器与 ROS bridge 分发同一版本化原始观测，
每个消费者独立有界队列。队列满时每个消费者独立丢弃最旧项并记账；慢 ROS 或
写盘不能阻塞唯一物理线程，也不能替换 Recorder sink。

导航原始观测与估计不依赖 Recorder 是否处于 `RECORDING`。正式运动门仍可由
录制要求覆盖；关闭录制只应隐藏保存/正式资格，不应让调试导航测量凭空消失。

## 6. 控制来源

G3A 唯一可信导航测试来源为 `ros_navigation_test`。`NavigationControlMessage`
只在 IPC 边界接受 `cmd_vel/release/stop`，校验 run、Episode、backend generation、
navigation epoch、session、lease TTL 与有限速度。ROS 回调只替换受控命令，
不得写 qpos/ctrl 或调用 `mj_step`。

导航速度更新不取消独立 arm 轨迹。人工接管 base 会：

1. 撤销当前 ROS 导航权限；
2. 清零 base 命令并递增 `base cancel epoch`；
3. 保持 arm 所属控制组及目标；
4. 旧 cmd_vel 持续发布也不能自动恢复权限，必须显式取得新导航 session/epoch。

软件 Stop、录制门、命令过期、lease 过期和失效健康优先于导航输入。`release`
保持零底盘，不取消独立 arm；全局 Stop 按 G1/G2A 安全语义停止两个控制组。

## 7. ROS2 Humble 表面

最低话题/服务表面：

| 名称 | 类型/作用 |
|---|---|
| `/clock` | `rosgraph_msgs/Clock`，sim capture time |
| `/scan` | `sensor_msgs/LaserScan`，`laser_frame` |
| `/imu/data_raw` | `sensor_msgs/Imu`，`imu_link`，orientation unavailable |
| `/joint_states` | `sensor_msgs/JointState`，模型角与 URDF 名称 |
| `/odom` | `nav_msgs/Odometry`，`odom -> base_link` |
| `/tf` | odom/base、静态 base/laser/IMU、动态活动臂 |
| `/mobilecar/navigation_health` | 版本化 health JSON |
| `/mobilecar/cmd_vel_nav_test` | `geometry_msgs/Twist` 受控入口 |

G3A 不发布 `map -> odom`，不启动 SLAM、Nav2、到点规划或自动抓取。

backend 与 ROS bridge 之间的本地传输使用单客户端 Unix socket JSON-lines：

- 默认路径：`var/ipc/g3a_navigation.sock`，权限 `0600`；
- outbound：`NavigationStreamEnvelope`，每行一个 UTF-8 JSON；
- inbound：`NavigationControlMessage`，每行一个，最大 8 MiB；
- 每方向独立发送/接收线程与有界队列；无连接时 live 消息丢弃并计数；
- 重连后从下一 capture 开始，不重放断连前旧测量；
- bridge 只能连接一个，替换旧连接时不结束其他项目进程。

该路径属于服务器运行目录，不进入 release 源码包。

## 8. 健康与停止

正常目标：wheel/IMU/odom 平均至少 45 Hz，scan 平均至少 9 Hz。输入门为
wheel/IMU 最大 0.2 s、scan 最大 0.3 s。超过门必须得到 `STALE`/`FAILED` 和
显式 stop reason，底盘受控零速。

冻结开发门：

- 静止 30 s：平面漂移不超过 0.01 m，yaw 不超过 2 deg；
- 短程轨迹单次初始对齐后：终点平面误差不超过 0.05 m，yaw 误差不超过 5 deg；
- LaserScan/TF 与同 capture 几何参考独立检查，避免以里程计误差掩盖 ray frame 错误。

正式段默认不超过 60 s、一个 ideal seed。任何门失败保留原始结果，不静默放宽。
`world_sim` 只用于隔离评分和显式 `GT_DEBUG_ONLY`，不打成 odom/map。
