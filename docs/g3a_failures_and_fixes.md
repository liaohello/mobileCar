# G3A 失败与修复记录

日期：2026-10-17。以下均为实际服务器 job，不覆盖失败结果。

## 1. 小型 agent 委派无产物

- 首个小 fixture agent 只复述约束并结束，没有创建文件。
- 按约定记录并接管，生成
  `tests/fixtures/g3a/laser_self_occlusion_v1.py`，验证后才继续。
- 后续 A/B/C 新会话同样停在初始化或只返回通用就绪状态，没有源码交付。
- 主负责人接管后端、估计和 ROS，不把接管实现伪称为独立 agent 交付。

## 2. 测试 fixture 终止角错误

- `job g3a-core-1`：4-beam fixture 的角增量 0.1 却把最后一个 beam 角度写成 0.1。
- 合同正确要求 `angle_max=0.3`。修正 fixture，不改变合同。

## 3. 有界队列测试断言错误

- `job g3a-core-1/2`：容量 2 的快速生产测试要求零丢弃。
- 有界 drop-oldest 的冻结语义允许该测试发生丢弃。修正测试，验证顺序与账目，
  不放宽实际 profile 的 Recorder 容量。

## 4. 旧 lidar helper 尾部残留

- `job g3a-backend-1`：维护旧 G2A lidar 时旧 `_sensor_vectors` 尾部被误嵌，
  出现 `result/name is not defined`。
- 删除残留；`job g3a-backend-2` 16 项既有后端测试全部通过。

## 5. AF_UNIX 路径过长

- `job g3a-bootstrap-1`：socket 配置含 `../../`，超过 AF_UNIX 限长。
- 将应用 socket 路径解析为服务器项目绝对路径；不改变 release 内源码组织。
- `job g3a-bootstrap-2` 成功启动诊断模型、无 target_cube，并完成关闭。

## 6. 观测频率与唯一失败

- 初始 100Hz tick 每 tick 发导航，实际约 93Hz，估计队列积压，scan 被判 stale。
- 第一次节流使用固定每 2 tick；在重射线帧使控制循环约 88Hz 时，结果只有约 44Hz。
- 始终保留 45Hz 门；改为基于仿真时间 20ms/100ms deadline 调度。
- 还发现估计线程每帧同步等待物理锁；健康状态改为快速锁，失效时才进入物理锁。
- 最终静止：wheel/IMU/odom 48.32Hz，scan 9.68Hz；不是降低验收门。

## 7. scan 原点与自遮挡

- 首个物理 job 中激光原点位于 lidar 外壳内部，引入视觉 mesh 后 360 beam 全被标为
  自遮挡，无法测量房间。
- 将 `T_base_laser` 修正为实际水平扫描面高度 `z=0.08m`；保留机器人碰撞/视觉几何，
  不通过过滤机器人制造穿透。
- 最终 scan 为 267 HIT、89 SELF_OCCLUDED、4 NO_RETURN，独立几何 HIT 全部匹配。

## 8. 独立几何参考为空

- `SceneManifest` 当前没有把 MuJoCo contype 暴露成 `collision` 布尔，验收器错误过滤
  所有 box。
- 改为使用已与机器人隔离的 `world_links` 中静态 box，独立 OBB raycast；该段不调用
  backend scan 函数，也不使用估计 pose 掩盖 scan 错误。

## 9. G2A 质量 profile 错配

- G3A 诊断模型 hash 与 G2A profile 的 literal ID 不同，app 每次停止 Episode 后错误
  尝试生成 G2A 质量报告。
- G3A server profile 明确 `g2a_quality_enabled=false`；仍保留 Episode、Oracle、
  Eligibility 和 G3A 自身验收，不把不适用的 G2A 资格重标为通过。

## 10. 人工命令 TTL 边界

- 高负载下验收进程从取 snapshot 到 POST 手工接管偶尔超过原 0.5s，返回
  `EXPIRED_BEFORE_DISPATCH`。
- 验收的人工作业请求使用 2s TTL；这是命令构造时限，不改变 backend 执行、lease、
  stale 和 Stop 门。最终接管成功，旧 nav session 仍被拒绝。

## 11. ROS QoS 与静态 TF

- 首次 ROS probe 使用 reliable QoS，正确被 best-effort sensor publisher 拒绝。
- probe 改为 sensor best-effort；`/tf_static` 改为 transient-local reliable。
- 不把发布端改成 unreliable/reliable 混用以迎合错误消费者。

## 12. ROS probe JSON 类型

- ROS 数值数组中出现 NumPy scalar，标准 `json.dumps` 拒绝 `numpy.bool_` 类对象。
- probe 在写文件前递归转为原生 JSON 类型；验收门数值不变。

## 13. 旋转案例未达方向门

- 一次双向转动净 yaw 仅约 3.3deg，低于冻结的 >4deg 方向检查。
- 增加两方向实际转动幅值/时长后复验到 0.2375rad；未降低 yaw 误差或
  scan 几何门。

## 14. 证据索引后处理错误

- 首次索引脚本将 run/identity 集合组合写错，报 `unhashable type: dict`。
- 该错误只发生在验收后的索引步骤；修复脚本，对同一冻结 release 的原始报告重算
  hash/index，不重跑或重标物理/ROS 结果。

## 未解决但明确保留的限制

- covariance 未标定；IMU acceleration 不用于融合。
- 一次 scan 为瞬时理想时序，不是真实 D500 时序/噪声。
- 非 Recorder 导航消费者允许有界 drop-oldest；计数进入 telemetry/health 证据。
- 没有 SLAM/Nav2 到点、训练或实车资格声明。
