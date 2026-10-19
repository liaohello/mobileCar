# G3B 建图、定位、目标与安全接口 v1

日期：2026-10-17。状态：公共合同冻结。`interface_profile=g3b-1.0`，
`profile_id=g3b-nav2-omni-map-v1`；G3A `g3a-1.0` 保持兼容，不回写其历史结果。

## 1. 模式与 TF

只允许两种模式：

- `MAPPING`：slam_toolbox mapping 是唯一 `map -> odom` 发布者；
- `LOCALIZATION`：slam_toolbox localization 是唯一 `map -> odom` 发布者。

任何模式都禁止 AMCL 或第二个定位节点同时发布同一变换。G3A bridge/estimator 继续是
唯一 `odom -> base_link` 来源；静态 base/sensor 和活动臂 TF 来源不变。无定位时不发布
伪 `map -> odom`。

## 2. 时钟与身份

`/clock` 只从有效 capture sim time 产生。缺失 capture time、同一
`run_id/episode_id/backend_generation/navigation_epoch` 内的时钟回退、或旧身份迟到消息，
都不能推进/回拨时钟或切换当前运行身份。

允许 Reset 切换新身份，但新身份必须伴随 backend monotonic 前进和新的 Episode；
旧 identity 的延迟 observation/estimate/health 不可把当前状态切回旧值。收到新身份后，
目标门先关闭、旧 Action 取消、SLAM/Nav2/costmap/TF 缓存和速度队列由系统 supervisor
清理或重启，之后才可初始化定位和接受新目标。

## 3. 地图包

`MapPackageManifest` 至少包含：

- `map_id`、scene/model、G3B/navigation/sensor/scan profile hash；
- 分辨率、原点、occupied/free threshold、map/odom/base frame；
- 建图 run/Episode/epoch、可选一次显式初始 prior；
- 占据 YAML、占据图像、序列化 pose graph 及数据、manifest、RViz 配置；
- 每文件相对路径、SHA-256 与大小。

只保存 PNG/YAML 不构成可重启定位地图。地图兼容检查按冻结字段执行，整个源码 aggregate
不是地图兼容字段。scene、model、scan/sensor、frame 或坐标约定不兼容时明确拒绝，
不能自动改用附近地图或场景真值重建路线。

Humble 地图固定为 `mode=trinary`、`occupied_thresh=0.65`、`free_thresh=0.196`。
`205` 未知像素不得被解释为自由空间。supervisor 启动时校验 manifest 资产哈希、
兼容字段和初始先验格；每个目标在授权前还要通过保存地图的 occupied/unknown/
outside-map 预检。地图 ID 或兼容字段不匹配时，在 backend 授权前拒绝。

## 4. 目标与授权

`G3BControlMessage` 绑定：

`goal_id`、`goal_generation`、`map_id`、`run_id`、`episode_id`、
`backend_generation`、`navigation_epoch`、`session_id`、lease 和有限速度。

一个新目标只有完成以下步骤才可向 Nav2 发送内部 Action：

1. 地图兼容、定位 READY、scan/odom/Nav2 健康；
2. supervisor 生成新 goal generation 与唯一 authorization session；
3. backend 对 `authorize` 返回已接纳 receipt；
4. 旧目标达到终止状态并经过冻结 barrier。

取消、人工接管、健康失效或 Reset 首先关闭最终速度门并撤权/清零，再异步取消 Nav2
Action。旧 session、旧 generation、旧 Action 结果和重连队列不能绑定新目标。新目标必须
重新授权。raw Nav2 Twist 没有目标代数，因此 supervisor 使用终止屏障和内部门控，在
最终发送点绑定当前已授权 goal，禁止给迟到 Twist 追写新 goal ID。

## 5. 真实接纳回执

Unix socket `sendall` 成功只算传输成功，不算运动可执行。每个
`G3BControlMessage.request_id` 必须收到对应 `ControlReceipt`：

- `accepted=true` 只在 backend 返回 `DISPATCHED/APPLIED/RECEIVED`；
- 拒绝原因、backend status、control tick、sim/monotonic time 必须可关联；
- 正式验收至少包含一次控制门或 backend 的真实拒绝；
- 连续授权请求被拒绝时 supervisor 关闭速度门、取消 Action并停车。

控制周期为 10Hz。关门时允许最多 2 条已在管线、其变化不超过两个控制周期的 backend
dispatch 回执完成；随后必须有 governed 零速、撤权及物理停稳证据。超过该有界值，
或在零速之后仍有接纳回执，判失败。

## 6. 最终速度门

Nav2 controller、behavior/recovery、可选 smoother 和所有其他速度输出统一发布到
`/mobilecar/nav2/cmd_vel_raw`，只有 goal supervisor 的 governor 可向
`/mobilecar/nav2/cmd_vel_governed` 发布最终速度，并由 bridge 转成带
`nav2_goal` 身份的 IPC 控制。任何组件不得直连 G3A 测试速度入口或 backend。

门打开条件全部成立：

- 当前目标和 generation 已授权，backend authorization receipt 已接纳；
- 当前 map/运行/epoch/session 与 control 完全一致；
- 速度和加速度有限且在 G3B profile 内；
- scan/odom/localization/Nav2 健康，无 EStop、recording gate 或 lease 失效。

门关闭时发布零速度并撤权。崩溃/断连时 backend lease 和 bridge watchdog 作为最终后备。

## 7. 定位健康

定位 READY 不能只看 `map -> odom` 是否存在或持续发布。冻结监督信号：

- slam_toolbox 节点存在与存活；
- recent successful scan-match pose `/pose`；
- `map -> odom` 新鲜度；
- 原始 scan/odom 新鲜度；
- 当前运行/地图 identity。

运动和准备再次运动时要求最近 pose match ≤0.5s；静止允许最长 2.0s 显式
`STATIONARY_HOLD`，再次运动前恢复 ≤0.5s。节点或 match 失效产生 LOST/STALE/FAILED，
即使旧 TF 仍存在也必须停车。该检查只证明定位输出/匹配失效，不声称证明所有几何退化
或未知起点全局重定位。

开发验收可在显式 `enable_test_faults=true` 下通过
`/mobilecar/localization/fault_match_failure` 注入“scan/odom 和节点仍新鲜、匹配失败”
故障。该开关不是生产默认值；结果必须注明其为定位输出/匹配失效注入。

## 8. 场景、足迹和导航

- 两个短场景：G3A 不对称房间和宽门洞/L 形通路；
- 唯一固定保守 footprint：`(-0.16,-0.18),(0.46,-0.18),(0.46,0.18),(-0.16,0.18)m`，
  加 0.03m padding；全局/局部一致；
- DWB 必须启用实际 obstacle-footprint 碰撞检查，不能只检查中心点；
- 规划/平滑/最终门统一限制 `|vxy|≤0.05m/s`、`|wz|≤0.30rad/s`；
- 显式支持并实测 vy，不用绕行冒充横移；
- 目标来自已保存地图/人工标注；scene truth 只进隔离评分；
- 障碍、未知、地图外和无路目标明确拒绝/失败，不自动替换目标。

Nav2 内部 goal checker 的航向终止值为 `0.10rad`（约 `5.73°`），为估计误差保留
外部 `10°` 门余量；progress checker 使用 Humble `PoseProgressChecker`，平移
`0.05m` 或旋转 `0.10rad` 均计为进度，窗口 `25s`。该组合在 A02/A03/A04 开发失败
后明确验证，外部位置/朝向/停稳门未改变。

## 9. 到达与失效门

到达要求独立记录：

- Nav2 Action 成功；
- 隔离物理位置≤0.10m、yaw≤10°；
- 线速度≤0.01m/s、yaw rate≤0.03rad/s，持续≥0.6s；
- 无非预期机器人/臂-环境碰撞；轮地接触不失败。

健康检测到最终零命令目标≤0.15s，检测后物理停稳≤1.0s。报告注入→检测→零命令→
物理停稳分段耗时，不能把命令零速当实际停止。所有参数在正式 10 例前冻结，失败不静默
放宽。

## 10. Goal 范围

本接口覆盖 MAPPING、LOCALIZATION、显式一次初始 prior、Nav2 到点、取消、接管、
有限臂活动、scan/定位失效和完整 Reset。不包含 Frontier、未知目标搜索、自动抓取、
视觉 SLAM、VLN/VLA、训练和实车。

服务器 RViz 使用 `nav2_rviz_plugins/GoalTool`；该插件直接调用顶层
`/navigate_to_pose` Action，由 supervisor 统一授权。`/goal_pose` PoseStamped 中继
作为备用标准工具入口。GUI 证据包含鼠标选工具、点击/拖动朝向、实际 goal pose、
Action 结果、物理到达和截图。
