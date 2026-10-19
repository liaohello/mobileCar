# G3A之后的实施计划：G3B建图、重载定位与到点

日期：2026-10-17；状态于2026-10-18更新：**G3B已完成，本文件保留历史规划依据**。原规划回合仅编辑文档；实际结果见[G3B报告](g3b_report.md)。当前计划见[后G3B计划](post_g3b_execution_plan.md)，不再按下文重开G3B。

完整执行规范见[服务器执行政策](server_execution_policy.md)，可交给agent的任务正文见[G3B Goal](goals/G3B_mapping_navigation.md)。

## 1. 基线与下一步判断

G1、G2A、G2B、G3A均已完成。G3A冻结身份：

- release：g3a-20261017T194117Z-aa6a6e987115；
- aggregate：aa6a6e9871159310cf3d2499847e181a7e962f7a8c552c9d5304461806848859；
- 接口：g3a-1.0 / g3a-nav-50hz-laser10-v1，360 beams、10Hz瞬时scan；
- 服务器结果：核心88 passed、bridge 3 passed、物理6/6，另有ROS Humble端到端证据。

依据：[G3A报告](g3a_report.md)、[接口](g3a_interface_v1.md)、[复现](g3a_reproduction.md)、[证据索引](evidence/g3a_server_artifact_index.json)。这些是已有结果，本轮未复跑。

下一步执行G3B：让车辆在两种简单室内环境中，通过实际激光建图，保存地图及位姿图，重启加载后定位，并到达人工指定的位置和朝向；取消、人工接管、传感器或定位失效时正确停车。

G3A的小漂移是在ideal仿真和短轨迹下取得，不能解释为真实硬件定位精度，也不能替代G3B的实际SLAM和路径跟踪验证。

## 2. 实施顺序与可并行部分

| 工作包 | 具体结果 | 依赖与并行 |
|---|---|---|
| P0 公共接口和冻结配置 | 地图包、目标/权限代数、健康、速度链、case合同；明确文件所有者 | 主负责人先完成，启动一次小型委派验证 |
| P1 建图与重载定位 | MAPPING/LOCALIZATION两模式；占据地图+位姿图；唯一map→odom；定位健康 | A可与P2配置、P3场景/评分开发并行 |
| P2 Nav2与目标控制 | 全向规划/控制、目标生命周期、取消/接管、新目标重新授权、服务器RViz | B消费P0合同；公共backend/bridge改动由主负责人集成 |
| P3 场景与有限验收 | 复用房间、增加宽门洞/L形通路；臂包络；隔离评分和证据 | C可并行写资产、case和评分，不能另起物理实例 |
| P4 单场景串通 | 观测→SLAM→Nav2→受控bridge→实际底盘；GUI设点 | P1/P2/P3集成后顺序执行 |
| P5 两场景有限验收 | 正式10例、拒绝/接口子检查、同release受影响回归与交付 | 一个服务器物理队列，结果绑定最终release |

并行的是源码实现、参数设计和证据工具；MuJoCo运行、建图和正式导航验收统一串行。服务器多GPU不改变单物理实例限制。

G3A委派没有产出有效代码，本轮执行不能直接宣称“三agent并行完成”。先给一个小任务，要求提交实际文件、内容摘要、未执行项；主负责人核对文件后才扩大并行。若仍无产物，记录并接管，避免反复创建空会话。

## 3. G3B真正需要补齐的衔接

以下为本地静态阅读发现的集成需求，不是新的服务器失败结论。

1. **一个目标对应一次明确导航授权。** 当前bridge固定session，输入超时会release，而backend会撤销该session。因此仅重映射Nav2速度话题无法完成“取消后再设点”。新增目标监督和授权握手，绑定goal UUID、目标代数、地图、Episode和navigation epoch；新目标显式取得新权限，旧速度与旧Action结果不能复活。
2. **看见真实接纳结果。** 当前bridge的socket发送成功只证明消息写入连接，不能证明backend允许移动。G3B须暴露accepted/rejected、原因及实际dispatch状态。人工接管先在命令边界撤权/清零，再异步等待Action取消结果。
3. **定位健康独立于TF是否持续发布。** slam_toolbox可以周期发送map→odom；有TF不代表最近扫描匹配成功。监督原始scan、有效匹配/pose、TF、节点存活及时间代数；区分静止时按配置跳过匹配与运动中失去匹配。冻结可用信号、超时和恢复规则，不能把旧定位换新时间戳。
4. **麦轮参数匹配实际低速。** Humble官方Nav2示例按TurtleBot配置，横向速度/加速度为零，且progress/stop阈值不适合当前0.05m/s限速。初选NavFn+DWB，显式调整横移采样、速度/加速度、最小速度阈值、进度和停稳判据，所有输出经过相同门控。
5. **覆盖完整机器人及允许臂活动的碰撞包络。** 固定保守footprint从模型和声明臂范围生成/验证，覆盖夹爪、相机等投影并留余量；全局与局部配置一致。实际使用footprint碰撞检查，不能只填polygon却仍只检查底盘中心。障碍应覆盖雷达扫描高度，本阶段不证明2D激光能发现桌面悬空障碍。
6. **时钟推进不能受异步消息顺序影响。** 当前bridge从多类消息转发clock；同epoch迟到消息不得回拨时钟，缺capture sim time不能用monotonic回填。运行身份也只能按明确的Reset/启动切换，不能被迟到的旧Episode消息反向覆盖。
7. **Reset与地图重启处理整个ROS状态。** G3A清理的是底座/bridge；新增SLAM、Nav2、costmap、TF缓存和目标状态也须清理或重启。重启验证使用新进程和新Episode，旧目标自动恢复属于失败。

另需验证真实非零人工驾驶接管：backend普通SET_BASE_TWIST路径仍会取消arm。G3B若承诺接管保持独立arm目标，应以专用profile显式修补并测试，同时保留G1默认语义，不因ZERO_BASE子检查通过就推断全部接管路径通过。

相关实现入口：src/mobilecar/backends/mujoco/backend.py、src/mobilecar/ros_ipc/server.py、src/mobilecar/contracts/navigation.py、ros2_ws/src/mobilecar_navigation_bridge/mobilecar_navigation_bridge/bridge_node.py。修改这些共享文件只由主负责人或明确转交的唯一所有者执行。

## 4. 冻结的能力范围

- ROS 2 Humble，复用G3A runtime与bridge；按服务器实际已装版本锁定slam_toolbox/Nav2，不追随rolling/main接口。
- MAPPING由人工驾驶/明确来源的开发驾驶建图；若是agent操控，保留agent来源，不称人类示教。
- LOCALIZATION加载占据地图和序列化位姿图/扫描资产，不启动AMCL与slam_toolbox同时发布map→odom。
- 两个场景：已有不对称房间，以及宽门洞/L形通路；一个ideal种子、两种有限臂模式，不扩成随机场景矩阵。
- 服务器RViz设位置/朝向、取消及查看状态；已有三维网页继续控制臂，所有自动GUI操作在服务器。
- 初期保留当前0.05m/s平移、0.30rad/s偏航上限。若确需修改，先冻结新G3B参数并验证；不修改G3A历史profile或为了超时临时加速。
- 地图目标来自已观测地图或人工标注，不从场景真值自动生成导航路线。GT仅进入隔离评分。
- 允许一次记录明确的初始定位先验；未知起点全局重定位、Frontier、目标搜索、近场抓取交接、VLN/VLA、训练和实车均另立Goal。

## 5. 有限验收与完成门

正式集合由旧草案8例细化为10例，补上独立定位故障与Reset，避免只有scan故障就宣称“失定位停车”：

| 正式组 | 数量 | 要证明的行为 |
|---|---:|---|
| 两场景×臂保持/有限活动 | 4 | Action成功，物理到达并停稳，无非预期碰撞 |
| 两场景保存后进程退出/重启加载 | 2 | 新Episode/进程加载同地图，实际扫描定位后到点 |
| 取消及人工接管 | 1 | 取消与非零人工驾驶均撤销旧目标；松手不恢复；显式新目标可运行 |
| scan失联 | 1 | 保留其他输入时失效停车，恢复不自动续航 |
| 定位输出/匹配失效 | 1 | scan/odom继续，定位不可用时停车；TF仍可见不能掩盖失效 |
| Reset/时间代数切换 | 1 | 清理旧目标、速度、TF/定位/路径；显式初始化后新目标可运行 |

到达门：位置≤0.10m、yaw≤10°；平移≤0.01m/s、yaw rate≤0.03rad/s持续≥0.6s。Action和物理评分分别报告。一次初始坐标对齐后不逐段重新对齐。

初期建议每次到点≤120s仿真时间、≤180s墙钟时间，每场景建图准备≤300s仿真时间、≤420s墙钟时间；按实际短路线在正式前冻结。不能继承G3A单段60s而让低速导航必然超时，也不能通过无限重试延长验收。每个job另有总超时与关闭记录。

独立列出：横移实测、NaN/+Inf实际消费者行为、足迹/臂包络、目标拒绝、同epoch地图不兼容拒绝、控制接纳回执及GUI操作。它们不增加10例的成功分母；失败同样阻止complete。拒绝障碍、未知、地图外或无路目标，不能悄悄换成附近可达点算成功。

健康检测到命令门清零建议≤0.15s；失联检测时限按传感器门和定位策略预先冻结。报告故障注入→检测→零命令→物理停稳四个时间，不将命令归零直接当停车。失效后物理停稳建议≤1.0s，确认当前制动参数后冻结，失败不静默放宽。

只对本轮实际修改范围做定向验证，版本稳定后跑一次相关完整回归。无具体影响，不重做G2B编码/loader、G2A长采集和全浏览器性能。

## 6. 输出与后续路线

本地保存G3B源码、场景、配置/依赖锁、接口、地图包manifest、复现说明、逐例结果和小证据；原始Episode、地图/位姿图大资产、bags与完整日志留服务器。最终所有结果对应同一release/aggregate，回传默认≤20MB。

G3B完成后停止。下一阶段候选为G4A：定义一个完整移动操作任务，收集少量来源明确的成功示教并核对部署视图可观测性和训练资格；然后独立启动G4B固定底盘近场ACT基线。G3B到达READY不等于相机看见物体、IK可达或能抓取，这些不加入本轮完成门。

## 7. 技术依据

- [slam_toolbox Humble说明](https://github.com/SteveMacenski/slam_toolbox/blob/humble/README.md)：占据地图保存与位姿图序列化用途不同。
- [Humble定位实现](https://github.com/SteveMacenski/slam_toolbox/blob/humble/src/slam_toolbox_localization.cpp)及[公共实现](https://github.com/SteveMacenski/slam_toolbox/blob/humble/src/slam_toolbox_common.cpp)：成功匹配pose与周期TF应分开监督；采样节流会影响静止pose更新。
- [Nav2 Humble示例配置](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)：示例的横移、速度与进度参数须按本车调整。
- [Humble obstacle layer](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp)与[ObstacleFootprint critic](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp)：消费者参数和footprint检查需实测。

以上为接口依据；服务器实际安装版本、可用插件及行为仍由G3B执行时核对和锁定。
