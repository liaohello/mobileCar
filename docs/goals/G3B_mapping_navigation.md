# G3B Goal：服务器建图、重载定位与到点导航

日期：2026-10-17；状态于2026-10-18更新：**G3B已完成，正文保留为历史执行合同，不重复启动**。结果见[G3B报告](../g3b_report.md)与[完成审计](../evidence/g3b_completion_audit.json)。当前后续规划见[后G3B计划](../post_g3b_execution_plan.md)和[G4A-1提示词](G4A1_nearfield_readiness.md)。历史预算诊断和续跑说明不再作为当前执行入口。

---

请在 C:/Users/Administrator/Desktop/embodied_intelligence_proj/mobileCar 创建并持续执行G3B：

**使用G3A已验收的测量、里程计、TF和唯一MuJoCo后端，在服务器实现ROS2 Humble下的slam_toolbox建图/保存/重载定位与Nav2到点，支持人工设点、取消/接管、失定位/扫描失联停止，以及有限机械臂活动。**

## 1. 基线与前置门

先读AGENTS.md、docs/server_execution_policy.md、docs/post_g3a_execution_plan.md，以及docs/g3a_report.md、docs/g3a_reproduction.md、docs/g3a_interface_v1.md、configs/profiles/g3a_navigation_v1.yaml和docs/evidence/g3a_server_artifact_index.json。

基线release为g3a-20261017T194117Z-aa6a6e987115，aggregate为aa6a6e9871159310cf3d2499847e181a7e962f7a8c552c9d5304461806848859；G3A核心88 passed、bridge 3 passed、物理6/6和ROS端到端结果均为历史已完成证据。

保留G1/G2A/G2B/G3A成果、原始记录和train_qualified=false结论，不覆盖历史报告或重标旧Episode。无需重开G3A；本轮只修补新增导航集成所需缺口。G2B loader结果不参与导航验收，也不重跑导出。按实际Goal身份创建/继续，不设置用户未请求的token预算。

用户已明确不需要人为token预算。create_goal时**完全省略token_budget字段**；0不是无限预算，不能在0被拒绝后擅自改成1000000或其他正数。创建后立即get_goal，确认tokenBudget=null。已有错误budget_limited状态先按恢复说明由用户控制清除/调整，不通过update_goal伪造恢复，不编辑内部数据库，不重做已经保存的实现。

## 2. 服务器和运行隔离

所有安装、colcon/前端构建、测试、MuJoCo、ROS/RViz及评估在服务器。本地仅源码/配置/锁/小报告。复用已有SSH、发布/作业工具，不重建另一套同步系统。

先复核G3A锁定环境，补Humble对应的slam_toolbox/Nav2/RViz必要组件；记录实际安装版本和参数/插件身份，不混rolling/main接口，不改全局Conda或G2B导出环境，不恢复旧A1暂停任务。需要系统组件时遵循服务器政策，仅补必要包；服务器失败时不回退本地或本机WSL。

保留唯一物理后端；ROS节点只消费安全原始观测/估计并提交受控命令，不读取GT_DEBUG snapshot当定位，不构造第二个Simulation。

一次最多一个物理实例，运行使用独立ROS_DOMAIN_ID、端口、地图/日志和进程组；只回收本任务进程，原始runs/bags/地图留服务器。

## 3. 分工

先冻结命令、地图包、状态、取消和评估接口，证明一个小任务有有效交付后再扩大：

- 主负责人：公共合同、app/backend/IPC/bridge共享接线、配置集成、依赖锁、部署与服务器队列、最终验收。
- A：独立mapping/localization包、SLAM参数、地图包保存/加载和定位健康模块及测试。
- B：独立goal supervisor包、Nav2配置/BT、Action生命周期、取消/接管和服务器RViz设置。
- C：两个场景、臂包络工具、有限case runner、隔离评分与证据。

开始前写工作包文件所有权表；A/B/C不能并发修改同一个launch、参数文件、backend或bridge。公共部分只由主负责人或明确转交的唯一所有者实现。物理运行只由主负责人排队，agent可并行做源码、配置、静态审查和服务器受限软件检查。

G3A委派均无有效代码产物。此次先派一个边界清楚的小任务，要求实际新增/修改文件和可核对回执；核对成功后才扩大。若无产物，记录并接管，不反复创建空会话；不得把作者自检描述为独立审查。

## 4. M0：明确两种模式和TF

MAPPING：使用G3A激光与估计里程计在线建图，允许人工驾驶，再向已知可通行区设点。

LOCALIZATION：加载已保存占据地图及定位所需位姿图/扫描资产，退出重启后定位/设点。不能只保存PNG就称位姿图已序列化，不能只冻结map→odom代替重启定位。地图包为占据图像+YAML、序列化pose graph与其数据、manifest和参数身份，使用Humble实际服务并核对文件内容。

明确唯一发布者：slam_toolbox mapping或localization二选一发布map→odom，G3A估计发布odom→base，bridge转发TF；关节/传感器TF来源保持一致。不让AMCL、robot_state_publisher或静态TF重复发布已有边。所有导航消费者use_sim_time=true，capture stamp和QoS与实际输入相容。

先补G3B时钟门：当前bridge会从observation/estimate/health异步消息逐条转发/clock，因此同epoch迟到消息不得回拨clock；缺capture sim time应拒绝/诊断，不能用backend_monotonic_s冒充。只接受当前run/Episode/generation/epoch，旧身份消息不能反向切换运行身份。header仍保留原capture时间，不通过重打时间掩盖迟到。

Reset/时间回跳需先撤权/取消旧目标，再清理或重启SLAM、Nav2、costmap、TF缓存、速度平滑与队列、定位及健康。仅重置G3A估计器不算整个导航系统Reset成功。

一次显式初始定位/人工先验可以配置并记录；不要求未知起点全局重定位，禁止每步用world_sim纠偏。导航器和地图不能读取场景真值地图、目标真值或Oracle。

地图包绑定map_id、scene/model及sensor/TF profile hash、分辨率/原点/坐标约定、参数/软件版本、建图run/Episode、初始先验和所有文件hash。与安全相关的scene/model/传感器变更明确拒绝，不以整个源码aggregate不相等一概拒绝兼容地图；兼容字段策略冻结并测试。

实际重载流程：停止并确认旧SLAM/Nav2进程退出→新物理运行/Episode与新odom原点→加载原地图包→一次显式初始定位先验→确认scan、odom、有效匹配与TF就绪→激活Nav2→新目标到达。不得只在原进程重读路径而称“重启定位通过”。两个模式的地图话题也保持唯一来源。

定位健康不能只依赖map→odom持续发布。监督原始scan、最近有效匹配/pose、TF可用性、节点存活和时间代数；IPC connected不能覆盖上游FAILED。针对静止被minimum_travel_distance等配置正常跳过匹配，声明有限静止保持与再次运动的准入条件，或配置并证明匹配持续更新；运动中丢失匹配必须被检测。可使用实际pose/诊断或窄扩展暴露成功处理计数，不改写SLAM核心来伪造状态。

冻结健康阈值及其可实现性；执行一项“scan/odom仍新鲜，但定位不再成功更新”的故障检查。若缺乏算法失定位置信信号，报告为定位输出/匹配失效检测，不能宣称已证明所有几何退化或全局失定位检测。

## 5. M1：两个简单场景与交互

在G3A诊断场景上复用一个房间，另加宽门洞/L形通路。真实模型加载对应场景，保持自由底盘、可动臂、有限力控制、真实接触和link4相机；joint5旋转夹爪、独立驱动两指开合。不依赖target_cube或抓取Oracle。

第一轮采用覆盖底盘和声明臂活动范围的固定保守footprint，包含相机、夹爪等投影与余量，实际验证活动轨迹及中间姿态没有越界；避免只检查几个端点。全局/局部costmap保持同一包络，局部控制使用实际footprint碰撞检查。不能只填polygon却仍只用中心点避障；不缩小包络、焊死臂、关闭碰撞或用qpos搬车来通过。

场景障碍需覆盖实际激光扫描高度且保证足迹有物理余量；地图来自实际scan，不从场景几何直接生成导航地图。2D激光对桌面/悬空障碍的覆盖不在本轮泛化结论内。

交互优先服务器RViz2的Nav2 Goal工具：点击位置、拖动朝向、取消、重新设点、查看地图/扫描/路径/状态。已有浏览器三维界面保留驾驶与臂控制，不同时开发一套新网页导航地图。

实际GUI证据和Action客户端测试分开。只调用Action不算鼠标交互完成；若GUI工具不可用，报告未验证，不伪造截图或通过。

## 6. M2：导航、取消与健康

初选Humble NavFn全局规划+DWB全向局部控制，显式支持麦轮vy并验证横移实际发生，不把绕行或先转向当横移证据。DWB应配置适用的ObstacleFootprint等实际包络检查，实际插件/版本以服务器为准，必要替代说明理由。

Humble示例参数不是本车参数：显式配置vy上下限、vy采样、横向加减速度、controller最小速度阈值、progress checker、goal checker、平滑器和停稳阈值。初期沿用平移0.05m/s、yaw 0.30rad/s上限，保证规划器/平滑器/bridge限幅一致；若改变，先冻结G3B专用配置及定向验证，不改G3A历史profile。避免照搬0.5m进度半径或大于实际车速的stopped阈值。

建立目标监督层，统一RViz和Action客户端入口，向内部Nav2提交唯一活动目标。目标合同至少含goal UUID、goal generation、map_id、run/Episode/generation/navigation epoch、来源和授权session；状态区分待就绪、执行、取消、接管、故障和到达。版本化新增G3B控制合同/profile，保持G3A测试来源兼容，不把历史ros_navigation_test数据重新标为Nav2。

当前bridge固定session且release会使其被backend拒绝复用。因此不能只remap /cmd_vel：新目标显式授权新session，目标结束/取消/接管/故障时撤权，显式新目标才重新授权。受控命令应能追溯到目标和有效代数；迟到速度、旧Action结果和重连队列不能绑定到新目标。

raw Twist不携带目标代数；监督层必须定义目标切换屏障、旧Action终止/输出静默和队列丢弃策略，或增加有代数的内部命令封装，不能仅在回调时给旧速度补上当前goal ID。全部速度来源，包括behavior/recovery和可选平滑器输出，进入同一最终门控，禁止旁路直接驱动backend。

IPC socket发送成功不等于backend接纳。补齐可关联回执或等效实际dispatch状态；界面/监督层能区分请求、接纳、拒绝、应用和失效原因。正式证据包括至少一次真实拒绝，不能只报告sendall成功。

复用G3A独立控制组：每条导航速度更新不取消臂。取消/人工接管先在最终命令边界撤权并清零，再异步取消Nav2 Action，不等待Action响应才停车。接管后非零人工驾驶、松键、重连均不能恢复旧目标；独立arm目标按G3B合同保持。当前backend普通人工SET_BASE_TWIST仍有取消arm路径，必须对真实非零接管做定向验证，必要时在G3B profile明确修补，保留G1默认行为的兼容性。软件全局Stop、租约和录制门始终优先。

障碍/地图外/未知/无路目标明确拒绝或失败；不自动探索未知空间。规划通过不保证实际可行，跟踪中继续碰撞/进度/定位/测量健康检查。

scan/定位过期或不可用时按冻结策略撤权、停车并终止/取消目标；重新定位/重新发目标需要明确恢复，不用旧pose配新stamp。健康门和失联watchdog使用steady clock，暂停仿真或停止/clock不能令其挂起。到达后用新鲜状态确认连续停稳。

READY仅表示导航到达，不等于目标可见、IK可达或可抓取，近场交接另包。

## 7. M3：有限服务器验收

固定一个ideal种子、两个简单场景、初始先验、目标、活动范围、时间限制和判据。case使用专用G3B导航记录profile，保存观测、估计、控制生命周期与目标/地图身份；未启用RGB-D时明确缺失，不套用G2A RGB-D资格，不把导航Episode标为可训练人类示教。

先串通一个场景、一张地图和一个目标，再进行第二场景及正式集合。正式前冻结case manifest、参数和验收阈值，10例：

1. 2场景×臂保持/有限活动：4例到达；
2. 2例保存地图后退出/重启加载定位，再到达新目标；
3. 1例取消/非零人工接管，含松手不恢复及显式新目标可运行；
4. 1例扫描失联停止及恢复不自动续航；
5. 1例定位输出/匹配失效停止，scan/odom保持新鲜；
6. 1例Reset/时间代数切换，旧目标/命令/TF清理，新初始化后新目标可运行。

建图准备、障碍/未知/地图外/无路等拒绝子检查、不兼容地图拒绝、连续两个目标、横移、footprint/臂包络、命令回执、实际GUI操作、开发探针和修复复验单独计数，不混分母，不扩成多种子/噪声大矩阵。所有必需子门也须通过，不能只看10例分母。

NaN/+Inf在G3A只证明桥接映射；本轮对实际slam_toolbox和Nav2消费者做小型定向检查，保留SELF_OCCLUDED/INVALID不清自由区、NO_RETURN按冻结策略处理。可保守不利用+Inf清除，但须报告取舍；禁止把所有NaN换成max range。检查足迹内自动清除和激光外参/高度参数没有令本体或障碍被错误处理。

首轮到达门：位置≤0.10m、朝向≤10°，平移≤0.01m/s、yaw≤0.03rad/s持续≥0.6s。正式前核对并冻结；失败后不静默放宽。

建议每个到点段≤120s仿真时间、≤180s墙钟时间；每场景建图准备≤300s仿真时间、≤420s墙钟时间，按短路线在正式前冻结，job另设总超时。不要继承G3A 60s短段而使低速路径必然超时；不得用无限重试、临时加速、缩短目标或放宽容差制造通过。

健康检测→最终门控零命令建议≤0.15s，检测后物理停稳建议≤1.0s；具体测量输入和定位失效检测时限、物理制动门在正式前按配置冻结。报告注入→检测→零命令→实际停稳的分段耗时、最大位移/制动距离及物理速度，不以零命令代替停车。

Nav2 Action结果与隔离物理评分分别记录。目标来自已观测地图/人工标注，不从真值生成路线；评分使用导航之外保存的固定map/world坐标对齐及物理接触，不参与控制。无非预期车体/臂—环境碰撞，轮地接触不算失败；不以Nav2自身pose作为唯一成功证据。估计误差只用每条连续轨迹一次初始对齐，不逐段或逐目标重对齐。

报告到点/停稳误差、路径/耗时、取消/接管、定位与扫描健康、臂活动对照、地图/位姿图重启证据和服务器资源。没有实际回环约束证据时，不把返回起点当回环通过。

修复先定向复验，版本稳定后一次受影响回归；无新修改不反复全测。不重跑G2B官方loader或G2A完整性能/浏览器序列，除非具体改动影响并说明。

## 8. 完成、交付、停止

交付docs/g3b_report.md、g3b_reproduction.md、g3b_interface_v1.md、g3b_demo_record.md、g3b_failures_and_fixes.md，G3B专用模型/profile/SLAM/Nav2/BT/RViz配置、依赖锁和case manifest；地图包manifest、10例及子门结果、GUI证据、真实agent回执、docs/evidence/g3b_server_artifact_index.json及新release/aggregate。

最终关键结果、逐文件源码hash和受影响回归对应同一冻结release；不将开发阶段不同release的成功拼成最终通过。交付可复现启动/停止/建图/保存/加载/设点/取消步骤及本任务进程归属。

本地只回传小报告/配置/锁/索引和少量预览，默认≤20MB；地图大文件、bags、原始runs和日志留服务器。保留旧记录与用户未提交工作，不自动提交推送。

本Goal不含Frontier、视觉SLAM、目标搜索、自动抓取、毫米级停车、未知起点重定位、VLN/VLA、训练或实车。关键门缺失不得标complete；完成后交付并停止，不自动开始示教或学习。

官方接口依据见[后G3A计划的技术依据](../post_g3a_execution_plan.md#7-技术依据)。运行时仍须核对服务器实际Humble包版本；文档存在或服务返回成功不能替代物理验收。
