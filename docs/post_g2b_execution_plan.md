# G2B之后的实施计划：先导航基础，再建图与到点

日期：2026-10-17；状态于2026-10-18更新：G3A/G3B均已完成，本文件保留历史规划。当前后续见[后G3B计划](post_g3b_execution_plan.md)。

本文件保留G3A启动前的规划依据；下文“A0-A4”等是已完成的原计划，缺口表不是当前源码状态。G3B当前范围和10例验收以[G3A之后的计划](post_g3a_execution_plan.md)及[G3B Goal](goals/G3B_mapping_navigation.md)为准，替代本文件的8例草案。

当前提示词：[G3B建图与到点](goals/G3B_mapping_navigation.md)。G3A执行结果见
[G3A报告](g3a_report.md)，验收身份为release `g3a-20261017T194117Z-aa6a6e987115`、
aggregate `aa6a6e9871159310cf3d2499847e181a7e962f7a8c552c9d5304461806848859`。
执行位置继续遵循[服务器规范](server_execution_policy.md)。

## 1. 当前基线和阶段判断

G1操作与原生录制、G2A服务器采集/时钟/外参/输入可靠性、G2B离线开发出口均已完成。

G2B保存身份：

- release：g2a-20261017T181443Z-ab5d023c98d3
- aggregate SHA-256：ab5d023c98d3d45d6c0c9b530ce91423cdcb4453d7a0fd0a9a750850e76566bc
- LeRobot 0.4.4，dataset v3.0，独立Python3.12环境。
- D0为14行、D2为272行，仅dev分区；format_valid/export_valid=true，train_qualified=false。

依据：[G2B报告](g2b_report.md)、[复现](g2b_reproduction.md)、[证据索引](evidence/g2b_server_artifact_index.json)。这是保存结果，本轮没有复跑。

这些成果证明数据能正确流入训练格式，并不证明导航BC数据充足、任务学习或实车能力。下一步补齐导航；训练仍需要后续任务成功、来源和可观测性明确的数据。

## 2. 为什么将原G3拆成G3A/G3B（G3A启动前的历史判断）

G3A启动前的源码有以下导航前置缺口，现已由G3A处理，不能继续当作待实现项：

| 当前实现 | 需要补齐 |
|---|---|
| backend._scan_laser以世界方向射线测距，却标base_link；无回波状态/本体遮挡语义 | 正确安装系与capture姿态、局部LaserScan、HIT/NO_RETURN/INVALID/SELF_OCCLUDED |
| 已有capture sim/monotonic/tick，尚无ROS时钟和估计消息 | /clock、采样stamp、Odometry/健康/TF；不回退旧时间语义 |
| 编码器含轮测量也含仿真真实底盘速度，IMU异常仍补零 | 消费者字段白名单与无效标记；轮/IMU独立里程计 |
| 底盘请求统一取消机械臂动作 | 区分人工接管与导航来源；独立控制组、取消代数和不恢复旧目标 |
| 单sink主要进入Recorder | 非阻塞原始观测分发；ROS不能替换Recorder或轮询录制文件当实时输入 |
| 后端强依赖target_cube，场景只有抓取台面 | 可选抓取Oracle、无方块场景、独立传感器诊断环境 |
| 尚无实际odom/ROS导航实现 | 先证明观测、估计和桥接，再调SLAM、地图与规划器 |

G2A已经修正capture时间与base外参，不能继续把这些已修复项当成待重做任务。新增导航工作保持g2a-1.0语义，必要扩展显式版本化。

## 3. 路线与停止边界

~~~text
G1 + G2A + G2B 已完成
  ↓
G3A 导航观测 + 轮/IMU估计 + ROS桥接 + 控制通道
  ↓ 已验收接口、运行profile、场景和数据健康
G3B 建图 + 保存/重载定位 + 到点 + 人工接管
  ↓
G4A 少量完整任务示教 + 视图可观测性 + 训练资格
  ↓
G4B ACT固定底盘近场基线，再独立评估语言VLA
  ↓
语言子目标/VLN、Frontier、近场可操作性交接与联合任务
~~~

G3A不包含SLAM/Nav2到点，G3B不包含自主搜索/抓取/训练。每个Goal完成后停止；下一阶段由用户分别启动，不能以路线图为全部任务的自动授权。

G4A的样本数量与成功/来源规则届时按具体训练profile制定。G2B开发profile中的数量门不是普适的数据充分性证明，不能仅凑满某个Episode数就宣称可学。

## 4. G3A具体实现

### A0：服务器和最小公共接口

复用tools/server/remote_job.py、G2A runtime专用环境与已有服务器目录。只核当前连接/资源和ROS包状态，不重新跑GPU、5Hz和LeRobot验收。

优先把MuJoCo runtime和ROS2 Humble桥接进程隔离；桥接读取明确的原始观测并提交受控命令，不能创建第二个Simulation。依赖只放专用环境/工作区，不全局替换已有Python、NumPy、MuJoCo或Conda。

新增版本化NavigationObservation、EstimatedBaseState/health与来源明确的控制通道。world_sim状态保留给隔离评分/GT_DEBUG，不改名为odom/map；G3A不发布假的map→odom。

### A1：导航测量

- 激光安装参数从同源模型/明确配置得到；世界射线由capture时刻base姿态和安装变换生成，消息角度与range在laser_frame描述。
- 新导航profile明确角范围、beam数量、range、频率和瞬时扫描假设。不得把重复端点当两个独立方向，不虚构真实D500逐束时延/噪声。
- 实际检测机器人遮挡；不能沿用“过滤机器人后射线穿过手臂看后方”的理想结果。遮挡方向无效，不能变成清除射线。
- HIT、NO_RETURN、INVALID、SELF_OCCLUDED在原始协议中区分。转成LaserScan的NaN/+Inf策略须与Humble消费者一致并做定向检查。
- wheel/IMU只包含可用测量与质量，缺失/异常明确无效；排除qvel直接计算的底盘速度、世界位置等特权字段。

仿真传感器内部使用MuJoCo状态生成测距/本体测量是正常实现；限制施加于估计/导航消费边界。不能用简单禁止全部world访问的检查，误把传感器生成与Oracle评分也禁止。

建议导航profile：轮/gyro采样与估计50Hz、雷达10Hz、用于自遮挡的关节状态与scan同capture绑定。消息实际生成/发布/使用率均测量；不将G2A的10Hz记录率直接解释为估计器频率。

### A2：轮/IMU里程计

第一版采用可解释的平面轮/gyro融合即可，优先复用并验证现有麦轮正运动学。只有实际实现滤波方程时才命名EKF。

明确轮序、半径/符号、模型角/用户角、IMU坐标、平面投影和初始原点。允许一次显式初始先验，禁止每步读取真实底盘位姿纠偏。

输出odom系pose、base系twist、来源、协方差与健康。协方差可为有标记的开发参数，不能全部置零冒充已标定精度；未经估计的维度明确处理。

### A3：原始分发与ROS

原始观测在采样侧分发给Recorder和导航消费者，分别有界排队；慢ROS/写盘不阻塞物理，不能替换唯一sink导致另一消费者失效。

导航订阅不能依赖Recorder处于RECORDING。录制开关只控制保存与正式运动门，不应令未录制调试时的导航测量消失；传感器有效零值也不能被当成缺失。

最低ROS接口：

- /clock，使用sim time；
- /scan，capture stamp与laser_frame；
- /imu/data_raw，轴向/有效性明确；
- /joint_states，按所用URDF/FK约定发布模型角及关节名；
- /odom及odom→base_link（或明确的平面基座约定）；
- base到laser/IMU/活动臂的TF，每条变换只有一个来源；
- 受控速度输入、停止/接管与健康诊断。

无定位时不生成map。Reset/时间回跳清理估计、队列和控制代数，旧消息不续接新Episode。

ROS消息stamp使用capture sim time；/clock产生/转发、IPC接收、心跳和失联看守用独立steady调度，不能等待尚未产生或已暂停的sim time。跨进程monotonic须证明同一时钟域，否则显式映射/拒绝比较。

IMU原始无效状态须映射到ROS消费策略：gyro缺失时停发相应正常数据并发布诊断/触发健康门，不继续发零值。未估计orientation按Humble Imu约定标不可用；gyro、acceleration和odom协方差分别说明来源，不以单位四元数/全零协方差暗示测得完美姿态。

### A4：控制组与可选场景

保留G1/G2A人工驾驶默认语义；新增可信的导航/桥接来源。导航速度更新不逐次取消臂，同一组仍互斥。人工接管撤销导航权限并置取消代数；松手/重连不因旧/cmd_vel仍在发布而恢复。

G3A用受控ROS速度测试验证此通道，不把它称作Nav2或到点成功。软件停止、记录门和租约覆盖所有来源。

增加一个不含target_cube的非对称诊断房间；抓取Oracle返回NOT_APPLICABLE/明确不可用，不能因无目标崩溃，也不能把无评分写成任务成功。机器人碰撞与轮地接触检查仍有效。

## 5. G3A有限验收

所有实际运行在服务器，一次一个物理实例。先软件/几何/消息定向验证，再做小集：

1. 静止：轮/IMU/odom、frame和TF，30秒内报告漂移/频率/年龄。
2. 平移：前后/横移及一个组合段，检查轮序、运动方向和相对位移。
3. 旋转：双向转动、非零yaw与平移后扫描墙面方向/距离一致。
4. 有限臂活动与遮挡：nav通道不取消臂，真实遮挡方向不清自由区。
5. 失联/过期和接管：缺轮/IMU/扫描或ROS输入时按声明模式降级/停车，松手不恢复旧动作。
6. Reset：新Episode/时间代数、估计原点与TF/消息无旧数据混入。

单个正式段默认≤60秒，总体不扩成噪声/多种子大矩阵。是否将多个行为合成一条轨迹在运行前固定；负例和正常段分开统计。

暂定开发门：正常轮/gyro/odom平均≥45Hz，scan≥9Hz；估计输入最大年龄0.2秒、scan年龄0.3秒，超限有显式健康/停止原因。旧G1/G2A控制/租约/停车门不放宽。

短程轨迹用隔离真值做一次初始坐标对齐，终点平面误差≤0.05m、偏航误差≤5°作为暂定门。静止30秒平移漂移≤0.01m、yaw≤2°。正式前核对profile并冻结；不重置每条腿的估计或逐段对齐求通过。

LaserScan/TF几何比较用同capture的几何参考和已声明数值容差，不用里程计误差掩盖射线坐标错误。ROS消息、桥接健康与物理评分分开报告。

只跑受影响的合同、后端、数据兼容、估计/ROS测试及上述小集；稳定后一次相关回归。不重新执行G2B编码/官方loader、G2A长采集或浏览器序列，除非实际改动影响它们并说明。

## 6. agent与文件所有权

主负责人：公共合同、app/CLI/config、部署/依赖、原始分发装配、最终验收与文档。

- A：src/mobilecar/backends/和传感器、场景可选目标逻辑、对应backend测试；
- B：src/mobilecar/estimation/及tests/estimation/，只消费冻结观测合同；
- C：独立ROS桥接/工作区和tests/bridge/，不创建仿真、不改A/B内部实现。

静态场景资产/TF静态参数由A维护，跨模块接线由主负责人处理。先以一个小模块/fixture证明agent有效交付，再扩并行；无有效产物记录接管，不循环派发。

不让agent独立启动多个物理作业；统一服务器队列。独立审查只在确有非作者报告时声明。

## 7. G3B范围与交付门

G3A接口/健康/估计门通过后，G3B使用Humble、slam_toolbox、Nav2，在两个简单场景做人工建图、占据地图+位姿图保存、重启定位、map目标和取消/接管。

Nav2采用经过Humble实际验证的全向配置；初期用覆盖有限臂活动的保守footprint。首版优先服务器RViz2设点，保留现有三维界面操作臂；不同时重做一套网页导航地图。

正式小集：2场景×臂保持/有限活动4个到达、2个重启加载定位、取消/人工接管和scan失联各1个；准备建图、拒绝子检查和修复复验单列分母。

到达暂定≤0.10m、≤10°，平移≤0.01m/s、yaw≤0.03rad/s持续0.6秒。Nav2 Action成功与隔离物理判据分别通过。到点不代表具备抓取视野/IK可达性。

不含Frontier、目标搜索、自动抓取、毫米级停车、训练、全身策略或实车。完成后交付地图包、启动方式、逐例结果和下一包边界并停止。

## 8. 服务器和交付约束

本地只保留逻辑代码、必要资产、配置/锁/清单和小报告。所有环境构建、ROS、lint、测试、仿真、GUI与指标重算在服务器。原始runs、bags/日志、地图留服务器，不回退本机WSL。

G2B的LeRobot环境和数据不需要参与G3。复用服务器工具和runtime环境，ROS仅补缺失必要组件，不恢复旧A1暂停任务、不改其他任务进程和环境。

每Goal保存新release/aggregate、源与模型/profile身份、逐例结果、真实失败和server_artifact_index。本轮仅写计划；执行Goal另外启动。
