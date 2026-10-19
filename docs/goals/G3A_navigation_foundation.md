# G3A Goal：服务器导航观测、里程计与ROS桥接

日期：2026-10-17。状态：**G3A已完成，正文保留为历史执行范围，不重复启动**。结果见[验收报告](../g3a_report.md)；当前后续为待单独启动的[G3B](G3B_mapping_navigation.md)，计划见[后G3A计划](../post_g3a_execution_plan.md)。

---

请在 C:/Users/Administrator/Desktop/embodied_intelligence_proj/mobileCar 创建并持续执行G3A：

**保持现有G1/G2A/G2B成果，在服务器实现可用于导航的激光/轮/IMU测量、独立平面里程计、健康/TF、原始观测分发与ROS2 Humble桥接，验证导航来源与机械臂并行、人工接管和失效停止。交付一个无抓取方块依赖的诊断场景及有限验收。**

本Goal不安装/启动SLAM或Nav2，不做建图、到点、Frontier、自动抓取、新示教、导出、训练或实车。完成后停止，不自动进入G3B。

## 1. 必须阅读

- AGENTS.md
- docs/server_execution_policy.md
- docs/post_g2b_execution_plan.md
- docs/g2a_interface_v1.md、docs/runtime_api_v1.md
- docs/g2a_server_report.md、docs/g2b_report.md
- docs/evidence/g2a_server_artifact_index.json、g2b_server_artifact_index.json
- 当前backend、contracts、control/wheel、app与必要配置。

G2B最终release为g2a-20261017T181443Z-ab5d023c98d3，aggregate为ab5d023c98d3d45d6c0c9b530ce91423cdcb4453d7a0fd0a9a750850e76566bc。其format/export有效、train资格false保持历史事实，不覆盖原始记录或报告。

G2A已有capture sim/monotonic/tick和正确base外参，不重做旧缺陷修复。新增导航profile和字段保持兼容，必要语义变化显式版本化。

## 2. 执行位置与自治范围

本地是源码主副本，只编辑/读取文件、制作同步清单并取小报告。所有安装、构建、lint、测试、MuJoCo、ROS和指标计算只在服务器，不回退本机WSL。

复用tools/server/remote_job.py与现有server目录、runtime环境；remote.md仅解析连接，不打印/上传凭据。核对当前ROS2 Humble及系统Python状态，不把旧环境文档当实时事实。

MuJoCo runtime保持现有独立Python3.10/MuJoCo3.1.6；ROS桥接优先独立进程/环境，通过非阻塞观测与受控命令接口连接。不得为了rclpy全局替换NumPy/MuJoCo或改动Conda。

本Goal允许必要代码、服务器专用依赖/ROS基础包、受影响测试与有界物理运行。保留未提交工作，不自动提交/推送，不结束其他任务、不覆盖旧mujoco_learning或恢复A1暂停Goal。

检查当前Goal身份，无冲突再创建，不自行设置token预算；已有同Goal则继续，其他未完成Goal保留。按工具真实规则处理状态。

同时最多一个服务器物理实例、一个mj_step所有者。ROS节点、估计器和Recorder不能各建Simulation。只回传小报告/索引，原始runs/日志留服务器。

## 3. 工作包与可检查委派

主负责人先冻结观测、估计、健康、控制来源和Reset代数合同，再派任务。

- 主负责人：公共合同、app/CLI/config、分发装配、依赖/部署、集成和验收。
- A：backend/传感器/可选目标场景，及自己的测试。
- B：src/mobilecar/estimation/、tests/estimation/；只消费测量白名单。
- C：独立ROS桥接工作区、tests/bridge/；只消费原始观测/估计并提交命令。

A拥有模型/诊断场景资产，主负责人集成跨模块配置。一次小模块/fixture形成实际文件/补丁后再扩大并行；空返回/只等待而无产物时记录并接管，不循环空派发。测试统一经过服务器队列。

## 4. N0：原始观测与分发

当前_scan_laser仍是world-aligned ray，不能直接输入SLAM：

1. 从同源模型或明确版本配置取得T_base_lidar，在capture时刻用实际base姿态和安装变换生成世界射线，发布laser_frame中的角度/range。
2. 固定新导航profile的beam数、非重复角范围、min/max range和10Hz目标。若用瞬时整圈，time_increment如实为0；不虚构真实D500逐束畸变/标定。
3. 输出HIT/NO_RETURN/INVALID/SELF_OCCLUDED，测到机器人应遮挡而不是穿透看到后方；机械臂遮挡依据同capture关节状态。
4. ROS LaserScan转换NaN/+Inf与原始状态对应；遮挡/无效不能成为无遮挡清除射线，越界不补假测距。
5. 轮角/轮速、gyro/IMU与关节反馈携带capture sim time、tick、monotonic及质量。IMU缺失/异常无效，不补零冒充正常。
6. 估计输入白名单排除真实底盘速度/世界pose/Oracle；保留GT_DEBUG仅作隔离评分。
7. 新增非阻塞原始观测分发，Recorder保持原功能，ROS不替换其sink、不从录制文件或UI摘要伪装实时输入；消费者慢/断开明确记账。

导航测量订阅不以Recorder=RECORDING为前提，录制开关只控制保存/正式运动门。零gyro等合法静止测量不是缺失；无效由实际采样失败/质量定义。

仿真传感器可在后端内部用MjData生成测量；估计器只拿测量白名单，不能得到动态世界pose/速度。不要把这两个边界混为“任何传感器代码都禁止访问世界状态”。

默认导航profile可采用轮/gyro与估计50Hz、laser10Hz；实际频率、年龄、队列/丢弃和clock均可检查。不要改变旧G2A默认profile来隐藏新需求。

## 5. N1：独立估计与健康

实现平面轮/gyro里程计，复用并验证麦轮正运动学、轮序/符号/半径。可用简单明确的融合；只有实现对应方程才称EKF。不要为本Goal新增复杂定位算法。

说明IMU轴到base转换、gyro积分、平面高度/roll-pitch近似、原点/初始先验。允许一次显式初始设置；不得逐tick读取base world pose纠偏。

输出odom系pose、base系twist、协方差/来源/health/last measurement；无效/过期时不能重发旧pose配新stamp伪装新鲜。

协方差开发值有配置/来源，未估计维度明确，不能全部置零声明完美。暂定输入最大年龄0.2s、scan0.3s；故障模式与恢复条件在运行前固定。沿用G2A的控制/租约/停车安全门。

## 6. N2：ROS桥接

最低接口：/clock、/scan、/imu/data_raw、/joint_states、/odom、TF、受控速度输入和health诊断。所有依赖版本按实际Humble核对。

ROS header使用capture sim time，节点use_sim_time；TTL/进程看守仍接收端monotonic。不要把publish/arrival时间当capture stamp。

/clock产生/转发、IPC接收、心跳和失联看守使用独立steady调度，不能等待尚未产生或暂停的sim time。跨进程monotonic须声明并证明同一时钟域；异域须显式映射或拒绝直接相减。

/imu/data_raw没有通用valid字段。gyro缺失时按冻结策略停发相应正常数据、发布诊断并使消费者健康失效，不能继续发零值正常IMU；未估计orientation按Humble Imu约定标不可用。gyro/acceleration和odom协方差各自有来源，不能用单位四元数或全零协方差暗示真实测得姿态/完美精度。

TF仅发布odom→base_link或明确的平面基座约定，以及base到传感器/活动臂。每条TF一个来源；无SLAM/定位时不发布伪map→odom。JointState按所用URDF/FK的模型角/名称发布，不混用户角。

官方消息依据：

- https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg
- https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg
- https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg

LaserScan角度/射线时间、Odometry pose parent与twist child frame按该发行版定义实现。QoS与传感器/TF消费端实际匹配，不能只证明消息类型存在。

Reset/时间回跳须清理估计、缓存、队列及控制权限，旧消息不能进入新Episode；原始记录保留代数/边界。

## 7. N3：命令组、诊断场景和有限运行

新增可信ROS导航测试来源；在此模式下base速度更新不取消独立arm目标。G1人工接管语义保持：接管撤销导航权限、取消未来运动，松手/重连不因旧/cmd_vel还在发布而重新接管；恢复需显式新权限/新目标。

软件停止、记录门、过期请求和租约覆盖ROS输入。ROS回调只更新受控命令，不写qpos/ctrl或步进。G3A的速度测试不是Nav2或导航成功。

增加一个非对称带墙/障碍诊断房间，允许不含target_cube。加载器真正加载该场景；抓取Oracle缺目标时明确NOT_APPLICABLE/不可用，不能崩溃、假success或关闭机器人碰撞。保持自由底盘、动态臂和link4相机。

## 8. 先冻结案例，再服务器验证

先做消息/几何/估计/控制定向测试，再小集物理：

1. 静止30s：测量、odom漂移、频率/年龄与TF；
2. 前后/横移及一个组合段：轮序与位移方向；
3. 双向转动与非零yaw/平移：scan方向/墙面几何；
4. 有限臂活动/遮挡：独立控制、遮挡不清自由区；
5. ROS输入/轮IMU/扫描过期与人工接管：明确健康、停车、不自动恢复；
6. Reset：新时间/消息/odom原点，无旧代污染。

各正式段默认≤60s，一个ideal种子；不扩噪声/多场景/多种子矩阵。正常/故障与开发探针、修复复验单列分母。

暂定门：wheel/gyro/odom平均≥45Hz、scan≥9Hz；输入和scan年龄门同上；静止30s漂移≤0.01m、yaw≤2°；短程轨迹终点平面误差≤0.05m、yaw≤5°。实际运行前核对场景/距离和profile后冻结，失败后不静默放宽。

真值只在隔离评分；每条连续轨迹一次初始坐标对齐，不每段重置估计求通过。scan/TF几何用同capture解析参考和明确数值容差，不以里程计误差作借口。

ROS消息/健康与物理评分分开，CLI退出0不能代替各gate。无需GUI；可选RViz检查不能冒充本Goal以外的到点导航。

最终只做一次受影响合同/后端/估计/桥接和必要数据兼容回归。不重跑LeRobot导出/编码/loader或G2A完整浏览器/5Hz长序列，除非具体改动影响并说明。

## 9. 完成、交付、停止

交付g3a_report、reproduction、interface/profile、failures_and_fixes、实际agent回执和server_artifact_index；说明服务器专用启动/停止、话题/TF检查、原始记录/评分/版本路径。

本地保存源码、必要资产、配置/锁和小报告；服务器留runs/bags/完整日志。新release/aggregate与旧G2A/G2B分开，不覆盖旧数据。

必须证明测量/估计/TF正确、原始分发与Recorder共存、ROS受控输入、有限臂活动/接管/故障/Reset。未完成关键门如实报告，不伪标complete。

按已授权范围自主实现修复，不反复确认常规步骤；完成后停止，不自动进入G3B、SLAM/Nav2、示教采集、训练或实车。
