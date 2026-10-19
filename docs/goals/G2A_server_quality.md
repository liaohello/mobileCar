# G2A Goal：服务器执行、观测质量与遥操作可靠性

> 已完成的历史提示词：最终结果见[G2A报告](../g2a_server_report.md)。下一首选为[G2B v2](G2B_lerobot_export.md)，不重新启动本Goal。

日期：2026-10-17。状态：待执行提示词。本文件保存不代表已启动Goal。

---

请在 C:/Users/Administrator/Desktop/embodied_intelligence_proj/mobileCar 创建并持续执行G2A：

**保持G1功能，在服务器建立可追溯运行基线，修正观测采样时间和base外参，消除TCP求解对物理循环的长锁阻塞，恢复5Hz图像profile，验证真实浏览器输入链，并提供版本化质量/资格报告。**

## 1. 依据和硬约束

先读适用AGENTS.md及：

- docs/server_execution_policy.md
- docs/post_g1_execution_plan.md
- docs/runtime_api_v1.md、docs/interface_contract_v1.md
- docs/acceptance_report.md、docs/failures_and_fixes.md

G1已完成，保留其报告和原始记录，不重标历史结论。新Goal不自动恢复旧mujoco_learning/A1任务。

用户要求后续所有安装、依赖解析、构建、lint、测试、MuJoCo、渲染、Playwright、重算和导出在服务器。本地只编辑/读取源码、配置/文档、制作传输清单和保存小报告；服务器失败不得回本地或本机WSL运行。

本Goal包含必要本地代码修复、源码同步、服务器专用环境安装、有限服务器验证与报告。常规修复无需反复确认。不提交/推送Git、不清理用户环境/数据、不结束其他任务，不训练、不发布、不实车。

## 2. Goal和服务器预检

检查本会话Goal身份；无冲突时创建本Goal，不自行设置token预算。已有同Goal则继续；不同未完成Goal保留并报告，不伪造完成腾位置。

从未跟踪remote.md解析连接字段，优先既有SSH私钥；不打印、上传、提交凭据，不直接执行配置中的任意命令文本。当前proj_path为 ~/Desktop/mobileCar，远端解析绝对路径并核对现有内容。

在服务器复核OS、Python、GPU、EGL/实际renderer、Chromium、磁盘和活跃任务。先按G1锁文件建立专用Python3.10/MuJoCo3.1.6环境，Node构建也在服务器；不替换全局/Conda环境。

使用带文件hash的隔离release，包含当前必要未跟踪实现和资产；上传后核对。原始runs等不从本地整树同步。独立输出放服务器var目录，本地默认仅回传≤20MB摘要/小证据和远端索引。

同一时刻最多一个服务器物理仿真实例。资源/端口冲突避让，不kill其他任务；记录本任务PID/进程组并有界回收。预检失败时继续可做的代码工作，明确未运行项。

## 3. 多agent分工

主负责人冻结公共字段、版本、文件所有权及服务器运行队列后，最多三个执行agent：

| 所有者 | 文件/职责 |
|---|---|
| 主负责人 | AGENTS/公共合同、src/mobilecar/app.py、cli.py、config.py、configs/default.yaml、依赖锁、远端作业/同步工具、集成测试和报告 |
| A | src/mobilecar/backends/、planning/相关实现和tests/backend/；时钟/变换/调度/锁与规划副本 |
| B | src/mobilecar/ui/及tests/ui/；Three.js渲染负载、实际页面输入/心跳、服务器前端构建 |
| C | src/mobilecar/data/、tools/data/、tests/data/；覆盖/时效、profile资格与只读质量输出 |

表中planning/指src/mobilecar/planning/，不得创建根目录同名第二套包。需要改公共文件由所有者处理，配置/新字段交给主负责人集成。各agent可并行编码，测试仅通过服务器队列。先有明确工作包和回执，再实施。

agent无有效交付时记录并调整/接管，不循环派发相同空任务，不冒充独立审查。

## 4. 必须完成的定向修复

### 4.1 时间与坐标

- 在复制用于渲染的物理状态时保存capture_sim_time、control_tick、capture_backend_monotonic和反馈引用。另存render完成、publish/arrival、persist时刻。
- 相机、关节、变换来自同一capture快照。不能以渲染结束时间当measurement，不能把不同clock域当作可直接相减。
- 修正T_base_link4/T_base_camera/T_base_tcp：由world变换左乘inverse(T_world_base)。相对T_link4_camera和T_TCP_camera保留正确方向；相机仍固定link4。
- 消除激光等payload与外层同名时间混域；明确每个时间的clock及来源。旧记录缺capture时间时显式未知，不反推/补造。
- 兼容读取mobilecar-1.0历史记录；新语义写入显式profile/clock/transform版本，必要时升级合同并给映射。

### 4.2 控制、采样和性能

- TCP求解/长碰撞检查只在短锁下复制状态，再离锁计算；完成后复核模型、目标、状态与cancel epoch，拒绝过期结果。控制和停止检查不等待长任务。
- 记录control、锁等待、capture、render、encode、backend暂存、Recorder写入/排空各段耗时。不要未经测量把全部瓶颈归因GPU或磁盘。
- 用计划deadline管理采样周期，明确跳过/追赶策略；不以每轮额外sleep(interval)叠加工作耗时。不能复制旧图像凑FPS。
- 分离传感器实际采样、控制tick、向Recorder发布和可靠写入频率；每10tick发布的10Hz流不写成100Hz丢包。
- 保持控制事件无静默丢失、媒体成对、关闭停车反馈、背压/故障停车、单物理所有者及G1安全阈值。

### 4.3 真实页面输入

- 在服务器运行实际Chromium/WebGL并记录renderer。优化必要的场景负载、阴影/像素比/刷新和异步任务，保持可交互三维能力。
- 真实keydown/keyup、滑块、TCP预览/执行、失焦和断连链在渲染/规划负载下可工作。
- API直接调用可作后端测试，不能代替本Goal实际页面输入验收。不能单靠延长0.5s租约掩盖主线程卡顿；失焦/断连仍必须停止。

### 4.4 数据质量与资格

- coverage依据冻结profile的计划样本数/窗口，分别计生成、交付、可靠写入、唯一帧及跳过原因；不以观测平均rate反推expected。
- 记录sim/wall频率、间隔分布、RTF、观测年龄和因果对齐，未知与不合格给明确原因。
- 建立策略输入allowlist；不要把整个RobotSnapshot/feedback或world_sim位置透给训练。UI的GT_DEBUG视图保留标签并与PolicyObservation隔离。
- 新增按action/observation profile的资格门，分开完整性、物理结果、来源、可观测性与时间质量。原始数据包含Depth/Lidar不能据此否决未来RGB-only输入；第三人称依赖另评。
- 旧G1数据只读，质量报告输出新版本/新目录并绑定输入hash。既有D2=true不能自动升级为训练就绪。

## 5. 有限验证和门槛

先落盘profile和用例。服务器先复现最小导入/模型启动与必要基线，不重复G1全部物理矩阵。修复稳定后仅做一次相关完整软件回归。

短小正式集合：

1. 非零底盘平移/yaw外参、q5/开口不变量与时间因果定向检查。
2. 60s静止录制和60s运动/臂操作录制，实际窗口与初始化/收尾预先声明。
3. 30s服务器浏览器输入序列，含持续驾驶/释放、TCP预览负载、失焦和重连不恢复旧动作。
4. 一个受控过载或写盘故障反例，验证正确降级/停车/不完整状态；恢复后检查新片段。

固定原5Hz图像profile及原分辨率：唯一RGB-D平均wall频率≥4.5Hz，p99间隔≤0.4s；同时报告最大间隔、RTF和sim频率。对10Hz原生发布流报告实际值与时效，不能用自洽coverage冒充目标满足。可选一次有界10Hz图像探针，不作为本Goal完成条件。

租约/停止/位置阈值沿G1，不为通过静默放宽。所有事件/媒体/尾部和输入hash通过；输出的资格仅对声明profile有效。

每个动作/测试/作业有超时，同源同配置同故障不无限原样重试。未达到门槛就报告未完成和瓶颈，不用降分辨率的另一个profile冒充原profile通过。

## 6. 完成和停止

交付本地代码与小型文档、服务器可复制运行方式、部署/环境/模型/配置hash、质量/性能和浏览器结果、原始记录的远端索引、已知限制及G2B/G3接口版本。

至少证明新时间/坐标语义正确、5Hz冻结profile达标、实际页面输入链可靠、停止/录制故障保持正确。代码存在、API脚本成功、GPU可见或历史G1通过不等于本Goal完成。

按Goal工具真实规则处理状态。完成G2A后停止，不自动开始G2B、导航、训练或实车。
