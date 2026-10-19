# G4A-1之后：真人示教采集、数据划分与正式出口

日期：2026-10-18。状态：**规划，尚未启动G4A-2**。用户已选择真人遥操作：用户本人观察并控制，agent准备界面、记录、检查与导出。本轮不创建Goal、不连接服务器、不运行测试。

执行正文：[G4A-2真人示教与数据出口](goals/G4A2_human_demonstrations_export.md)。

## 1. 已完成基线

G4A-1 release：g4a1-20261018T125226Z-134032d390f8。

Aggregate：134032d390f83a4875e8bd936b9dead4fec9fbcd55e6c251cfde3ff5c7da4747。

六例符合各自预期，三例抓取抬升/双侧保持通过；原生format/data通过，最终受影响回归130 passed。parent split、正式导出和训练资格尚未建立。依据：[报告](g4a1_report.md)、[接口](g4a1_interface_v1.md)、[就绪报告](evidence/g4a1_readiness.json)及[完成审计](evidence/g4a1_completion_audit.json)。

这些是已有服务器结果，本轮没有复跑。G1–G4A-1原始数据、报告、资格和已知例外只读保留。新Goal省略token_budget，核对返回tokenBudget=null。

## 2. 脚本数据是什么，以及此次选择

有两种完全不同的用途：

- 接口fixture：合成或随机输入，用来检查维度、单位、拒绝和序列化；不是机器人任务示教。
- 流程脚本示范：程序实际驱动物理机器人完成观察、接近、闭爪、抬升、保持，记录真实观测与动作。它可以是真实任务数据，但来源是scripted，输入权限也要如实说明。

现有G4A-1 runner属于真实流程脚本开发验证，但使用已知case坐标、world_sim规划和接触状态。它不能直接改名为视觉专家或人类示教。

用户已选择下一阶段由真人实际操作。因此**G4A-2不开发批量自动专家来代替用户**。允许机械臂安全规划、轨迹插值和5Hz发送器作为明确的遥操作辅助；操作者、辅助算法和自动化传输分别记录，不能把每个周期的重复发送都说成一次新的人类决策。

## 3. 新阶段需补齐的具体接口

以下是训练数据阶段的新增要求，不改写G4A-1的有限dev完成结论：

| 当前实现 | G4A-2需要 |
|---|---|
| main.js的G4A1动作source固定为agent_ui；G4A1合同没有human_teleop动作来源 | 新版本human来源合同；UI事件、operator、自动辅助及帧来源一致核对 |
| runner按已知case offset生成world_sim目标，并读取bilateral_contact | 真人只看声明RGB-D/本体视图；Oracle仅在独立评分中使用，不向操作页面透露目标/接触真值 |
| teacher_plan_id使backend执行缓存轨迹；普通六维输入走另一条路径 | 训练动作走普通六维部署路径；辅助轨迹展开为逐时刻目标，不依赖隐藏plan ID |
| UI把decision/issued填成arrival+1µs | 记录真实UI事件、服务端接收/采样/发送时间和时钟域，不倒填决策时间 |
| readiness的observability仅检查全图Depth有效率；causality只检查事件种类 | 目标/夹爪分阶段可见性及逐动作因果、回执/应用tick审计 |
| G2B按旧actuator_reference、单父Episode与dev导出，Depth为sidecar | 独立G4A-2目标层reader/多父采集manifest、分组split和RGB-D loader验证 |

本地核对入口：tools/g4a1/run_formal.py、src/mobilecar/ui/web/main.js、src/mobilecar/contracts/g4a1.py、src/mobilecar/backends/mujoco/backend.py、src/mobilecar/data/g4a1_readiness.py、src/mobilecar/data/export/。

## 4. 实施顺序

| 阶段 | 结果 | 谁参与 |
|---|---|---|
| H0 采集准备 | 独立真人采集页、来源/真实时钟、5Hz普通六维目标流、Depth/动作出口小型技术验证 | agent实施；自动化记录仅dev |
| H1 真人试点 | 2条完整抓取成功、1条主动取消/失败；检查真实视野、动作和追溯 | 用户实际操作，agent记录检查 |
| H2 冻结采集设计 | 布局组、任务profile、数量、split、动作层/模态及资格规则 | agent提交可审阅清单，按用户实际安排采集 |
| H3 小批量真人采集 | 默认24条成功示教，所有失败/取消也保存 | 用户操作，可分3批，每批约8条；以试点测得耗时估计时长 |
| H4 导出与资格 | 官方LeRobot读全样本/batch、RGB-D加载、父任务防漏、训练入口审计 | agent服务器执行 |

先把H0/H1打通，不在错误source、隐藏轨迹或视野不充分时直接收集24条。H1默认只进dev，不借三条试点凑正式数量。

“真人试点与采集”确实需要用户参与，不能无人值守完成。用户不在线时先完成不依赖真人的源码、fixture、导出和检查准备；随后提出一次具体操作请求并等待，不循环空跑，不冒充用户，不把未采集部分标完成。

## 5. 真人采集方式

- 使用服务器上的采集页面/浏览器及既有远程桌面访问方式。安装、自动浏览器、渲染和测试都在服务器；不因访问失败回退本地测试或擅自开放公网服务。
- 页面只显示link4 RGB-D、本体/局部FK、安全状态和必要操作控件。独立页面及观测白名单，不让第三人称真值画布、world_sim坐标或实时Oracle继续同屏/同响应输入。
- 用户选择目标、移动机械臂、决定闭爪和抬升。可用相机像素/Depth/局部FK辅助预览，但不能自动吸附仿真物体中心。
- 每条记录含operator/session、human事件、辅助模式、control generator、使用的图像sample、原始意图和实际六维流。没有真实用户操作确认的自动化记录不得进入human集合。
- 自由底盘停车、link4挂载、五轴/独立开合及G4A安全门继续保留；不加入导航前置或联合移动抓取。

## 6. 动作与模态合同

正式动作仍是五轴绝对用户角rad加夹爪宽度m。人类TCP/滑块意图可经安全规划生成5Hz目标流，但实际下发和数据标签都必须是同一普通六维接口，teacher_plan_id为空；不能把整段轨迹终点复制到所有帧，不能用执行后反馈冒充目标。

分别记录human_intent_time、server_receive、observation_received、control_decision、approved/dispatch/actual tick。冻结“显示完成sample ID→human intent ID→服务端接收→控制帧→应用tick”的关联回执链。UI与backend时钟不同则分别保留或建立有证据的映射；不拿performance.now直接减time.monotonic，不制造arrival+1µs时间。无法计算的跨域延迟标UNKNOWN，不能仅按异域数值排序判因果通过；有跨域时限门时须另证映射或保守时延界限。

本批声明RGB-D+本体用途；不直接宣布RGB-only ACT资格。保留原始米制Depth/valid mask，模型输入存储与批量加载在采集前用小样本证明。优先验证官方数值Array特征；若采用lossless Depth sidecar与显式RGB-D wrapper，则将官方核心loader和扩展loader结果分开报告。Depth不能伪彩图冒充RGB，也不能只存在原始目录便称模型能够读取。

LeRobot仍优先复用0.4.4专用导出环境和锁；不升级G2A MuJoCo/runtime。该版本官方ACT配置只支持单观察步，RGB-D通道/预处理不应被说成自动兼容；本Goal只验证数据读取和张量合同，不训练或运行策略。

## 7. 首批规模、覆盖与划分

默认正式目标：**24条成功真人示教**，不把这一数量解释为学习效果保证。

- 12个预先冻结的合法初始布局/起始姿态family，每个2条成功；
- 8个family分train：16条；2个family分validation：4条；2个family分test：4条；
- 在小工作区内覆盖位置和合法观察/起始姿态变化，不重复播放同一轨迹填数；
- 正式总尝试默认最多36次、每family最多3次；失败不覆盖，不能无限挑成功；
- 同一父任务、恢复、裁剪、重复导出和同family不能跨split；输出子段不算独立父Episode；
- test用于后续评估，不据其结果选择训练参数；所有归一化/采样统计只用train，不能直接使用全包统计造成泄漏。

若用户在正式冻结前改变数量/时间安排，以用户要求为准并相应重定用途门。未达覆盖或数据质量时保持资格false，不临时降低门槛。用户可以分批操作；实际耗时以H1试点为依据，不承诺固定每条耗时。

## 8. 验收与完成边界

采集数据以原有抬升≥0.05m、双侧有效保持≥2s及停车/接触门作独立物理评分。保留人类成功/失败自评，两者不相互替代。

新资格报告分别给出format_valid、export_valid、physical success、source provenance、learner modality/可观测性、action deployability、causality、parent/family split与training_qualified。G4A-1的布尔值不直接继承；training_qualified只对明确命名的人类RGB-D/六维用途profile成立，不代表策略效果、RGB-only、语言VLA或实车。

必需负例包括：自动数据冒充human、含teacher_plan_id隐藏轨迹、未来/伪造时间、跨Episode/epoch、媒体缺失、Depth错单位/错sample、父任务跨split、同family跨split及只用全图Depth有效率冒充目标可见。

官方writer/finalize、全样本loader、batch、分split读取和逐行来源映射均实际在服务器验证；Depth/Mask也验证张量、单位、有效编码及对应sample。先排空Recorder、完成尾部/原生报告和首次评分后封存，以封存manifest为基准核对后续只读审计/导出前后hash一致；新资格、split和出口报告写外部派生目录。

优先在H0/H1完成代码稳定和出口验证后再冻结正式采集release。采集和导出身份分别保存；若采集后只修离线代码，可用新导出release只读重算，并核对采集runtime/model/profile子集兼容，不让用户无故重录。改变动作、相机或runtime语义则创建新cohort，不给旧记录追溯换标签。

本地保存源码、配置/锁、manifest、精简报告和默认≤20MB证据。原始媒体、数据集、完整日志留服务器。共享模块未受影响时不重跑导航矩阵；有实际改动才做对应回归。

完成后停止。下一步G4B再单独验证模型/Depth适配与训练；本Goal不训练、不上传Hub、不进入实车。

## 9. 多agent与官方依据

主负责人独占公共合同、backend/应用装配、时间/权限、服务器队列和最终验收。A负责真人采集界面/意图来源，B负责多Episode导出与RGB-D读取，C负责分组、资格和证据；每文件一个所有者。先派一个含具体路径与内容的小任务，确认实际产物再扩大，无产物如实接管，不继续循环空lane。一个物理实例。

官方核对依据：[LeRobot 0.4.4数据特征转换](https://github.com/huggingface/lerobot/blob/v0.4.4/src/lerobot/datasets/utils.py)、[ACT配置](https://github.com/huggingface/lerobot/blob/v0.4.4/src/lerobot/policies/act/configuration_act.py)、[ACT视觉实现](https://github.com/huggingface/lerobot/blob/v0.4.4/src/lerobot/policies/act/modeling_act.py)。文档和源码支持技术设计，正式兼容性仍须用服务器锁定环境的小样本验证。
