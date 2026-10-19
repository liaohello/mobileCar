# G2A之后的实施计划 v1

> 阶段状态更新：G2B已完成，见[G2B报告](g2b_report.md)。当前路线改见[G2B之后的计划](post_g2b_execution_plan.md)，下一首选为G3A；本页保留历史，不再次启动G2B。

日期：2026-10-17。状态：后续规划；本轮未创建Goal、连接服务器或执行测试。

## 1. 已完成的基线

G1完成操作/物理抓取与原生录制；G2A完成服务器运行、时间/变换、5Hz采集、浏览器可靠性和质量/资格门。

G2A保存身份：

- release：g2a-20261017T163244Z-5cbe582fa6b2
- aggregate SHA-256：5cbe582fa6b266d074d22bd4c120ad381797a440e37834ab8e9a6c7e03558514
- wire：mobilecar-1.0；语义：g2a-1.0
- observation profile：g2a-rgbd-5hz-original
- action profile：g2a-teleop-action-1.0
- 保存结果：55项回归通过；静止/运动分析窗各250组持久RGB-D、5Hz；实际页面与故障门通过。

依据：[服务器报告](g2a_server_report.md)、[接口](g2a_interface_v1.md)、[证据索引](evidence/g2a_server_artifact_index.json)。这是保存结果，本轮没有复跑。

G2A报告中的250样本属于预先声明的5–55秒分析窗，不是将60秒全段改标为250帧。初始化和收尾保留。

增加本规划文档后，当前工作树的新aggregate自然可能不同；G2A hash保持历史身份，新Goal另建release和清单，不重写G2A报告或要求新源码伪装成旧release。

## 2. 推荐顺序

| 阶段 | 目的 | 入口/完成门 |
|---|---|---|
| **G2B：离线数据出口，下一首选** | 从服务器现有记录生成D0/D2开发数据集，验证动作、因果对齐、格式与可追溯性 | [新版Goal](goals/G2B_lerobot_export.md)；官方loader可读、逐行来源可核、原始数据不变 |
| **G3：估计导航** | 雷达/轮/IMU观测适配、无动态真值里程计、TF、地图保存/加载定位、到点与接管 | [导航Goal](goals/G3_server_navigation.md)；独立有限导航矩阵 |
| **G4A：示教采集试点** | 形成任务成功、视图/来源/频率明确的少量完整数据，评估link4相机可观测性 | 使用G2B的资格和出口，不把开发片段直接扩成专家集 |
| **G4B：ACT近场基线** | 先固定底盘、固定动作profile验证学习闭环，再独立评估语言条件VLA | 必须有对应train-qualified数据；先小规模加载/拟合/有界闭环评估 |
| **后续：VLN与联合任务** | 语言子目标、Frontier、多阶段切换、近场重新观察/可达性与有界重定位 | 导航、数据、操作学习各自验收后集成 |

G2B和G3技术上可并行：G2B只读数据、不改backend；G3改导航/后端，分别使用固定release与环境。考虑前两轮执行agent无有效交付，默认先完成G2B，不自动再启动一个G3 Goal。

不在G2B重做G2A性能/渲染验证，不新增示教、不训练、不将导航加入同一完成条件。

## 3. G2B的具体输入

从服务器索引定位，不回拉媒体：

| 输入 | 身份 | 使用方式 |
|---|---|---|
| G2A运动/臂 | run_81ccba839bec4e1991278b913fa2f1b3 / episode_c20c70aa2ada4e4fbd4c1a9ec5672090 | D0/D2开发导出候选，逐行重新检查 |
| G2A静止 | 同run / episode_32975d5ce7eb4b10a58588858de103d7 | 观测和“无示教动作”拒绝对照 |
| G2A故障 | artifact index中的写入故障记录 | 不完整记录拒绝对照 |
| G1历史 | 仅使用现有服务器副本或合成旧schema夹具 | 时间未知/旧版本兼容检查，不为测试上传大媒体 |

具体路径及关键文件hash以docs/evidence/g2a_server_artifact_index.json为准。原始文件只读；报告输出独立目录。

运动/臂Episode任务结果为failure，因为该性能片段没有完成抓取。它可以验证数据出口，但不能改标成功任务或人类示教。

## 4. 三种结论分别交付

1. **format_valid**：目标文件符合锁定的LeRobot格式，官方读取/批加载成功。
2. **export_valid(profile)**：所选行的来源、动作层、时间、缺失处理和输入白名单满足声明的开发出口。
3. **train_qualified(training_profile)**：对特定训练任务，来源、结果、可观测性和覆盖也足够。

G2B允许前两项成立而第三项为false。agent_ui、third_person_3d、失败片段必须保留；不能为了Goal通过改标签，也不要求临时采集人类成功数据。

当前G2A的D0/D2布尔值是候选门，不是上述逐行结论。PolicyObservation构造器也不替代跨流对齐器。

## 5. G2B实现包与范围

### E0：冻结出口

只读盘点已有G2A记录和工具，复用tools/server/remote_job.py、现有源清单和报告路径；不重写部署体系。

锁定一个可取得的LeRobot release/commit、数据格式和独立环境。G2A Python3.10/MuJoCo环境不升级。官方main本次读取声明Python≥3.12；这不等于该main版本一定已在PyPI发布，执行时核对选定包。[官方依赖](https://github.com/huggingface/lerobot/blob/main/pyproject.toml)

G2B默认5Hz出口，不升到10/20Hz。冻结grid origin、可用时刻、最大年龄/跨流偏差、gap policy、动作层、单位与feature键。建议开发阈值：图像年龄≤0.4s、本体反馈年龄≤0.2s、图像/本体capture差≤0.2s；这些是独立的出口门，不是由G2A的p99统计门自动推导。正式导出前固定，超限拒绝/拆段。

### E1：只读Reader和因果对齐

同一run/episode/时钟内，在决策网格时刻选择已经到达消费边界的观测。图像capture、publish/arrival和persist有不同用途：persist证明资产可靠保存，不自动代表在线可用时刻。

记录图像、本体的采样与到达时刻、source sample/control tick和选择偏差。缺值不补零；不做未来反馈插值，不复制旧图像伪造新的有效采样。缺口拆成连续片段并保留父Episode，不能跨Reset/恢复拼接。

D0观测优先使用已记录的轮测量、IMU和可用RGB。名称为base_twist_body的值仍需查来源；若直接由仿真qvel计算，则不当作实测里程计输入。当前激光方向/frame尚待G3验证，默认不纳入D0首版观测；这不阻止body速度动作出口。

### E2：固定动作语义

- D0：实际生效的body速度参考[vx,vy,wz]，m/s、m/s、rad/s；不得用反馈底盘速度作动作。
- D2：执行参考层的五关节绝对参考转换为用户角rad，加米制夹爪目标，六维。模型角→用户角必须绑定零位/方向/模型版本。
- preview、rejected、未dispatch目标不作为有效动作。每行引用request/dispatch/control tick与生效区间。
- 如果记录只能提供不同动作层，必须另定义并命名profile，不能在同一输出列混入宏目标、轨迹参考和反馈。

执行参考型D2是本Goal的开发出口；未来部署策略适配器必须使用相同语义另验，不能直接宣称已有ACT控制接口。

### E3：写入、读取和追溯

通过选定版本的官方writer创建数据，完成所需save/finalize。默认RGB视频或该版本正式支持的图像存储，原始米制Depth/Mask保留在服务器并通过sidecar引用；不把伪彩深度写成RGB观测。

逐行sidecar保存源run/episode/segment、frame/sample、command/tick、时钟/原始时刻、动作层、排除/拆段原因和规则hash。标准数据文件仅保留profile明确选择的观测/动作，GT_DEBUG、物体/世界真值、Oracle不进入特征。

拆段仍属于同一父Episode/同一次源任务，不能把其子段分别分到train/validation。只有一条可用真实源时输出dev集；train/validation可以为空，不为了非空而泄漏或生成新示范。

### E4：有限验收

- Motion分别产生非空D0、D2开发出口；若仅子段合格，报告实际覆盖，不能拿静止填充。
- Static返回无示范动作的明确拒绝/降级；故障/缺时刻/跨流不匹配等反例正确处理。
- 锁定版本官方loader逐样本和小batch读取成功，首/中/末行追溯；shape、单位、动作层、视频帧数/时间一致。
- 原始文件hash前后不变，输出完整manifest/checksum；train-qualified=false可以是正确结论。

验证仅在服务器。默认不启动MuJoCo、浏览器或新的采集，不重跑55项动力学/UI基线。只做出口模块、相关data/contract回归和一次稳定版本的出口验收；有实际公共运行时代码变更才扩大回归并说明。

## 6. agent安排与开销控制

保留可并行方案，但先证明委派有效：

1. 主负责人先冻结Reader→Aligner→Writer小接口及文件所有权。
2. 先给一个执行agent一个可直接交付的小模块/fixture任务。收到可核对文件/补丁后，再并行第二个独立模块；不是先铺开三条等待链。
3. 最多两个同时执行子任务：A读入/对齐，B官方writer/loader；主负责人负责profile、远端运行、集成和验收。
4. 最后可安排一次非作者只读审查。若agent出错、只有澄清/等待而无产物，记录并接管，不循环派发相同空任务。

各阶段只保留能决定下一步的输入、补丁和结果；不重复阅读全部G1/G2A大日志。不自行设置用户未请求的token预算，依靠范围、单次阶段验收和不重复全测控制开销。

## 7. 交付与后续

代码/profile/独立锁文件保留本地，测试、视频/Parquet、数据集、日志在服务器。回传导出清单、资格/排除报告、样例行追溯、loader/JUnit结果和server_artifact_index，不回传完整媒体。

G2B完成后停止。G3导航已有独立提示词，下一次启动时读取G2A最终接口；不将G2B作为导航必须前置。训练提示词在数据采集试点后制定，不把当前开发数据出口直接变成训练实验。
