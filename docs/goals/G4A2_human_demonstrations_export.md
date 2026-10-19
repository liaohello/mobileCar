# G4A-2 Goal：真人遥操作示教、分组划分与正式数据出口

日期：2026-10-18。状态：**待用户启动的执行提示词**。保存本文件不等于已创建或启动Goal。规划依据：[G4A-1之后的计划](../post_g4a1_execution_plan.md)。

---

请在 C:/Users/Administrator/Desktop/embodied_intelligence_proj/mobileCar 创建并持续执行G4A-2：

**复用已完成的G4A-1，在服务器补齐真人采集界面、真实来源与时钟、普通六维目标流及RGB-D出口；先由用户本人完成三条试点，再冻结并采集首批真人示教，完成父任务/布局组划分、官方LeRobot全样本读取及明确用途的数据资格审计。满足合同后交付并停止。**

这里的真人是用户本人观察画面并控制。agent负责工具、引导、采集、检查与导出，不操纵浏览器完成抓取后声称是人类操作。允许明确标注的安全规划、插值和发送辅助；不得使用特权任务脚本代替用户。

## 1. 读取基线与Goal身份

先读取AGENTS.md、docs/server_execution_policy.md、docs/post_g4a1_execution_plan.md、docs/g4a1_report.md、g4a1_interface_v1.md、g4a1_reproduction.md、g4a1_failures_and_fixes.md及evidence下readiness/formal_report/completion_audit，再按改动读取G2A时序、G2B出口和现有实现。不要为阅读材料重跑历史矩阵。

最近完成基线：

- release：g4a1-20261018T125226Z-134032d390f8；
- aggregate：134032d390f83a4875e8bd936b9dead4fec9fbcd55e6c251cfde3ff5c7da4747；
- 六例符合各自预期、最终受影响回归130 passed；
- 原生format/data通过，但parent_split与正式export未评估，training_qualified=false。

G1–G4A-1已完成，不重新激活旧Goal，不改写历史数据或资格。G3B的TF补充判定和contacts.py历史provenance例外继续按原报告保留。

先get_goal。同范围无预算active Goal可续作；其他不同未完成Goal先保留并说明冲突，不伪造完成以腾出Goal。新建G4A-2时只提供objective，**完全省略token_budget字段**，不传0、null或任何自拟数值。创建后立即get_goal，核对tokenBudget=null、remainingTokens=null；如出现数值上限，先说明真实字段/来源，不能继续消耗后再归因默认。不给工作包另建带预算子Goal，不修改Codex内部数据库。

## 2. 服务器执行与协作

本地仅读/编辑源码、资产、配置/锁、manifest和精简证据。依赖安装/解析、构建、lint/typecheck、测试、MuJoCo、渲染、自动浏览器、数据分析/重算/导出全部在服务器；连接失败不得改用本地或本机WSL。沿用remote.md必要连接字段及tools/server/remote_job.py，不打印/提交/同步凭据，不执行配置中任意命令文本。

本地是源码主副本；服务器每次执行使用唯一release ID和逐文件哈希，不在release目录中直接修代码。最多一个物理仿真实例、一个mj_step推进者；主负责人统一排队并记录PID、端口、设备、时限和输出目录。只清理本任务进程，不删除旧环境/runs，不自动提交或推送Git。

主负责人独占公共合同、backend/应用接线、权限与时钟、依赖、服务器作业和最终验收。A负责独立真人UI与意图模块，B负责独立多Episode出口/RGB-D加载模块，C负责独立分组、资格与追溯工具。先列每文件唯一写入者，现有公共文件由主负责人集成。

先派一个路径/内容明确的小任务，实际核对文件和回执后才扩大并行。没有产物就如实记录并接管，不循环派空lane，不冒充独立实现或独立复核。代码工作可并行，物理和自动浏览器作业统一排队。

## 3. H0：先实现采集入口与小型技术验证

### 3.1 真人观察/操作界面

建立独立采集页面和服务端观测白名单，只提供link4 RGB、米制Depth/有效mask、本体关节/宽度、局部FK、安全与记录状态及必要控件。不能只隐藏3D画布却继续把world_sim物体位置、实时Oracle、真实接触/物体抬升或第三人称图像传给该页面。调试界面可保留在另一路径，但同条正式示教不能混用。

用户可用关节/夹爪控制、局部TCP点动或基于真实像素/Depth的安全预览，自己决定接近、下探、闭爪、抬升与结束。不得自动吸附已知物体中心、读取case offset代替视觉选择，或把Oracle作为操作反馈。后端碰撞/停车安全检查仍保留，其介入单独记录。

相机保持link4，joint5旋转夹爪，独立两指开合；自由底盘显式停车、有限力/真实碰撞不变。禁止改挂TCP、增外部相机、运行中qpos搬运、焊底盘、附着物体或改物理来制造成功。五轴继续位置优先/受限姿态，不承诺任意6D姿态。

若目标或夹爪不可见，先在既有挂载内调整合法观察/机械臂起始姿态；不能用第三人称/特权坐标补操作后仍给受限视图资格。视野不能满足则停在试点问题报告，不批量采集。

### 3.2 来源、意图与普通六维动作

新建版本化G4A-2合同，不把G4A1的agent_ui记录重标human。区分operator_kind、operator/session、executor/assist mode、control_generator、automation工具来源、human意图及privileged input声明。真实人类操作有明确会话确认和输入事件关联；仅source字符串或下拉选项不足以证明真人。

所有自动化fixture、agent浏览器试验和脚本Episode保持automation/test/dev来源，不能计入真人试点或正式数量。人类选择的辅助规划如实记为planner_assist，不假称纯手动逐帧控制。

动作统一为[q1,q2,q3,q4,q5,gripper_width]，绝对用户角rad、宽度m。实际下发与训练标签使用同一普通六维路径；teacher_plan_id为空。辅助规划在发送侧展开为有速度/加速度限制的5Hz目标流，每个目标具有独立序号/有效期/回执。禁止用隐藏缓存轨迹执行、却给学习器只存整段终点；也不能把执行后反馈冒充目标。

人类输入事件与5Hz控制帧分开：持续持有同一意图可以生成多帧，但不是每帧都有新的人类决策。记录human_intent_time/原始输入、所见display sample、服务端接收、控制采样/生成、approved、dispatch、reference及首次实际应用tick。冻结“显示完成sample ID→human intent ID→服务端接收→控制帧→应用tick”的关联回执链；显示完成不等于声称证明人类视线。浏览器与后端时钟域分别保留，或建立有证据的映射；禁止arrival+1µs倒填、直接比较不同单调时钟或使用未来观测。无法计算的跨域延迟标UNKNOWN，不凭异域数值排序判通过；存在跨域时限门时须证明映射或保守时延界限。

同时保留原始多频记录。固定5Hz学习样本采用明确的因果窗口：图像新鲜度不超过0.4s、本体不超过0.2s、图像/本体skew不超过0.2s；同capture FK可追溯。学习观测选择以实际控制生成/消费时刻为准，不能以事后收到的图像倒配动作。人类原始意图实际基于哪张画面另行保存，不声称人已看过尚未显示的新帧。

失焦/断连、Stop、传感器失效、记录写入失效和Reset撤销旧许可，停止目标生成并进入既有安全停车/保持路径，不自动开爪。后台5Hz发送器不能在无人在线时自行续租旧人类意图或继续旧规划。恢复要显式新许可，旧epoch/session/帧/预览继续拒绝。

### 3.3 在真人批量前证明出口可行

复用锁定的lerobot==0.4.4导出环境和G2B官方writer/finalize/loader经验；新建G4A-2目标层profile/多父reader，不破坏G2B旧actuator_reference/dev出口。MuJoCo runtime与LeRobot依赖继续隔离。

先用明确标automation/dev的小样本验证六维动作、多Episode和RGB-D全链路，不能等用户录完才发现Depth无法读。优先验证官方数值Array特征的米制Depth与mask；如不适合，则冻结lossless sidecar+显式RGB-D wrapper方案，分别报告官方核心格式门和扩展模型输入门。不得把伪彩Depth当RGB，或只在原始目录保留Depth便宣布学习器可读。

记录RGB/Depth/Mask的shape、dtype、单位、NaN与valid编码、sample一致性、批量张量和预处理版本，抽查首/中/末数值。RGB-D是本批声明的输入；不宣布RGB-only ACT资格。ACT模型Depth适配、单/多观察步、训练与策略推理留到G4B；本Goal只证明明确的数据张量与读取合同。

## 4. H1：三条真正的真人试点

在独立工作完成后，给用户一次具体采集请求：服务器访问方式、当前视图、开始/停止/取消方法、预期三条任务和耗时估计依据。使用既有远程访问，不擅自开放公网服务，不让用户本地启动仿真/构建。

用户本人操作两条成功抓取及一条主动取消或失败。每条封存来源、输入、显示帧、实际目标流和完整原生记录，全部仅dev。agent可指导控件使用和安全流程，不能在这三条中代替用户移动/闭爪，也不向用户提供仿真真值目标。

核对OBSERVE/pregrasp、CLOSE、LIFT、HOLD阶段实际显示的目标/夹爪可见性、遮挡与Depth有效区域。不能仅凭全图Depth有效比例判定可观测；允许保存用户对实际帧的标注及精简样例，遮挡/离画或不确定时按预先声明规则拒绝或降级。

逐动作追溯输入/辅助生成→观测到达→控制帧→许可→dispatch→首次应用tick，不能只检查事件类型集合存在。确认普通六维动作确实应用且能独立消费，无隐藏plan依赖。检查取消/断连及新许可恢复；完整性与任务成败分开。

试点若不通过，先修最小问题并按受影响项重验，不继续收集批量。使用实际试点时长估计正式采集安排，不承诺固定几分钟录完。

用户不在线时继续所有不依赖真人的工作，随后保存明确恢复点并等待真实输入；不要循环空跑、合成human、把准备完成说成采集完成，或擅自标记Goal paused/complete。状态修改遵守Goal工具实际规则。

## 5. H2/H3：冻结正式小批量方案并采集

默认方案是24条成功真人Episode：12个合法初始布局/起始姿态family，每family 2条；train 8个family/16条，validation 2个/4条，test 2个/4条。H1三条dev不凑正式数。该数量是首批工程合同，不保证学习效果。

先在小工作区冻结12个family的参数/容差、split、任务指令、随机种子/初始设置规则、操作者会话、辅助方式、可观测门、成功门与用途profile，再采集。布局可沿已验证范围包含合法位置/起始姿态变化，不直接把未验收大范围或新物体纳入。agent可在Episode开始前合法设置场景，但不能给操作页面泄漏物体真值。

用户可在正式冻结前调整数量/时段，随之明确修改覆盖/用途合同；未收到调整则沿用上述默认。可分3批约8条，不把等待真人当无限运行仿真的理由。单Episode默认安全超时为300s仿真/420s墙钟，正式profile中记录；超时按失败/取消封存。若试点表明需要更合理时限，在正式采集前依据实测冻结，不能事后改已执行案例。

正式总尝试默认最多36次、每family最多3次，所有失败/取消保留并计数。未在上限内达到目标，就报告实际数量/原因及未满足门，等待用户决定补采或修订下一批，不无限追成功、不偷偷放宽质量门。尝试上限和操作安全超时不是token预算。

同一父任务、恢复/retry链、裁剪、重复导出及同family不跨split；分组清单在采集前冻结。不能把一条长记录切成多段凑独立父Episode。成功/失败的实际比例完整报告；正式失败记录作为诊断用途独立保存，不混进成功示教集，仍遵守同family split。

test保持隔离，不能依据其结果调训练参数；后续归一化/采样统计只由train拟合。若官方metadata默认含全包stats，明确不将其作为训练归一化依据，另交付train-only stats及选择规则。

## 6. H4：只读导出与资格验收

源Episode先排空Recorder、完成记录尾部/原生报告与首次评分，再生成封存manifest。封存后validator/Oracle/出口只读或写外部派生目录，不改原始文件；以封存manifest为基准核对后续审计/导出前后SHA-256一致。派生manifest记录采集release、model/scene/camera/profile、operator/assist、family、parent、split、结果及文件来源，不把新split/出口统计补写入封存源。

正式出口至少完成：

1. 官方writer写入/finalize、官方loader读取全部RGB/本体/六维动作样本，各非空split实际读取并通过batch；不是仅检查文件存在或取一个样本。
2. 声明的RGB-D loader读取全部Depth/Mask，sample对应、单位、shape/dtype/有效编码正确，批量张量合同通过；若使用wrapper，官方与wrapper两项结论分开。
3. 首/中/末和所有关键闭爪/抬升/保持段可映射到父Episode、human事件/辅助目标、图像/本体sample、命令序号、权限和实际应用tick。
4. 无重复图像凑5Hz、未来匹配、缺帧伪造、动作/反馈混用、跨epoch拼接；丢帧按冻结规则拒绝或切出明确子段，保留原时间和父身份，不静默压缩时间，不丢掉关键失败段后假称完整示教。
5. 父任务/family防泄漏、24条与分split数量、覆盖、train-only stats和全部原始尝试统计正确。

至少有以下有意义的拒绝测试：自动化冒充human、teacher_plan_id隐藏轨迹、未来/倒填时间、跨epoch/Episode、失焦后发送器自续租、缺RGB/Depth/Mask、Depth单位/样本错配、父任务或family跨split、只有全图Depth有效而目标不可见。测试和技术fixture全部服务器执行且明确dev来源。

物理评分沿用G4A-1冻结门：抬升至少0.05m、双侧有效保持至少2s及原停车/接触安全约束，逐例给出真实结果；人类自评不代替独立物理证据。

最终报告分别给出原生format_valid/data_complete、physical result、human source provenance、观测/模态资格、普通六维action deployability、逐动作causality、parent/family split、export_valid及training_qualified。训练资格仅针对明确命名的human RGB-D/proprio/5Hz六维数据入口profile，所有必要门满足才可true；不代表ACT效果、RGB-only、语言泛化或实车可用。未达门时保留false和原因，不能因报告齐全标Goal完成。

## 7. Release、回归和证据

优先在H0/H1稳定采集/出口代码后冻结正式采集release并逐文件核对，再录正式集。采集身份与导出身份分别保存；只修离线代码可用新export release只读重算，核对采集runtime/model/action/camera/profile子集一致，不要求无关的重复真人录制。若改变这些运行语义，则新cohort/新采集release，旧记录不追溯换标签。

最终源码/配置清单、实际服务器采集身份、出口身份、测试/loader报告各自可核对，不能把新aggregate倒填成旧采集身份。公共控制/记录改动做必要受影响回归；无关导航、建图、RViz和历史出口矩阵不自动重跑。所有必需受影响检查必须通过，已知历史例外如实列出。

交付：

- docs/g4a2_report.md、g4a2_interface_v1.md、g4a2_reproduction.md；
- docs/g4a2_human_operator_guide.md、g4a2_demo_record.md、g4a2_failures_and_fixes.md；
- 采集/动作/观测/出口/资格profile、冻结family与parent split manifest；
- docs/evidence/g4a2_readiness.json、g4a2_collection_report.json、g4a2_export_report.json；
- docs/evidence/g4a2_server_artifact_index.json、g4a2_completion_audit.json及最终JUnit；
- docs/workpackages/G4A2_agent_receipts.md，如实记录实际产物与主负责人接管。

本地仅回传精简报告、hash、必要截图/样例，默认≤20MB；原始RGB-D/Mask、Episode、完整数据集和日志留服务器，以路径/大小/hash索引。恢复点至少记录完成阶段、实际缺口、下一作业和是否等待真人。

## 8. 完成和停止边界

只有H0准备、H1真人试点、冻结正式集、来源/物理/时序/模态/划分/出口门、必需回归及证据完整，才能标记G4A-2 complete。仅工具就绪、fixture通过、用户暂未操作或数量不足都不是完整交付。

结束时只清理本Goal作业并核对无遗留物理进程，给出release身份、数量/失败率/资格结论、实际限制及文件入口。全过程不下载训练权重、不训练/运行策略、不Hub上传、不执行联合移动抓取、导航扩展或实车；不自动进入G4B。
