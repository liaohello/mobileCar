# G2B Goal：服务器离线数据出口 v2

> 已完成的历史提示词：见[G2B报告](../g2b_report.md)。下一阶段使用[G3A](G3A_navigation_foundation.md)，本提示词不再次执行。

日期：2026-10-17。基于已完成G2A收敛。状态：可交给下一执行会话；保存不代表已启动Goal。

---

请在 C:/Users/Administrator/Desktop/embodied_intelligence_proj/mobileCar 创建并持续执行G2B：

**只读服务器现有G2A Episode，实现5Hz的D0速度参考、D2关节/夹爪参考两种开发数据出口，完成因果对齐、逐行追溯、LeRobot正式格式写入与官方loader验证。区分format-valid、export-valid和train-qualified，不新增采集、不训练、不启动导航或仿真。**

## 1. 先读实际基线

完整阅读适用AGENTS.md，以及：

- docs/server_execution_policy.md
- docs/post_g2a_execution_plan.md
- docs/g2a_server_report.md、docs/g2a_interface_v1.md
- docs/evidence/g2a_server_artifact_index.json
- configs/profiles/g2a_rgbd_5hz.yaml、g2a_action_teleop_v1.yaml
- 当前contracts、data/g2a_quality与必要的data代码，不重复全读G1/G2A大日志。

已完成G2A身份：

- release：g2a-20261017T163244Z-5cbe582fa6b2
- aggregate：5cbe582fa6b266d074d22bd4c120ad381797a440e37834ab8e9a6c7e03558514
- wire mobilecar-1.0；语义g2a-1.0；5Hz原分辨率profile。
- motion：run_81ccba839bec4e1991278b913fa2f1b3 / episode_c20c70aa2ada4e4fbd4c1a9ec5672090
- stationary：同run / episode_32975d5ce7eb4b10a58588858de103d7

路径、关键文件hash、故障记录以artifact index为准。G2A motion是agent_ui/third_person_3d，任务failure；D0/D2只为声明profile内的候选，不是人类成功示范或逐行合格证明。静止数据用于无动作拒绝对照。

G1缺capture时间的数据，只能格式/兼容检查，不补造时刻。若服务器没有G1媒体，用合成旧schema反例，不从本地上传大媒体。

## 2. 执行位置、权限与Goal

所有依赖安装/解析、数据检查/转换、测试、视频编码和加载验证在服务器；本地仅编辑/读取源码、profile、锁、清单和小报告。不因远端失败在本地/本机WSL执行。

复用tools/server/remote_job.py和现有部署清单。remote.md只用于解析连接，不输出/上传凭据。新代码冻结新release和aggregate；G2A历史release、报告、源数据保持不变，不重新标注或要求新源码匹配旧aggregate。

专用导出环境与envs/g2a-py310-mujoco316隔离。允许本Goal必要的本地编辑、服务器同步、专用依赖和离线验证，无需常规步骤反复确认；不提交/推送Git、不清理既有环境/数据、不下载模型权重、不上传数据到Hub。

检查当前Goal身份：无冲突时创建本Goal，不自行设token预算；已有本Goal则继续，不以完成其他未完成Goal腾位置。状态依实际工具规则处理。

## 3. 小范围并行，先验证委派有效

主负责人先冻结Reader/Aligner/Writer边界、精确文件所有权和一个fixture。

先给一名执行agent一个小模块或fixture任务；得到可核对文件/补丁后，最多并行两个执行agent：

- A：只读Reader、时钟/有效区间索引、Aligner及自己的测试。
- B：官方writer、媒体/feature映射、loader核对及自己的测试。
- 主负责人：profile、资格、CLI、独立依赖锁、服务器作业、集成和验收。

建议路径src/mobilecar/data/export/、tools/export/、configs/export/、tests/export/，各文件先明确所有者。不改backend、UI、ROS或公共运行时配置；需要既有data入口/公共类型变化由主负责人集成并说明。

agent出错、仅等待/澄清且无实际产物时记录并接管，不循环重新派发同一空任务。阶段末可做一次非作者只读审查，有效审查与主负责人自检分开表述。无有效委派时串行完成，不声称三线独立实现。

## 4. E0：冻结环境和出口profile

服务器先核对源数据可读、关键hash和现有质量报告，不运行MuJoCo或renderer probe。

选定一个实际可取得的LeRobot release或commit，记录Python、torch、编解码器及格式版本并锁定；不无约束安装main。官方参考：

- https://github.com/huggingface/lerobot/blob/main/pyproject.toml
- https://huggingface.co/docs/lerobot/en/lerobot-dataset-v3

规划时官方main声明Python≥3.12；不等于其当前main版本必已发布到PyPI。执行时核定一个兼容版本和独立环境，不为导出升级G2A Python3.10/MuJoCo。默认只读文件，避免导入整个仿真包；不使用GPU渲染或模型推理。

首版5Hz，不升级到10/20Hz。profile至少固定：

- feature名称、顺序、shape/dtype、单位、坐标与归一化策略；
- 动作层及生效区间；
- grid origin、消费边界/可用时刻、最大年龄、跨流偏差、gap policy；
- 来源/结果/阶段规则、输出用途和资格schema版本。

开发初值：图像年龄≤0.4s、本体反馈年龄≤0.2s、图像/本体capture偏差≤0.2s。正式导出前固定；它们不是G2A p99间隔统计的替代品。超过时拒绝/拆段，不补零、借未来状态或改标签求通过。

## 5. E1：只读解析与因果对齐

源Episode只读；所有质量重算和导出写服务器新的export/report目录。不要调用默认回写原目录的旧validate/eligibility入口；复用时显式关闭回写或用只读包装。

每个决策网格时刻t，选择同run/episode且在t之前已经到达消费边界的有效样本。使用capture sim time/control tick与monotonic明确映射，不能直接混比时钟。

- capture是图像对应物理时刻，publish/arrival决定在线可用性；
- persist证明可靠写入，不能自动作为所有消费者的在线时刻；
- 不把未来反馈插值冒充在线可用观测；
- 不复制同一旧图像伪造新的有效采样；缺口显式拒绝或拆为连续片段；
- 保存窗口选择及首尾裁剪原因，不默认删除初始化/收尾，不能跨Reset/恢复拼接。

G2A的PolicyObservation构造器不是跨流匹配器。每行必须独立检查样本身份、时钟、可用性、新鲜度和对应反馈。

## 6. E2：D0/D2的具体语义

### D0开发profile

动作是实际生效的body速度参考[vx,vy,wz]，单位[m/s,m/s,rad/s]、base_link。使用批准/dispatch/执行参考的明确生效区间，不用测量底盘速度作动作标签。若从wheel target推导body参考，记录所用运动学/参数hash和派生规则。

观测首版使用可验证来源的轮测量、IMU和需要的RGB。body_twist即使名称在白名单中，也须核对来源；由MuJoCo qvel直接算出的底盘速度作为特权诊断，不冒充轮/IMU估计。G1/G2A雷达几何尚待G3专项验证，默认不纳入首版D0特征。

### D2开发profile

动作固定选执行参考层：五轴arm_model_reference_rad转换成用户角rad，另加gripper_width_target_m，共六维。绑定五轴顺序、零位、方向和模型/配置hash，不能混入宏目标或实测反馈。

观测为五轴实际用户角、实际夹爪开度及命名的link4 RGB；原始Depth/Mask通过sidecar保留或明确profile使用，不把伪彩深度当RGB。

preview/rejected/未dispatch目标不是已执行动作。STOP-only静止段不能成为操作示范；真实运动中的合法零速/保持段依据生效命令追溯，不能一概删除或填假动作。

执行参考型D2不等于已完成策略部署适配；后续训练/在线控制须使用相同动作语义另验。

## 7. E3：官方写入、来源和资格

使用锁定版本官方writer并完成必要save/finalize，通过官方LeRobotDataset读取和小batch加载。默认选择该版本正式支持的RGB视频或图像存储，记录编解码器、fps/时间重基准、分辨率与帧对应；有损视频不宣称字节等同原PNG。

逐行sidecar至少保存run/episode/recording_segment、source frame/sample、request/dispatch/control tick、生效区间、原始时间、选取偏差、动作层、规则hash和拆段原因。

输出三类独立结论：

1. format_valid：标准格式可被官方读取；
2. export_valid(profile)：逐行来源/动作/时间满足本开发出口；
3. train_qualified(training_profile)：任务/来源/可观测性/覆盖满足指定训练用途。

本Goal允许前两项通过而第三项为false。现有agent_ui、third_person_3d、task_result=failure原样保留；不伪造人类来源、成功标签、腕部可观测性或训练资格，也不因缺人类成功示范擅自新增采集。

使用明确输入allowlist和来源检查；world_sim、物体真值、Oracle、整对象diagnostics不进策略特征。报告/sidecar可以引用隔离评分，但不能变成观察特征。

拆段保持父Episode身份，同父源的子段不能跨train/validation。仅一条合格真实源时输出dev集即可；用独立合成Episode测试split规则，不能将同一真实Episode拆开凑非空训练/验证集。

D1语言子目标、D3联合训练、真实任务泛化均不在本Goal。

## 8. E4：有限的服务器验收

固定小输入集合：G2A motion、stationary、一个故障/不完整记录，以及少量合成负例；不重新录制。

必须交付：

- motion产生非空D0和D2开发出口，官方loader逐样本/小batch可读；
- 首/中/末样本核对shape、单位、时间、动作层和source-row映射；全部行通过自动身份/因果检查；
- stationary为无示范动作的拒绝/观察对照，故障记录不得进入完整出口；
- 负例覆盖未来到达样本、错时钟/跨Episode、缺媒体、长缺口、未知动作、rejected/preview标签、真值混入和父Episode split泄漏；
- 源文件hash前后不变，输出manifest/checksum完整，源资格和输出资格不混淆；
- train_qualified=false、空train/validation分区可为正确结论，不阻塞本开发出口验收。

只运行export模块和受影响data/contract定向回归，并在代码稳定后做一次出口验收。不重跑55项动力学/UI基线、60s采集、renderer或浏览器；没有运行时变更或相关失败证据，不扩大验证。

任何CLI退出0只是执行状态，必须读取profile gate/逐行错误/loader结果。无法得到非空D0/D2或来源因果不明时报告未完成，不补造数据。

## 9. 交付和停止

本地保留代码、profiles、独立依赖锁、g2b_report/reproduction和小型证据索引；原始记录、视频/Parquet/数据集和完整日志留服务器。复用现有同步/作业工具，回传遵循20MB默认上限，不拉媒体。

记录最终release/aggregate、环境、输入数据hash、profile hash、有效/排除行数和原因、format/export/train三个结论、官方loader结果及复算命令。

常规实现/修复在范围内自主推进，不重复询问许可；保留用户未提交工作，不自动提交推送。任务完成后将本Goal按实际工具规则结束并停止，不自动进入G3、示教采集、训练或实车。
