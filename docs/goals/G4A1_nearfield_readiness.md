# G4A-1 Goal：近场抓取与示教采集准备

日期：2026-10-18。状态：**已完成，以下保留原执行合同**。结果见[服务器报告](../g4a1_report.md)及[完成审计](../evidence/g4a1_completion_audit.json)。不重新启动本Goal；后续见[G4A-2计划](../post_g4a1_execution_plan.md)。

---

请在 C:/Users/Administrator/Desktop/embodied_intelligence_proj/mobileCar 创建并持续执行G4A-1：

**复用G1–G3B已完成的物理后端、安全控制与原生记录，在服务器完成固定底盘近场方块抓取、受限link4 RGB-D/本体操作视图、可部署六维动作接口、少量来源明确的开发示范及采集就绪报告。通过同一冻结release上的六个正式案例与必需子门后交付并停止。**

本Goal不依赖真人实时参与；agent_ui/scripted试点只证明操作/采集管线，本轮training_qualified=false。实际人类示教、批量训练数据、LeRobot正式出口、训练及导航到抓取的完整交接留给后续独立Goal。

## 1. 必须阅读与基线

先阅读AGENTS.md、docs/server_execution_policy.md、docs/post_g3b_execution_plan.md、G3B report/reproduction/completion audit及failures_and_fixes，以及G1抓取报告、G2A interface/profile、G2B report和现有数据出口实现。

最终导航基线：

- release：g3b-20261018T103442Z-a4081b0147bd；
- aggregate：a4081b0147bd871a5161aa2b574f534dec44fa66cf6a47969f94aab63a51d9a5；
- 10例符合各自预期、7例Action SUCCEEDED、最终受影响回归123 passed；
- G3B导航Episode training_qualified=false。

只承接实际已验收范围，不外推Room B真正L转弯、任意移动臂姿或可抓取性。严格TF检查与补充单源修正判定、contacts.py历史provenance排除如实保留，不重写G1–G3B报告、manifest或原始数据。

## 2. Goal身份与无自设预算

先get_goal：已有同范围且无预算的active Goal则继续；其他不同未完成Goal保留并报告冲突，不伪造完成。已complete的G3B不重新激活。

新建G4A-1只提供objective，**完全省略可选token_budget字段**。不传0、1、1000000或任何自拟数值，不把token_budget=null作为调用参数。创建后立即get_goal，核对返回tokenBudget=null；remainingTokens=null是无自设上限。若仍出现数值上限，先说明真实字段/来源，不继续消耗后才解释为默认值。子agent是工作包，不需要再创建子Goal。

无自设token预算不取消作业超时、租约、停车门或平台限制。平台中断时留下精简恢复记录，不将未完成验收标complete，不编辑Codex内部数据库。

## 3. 执行位置和协作

本地仅读取/编辑源码、必要资产、配置、锁、清单和小报告；所有依赖安装、构建、lint/typecheck、测试、MuJoCo/渲染、自动浏览器、数据分析均在服务器。复用已有SSH与tools/server/remote_job.py，凭据只读取必要字段，不打印、提交或同步。服务器不可用时不改用本机/本机WSL执行。

本地源码为主副本，服务器使用唯一ID隔离release；逐文件hash核对后运行。单一物理实例和mj_step所有者，明确作业PID、设备、端口、超时和输出路径；只清理本任务进程。不自动提交/推送，不删除旧环境或runs。

主负责人先冻结合同和每文件唯一写入者：

- 主负责人：公共contracts、app/config、backend/记录接线、安全与权限、部署/依赖、最终验收和报告。
- A：独立nearfield观测/几何模块、受限界面及对应测试；与主负责人约定UI公共文件所有权。
- B：独立manipulation动作适配/权限模块及测试；不并发修改backend、contracts或A文件。
- C：独立采集就绪报告、阶段标注、case/追溯/评分工具及测试；不改A/B内部或启动第二个物理实例。

先验证一个窄任务有实际可核对文件和回执，再扩大并行。之前G3B执行lane无产物，不重复空派发；无有效产物立即记录并接管，不冒充独立实现/复核。物理作业由主负责人统一排队，源码可并行。

## 4. N0：任务、场景与停车许可

任务限定为单个20mm方块的接近、闭爪、抬升、保持，不增加放置、堆叠、搜索或全身规划。复用G1几何和已验证抓取机制，必要时建立G4A-1专用场景/初始配置以形成明确可见可达工作区，并冻结model、scene、arm_spec、相机与参数hash。

三个初始布局为中央、小横向偏移、小纵向偏移；偏移建议10mm，具体方向和位置在开发检查后、正式执行前固定。初始设置只发生在Episode前；运行中不写qpos搬车/搬物体、不焊死底盘、不附着方块、不关闭碰撞或改摩擦/重力制造成功。

底盘保持自由动力学。用显式准备的停车位，记录park_source=prepared_pose，不要求Nav2先导航，也不宣称导航到近场交接已完成。进入操作前，基于可用测量确认平移≤0.01m/s、yaw rate≤0.03rad/s持续≥0.6s，并确认没有旧导航授权。

建立最小nearfield许可：绑定run/Episode/generation/epoch、操作session、阶段、模型/profile及取消代数。只允许一个底盘/操作权限持有者；操作期间拒绝非零底盘请求，人工需要移动时先明确取消/退出nearfield。软件Stop始终可用。

底盘移动、测量失效、Reset、权限撤销和人工接管使旧操作许可/预览失效；恢复后需显式新许可。控制健康只消费测量/估计，真实漂移只用于隔离评分。阶段可用OBSERVE/READY/APPROACH/CLOSE/LIFT/HOLD/DONE或ABORTED，阶段标签不能替代实际物理成功。

相机固定link4；joint5旋转夹爪、独立驱动开合。不得改挂TCP或增加外部相机绕过视野门。五轴采用位置优先/受限姿态，姿态残差单列，不承诺任意6D姿态。

## 5. N1：受限观测、几何与操作入口

建立G4A-1独立白名单观测，复用现有capture实现：

- link4 RGB、米制Depth、Depth有效mask；
- capture sim/monotonic/tick、sample/feedback身份、consumer arrival及质量；
- 五轴实测用户角、夹爪实测宽度；
- 相机内参、固定挂载及同capture关节FK可重算引用。

不把G2A整个PolicyObservation类的所有可选字段自动当白名单；特别排除真值底盘速度、目标世界位姿、完整diagnostics、Oracle接触、场景物体坐标、第三人称操作状态和实例分割真值。Depth有效mask是传感器有效性，不是物体分割mask。

受限采集视图只提供上述可用观测、必要状态和安全操作控件。不能仅隐藏第三人称画布却继续给控制客户端推送真值snapshot；诊断入口与控制消费进程隔离并做字段/请求审计。已有第三人称调试界面可保留为明确GT_DEBUG，不能将其操作记录标成受限视图示范。

复用滑块/TCP位置预览，优先提供可靠link4主视图、关节/开口和局部深度读数，不重做整个三维界面。若需要像素点选辅助，只根据实际RGB像素、有效Depth、标定和capture时FK生成局部候选点，记录点击样本、像素和推导过程；不吸附目标真值中心。TCP/IK辅助结果仍须经过碰撞检查和本Goal动作适配器。

标定/几何有限检查至少覆盖两种合法q1–q4姿态、中心与离轴像素合计六个参考点，报告重投影及三维误差；建议门为≤1.5像素、≤3mm，正式前固定。核对米制Depth、光学轴、图像翻转和T_arm_camera，禁止重复深度转换/翻转。参考几何仅进入隔离测试，不能作运行时目标真值。

控制使用图像时刻局部几何，不用“当前关节”配旧图像，也不让map→odom修正直接跳动局部抓取目标。q5/开口对相机及TCP关系做对应检查。

teacher使用RGB-D则来源view_source明确为link4_rgbd_proprio，RGB-only资格为NOT_EVALUATED。不在本Goal开发视觉检测模型、目标搜索或通用自动抓取专家。

## 6. N2：可部署六维动作适配器

冻结独立动作profile，建议决策/控制目标频率5Hz：

action=[q1_target_user_rad,q2_target_user_rad,q3_target_user_rad,q4_target_user_rad,q5_target_user_rad,gripper_width_target_m]。

这是绝对目标，五轴rad、夹爪m，不是实测关节反馈、TCP宏目标或底层100Hz执行参考。user/model零位与方向、上下限、最大目标增量/速度/加速度、有限力、调度及低层插值/平滑明确记录。若低层再次平滑，保留其身份和时间，不宣称与历史actuator_reference数值等价。

UI/teacher和未来policy必须走相同适配器；TCP规划只能作为teacher候选生成器。定义动作帧共同身份、seq、所依据的observation sample、decision/issue/expiry、权限和实际应用tick；arm与gripper同帧一致应用或整体拒绝，不能用不同时间/不同owner的两个值拼成一次六维动作。

profile/帧明确声明joint_order和units；拒绝声明不匹配、维度/非有限/越界等情况，不依赖猜测数值来判断单位。

在唯一物理推进者处更新控制目标，不在ROS/HTTP/浏览器回调里step或写qpos。长规划离锁，执行前复核观测、实际起始关节、版本、lease和取消代数。保留G1旧命令兼容，新动作profile显式启用。

记录requested_action、approved_target、dispatch、actuator_reference和feedback为不同层。action_t定义为消费obs_t后制定并接纳的本合同目标层；不得拿decision前最新执行参考冒充之后的监督动作。至少能追溯obs capture→consumer arrival→decision→request→approve→dispatch→actual control tick。

命令过期、客户端失联、Stop、接管或Reset后不得执行旧帧或恢复旧序列。默认中止策略为停止新轨迹并在有限力下保持最后安全关节/夹爪目标，不自动张开夹爪；具体策略正式前冻结，不能把“无新命令”解释为松爪。显式新许可才能继续。

在不训练、不做模型推理的条件下，用少量安全目标直接驱动适配器，验证单位、关节顺序、零位、夹爪开口、限速、回执和实际物理变化，不能只验证序列化或转换函数。

## 7. N3：原生记录与采集就绪报告

沿用原生多频Episode，不覆盖旧数据。G4A-1使用独立observ/action/task/quality profile，保存来源、阶段、控制许可、模型/参数hash和任务/采集父身份。

RGB-D目标640x480、5Hz，保持RGB/Depth/有效mask同sample/capture，关节反馈与FK对应同capture。保留capture/publish/consumer arrival/persist语义；operator真正看到的sample和decision关联，不能用写盘时间代替在线可用时间。初始化/分析/收尾均报告，重复像素与重复sample分开，不能复制旧帧补频率。

建议沿用图像最大年龄0.4s、反馈0.2s、capture skew0.2s，按当前低速任务正式前冻结；若任务需要更紧门先声明，不静默放宽。未来插值、缺值补零、旧图像冒充新决策以及跨Reset拼接禁止。无新有效观测时安全保持/中止，不能继续发布依赖过期画面的动作。

对APPROACH/CLOSE/LIFT/HOLD逐段给出目标与夹爪可见、遮挡、出视野、UNKNOWN、Depth有效性和关键截图/短片索引。允许真实闭爪遮挡，明确哪些动作依赖历史观测；不能以Oracle知道物体位置替代可观测性。标注者身份如实写agent_review/自动方法/实际human，不伪装人工评价。

独立报告以下结论，不合并成一个“合格”：

1. 原生format_valid/data_complete；
2. 物理task_result及Oracle证据；
3. 观测覆盖与可观测性；
4. 动作部署接口及因果关联；
5. operator/view来源与干预/恢复；
6. parent Episode/collection family与切分防漏；
7. training_qualified及逐项原因。

现有G2B资格函数未逐项实现配置声明的全部门，不修改旧profile让历史数据变true。为G4A-1新增显式检查，所有声明门逐项给PASS/FAIL/NOT_EVALUATED；缺证据不默认通过。数量不是训练充分性证明，本轮所有开发试点用途dev、training_qualified=false。

source=human_teleop只允许真实人操作；agent实际操作受限界面为agent_ui；代码控制器为scripted；fixture保持synthetic_fixture。自动化本Goal不需要等待真人；真人示教与其资格留给G4A-2。

保存parent_episode_id、recording segment、collection family、Reset/恢复和裁剪来源。只读回放/审计输出到外部报告目录；源目录前后hash一致。同父或同次恢复的子段不能跨split；用小型负例验证防漏，fixture不冒充物理记录。

明确源封存边界：首次采集收尾及初次物理评分产物完成后封存；后续回放、资格审计和对齐预览只读。Oracle重复复算使用write=false或等效外部输出，不因其默认写入行为改动已封存Episode。

只做小型对齐预览/sidecar及首中末、闭爪/抬升追溯，不运行LeRobot writer/编码/loader、不发布Hub、不划正式train/validation；export_valid=NOT_EVALUATED。本Goal的原生format_valid不等于LeRobot格式通过。

## 8. N4：服务器有限验收

先开发单一中央布局完成一次真实闭环，再冻结六例case manifest、动作/视觉/物理阈值和配置；不开展多种子、噪声或对象大矩阵。

正式六例：

- M01：中央布局，受限观测操作完成真实抬升和保持。
- M02：小横向偏移，重新观察并成功抓取。
- M03：小纵向偏移，重新观察并成功抓取。
- M04：关键阶段注入图像陈旧或Depth失效，受控中止、保持安全目标，恢复不续旧动作。
- M05：Stop/实际人工来源控制组接管测试；旧动作帧被拒绝，显式新许可后安全目标可执行。自动探针模拟manual来源须标为测试注入，不称真实人类示教。
- M06：Reset/new epoch，旧帧/旧预览/旧session拒绝；重新初始化后新安全目标可执行。

成功门沿用G1隔离物理条件：方块相对开始抬升≥0.05m，双侧有效接触保持≥2s，无非预期机器人/环境碰撞。台面初始支撑、轮地接触及夹爪/目标接触按冻结允许表处理；不因有接触就全判失败，也不忽略arm/table碰撞。

操作期间自由底盘平面漂移≤0.01m、yaw≤2°建议作为正式门，隔离真值只评分不控制。建议每例≤120s仿真时间、≤180s墙钟，另有job关闭超时；正式前按任务冻结，不无限重试，不靠临时改变物理参数或目标完成。

M04/M05报告故障/撤权→检测→安全目标/零速度→物理停止的各层时间及是否保持夹爪；检测到安全目标建议≤0.15s、随后机械臂停动建议≤1s，实际速度阈值按arm规格正式前冻结。无确认不能把command accepted当physical stopped。

必需子门单列，不混入六例成功分母：

- 几何/光学标定与capture对应、q5/开合关系；
- 三组安全六维目标真实执行，错误单位/顺序/非有限/越界/过期/旧身份拒绝；
- nearfield与非零底盘命令互斥，最终动作/拒绝回执可追溯；
- 白名单与真值泄漏拒绝、四阶段可观测性与来源；
- 原生质量、因果追溯、源hash不变、父任务split负例；
- 至少一次真实服务器浏览器受限视图操作；仅直接API脚本不能冒充GUI通过。

三条成功开发示范不当作统计成功率或训练充分性；准备、失败、修复复验、注入负例全部独立记账。正式六例及全部必需子门通过才可complete。

## 9. 回归、交付与停止

先定向验证，稳定后在同一冻结release做一次受影响回归。导航/TF/地图未改则不重跑G3B十例；共享权限/backend变动补相关G3B控制回归，确有影响才扩大。不重做G2A全性能或G2B官方导出矩阵，不以重复全测替代修复。

历史contacts.py provenance失败保留且单列；为当前release检查实际源码、传递include资产、profile、参数和构建产物身份。排除项不得写成通过，新增活动文件不得借历史例外漏审。

交付：

- docs/g4a1_report.md、g4a1_reproduction.md、g4a1_interface_v1.md；
- docs/g4a1_demo_record.md、g4a1_failures_and_fixes.md；
- 独立观测/动作/task/quality profile、case manifest及六例/子门结果；
- 开发Episode和阶段可观测性/因果sidecar索引、逐项采集就绪报告；
- docs/workpackages/G4A1_agent_receipts.md；
- docs/evidence/g4a1_server_artifact_index.json、最终JUnit、新g4a1-... release/aggregate与逐文件清单。

正式结果、UI构建和受影响回归绑定同一冻结release；本地与release清单逐文件对齐，不能混合不同开发版本的通过。原始RGB-D、Episode、完整日志留服务器，本地默认≤20MB小报告/证据。

维护精简恢复记录：当前release、最后job/问题证据、已通过/未完成门、下一动作、实际Goal状态。完整验收后标complete并停止，不自动进入G4A-2、LeRobot正式导出、ACT/VLA训练、移动抓取联合任务或实车。
