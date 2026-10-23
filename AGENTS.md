# mobileCar 工作约束

## 执行位置

用户已明确要求：后续安装、依赖解析、前端构建、lint/typecheck、单元/集成测试、MuJoCo、渲染、自动浏览器、数据重算/导出和训练全部在服务器执行。

本地只编辑/读取源码、配置和文档，制作/传输源码清单，保存精简报告及必要的小证据。不得因服务器连接或环境失败改用本地/本机WSL运行测试。不要删除已有本地环境、runs或用户文件。

规范见 docs/server_execution_policy.md。连接字段从未跟踪的remote.md读取；不打印、提交或同步私钥/口令。不直接执行配置中任意命令文本。

## 基线和后续任务

- G1、G2A、G2B、G3A、G3B已经完成；docs/acceptance_report.md、docs/g2a_server_report.md、docs/g2b_report.md、docs/g3a_report.md、docs/g3b_report.md及对应原始记录是历史证据，不覆盖或重新标注。
- G3B最终服务器release为g3b-20261018T103442Z-a4081b0147bd，aggregate为a4081b0147bd871a5161aa2b574f534dec44fa66cf6a47969f94aab63a51d9a5。正式10例按各自预期通过，最终受影响回归123 passed；导航Episode training_qualified=false。TF严格检查/补充单源判定及contacts.py历史provenance例外按原报告保留，不外推任意L转弯、任意移动臂姿或可抓取性。
- G4A-1已完成：release为g4a1-20261018T125226Z-134032d390f8，aggregate为134032d390f83a4875e8bd936b9dead4fec9fbcd55e6c251cfde3ff5c7da4747；六例符合预期、受影响回归130 passed。其parent split/正式出口未评估、training_qualified=false；docs/g4a1_report.md及原始证据只读保留。
- 8792驾驶链排查与最小修复已完成（2026-10-19）：release为drvfix-20261019T044549Z-0a80415b2577，aggregate为0a80415b25778927158b53a48d4af8e4ea1530934554b71ae3db763003d76369。根因为软件停止锁存与强制录制两道既有门控在浏览器内静默拦截、驾驶区无持久说明；已增加门控状态/一键处置/目标-受理-实际读数与后端拒绝留痕，未放宽停止、录制、近场互斥和租约保护。回归129 passed（另有contacts.py历史provenance例外），57项驾驶链路检查通过；自动化不算真人示教。报告见docs/drive_path_diagnosis.md，真人H1仍待用户操作。
- 当前规划见docs/post_g4a1_execution_plan.md；下一阶段为待用户启动的G4A-2，正文见docs/goals/G4A2_human_demonstrations_export.md。用户已选真人遥操作：用户本人观察并控制，agent准备界面、记录、检查、划分和导出；自动化/脚本不得冒充human。先完成采集工具与三条真人试点，再按冻结方案小批量采集。不自动训练、联合移动抓取、Hub上传或实车。
- G3B已complete，不恢复其旧Goal做新阶段；旧tokenBudget=1000000保留为历史元数据。新Goal必须省略token_budget并立即核对null；预算诊断记录不再是当前执行指令。
- 相机保持link4挂载；joint5旋转夹爪，独立驱动控制两指开合。不得误迁移“相机改挂TCP”的旧提案。
- 本地是源码主副本；服务器使用带文件哈希的隔离release执行。保留所有未提交和未跟踪工作，不自动提交/推送Git。
- 每文件唯一写入者；公共合同、应用装配和依赖由主负责人集成。每个仿真实例只有一个物理推进者。
- 初期全项目同时最多一个服务器物理仿真实例；多个agent不等于多个GPU/仿真worker。
- 只读回放和离线导出不能修改原始Episode；数据完整、物理成功、训练资格分别表述。
- 用户只索要计划或Goal提示词时，不自动创建或启动Goal；执行范围以实际发送的Goal为准。
- G2B/G3A/G3B已结束；其离线限制、有限ROS及导航验证属于历史Goal范围，不自动重跑导出/导航/浏览器矩阵或进入新任务。执行范围以用户实际发送的新Goal为准。
- 协作先验证一个小任务能产生可核对交付，再扩大并行；无产物时记录并接管，不循环派发空任务，不冒充独立实现或审查。

## Goal预算

- 用户明确不需要人为设置token预算。创建Goal时完全省略可选token_budget字段，不传0、1或自行估算的正数，也不给子任务/子Goal自行加token额度。
- 创建后立即get_goal核对tokenBudget=null；remainingTokens=null是无自设预算，不是零剩余额度。若出现数值上限，先说明实际字段及来源，不能继续消耗后才声称系统默认。
- 已有budget_limited Goal不能通过update_goal伪造恢复或完成，也不能直接修改Codex内部数据库。通过当前客户端支持的用户控制移除错误预算；若需清除并重建，仅重建Goal元数据，保留源码、地图、运行记录、恢复点和原验收合同。
- token预算与账户用量、上下文窗口、单次执行限制、作业/动作安全超时分别判断。没有真实停止事件/状态证据时，不把“停止”泛称为预算耗尽；无自设token预算也不意味着可以绕过平台/账户限制。
