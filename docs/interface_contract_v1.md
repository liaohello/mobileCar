# mobileCar 最小接口与原始数据契约 v1

日期：2026-10-17。状态：供首个 Goal 实施的契约基线；尚无对应实现。

ContractVersion：mobilecar-1.0。字段调整由主负责人维护版本和兼容映射。A/B/C 在开发前使用同一份样例消息与能力描述。

## 1. 原则和数据流

~~~text
Keyboard / Joint UI / TCP target / Future policy
                ↓ CommandRequest
       CommandMux + SafetySupervisor
                ↓ SafetyDecision
           Backend dispatch
                ↓ ActuatorReference
        limited controller → MuJoCo
                ↓ Feedback / SensorObservation
             immutable snapshot

Recorder 订阅各阶段事件及原始观测；UI 只发请求、读快照。
~~~

RobotBackend 对上层不暴露可写 MjData。每个实例一个物理推进者，规划使用副本；Recorder、UI 不调用 mj_step。

## 2. 身份、时钟与顺序

| 字段 | 语义 |
|---|---|
| run_id | 本次进程运行身份 |
| backend_generation | 后端重建代数；重启不可接受旧代命令 |
| episode_id | 重置或新任务的 Episode 身份 |
| recording_segment_id | 一次连续录制片段；停止后重开产生新身份，引用前片段但不假装无间断 |
| session_id / operator_id / operator_kind | 输入会话、操作者及来源类别 |
| command_id / command_seq | 请求唯一ID及会话内递增序号 |
| control_tick_id | 唯一后端控制周期身份 |
| stream_id / sample_id | 流内采样ID；不同传感器ID不保证全局唯一 |
| event_seq | Recorder追加顺序，不代表测量先后 |
| frame_id | 相机帧ID，引用仍需run/episode/stream上下文 |
| model_id / config_id / calibration_id | 有哈希及状态的版本引用 |
| target_revision / plan_id / cancel_epoch | 目标修订、计划及取消代数 |

观测引用键至少为 (run_id, episode_id, stream_id, sample_id)。命令还绑定 backend_generation。

每个时间字段声明 clock_id。时间域至少区分 sim、backend_monotonic、device、wall_utc。

- measurement_time：测量或图像曝光对应时刻。
- arrival_time：观测到达消费边界的时刻。
- decision_time、dispatch_time：裁定与实际下发时刻，后者由物理所有者记录。
- feedback_time：反馈采样时刻；不能用请求时间或等待结束时间替代。
- wall_utc：跨机器对照，不用于直接比较不同设备的超时。

命令使用 requested_ttl_s；后端在接收端单调时钟生成 expires_at。客户端绝对截止时间仅作诊断。重复请求幂等返回原响应；较旧序号不重新执行。

请求TTL约束首次dispatch，所有权租约约束持续运动。底盘速度需要按住刷新；长臂计划仅首次提交，由独立会话心跳刷新权限，不能重复启动轨迹。租约失效取消未完成运动并制动/保持；已经执行的区间仍保留。单动作另有max_duration_s，不允许无限执行。

最小请求示例（纯合成夹具，不是运行证据）：

~~~json
{
  "contract_version": "mobilecar-1.0",
  "run_id": "example_run",
  "backend_generation": 1,
  "episode_id": "example_episode",
  "session_id": "example_session",
  "operator_id": "fixture",
  "operator_kind": "synthetic_fixture",
  "command_id": "example_command_1",
  "command_seq": 1,
  "command_type": "set_base_twist",
  "requested_ttl_s": 0.5,
  "payload": {
    "frame_id": "base_link",
    "vx_m_s": 0.03,
    "vy_m_s": 0.0,
    "wz_rad_s": 0.0
  }
}
~~~

多流延迟/乱序是合法情况。validator按流、时钟和关联检查因果，不要求整份日志measurement_time递增。

## 3. 动作通道和坐标

### 3.1 通道

| 通道 | 值 | 单位/参考系 |
|---|---|---|
| base_twist | vx, vy, wz | m/s、m/s、rad/s，base_link；前/左/逆时针为正 |
| arm_joint_goal | joint1…joint5 | 用户角rad，名称/顺序固定 |
| gripper_width_goal | width | 米制开口，按有效模型限制，不能假定最小为0 |
| tcp_target | position + optional orientation | 明确目标参考系；首版position_first |

能力协商说明支持的通道、限位、单位、速率、停止策略和位置/姿态容差。缺通道表示不更新，不补零；显式zero_base/stop/cancel表示停止。

上层关节统一用户角。后端保存转换后的模型角及零位版本用于诊断，不能把两者混写到同一个q字段。

### 3.2 参考系

- base_link：车体坐标系。
- arm_base：机械臂安装基座。
- grasp_tcp：夹爪固定结构上的工具参考系，不随两指开合变化。
- rgbd_optical：相机光学系，轴向需显式声明；与MuJoCo渲染相机坐标分别标识。
- map、odom：有来源的估计坐标系，缺少定位时不可用。
- world_sim：仿真世界，仅供已声明GT_DEBUG与隔离诊断，不能默认为map。

所有变换以 T_parent_child 命名，表示将child坐标变到parent。公共旋转采用右手系，四元数字段明确为xyzw；MuJoCo内部wxyz须由适配器转换。rgbd_optical采用X右、Y下、Z前，保留与MuJoCo渲染相机坐标的显式转换及来源。矩阵存储顺序写入元信息。界面可显示mm/deg，消息内部保持m/rad。

TCP世界目标提交后保持固定。车体系目标若开放，在提交时转换一次，并记录转换依据；移动底盘后不悄悄改写已提交目标。

未提供TCP朝向时，采用提交时实际姿态作为可放松软偏好，不隐含单位旋转；位置到位与旋转残差分别输出。

## 4. 后端与命令生命周期

建议最小接口：

~~~python
class RobotBackend:
    def capabilities(self): ...
    def start(self): ...
    def submit(self, request): ...       # 接收回执，不等于到位
    def snapshot(self): ...              # 不可变/受控副本
    def cancel(self, request): ...
    def stop(self, request): ...
    def reset(self, request): ...
    def close(self): ...
~~~

TCP规划由独立Planner/IK接口产生计划；执行前重新检查实际状态、身份、模型、限位、目标修订与取消代数。计划对象不能直接绕过submit执行。

命令响应区分：

- RECEIVED：已接收，尚未证明执行。
- REJECTED：未执行，保留原请求与原因。
- EXPIRED_BEFORE_DISPATCH：首次下发前过期，未执行。
- DISPATCHED / APPLIED：后端在某控制周期接受并下发，不等于实际到位。
- REACHED：有新鲜反馈满足本动作到位条件。
- CANCELLED / FAILED：终止，原因与最后执行/反馈可追溯。
- LEASE_EXPIRED：已执行命令的权限到期，保留生效区间/最后control_tick及停止原因，剩余部分不再执行；不能反写成从未执行。

允许APPLIED作为兼容显示标签，但持久化字段必须说明它对应dispatch、接口目标还是轨迹参考，不能含糊复用旧语义。

### 4.1 所有权与停止

底盘base和机械臂arm是执行器组；同一会话可同时拥有两组，不同来源不可争用同组。arm_joint_goal和tcp_target在arm组互斥，gripper_width_goal也属于该组。闭爪/持物期间接管先取消旧运动、保持开口，不允许新来源单独覆盖夹爪。未来导航通常占底盘，VLA通常占臂；首版不开放未经约束的自主全身叠加。

停止和接管优先于普通运动。记录关节保持、夹爪保持和底盘制动策略。软件停止锁存后拒绝新运动，显式复位且重新取得控制权后才允许新请求；不重放旧目标。

区分松键零速、取消轨迹、软件停止、暂停仿真、Reset。暂停不是实测停稳；清零所有ctrl也不是合理的默认持物停止策略。没有硬件停止链路时不宣称硬件急停。

接触规则分允许与禁止：轮地和批准抓取阶段的指腹—选定目标接触允许；非预期自碰撞、车体/臂与环境接触禁止。接触类型、目标身份、允许阶段和控制限制由后端统一发布，不能以“有接触”直接判所有动作失败。持物包络继续检查环境碰撞，物理上不增加吸附约束。

## 5. 多层动作记录

同一command_id串联：

| 记录 | 必须表达的含义 |
|---|---|
| requested | 原始输入、设备、视图来源、请求值、坐标、通道、时间和有效期 |
| approved | 安全层批准的接口动作；拒绝/限幅原因及规则版本 |
| dispatch | 哪个control_tick实际接收哪条动作、下发时间及延迟 |
| actuator_reference | 每周期轨迹/限速参考、轮目标、关节目标、夹爪目标、控制输出和饱和信息 |
| feedback | 实测状态及样本引用；作为观测，不默认当专家动作 |

被拒绝、取消、过期的原始请求也保存。旧proposed/commanded/applied字段可保留兼容，但提供逐字段映射。

一个长目标可关联多个control_tick和actuator_reference；相同目标不需要伪造新请求。新的安全约束或限幅结果记录生效周期。

导出标签由action_profile明确选择。例如部署接口为关节绝对目标时可选approved目标；底盘连续监督应依据实际生效的速度参考及其时间区间。目标、实际轨迹参考和反馈不能不加说明地互换。

## 6. 原始观测和图像时刻外参

SensorObservation包含身份、测量/到达时刻、clock_id、frame_id、来源、质量、值或资产引用。缺失/过期/无效值显式记录，不用零填充冒充测量。

RGB：原始RGB8；深度：米制光轴深度、非法值NaN或显式掩码，并记录注册关系/内参/有效范围。缩放预览、伪彩、框、目标手柄及诊断图像另存。

图像帧至少保存：

- stream/sample/frame身份、原始RGB和Depth引用。
- 图像测量时刻、内参版本、分辨率、深度约定。
- 图像时刻关节反馈样本/插值括号、FK和安装标定版本。
- T_base_link4、T_link4_camera及所采用光学系转换，或可重算它们的完备引用。
- 若map位姿可用，保存图像时刻T_map_base、估计来源、质量/外推界和地图版本。
- 对齐误差、缺失、过期、反馈无法括号插值等质量标记。

已确认的关系：

~~~text
T_base_camera(t) = T_base_link4(q1..q4,t) @ T_link4_camera
T_map_camera(t)  = T_map_base(t) @ T_base_camera(t)
T_TCP_camera(t)  = inverse(T_base_TCP(q1..q5,t)) @ T_base_camera(t)
~~~

T_link4_camera在同一安装标定版本内恒定；T_TCP_camera随q5变化。开口变量不进入固定TCP或相机的运动学链。无map估计时不生成伪map变换。

## 7. 原生 Episode 目录和生命周期

以下为建议布局，实际文件格式由C实现并固定版本：

~~~text
runs/<run_id>/
  manifest.json
  episodes/<episode_id>/
    episode.json
    events.jsonl
    commands.jsonl
    control.jsonl
    observations/*.jsonl
    transforms.jsonl
    assets/rgb/*
    assets/depth/*
    maps/*                 # 有实际地图时才存在
    assessments/*          # Oracle/人工评价，默认不供策略
    quality_report.json
~~~

manifest记录软件/配置/模型/资源/标定身份、随机种子、场景、支持能力、时钟、约定、运行模式和文件校验。

episode.json记录任务语言、目标描述、初态引用、操作者/视图来源、采集开始结束、map模式/版本、结束原因。

任务阶段可用SEARCH、NAVIGATE、OBSERVE、APPROACH、MANIPULATE、VERIFY，但G1的人工驾驶不能被记作自主导航成功。机器模式变化自动记事件；高层子目标、目标确认与原因允许人工标记。

结果为success/failure/partial/cancelled/unknown；人工标签和独立自动评分分开。Reset结束当前片段并换episode身份，不能静默接在原轨迹后。暂停编辑/瞬移初始化标记为边界，不导出成连续物理动作。

## 8. 写盘、回放和质量

Recorder使用有界队列。原始资产完成可靠写入后再提交引用；控制事件不静默丢弃。传感器丢帧、积压、写入错误和超期单独记账。

record_required=true用于正式采集：Recorder启动、起点状态与来源记录可靠写入后，才放行本片段运动。主动关闭先受控停车并结束片段，再进入DRAINING；重开产生新的recording_segment_id及起点/前片段引用，不必强制物理Reset，但不能拼成无间断完整示范。普通调试允许未录制运动，界面必须明确显示。

写盘故障通知后端受控停车，分别记recording_status=FAILED、audit_status=INCOMPLETE、termination_reason=RECORDING_FAILED；task_result按实际物理证据独立判断。录制状态至少区分STARTING、RECORDING、DRAINING、COMPLETE、FAILED。若故障使尾部无法写入，由validator根据缺完整结尾判不完整，不承诺坏盘仍能保存最终事件。

关闭时先拒绝新运动，执行有界制动及关节/夹爪保持，并继续必要物理推进、控制和反馈。记录停稳确认或超时终态后，再停采样、排空、写索引/结尾与校验，避免丢掉停车证据。异常退出没有完整结尾时不得默认为完整。

ReplayReader只读，不执行动作、不推进物理。它提供事件/图像/反馈查看；原始流离线重算与动力学重执行属于另外的能力，不能仅凭可播放宣称精确复现。

validator检查身份、跨Episode引用、缺资产/损坏、命令关联、合法单位、时钟域、同流因果、图像重复/缺失、动态外参来源、记录尾部、真值隔离、来源和阶段标签。

## 9. 真值、数据资格与数据视图D0–D3

默认Observation与Diagnostics/Oracle为不同类型和访问路径。不要将含truth的旧A1Snapshot整对象传给策略。公开观测缺估计位姿时显式UNKNOWN，不从Oracle补齐。

记录来源至少包含human_teleop、agent_ui、scripted、autonomous_policy、synthetic_fixture。

三种结论分开：

1. audit_status：记录是否完整、可关联。
2. task_result：任务实际结果及证据。
3. export_eligibility：对某种训练profile是否可用及原因。

正常人类示教不沿用“排除所有人工干预”的自主专家规则。救援接管、恢复、复位和不可见目标片段单独标记，允许保留有效子段。

| 视图 | 监督与必要条件 |
|---|---|
| D0导航BC | 底盘观测与已生效速度/轨迹；动作坐标和延迟明确 |
| D1语言导航 | 指令、历史、明确高层子目标/决策标签及可执行性；普通底盘轨迹不自动等于VLN数据 |
| D2近场操作 | RGB/按profile所需Depth、本体与夹爪、批准/执行动作；固定底盘阶段及视图来源明确 |
| D3联合任务 | 完整阶段、切换、接管/恢复、结果；不等于已具备端到端策略监督 |

LeRobot是后续离线出口。保留原生多频数据，按锁定版本/profile重采样并记录映射。遵循观测可用时刻和因果关系，不借未来反馈替代当前观测。数据集划分至少按Episode，后续泛化评估再按场景/对象划分。
