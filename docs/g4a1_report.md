# G4A-1 近场操作与采集准备验收报告

日期：2026-10-18。状态：**通过有限 G4A-1 验收**。所有正式结果绑定同一服务器
release：

- release：`g4a1-20261018T125226Z-134032d390f8`
- aggregate：`134032d390f83a4875e8bd936b9dead4fec9fbcd55e6c251cfde3ff5c7da4747`
- interface：`g4a1-1.0`
- 观测 profile：`g4a1-link4-rgbd-proprio-v1`
- 动作 profile：`g4a1-sixdim-absolute-goal-v1`

## 正式六例

| case | 结果 | 物理证据 |
|---|---|---|
| M01 中央方块 | PASS | 抬升 `0.096133 m`，双侧保持 `7.000 s` |
| M02 小横向偏移 | PASS | 抬升 `0.094111 m`，双侧保持 `6.800 s` |
| M03 小纵向偏移 | PASS | 抬升 `0.095001 m`，双侧保持 `7.050 s` |
| M04 图像/Depth失效 | PASS | 故障撤销、旧动作拒绝、恢复后新许可 |
| M05 Stop/接管 | PASS | session/epoch 撤销，旧帧拒绝，显式新许可 |
| M06 Reset | PASS | 新 Episode/epoch，旧帧/旧预览拒绝 |

三例成功均满足抬升至少 `0.05 m`、双侧有效接触至少 `2 s`；无制造性 qpos 搬动、

禁用碰撞、改摩擦/重力或相机改挂 TCP。M04–M06 的任务结果可以是预期 failure，但其
安全/恢复门分别 PASS，不能把物理成功与安全控制结论混合。

## 接口、观测与因果

- 六维动作固定为 `[q1_target_user_rad, ..., q5_target_user_rad, gripper_width_target_m]`，
  使用绝对 user 角/米制开口、固定 joint order 和 5 Hz 决策语义。
- requested、approved、dispatch、actuator reference 和 feedback 分别记录；动作绑定
  observation sample、capture/arrival、decision/issue/expiry、permission、run/Episode/
  generation/epoch/session。
- link4 RGB、米制 Depth、有效 mask、同 capture 五轴反馈/FK 和标定引用进入受限视图；
  world_sim、Oracle、第三人称操作状态和完整 diagnostics 不作为该视图输入。
- 实际浏览器端完成受限视图和六维动作点击；案例来源保持 `agent_ui`，教师候选为
  `scripted`，未冒充 `human_teleop`。

## 数据与资格

六例主 Episode 均为 `audit_status=COMPLETE`、format gate PASS、data-complete gate PASS，
动作因果链和观测来源 gate PASS。采集就绪报告逐项为：

| 门 | 结果 |
|---|---|
| format_valid | PASS |
| data_complete | PASS |
| task evidence | PASS |
| observability | PASS |
| action causality | PASS |
| source provenance | PASS |
| parent split | NOT_EVALUATED |
| training_qualified | false |
| export_valid | NOT_EVALUATED |

这些是开发用途数据，不是人类示教，也没有执行 LeRobot writer、编码、loader 或训练。

## 回归与限制

最终 release 的完整受影响回归为 `130 passed`，包含 7 个新增 G4A-1 合同测试；0 failure、
0 error。历史 `contacts.py` provenance 例外按原报告保留。已知限制包括：本阶段不证明
任意姿态可达、RGB-only 可观测性、正式 parent split、LeRobot 导出或训练资格。

服务器原始终端、RGB-D、Episode、日志和截图保留在服务器。小型结果见
[采集就绪报告](evidence/g4a1_readiness.json)、[正式报告](evidence/g4a1_formal_report.json)
和[证据索引](evidence/g4a1_server_artifact_index.json)。完成审计见
[g4a1_completion_audit.json](evidence/g4a1_completion_audit.json)。
