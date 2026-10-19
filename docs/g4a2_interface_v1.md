# G4A-2 真人示教、六维动作与数据出口接口 v1

日期：2026-10-18。状态：**H0 冻结候选**。基线为 G4A-1 release
`g4a1-20261018T125226Z-134032d390f8`，不修改历史 `g4a1-1.0` 数据语义。

## 1. 真人会话

- 页面来源固定 `human_teleop`、`view_source=link4_rgbd_proprio`。
- 用户必须在独立采集页点击“确认本人操作”，建立 human session；agent 的自动浏览器
  测试只能建立 `agent_ui`/`test_injection` 会话。
- session 绑定 run/Episode/generation、operator、输入设备、辅助模式、开始时间和
  reset/cancel epoch。
- 失焦、断连、Stop、传感器失效、Recorder 失败或 Reset 立即关闭 5Hz sender、撤销
  近场权限并保持最后安全目标，不自动续租、不自动开爪。

## 2. Human intent

每次真实输入事件记录：

- `human_intent_id`、session、operator、输入类型和原始输入摘要；
- 用户实际看到并显示完成的 `display_sample_id`；
- 浏览器 monotonic 时钟的 display/intent/sequence 时刻；
- 服务端接收 monotonic 时刻；
- 辅助模式 `manual` 或 `planner_assist`、control generator 和 privileged input 声明。

浏览器与后端时钟不能直接相减；无法建立映射时跨域 latency 为 `UNKNOWN`。持续移动可
由一个 human intent 扩展为多个 5Hz 目标帧，但不得伪造为多个独立人类决策。

## 3. 普通六维动作

```text
action = [q1,q2,q3,q4,q5,gripper_width]
```

单位固定 rad/rad/rad/rad/rad/m，joint order `joint1..joint5`。正式 human 动作：

- `source=human_teleop`；
- 走普通 `G4A2ActionRequest`/`g4a1` 六维执行边界；
- 实际下发窗口 `teacher_plan_id` 必须为空；
- 每帧有 seq、observation sample、human intent、有效期和 receipt；
- 辅助轨迹必须在发送侧展开为 5Hz 普通目标，不得把整段终点复制到所有帧。

记录 requested、approved、dispatch、actuator reference、actual tick 和 feedback 为
不同层。动作标签是目标层，不使用执行反馈或 TCP 宏目标冒充。

## 4. 受限采集页

独立路径 `/g4a2` 只消费：

- link4 RGB、米制 Depth/valid mask；
- 本体实测五轴/夹爪、局部 FK、安全/录制/权限状态；
- 用户自己的输入和显示 sample。

禁止注入 world_sim 物体/底盘 pose、Oracle 接触/抬升、第三人称画布、完整 diagnostics
或已知 case offset。调试页面保留独立路径，不能与正式 human 记录混用。

## 5. 数据用途

本批 profile：`g4a2-human-rgbd-proprio-5hz-sixdim-v1`。

- observation：link4 RGB + 米制 Depth + valid mask + 五轴 user 角 + 夹爪宽度；
- action：普通绝对六维目标；
- purpose：真人 dev pilot 或 main collection；
- H1 三条试点只进 dev，不计入 24 条正式目标；
- `training_qualified` 只对明确 human RGB-D/proprio/5Hz 六维用途成立，不代表 RGB-only、
  ACT 效果、语言泛化或实车。

## 6. 文件所有权

| 所有者 | 独占路径 |
|---|---|
| 主负责人 | `src/mobilecar/contracts/**`、`src/mobilecar/app.py`、`src/mobilecar/backends/**`、`src/mobilecar/ui/routes.py`、`src/mobilecar/ui/static/g4a2*`、公共 profile/config、服务器作业、最终验收/导出报告 |
| A | `src/mobilecar/human/input.py`、`tests/g4a2/test_human_input.py`、`docs/workpackages/A_G4A2/**` |
| B | `src/mobilecar/data/g4a2_reader.py`、`src/mobilecar/data/g4a2_export.py`、`tests/g4a2/test_export.py`、`docs/workpackages/B_G4A2/**` |
| C | `src/mobilecar/data/g4a2_qualification.py`、`tools/g4a2/split_manifest.py`、`tests/g4a2/test_qualification.py`、`docs/workpackages/C_G4A2/**` |

公共合同、backend/app/页面装配、依赖和最终索引只能由主负责人集成。

## 7. 完成边界

H0 工具与 dev 小验证不等于 G4A-2 完成。只有用户本人完成 H1 三条试点、冻结正式
cohort、完成默认 24 条成功及全部失败/取消保存和 H4 出口门后才能 complete。
