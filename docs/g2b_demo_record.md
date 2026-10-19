# G2B 演示记录

日期：2026-10-17。输入是现有 G2A 服务器 Episode；没有重新采集。

## 序列

1. 只读读取 motion/static/fault 三个 Episode。
2. 用固定 5 Hz 相位 `recording_start + 0.13 s` 建立决策网格。
3. 按 arrival 边界选择已到达图像和 fast observation；persist 仅作耐久性证据。
4. exact tick 从 `feedback` event 取得同一图像 capture 的六维本体快照。
5. D0 从 `actuator_reference.base_twist` 取实际生效速度参考。
6. D2 将 `arm_model_reference_rad` 前五维按 zero/direction 映射为用户角，并加
   `gripper_width_target_m`。
7. 使用官方 `lerobot==0.4.4` writer 写 D0/D2，H.264 视频和 v3.0 metadata。
8. finalize 后通过官方 metadata API 将 split 设置为 `dev: 0:1`。
9. 官方 loader 逐样本读取全部行，并读取 4 样本 batch。
10. 写 sidecar、source hash、环境、manifest 和总报告。

## 实际结果

| 项目 | D0 | D2 |
|---|---:|---:|
| format_valid | true | true |
| export_valid | true | true |
| rows | 14 | 272 |
| episodes | 1 dev | 1 dev |
| loader samples checked | 14 | 272 |
| image shape | `[3,480,640]` | `[3,480,640]` |
| state shape | `[10]` | `[6]` |
| action shape | `[3]` | `[6]` |
| train_qualified | false | false |

`train_qualified=false` 的原因原样保留：来源 `agent_ui`、任务结果 `failure`、
只有 1 个父 Episode。未改标签，也未生成 train/validation。

## 关键追踪

D0 首行来源为 image `rgbd_85aa0d00eed04a14a44d07a6dcfd27f8`，image control tick
`tick_000006712`，action reference `tick_000006726`。

D2 首行来源为 image `rgbd_b1496854feed4b99ac5b8793282ecc09`，image/feedback
`tick_000007149`，action reference `tick_000007165`。

完整首/中/末 sidecar 行在 [g2b_export_report.json](evidence/g2b_export_report.json)。
