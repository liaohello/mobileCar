# G3B 建图、重定位与 Nav2 到点验收报告

日期：2026-10-18。状态：**通过有限 G3B 验收**。所有最终证据绑定同一服务器 release。

- release：`g3b-20261018T103442Z-a4081b0147bd`
- aggregate：`a4081b0147bd871a5161aa2b574f534dec44fa66cf6a47969f94aab63a51d9a5`
- interface：`g3b-1.0` / `g3b-nav2-omni-map-v1`
- 服务器：Ubuntu 22.04、ROS 2 Humble、MuJoCo 3.1.6

## 正式 10 例

Nav2 Action `SUCCEEDED` 共 7 例；取消/故障/Reset 例分别为预期 `CANCELED` 或
`ABORTED`。所有 10 例隔离物理评分、停稳和无非预期环境接触门均通过。

| case | Action | 位置误差 m | yaw ° | 停稳 s |
|---|---:|---:|---:|---:|
| A01_hold_room_a | 4 | 0.0187 | 1.76 | 0.60 |
| A02_limited_room_a | 4 | 0.0023 | 4.21 | 0.60 |
| B01_hold_room_b | 4 | 0.0419 | 2.31 | 0.60 |
| B02_limited_room_b | 4 | 0.0425 | 2.15 | 0.60 |
| A03_restart_localization | 4 | 0.0156 | 3.55 | 0.60 |
| B03_restart_localization | 4 | 0.0310 | 3.56 | 0.60 |
| A04_cancel_manual_takeover | 5 | 预期接管/重授权 | — | 0.60 |
| A05_scan_loss | 6 | 预期故障停车 | — | 0.60 |
| B04_localization_failure | 6 | 预期故障停车 | — | 0.60 |
| A06_reset_epoch | 4 | 0.0220 | 2.39 | 0.60 |

Room A 横移已通过真实非零 `vy` 命令和物理位移确认。有限臂例分别确认约
`0.04rad` 实际关节变化；取消例确认非零人工接管、松手不恢复、臂目标保持和显式新
目标重新授权。Reset 例确认新旧 Episode/epoch 隔离、旧 ROS 栈退出、旧 receipt 拒绝及
新目标到达。

## 地图与定位

- Room A：`map_room_a_g3b_release_v1`，manifest SHA-256
  `aaee75864a1a1edc7bdf73734640078640b0b40b69020574e80eef81c1b6fd34`。
- Room B：`map_room_b_g3b_release_v1`，manifest SHA-256
  `a6e641a570c85e21523863f75e1346469c668993c1af0d75fe92ff4d1ea0382c`。
- 两包均含 YAML、PGM、posegraph、posegraph data 和 digest；`free_thresh=0.196`。
- 地图兼容字段全部一致；两个场景均通过 fresh process 加载 posegraph 后重定位到达。

## 子检查

- odom frame、地图外、unknown cell、篡改场景兼容字段：4/4 在 backend 授权前拒绝且无运动。
- NaN、+Inf 和 SELF_OCCLUDED：实际 scan 含 NaN/+Inf；Nav2 `inf_is_valid=false`，
  不使用 invalid/unknown 清自由区。
- footprint：全局/局部完全一致，包含 0.03m padding，DWB `ObstacleFootprint` 启用。
- map→odom：仅一个 slam_toolbox localization 源、无 AMCL；4.15Hz 运行流量。原始严格
  检查记录同 stamp 的 0.49mm/0.0017rad 更新，服务器补充审计确认属于同一 SLAM 源修正，
  不是第二发布者；原始检查保留。
- RViz：自动鼠标设点/拖动朝向，Action 成功，物理位置、yaw、停稳、截图全部通过。
- scan/定位故障：故障停车和恢复不自动续航通过；定位故障时 scan/odom 保持新鲜。
- 最终门：发布点零速、物理停稳、迟到速度和 backend receipt 关联门通过。

## 数据与训练资格

10 个主 Episode 均为 `audit_status=COMPLETE`、`format_valid=true`、
`data_complete=true`，无格式错误。少量 recording reset 生命周期 warning 不影响格式门。
这些是导航 Episode，不是人类抓取示教；`training_qualified=false`。

## 服务器资源与耗时

最终服务器预检为 128 CPU、503.51GiB 内存和 4 张 RTX 4090；G3B 队列始终最多一个
物理 MuJoCo 实例。正式10例墙钟耗时合计 688.234s，单例 19.113–214.922s，均值
68.823s；另设每例 360s 安全超时，未靠无限重试或放宽阈值完成。资源汇总命令 job 为
`var/jobs/g3b-final-resource-summary`。

## 回归

最终 release 先校验 295 个冻结源文件，再在服务器完成前端构建和完整受影响回归
`123 passed`，JUnit 为 0 failure / 0 error。仓库既有的 `test_provenance` 对历史
`contacts.py` hash 仍失败，本轮未修改该文件，作为已知既有问题保留。

完成审计重新核验正式10例、四个负向目标、footprint、NaN/+Inf、RViz GUI、两份地图
manifest、数据完整性和 `training_qualified=false`。服务器任务结束后，domain 140–144
的 `ros2cli` daemon 已用 `ros2 daemon stop` 正常关闭，独立进程审计确认没有遗留
MuJoCo、Nav2、slam_toolbox、RViz 或 G3B runner。审计结果见
[g3b_completion_audit.json](evidence/g3b_completion_audit.json)。本地 235 个非交付
文档源码/配置文件也逐项与 release manifest 一致。

原始报告、地图大文件、runs、日志和截图留在服务器。小型结果与索引见
[g3b_server_artifact_index.json](evidence/g3b_server_artifact_index.json)。
