# mobileCar G2A 服务器质量验收报告

日期：2026-10-17。结论：**G2A 范围内 PASS**。所有安装、前端构建、软件回归、MuJoCo、
EGL、full Chromium、质量重算和故障验证均在服务器执行；没有用本地测试替代。

## 1. 最终身份

| 项目 | 值 |
|---|---|
| wire contract | `mobilecar-1.0` |
| semantic interface | `g2a-1.0` |
| observation profile | `g2a-rgbd-5hz-original`, 640x480, nominal 5 Hz, 60 s |
| observation profile SHA-256 | `9473171d7460ba968813b40ad1cae2f392bbc168e85ef4524eced6f38b5e3042` |
| action profile | `g2a-teleop-action-1.0` |
| action profile SHA-256 | `4b037ff671fb766452630b234d5680383381a2e6597b5855a5d92131a5c9f7dc` |
| model SHA-256 | `3ca3d6d0a92a48bbfbce5ac7a04e37a0f7cf7a73297045ae327bf461ad50e280` |
| final release | `g2a-20261017T163244Z-5cbe582fa6b2` |
| release aggregate SHA-256 | `5cbe582fa6b266d074d22bd4c120ad381797a440e37834ab8e9a6c7e03558514` |
| file count | 163 |

服务器为 Ubuntu 22.04.5、Python 3.10.12、Node.js 20.20.2、MuJoCo 3.1.6、128 CPUs、
约 503.5 GiB RAM、4x RTX 4090。MuJoCo 实际使用 EGL
`NVIDIA GeForce RTX 4090/PCIe/SSE2`；Chromium 153 使用
`ANGLE (NVIDIA, Vulkan 1.4.303, NVIDIA GeForce RTX 4090)`，不是 SwiftShader。

## 2. 实现范围

- RGB/Depth capture 绑定同一物理快照的 sim time、backend monotonic、control tick、
  joint feedback sample，并分别记录 render、encode、publish、arrival、persist 各阶段。
- `T_base_link4`、`T_base_camera`、`T_base_tcp` 使用
  `inverse(T_world_base) @ T_world_*`；相机保持 link4 分支，开口不改变相机/TCP，
  q5 改变 TCP 与相机相对姿态。
- 5 Hz 图像和 10 ms 控制使用绝对 deadline；跳过槽位和原因进入 Episode 诊断，不复制图像凑帧。
- TCP 预览只在短锁内复制状态，IK、碰撞和轨迹参数化在锁外完成；回锁后复核 Episode、
  generation、目标修订、取消代数、model/profile、租约以及 base、arm、gripper 起始状态。
- Recorder 在 RGB/Depth/Mask 可靠写入并 `fsync` 后才发布持久化记录；队列溢出和控制写入故障
  会触发现有 STOP/锁存链。
- 质量门按 sample identity 而非像素 hash 计唯一帧；静止画面相同但 sample/asset 不同的 capture
  仍算独立帧，像素一致只作诊断。
- `PolicyObservationV1` 使用明确 allowlist。`world_sim`、物体真值、Oracle、整对象 diagnostics
  和第三人称手柄状态不进入策略输入；GT_DEBUG 保留在独立 UI/诊断路径。
- G2A 资格同时绑定 observation/action profile ID、版本和 hash。G1 的
  `assessments/eligibility.json` 保持历史结论，不自动升级。
- Three.js 改为按需/限频渲染，上限 1.5 pixel ratio、默认 512 阴影图；安全驾驶、零速、
  取消和停止输入走优先级路径，不被长 TCP 预览串行阻塞。失焦、隐藏和断连释放输入，
  重连不会恢复旧动作。

接口与兼容说明见 [G2A 接口](g2a_interface_v1.md)、[运行时 API](runtime_api_v1.md) 和
[工作包](workpackages/G2A.md)。

## 3. 5 Hz 真实 MuJoCo profile

分析窗口固定为 5-55 s；初始化 0-5 s 和收尾 55-60 s 保留并报告。两段 Episode
均为唯一 RGB-D 持久样本、完整 RGB/Depth/Mask 对，所有质量 gate 通过。

| 指标 | 静止 60 s | 运动/臂 60 s | 门槛 |
|---|---:|---:|---:|
| generated / delivered / persisted | 250 / 250 / 250 | 250 / 250 / 250 | pair complete |
| analysis 唯一持久样本 | 250 | 250 | >= 225 |
| average wall rate | 5.00 Hz | 5.00 Hz | >= 4.5 Hz |
| simulation sample rate | 5.00 Hz | 5.00 Hz | 报告 |
| wall p95 interval | 0.2228 s | 0.2302 s | 报告 |
| wall p99 interval | 0.2410 s | 0.2490 s | <= 0.4 s |
| wall maximum interval | 0.3416 s | 0.2930 s | 报告 |
| RTF | 0.9951 | 1.0000 | realtime |
| dropped / skipped | 0 / 1 backend deadline | 0 / 0 | 显式记账 |

初始化/收尾窗口没有被删除：

| 窗口 | 静止 unique/max | 静止 p99 | 运动 unique/max | 运动 p99 |
|---|---:|---:|---:|---:|
| 0-5 s | 26 / 0.5083 s | 0.4530 s | 24 / 0.2517 s | 0.2492 s |
| 5-55 s | 250 / 0.3416 s | 0.2410 s | 250 / 0.2930 s | 0.2490 s |
| 55-60 s | 25 / 0.2401 s | 0.2378 s | 25 / 0.2394 s | 0.2349 s |

IMU、编码器、关节反馈和 lidar 的发布流实测约 10.00 Hz，单列而不替代图像频率。

关键延迟（静止 / 运动，p99）：

| 阶段 | 静止 | 运动/臂 |
|---|---:|---:|
| capture -> render complete | 32.90 ms | 30.37 ms |
| encode | 81.94 ms | 83.62 ms |
| render complete -> publish | 84.77 ms | 86.86 ms |
| arrival -> persist | 183.67 ms | 176.86 ms |
| capture -> persist | 262.28 ms | 263.03 ms |

运动段 lock wait p99 为 62.20 ms、最大值 124.18 ms；这一段包含请求排队、短锁状态复制及
相机线程与物理线程竞争。TCP 长求解不在该锁内执行。

## 4. 真实页面与故障

30.5 s 的 full Chromium 序列包含持续驾驶/释放、夹爪滑块、TCP 预览/执行、失焦和断网重连。
共记录 186 个页面命令请求，全部 HTTP 可达请求返回 200。

| 检查 | 结果 |
|---|---:|
| preview 请求期间 keydown -> 新 `set_base_twist` 到达 | 0.119 s |
| blur -> `zero_base` 到达 | 0.0084 s |
| 失焦后保留驾驶键 | 0 |
| 断连后保留驾驶键 | 0 |
| 重连后自动恢复旧输入 | 否 |
| Episode audit | COMPLETE |

写入故障注入 Episode 在真实移动命令后触发 Recorder 失败：
`recording_status=FAILED`、`audit_status=INCOMPLETE`、安全锁存为 true、底盘停稳为 true。
随后启动正常实例，新移动命令 `DISPATCHED`，新 Episode 正常 `COMPLETE`，证明故障后恢复。

## 5. 数据资格

- 静止 Episode：质量 COMPLETE/PASS，但无真实底盘/臂演示动作，D0/D2/D3 均为 false。
- 运动/臂 Episode：在本报告声明的 observation/action profile 内 D0=true、D2=true；
  D3=false，任务 Oracle 为 failure，因为本 G2A 片段不执行完整抓取任务。
- 两者来源均为 `agent_ui`，只作为开发验收，不是 `human_teleop`，也不构成训练或部署资格。
- 本报告不把 quality PASS、D0/D2 或记录完整混同为策略效果、LeRobot 导出资格或实车能力。

## 6. 回归与证据

- 最终 release 全量相关回归：`55 passed`，1 条第三方 Starlette/httpx deprecation warning。
- JUnit：[pytest-g2a.xml](evidence/pytest-g2a.xml)。
- 验收汇总：[g2a_server_acceptance.json](evidence/g2a_server_acceptance.json)。
- renderer 复证：[g2a_renderer_probe.json](evidence/g2a_renderer_probe.json)。
- 浏览器截图：[g2a_browser_full_chromium.png](evidence/g2a_browser_full_chromium.png)。
- 质量与资格：[静止质量](evidence/g2a_stationary_quality.json)、
  [静止资格](evidence/g2a_stationary_eligibility.json)、
  [运动质量](evidence/g2a_motion_quality.json)、
  [运动资格](evidence/g2a_motion_eligibility.json)。
- 远端路径、原始 Episode hash、资产计数和复算命令：
  [g2a_server_artifact_index.json](evidence/g2a_server_artifact_index.json)。

服务器复现步骤见 [G2A 服务器复现](g2a_reproduction.md)，演示序列见
[G2A 演示记录](g2a_demo_record.md)，失败修复见
[G2A 失败与修复](g2a_failures_and_fixes.md)。

## 7. 已知限制

- 只验证了两个 60 s 片段和一个 30 s 页面序列，未做长时间 endurance、内存泄漏或多场景统计。
- 本 Goal 不含导航、SLAM/ROS2、自动视觉抓取、LeRobot 导出、训练或实车。
- RGB-D 为 5 Hz profile，不声明 10 Hz 或 20 Hz 训练 profile；无 GPU/磁盘瓶颈时也不自动升级。
- 静止场景存在像素相同但 sample/asset 不同的真实 capture；质量统计按身份计唯一帧，像素重复仅诊断。
- 服务器 release 不携带只读旧工程 `mujoco_learning`；release 文件 hash 在服务器验证，
  旧源绝对路径的二次 hash 只在源可用的本地审计环境验证。
- `agent_ui` Episode 不是人类示教，仍有人类复核和更广泛标定的后续要求。
