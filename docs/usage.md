# mobileCar G1 安装、启动与操作说明

> 后续执行位置更新：安装、构建、测试、仿真和自动浏览器均在服务器。本页保留G1命令用法，运行时先进入服务器的独立release/环境；本地不执行这些命令。见[服务器执行规范](server_execution_policy.md)。

日期：2026-10-17。合同：`mobilecar-1.0`。

## 1. 环境

已验证环境：Python 3.10、MuJoCo 3.1.6、Node.js 20（仅在重新构建前端时需要）。工程不读取
`mujoco_learning` 的可变资源，也不依赖旧工程的 `sys.path` 或 `experiments`。

在 `mobileCar` 根目录执行：

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[test]'
```

运行时依赖锁定于 `requirements.lock`；测试依赖锁定于 `requirements-test.lock`。浏览器 bundle
已随工程提供。只有修改 `src/mobilecar/ui/web/**` 后才需要：

```bash
npm ci
npm run build:ui
```

## 2. 启动 Mock

```bash
.venv/bin/mobilecar --backend mock
```

打开 `http://127.0.0.1:8787`。Mock 用于接口、界面和数据闭环联调，不构成物理证据。

## 3. 启动真实 MuJoCo

带无窗口 EGL 渲染的 Linux/WSL：

```bash
MUJOCO_GL=egl .venv/bin/mobilecar \
  --backend mujoco \
  --require-recording \
  --host 127.0.0.1 \
  --port 8787
```

有桌面 OpenGL 环境时也可去掉 `MUJOCO_GL=egl`。同一进程只创建一个物理推进线程。

`--require-recording` 会禁止首个正式录制起点可靠写入前的底盘、臂、夹爪和目标执行运动。
默认配置 `configs/default.yaml` 使用 MuJoCo 1 ms 步长、10 ms 控制周期、500 ms 命令 TTL/租约、
30 s 单动作超时、300 s Episode 超时和 5 s 停车确认窗口。

## 4. 操作

1. 来源保持 `agent_ui`（开发验收）或由人类操作者明确选择 `human_teleop`。
2. 勾选“未确认录制不允许运动”，点击“开始录制”；界面必须显示 `RECORDING`。
3. 三维视图中拖动旋转、滚轮缩放、右键平移。
4. 按住 `W/S` 前后、`A/D` 横移、`Q/E` 转向；松键、窗口失焦或轮询断连会释放驾驶并制动。
5. 五轴与夹爪滑块只更新本地目标，点击“执行关节目标”/“执行夹爪”后才发送动作。
6. TCP 坐标使用米；`world_sim` 仅用于明确标注的三维调试视图。先“预览并检查”，再“执行计划”。
7. 计划区域会显示位置优先、预测残差、姿态软残差、碰撞结论和实际反馈。
8. “取消自动轨迹”会取消剩余轨迹并保持；停止/接管优先于普通运动。
9. “停车并结束”先制动、保持夹爪、继续采集停车反馈，再进入排空、完整性校验和隔离 Oracle。
10. “只读回放”不会发送动作、调用 Reset 或推进物理。

文本输入框或 select 聚焦时键盘不会触发驾驶。

## 5. Episode

正式输出位于：

```text
runs/<run_id>/episodes/<episode_id>/
  episode.json
  events.jsonl
  commands.jsonl
  control.jsonl
  observations/*.jsonl
  transforms.jsonl
  assets/{rgb,depth,masks}/
  assessments/oracle_samples.jsonl
  assessments/task_assessment.json
  assessments/eligibility.json
  quality_report.json
  episode_complete.json
```

其中 RGB 为原始 RGB8，Depth 为米制光轴深度 `.npy`（无效值 NaN，另有 PNG 有效掩码）。Oracle
真值只进入 `assessments/`，默认策略观测和 UI 快照不提供 `map` 估计；第三人称视图的
`GT_DEBUG` 物体位姿有显式来源标签。

## 6. 校验与工具

```bash
.venv/bin/python -m mobilecar.data --root runs list
.venv/bin/python -m mobilecar.data --root runs validate <episode_id>
.venv/bin/python -m mobilecar.data --root runs assess <episode_id>
.venv/bin/python -m mobilecar.data --root runs eligibility <episode_id>
```

真实 MuJoCo + Chromium 验收脚本：

```bash
.venv/bin/python scripts/g1_acceptance.py --url http://127.0.0.1:8787
.venv/bin/python scripts/collect_g1_evidence.py --url http://127.0.0.1:8787
```

## 7. 测试

```bash
.venv/bin/python -m pytest tests/contracts tests/backend tests/data tests/integration tests/ui -q
npm run build:ui
```

Playwright 1.63.0 已搭配本机 Chromium 1243。全新机器若缺少浏览器，可执行：

```bash
.venv/bin/playwright install chromium
```

## 8. 已知限制

- 参数是未辨识仿真 profile，不代表实车动力学或标定。
- RGB-D 请求 profile 为 5 Hz；本机 EGL 实测约 1.5 Hz，频次以实际记录为准。
- `agent_ui` 成功 Episode 是开发验收证据，不是 `human_teleop` 或仅腕部 RGB 部署资格。
- 首轮不包含导航、SLAM/ROS2 部署、自主视觉抓取、Leader Arm、LeRobot 正式导出、训练和实车运动。
- 旧来源仓缺少顶层许可证文件；资产再分发前需权利人确认。
