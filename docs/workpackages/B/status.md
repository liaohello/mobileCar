# B 界面工作包状态

日期：2026-10-17。合同版本：`mobilecar-1.0`。

## 已实现

- Three.js 同源三维场景：消费 `SceneManifest` 的 body/geom/joint/mesh 描述，实际机器人、计划末端构型、TCP 目标使用不同视觉编码。
- 轨道相机旋转、缩放、平移与一键 fit；STL 网格由应用资产端点加载，模型哈希显示在视图工具栏。
- 键盘驾驶：W/S 前后、A/D 横移、Q/E 逆时针/顺时针转向；0.15 s 按住刷新、松键零速、窗口失焦和页面隐藏释放全部输入。
- 文本框、数值框、select 和 textarea 聚焦时不触发驾驶。
- 五关节滑块与夹爪米制开口滑块；滑块修改先在本地预览，显式执行才发送运动请求。
- TCP 坐标输入与三维目标拖动；预览和检查生成计划后才允许执行，目标修改立即作废旧 `plan_id`。
- 显示位置优先、碰撞结论、预测/实测位置误差、姿态残差、关节余量、拒绝原因、控制 tick、所有权和安全锁存。
- 开始/结束录制使用 Recorder 确认状态；中断连接只显示断连并释放驾驶，不恢复旧命令。
- 非模态只读回放：Episode 索引、事件时间线、validator、隔离 Oracle、资格报告和原始 RGB 引用；回放路径不发送动作。
- 回放打开时停止按钮仍可操作，避免只读面板阻塞安全操作。

## 自动化证据

- `npm run build:ui`：成功生成 `src/mobilecar/ui/static/app.js`。
- `python3 -m pytest tests/ui -q`：`3 passed`。
- 其中真实 headless Chromium 测试验证：
  - 同源 Three.js canvas 创建；
  - W 按下产生正向 base twist；
  - 松键产生 `zero_base`；
  - TCP 数值输入聚焦时按 W 不产生驾驶命令；
  - TCP 预览生成计划后才启用执行；
  - 录制状态从 `RECORDING` 变为 `COMPLETE`；
  - Episode 索引、事件时间线和 validator 报告可读取；
  - 回放打开后仍可点击停止。
- 最终全仓测试：`40 passed`；JUnit 见 `docs/evidence/pytest-g1.xml`。
- 合成界面截图：[ui_fixture_20261017.png](ui_fixture_20261017.png)。图中后端为 `synthetic_fixture`，只证明界面交互，不是 MuJoCo 或任务成功证据。

## 装配接口

- `mobilecar.ui.routes.create_ui_router()` 提供 `/`、`/static/{asset_path:path}`；已由 `app.py` include。
- UI 只调用 `docs/runtime_api_v1.md` 中端点，不导入 MuJoCo，不写 `MjData`，不另起物理线程。

## 最终真实集成

- 同源浏览器已连接真实 MuJoCo 后端和 Recorder，显示接近、停稳、三个 TCP 计划、夹爪闭合、抬升保持和只读回放；受控 agent_ui 通过应用 API 执行最终连续动作，键盘/滑块/拖动由真实 Chromium 交互测试覆盖。
- 真实成功 Episode：`episode_4414a5ec8ae645d29d8e7df7826160cb`；异常 Episode：`episode_4cb1700ce2e04a8f9c5276b623ddb43f`。
- 真实画面见 `docs/evidence/g1_mujoco_lift_hold.png`、`g1_replay_success.png` 和 `g1_replay_oracle_success.png`。
- 该证据来源为 `agent_ui`，不声明为 `human_teleop`，也不声明仅腕部 RGB 可部署资格。
- `base_link`/`arm_base` TCP 手柄仍按输入参考系显示；正式 G1 物理案例使用 `world_sim`。
