# G2A 失败、修复与协作记录

日期：2026-10-17。本文件只记录 G2A，不覆盖 G1 历史报告。

## 1. 协作接管

按要求创建了 A/B/C 三个执行 agent。它们连续返回澄清/等待状态，没有产生可核对的修改、
测试或回执。按 G2A “无有效交付时记录并接管”的规则，主负责人停止重复派发并接管三条线，
所有最终实现与服务器验证均由主负责人完成。没有把 agent 状态描述成独立审查或成果。

## 2. 服务器与环境

- 系统 Node 为 v12，保持不动；通过已安装 nvm 使用 Node 20.20.2 构建前端。
- 系统缺少 `python3.10-venv`，使用用户级 virtualenv 创建独立
  `envs/g2a-py310-mujoco316`，没有修改其他环境。
- 初次 provenance 全量回归在服务器上要求旧 `mujoco_learning` 的 Windows 绝对源路径存在。
  旧工程按策略未进入 release。测试改为源可用时双重 hash，服务器上验证 release 目标 hash
  和来源元数据；最终同 release 完成 55 项回归。

## 3. 观测和规划

- 初版规划离锁重构遗漏 `base_request_body` 字段位置和 orientation 局部变量，服务器定向测试
  立即捕获 TypeError/NameError；回到本地源码修复并按新 release 重测。
- capture 测试在物理线程未启动时没有 joint feedback sample。测试先执行一个真实控制 tick，
  保留“无反馈不伪造”的生产语义。
- 预览在原底盘仍有残余速度时正确返回 `base state changed while planning`。页面验收改为先
  等待连续停稳条件，再对当前 TCP 做第二次预览；没有放宽状态检查。
- 质量唯一性不能用 RGB 像素 hash。改为唯一 `(stream_id,sample_id)` 和唯一 RGB/Depth/Mask
  URI；像素相同只作诊断。静止场景的 270 个相同像素样本仍计为独立真实 capture。

## 4. 页面

- 全量页面最初把 preview、驾驶、zero/stop 放在同一 Promise 链，长预览会延迟安全输入。
  改为序号仍单调，但驾驶/零速/取消/停止走优先级发送；旧预览响应按 plan/control epoch 丢弃。
- Playwright 对真实夹爪 range 使用 `fill("0.032")` 报 malformed，因为目标值不符合
  `min + n*step`。验收改为在页面中按实际 min/step 设置并派发 input 事件。
- 停录后页面可能先看到 Recorder `COMPLETE`，而 G2A 重算仍在进行。验收改为等待
  `/episodes/stop` HTTP 完整返回后再读取 validator。
- 页面 renderer 查询最初从已有 Three context 取 debug extension 得到 null；验收改为独立
  WebGL 探测并在 full Chromium + Vulkan/ANGLE 参数下记录 NVIDIA renderer。

## 5. 资格

- 首轮 G2A 资格把停录自动生成的 STOP/保持动作误当成底盘/臂演示，导致静止 Episode D0/D2=true。
  改为从原始 `requested.command_id` 解析 command type，只接受非零 `set_base_twist` 和真实
  arm/gripper/TCP 动作；STOP-only 回归确认 D0/D2/D3 均为 false。
- G1 eligibility 文件不覆盖。G2A 报告另写目录并绑定 input hash、observation profile 和 action
  profile；`agent_ui` 继续标为开发证据。

## 6. 最终验证

- 同 release S4：5/5 gate PASS。
- 同 release 软件回归：55 passed。
- 同 release EGL + Chromium renderer probe：PASS。
- 原始服务器记录未被修改；G2A 输出使用新 run/report 目录。
