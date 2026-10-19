# G2A 服务器复现与启动说明

所有命令在服务器项目的 `releases/<snapshot_id>` 目录执行。本地不运行安装、构建、pytest、
MuJoCo 或浏览器。连接字段由 `tools/server/remote_job.py` 从未跟踪的 `remote.md` 读取，
不要把 `remote.md`、私钥或口令放进 release。

## 1. 同步与身份

在本地项目根目录：

```bash
python tools/server/remote_job.py preflight
python tools/server/remote_job.py sync
```

`sync` 生成：

```text
releases/<snapshot_id>/SOURCE_MANIFEST.json
releases/<snapshot_id>/SHA256SUMS
provenance/releases/<snapshot_id>.json
```

服务器解包后会执行 `sha256sum -c SHA256SUMS`。后续报告只使用同一个 `snapshot_id`。

## 2. 专用环境和前端

本次环境为 `envs/g2a-py310-mujoco316`。依赖锁不变时复用；不修改全局 Python/Conda/Node。
在 release 根目录：

```bash
. "$HOME/.nvm/nvm.sh"
nvm use 20
npm ci --no-audit --no-fund
npm run build:ui
```

`src/mobilecar/ui/static/app.js` 和 source map 是服务器构建产物，不由本地同步。

## 3. 启动真实 MuJoCo 页面

```bash
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONPATH=src \
  ../../envs/g2a-py310-mujoco316/bin/python -m mobilecar \
  --config configs/g2a_server.yaml \
  --require-recording \
  --host 127.0.0.1 \
  --port 8790
```

浏览器访问 `http://127.0.0.1:8790`。`configs/g2a_server.yaml` 继承默认配置并把运行/报告
输出放到服务器项目的 `var/`；不要改成 release 内运行目录。

## 4. 最终相关回归

```bash
mkdir -p ../../var/reports/<job_id>
MUJOCO_GL=egl PYTHONPATH=src \
  ../../envs/g2a-py310-mujoco316/bin/python -m pytest tests -q \
  --junitxml=../../var/reports/<job_id>/junit.xml
```

完整回归期间不要同时启动另一个物理仿真实例。

## 5. S4 验收

```bash
mkdir -p ../../var/reports/<job_id>
G2A_JOB_ID=<job_id> MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONPATH=src \
  ../../envs/g2a-py310-mujoco316/bin/python scripts/g2a_server_acceptance.py \
  --report-root ../../var/reports/<job_id> \
  --config configs/g2a_server.yaml
```

脚本顺序启动最多一个物理实例，执行：

1. 60 s 静止采集；
2. 60 s 底盘/臂/夹爪采集；
3. 30 s full Chromium 输入；
4. 写入故障与锁存停车；
5. 正常实例恢复。

只有五个 gate 全部为 true 时进程返回 0。原始 RGB-D、Episode 和完整日志留在服务器
`var/runs/`、`var/reports/`、`var/jobs/`。

## 6. 渲染器复证

```bash
mkdir -p ../../var/reports/<job_id>
MUJOCO_GL=egl PYTHONPATH=src \
  ../../envs/g2a-py310-mujoco316/bin/python tools/server/server_render_probe.py \
  --model assets/landerpi/model/robot.xml \
  --report ../../var/reports/<job_id>/probe.json
```

必须记录 MuJoCo EGL vendor/renderer 和 full Chromium WebGL renderer；不能把
`nvidia-smi` 可见或 SwiftShader 截图当作 GPU 渲染证明。

## 7. 离线重算质量/资格

```bash
PYTHONPATH=src ../../envs/g2a-py310-mujoco316/bin/python -m mobilecar.data \
  --root ../../var/runs quality <episode_id> \
  --output-root ../../var/reports/offline-g2a

PYTHONPATH=src ../../envs/g2a-py310-mujoco316/bin/python -m mobilecar.data \
  --root ../../var/runs g2a-eligibility <episode_id> \
  --output-root ../../var/reports/offline-g2a
```

历史 G1 Episode 缺 capture/persist 字段时只报告 `UNKNOWN`，不补造时刻，也不回写或升级
G1 的资格结论。

## 8. 回传

本地只取小报告、JUnit、质量/资格 JSON、renderer probe、截图和 artifact index。原始媒体
通过服务器相对路径和 hash 引用。最终本地小证据总量约 328 KiB。
