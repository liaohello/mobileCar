# G2B 服务器复现

所有命令在服务器项目根目录/release 内执行。G2B 不启动 MuJoCo、浏览器或训练。

## 1. 环境和依赖

独立环境：

```bash
envs/g2b-lerobot-0.4.4/bin/python --version
```

应显示 Python 3.12。完整锁文件为 `requirements-g2b-lerobot.lock`，其中锁定：

```text
lerobot==0.4.4
torch==2.7.1+cpu
torchvision==0.22.1+cpu
av==15.1.0
datasets==4.8.5
pyarrow==26.0.0
```

安装时先装独立 Python3.12 virtualenv，再按锁文件安装；不要升级
`envs/g2a-py310-mujoco316`。

## 2. 同步代码和输入索引

在本地：

```bash
python tools/server/remote_job.py sync
```

服务器 release 解包后执行 `sha256sum -c SHA256SUMS`。G2A 原始 Episode 保持只读：

```text
var/runs/run_81ccba839bec4e1991278b913fa2f1b3/
  episodes/episode_c20c70aa2ada4e4fbd4c1a9ec5672090
  episodes/episode_32975d5ce7eb4b10a58588858de103d7

var/runs/run_d669e1a5aa2149cfac566d9990c75703/
  episodes/episode_b08d13faa97341088a8c21c4136b4d2e
```

## 3. 定向回归

在 release 根目录：

```bash
mkdir -p ../../var/reports/<job_id>
PYTHONPATH=src ../../envs/g2a-py310-mujoco316/bin/python -m pytest \
  tests/export tests/data tests/contracts/test_g2a_contracts.py \
  tests/contracts/test_contracts.py -q \
  --junitxml=../../var/reports/<job_id>/junit.xml
```

这些测试只做离线导出、数据和合同检查，不运行 G2A 的 55 项物理/UI矩阵。

## 4. 正式写入与官方 loader

```bash
mkdir -p ../../var/reports/<job_id>
PYTHONPATH=src ../../envs/g2b-lerobot-0.4.4/bin/python tools/export/run_g2b.py \
  --run-root ../../var/runs \
  --dataset-root ../../var/datasets/<export_id> \
  --report-root ../../var/reports/<job_id> \
  --profile configs/export/g2b_export_v1.yaml \
  --run-id run_81ccba839bec4e1991278b913fa2f1b3 \
  --motion-episode-id episode_c20c70aa2ada4e4fbd4c1a9ec5672090 \
  --stationary-episode-id episode_32975d5ce7eb4b10a58588858de103d7 \
  --fault-run-id run_d669e1a5aa2149cfac566d9990c75703 \
  --fault-episode-id episode_b08d13faa97341088a8c21c4136b4d2e
```

退出码 0 只是执行状态；必须读取：

```text
var/reports/<job_id>/g2b_report.json
```

其中的 `format_valid`、`export_valid`、`train_qualified` 必须分别解释。当前允许
`train_qualified=false`。

## 5. 输出布局

```text
var/datasets/<export_id>/
  d0_dev/
    data/
    meta/
    videos/
    g2b/sidecar.jsonl
    g2b/source_episode.json
    g2b/split.json
    manifest.json
  d2_dev/
    ...
```

官方 `meta/info.json` 使用 codebase `v3.0`，split 为 `dev: 0:1`；train/validation 为空。
原始 Depth/Mask 不复制进 LeRobot feature，sidecar 保存其在源 Episode 的相对 URI 和 hash。

## 6. 复核命令

服务器上可用官方 loader 重读：

```bash
PYTHONPATH=src ../../envs/g2b-lerobot-0.4.4/bin/python - <<'PY'
from lerobot.datasets.lerobot_dataset import LeRobotDataset
for name in ("d0", "d2"):
    ds = LeRobotDataset(
        f"mobilecar/g2b-g2a-motion-{name}-dev",
        root=f"../../var/datasets/<export_id>/{name}_dev",
        video_backend="pyav",
    )
    print(name, len(ds), ds.num_episodes, ds.fps, ds.meta.info["splits"])
    print(ds[0])
PY
```

不要上传到 Hub，不要启动训练。
