# G4A-1 服务器复现说明

日期：2026-10-18。所有命令在服务器项目 release 目录执行；不包含本地测试。

## 固定 release

```bash
export G4A1_RELEASE=g4a1-20261018T125226Z-134032d390f8
export G4A1_AGGREGATE=134032d390f83a4875e8bd936b9dead4fec9fbcd55e6c251cfde3ff5c7da4747
cd "$HOME"/Desktop/mobileCar/releases/"$G4A1_RELEASE"
npm ci --no-audit --no-fund
npm run build:ui
```

## 正式六例

```bash
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
export PYTHONPATH=src:$PYTHONPATH
../../envs/g2a-py310-mujoco316/bin/python tools/g4a1/run_formal.py \
  --cases configs/g4a1/cases_v1.yaml \
  --report-root ../../var/reports/g4a1-formal-6-final-v1 \
  --python ../../envs/g2a-py310-mujoco316/bin/python
```

每例独立启动一个 MuJoCo app、录制 Episode、受限 link4 视图浏览器和 G4A-1 接口。正式
集合必须串行，禁止并行启动第二个物理实例。成功例使用固定中央/横向/纵向偏移；M04、
M05、M06 分别注入 Depth 失效、接管撤销和 Reset。

## 就绪报告

```bash
../../envs/g2a-py310-mujoco316/bin/python tools/g4a1/score_case.py \
  --formal-report ../../var/reports/g4a1-formal-6-final-v1/g4a1_formal_report.json \
  --runs-root ../../var/runs \
  --quality-profile configs/profiles/g4a1_quality_v1.yaml \
  --output ../../var/reports/g4a1-readiness-final-v1/g4a1_readiness.json
```

## 受影响回归

```bash
source ../../envs/g3b-humble-overlay/overlay_env.sh
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONPATH=src:$PYTHONPATH \
  ../../envs/g2a-py310-mujoco316/bin/python -m pytest -q \
  --ignore=tests/integration/test_provenance.py \
  --junitxml=../../var/reports/g4a1-final-regression-v1/junit.xml
```

最终结果为 `130 passed`。历史 `test_provenance` 的既有 `contacts.py` hash 例外单独
保留，不把排除项写成通过。

## 启动与停止边界

正式物理 runner 自己创建 app/浏览器进程，并在每例超时或完成后终止本任务进程组。
运行期间只允许一个服务器物理实例。原始媒体、Episode、日志、截图和完整 run tree
留在服务器；本地只回传摘要、JUnit、就绪 JSON 和一张关键截图。
