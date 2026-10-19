# C 数据执行回执

- 合同版本：`mobilecar-1.0`
- 独占路径：`src/mobilecar/data/**`、`tools/data/**`、`tests/data/**`、`docs/workpackages/C/**`
- 禁止路径：`src/mobilecar/contracts/**`、`src/mobilecar/app.py`、`src/mobilecar/cli.py`、`src/mobilecar/config.py`、A 后端/规划/传感器目录、B UI 目录、根依赖文件、`tests/contracts/**`、`tests/integration/**`
- 公共依赖：`EventRecord`、`SensorObservation`、`RobotSnapshot`、`EpisodeStartRequest/Response`、`RecordingState`、`ValidationReport`、`TaskAssessment`、`DataEligibility`
- 对外装配：`Recorder`（实现 `publish`、`publish_observation`、`recording_gate`、`status()`、`start`、`stop`、`validate`、`close`）；`ReplayReader(root)`；`validate_episode(root, episode_id)`；`assess_grasp_oracle(...)`；`evaluate_eligibility(...)`
- 验收命令：`PYTHONPATH=src python3 -m pytest tests/data -q`；`PYTHONPATH=src python3 -m mobilecar.data --root runs list`
- 边界：Recorder/Replay/Validator 不读取可写 MuJoCo 状态、不调用 `mj_step`；回放只读；Oracle 只消费显式 GT_DEBUG 样本并写入 `assessments/`。

本执行者未派发嵌套 agent。
