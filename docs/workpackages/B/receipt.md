# B 界面执行回执

- 合同版本：`mobilecar-1.0`
- 冻结依据：`docs/interface_contract_v1.md`、`docs/runtime_api_v1.md`、`docs/workpackages/G1.md`
- 独占路径：`src/mobilecar/ui/**`、`tests/ui/**`、`docs/workpackages/B/**`
- 禁止路径：`src/mobilecar/contracts/**`、`src/mobilecar/app.py`、`src/mobilecar/cli.py`、`src/mobilecar/config.py`、根 `package.json`/`package-lock.json`、`configs/default.yaml`、`tests/contracts/**`、`tests/integration/**`
- 依赖接口：`Capabilities`、`RobotSnapshot`、`SceneManifest`、`CommandRequest/Response`、`PlanPreview`、`EpisodeControlResponse`、`ValidationReport`，以及运行时 API v1。
- 验收命令：`npm run build:ui`；`python3 -m pytest tests/ui -q`；连接真实应用后进行 Playwright 浏览器交互验证。
- 验收重点：同源三维整车、轨道视角、键盘按住/释放/失焦停止、输入框不驾驶、五轴与夹爪预览/执行、TCP 输入/拖动、预览→执行、停止/取消/复位、录制和只读回放。

本执行者不派发嵌套 agent，不直接读写 MuJoCo 状态，不提交 Git。
