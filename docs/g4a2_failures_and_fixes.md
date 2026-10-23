# G4A-2 H0 失败与修复记录

日期：2026-10-18。本文记录工具准备阶段的失败；真人 H1 尚未开始。

## 1. Fixture agent 无产物

`/root/g4a2_fixture` 只返回通用就绪说明，没有创建文件。主负责人接管 fixture 和回执，
没有重复派发空任务，也没有把主负责人实现伪装成 agent 成果。

## 2. 页面 smoke 断言误判

首次页面 smoke 在正文中禁止字符串 `world_sim`，但页面说明文字本身包含
“不显示 world_sim”。产品页面没有第三人称容器；改为检查 DOM 中不存在
`#three-container` 和 `#snapshot-age`，复跑通过。

## 3. 新 release 静态 bundle 缺失

在 `g4a2-20261018T162535Z-5c9149d7be34` 首次运行 static UI 测试时，`app.js` 尚未在
该 release 构建，测试失败。先执行 `npm ci`/`npm run build:ui`，再运行同一测试，
结果为 `6 passed`。这是构建顺序问题，不是源码或验收门放宽。

## 4. 现有 G2B export 不覆盖 G4A-2 human action

G2B 读取旧 `actuator_reference` 且单父 Episode。G4A-2 新增独立 reader/qualification：
human session、human intent、显示 sample、普通六维请求、G4A2 link、官方 RGB-D writer
分开审计；G2B 历史结果不改写。

## 5. 小样本 RGB-D writer 限制

`lerobot==0.4.4` 的官方 writer/finalize 和 loader 已用 synthetic、2 Episode、6 frame
小样本验证；这不是训练数据。正式 human cohort 仍需在 H1 后使用真实记录重跑，
不能把 synthetic shape 通过当作 human/export/training 资格。

## 6. 真人试用发现图片闪烁及整车入口缺失说明

用户截图显示RGB破图/闪烁，以及不清楚第三人称和底盘前后左右在哪里。服务器只读核对时RGB资产接口为200/image/png且PNG签名正确；原整车`/`页面和bundle均为200，第三人称canvas及W/A/S/D/Q/E控件存在。没有用自动浏览器代替用户操作机械臂或底盘。

前端原实现每300ms用setInterval启动异步轮询并直接重写可见img.src；没有等待图片成功显示，也没有处理图片错误。显示sample及client_display时间在图片加载前被使用，不能据此证明用户实际看到了该帧。旧H0页面smoke只等待img节点，不能证明图片可解码或长期刷新稳定。

修复源码采用完成后调度的单飞轮询、新sample离屏加载/解码后替换、固定4:3画面区域、失败保留上张图并停止发送。成功绘制后才更新display sample/time；真人确认不再被动作发送器隐式调用。发送目标使用固定输入快照，5Hz发送器避免请求重叠。补充整车页面入口和录制期间切换提示，不改变本批固定底盘范围或相机挂载。

部署过程另发现本机Windows打包会把SHA256SUMS写成CRLF，以及把Windows私钥路径误转为WSL路径；修正跨平台路径条件和清单LF写入。首次失败的传输没有启动新物理实例或修改旧release。服务器系统Node过旧无法解析已有可选链；改用现有现代Chromium实际加载验证，不更改全局Node。

最终验证与部署状态以`docs/evidence/g4a2_live_deploy_acceptance_20261019.json`为准；修复前的试用记录保留，未由本次维护补标为H1合格数据。
（2026-10-19 更正：原稿此处的 `docs/evidence/g4a2_ui_preview_fix.json` 从未生成，服务器与本地均不存在；以上方 2026-10-19 部署核查与线上验收索引为准。）
