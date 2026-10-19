# G4A-1 开发失败与修复记录

日期：2026-10-18。本文保留开发失败和修复，正式六例只使用最终 release。

## 1. 受限观测 timing 读取错误

首轮 HTTP 500：`capture_*` 字段实际位于 capture `timing` 子结构，代码误读 metadata
顶层。修复后恢复 RGB/Depth/mask 与 capture/arrival 绑定；该轮没有发生动作。

## 2. fast sensor wheel qpos 索引错误

首轮物理线程在 `wheel_joint_positions_rad` 处抛出 ndim>1 索引错误，physics thread
退出而 sensor thread 继续。将 qpos 地址显式转为标量后恢复唯一物理推进线程。

## 3. prepared base 沉降误差

预置 base `x=0.138` 在落地后实际稳定在约 `0.129`，比 G1 抓取位远约 9 mm。保持自由
底盘，不焊死或改动力学；将 Episode 前 prepared qpos 修正为 `0.1473`，落地后实际
约 `0.1386`。正式物理成功门不变。

## 4. 动作权限和 lease 过期

首版 action 把用户权限过期时间错误缩短为 0.5 s，后续帧被拒绝；之后又发现浏览器
heartbeat 与轨迹时长不一致。权限和 arm lease 分开：权限 TTL 冻结为 120 s，六维动作
仍要求新鲜 observation 和有效 session；Stop/接管/Reset 仍立即撤销。

## 5. 图像新鲜度竞态

浏览器拿到观测后准备 UI 时 frame 已超过 0.4 s。未放宽 0.4 s 门；在动作按钮发送前
等待 `observation_backend_monotonic_s - capture <= 0.35 s`，并允许因果上已知的最近
样本在 backend cache 中核对 arrival/age。缓存不会把旧帧冒充新 sample。

## 6. 抓取轨迹与夹爪同帧闭合

直接把抓取动作与夹爪闭合放在同一个下探帧会在接触前闭合，导致物体未随动。保持六维
原子接口，但拆成两个顺序动作帧：先到抓取位、夹爪 25 mm 开；验证到达和稳定目标后，
再发送同目标、夹爪 18 mm 闭合。正式成功仍要求物理抬升 ≥0.05 m 和双侧保持 ≥2 s。

## 7. TCP 目标与碰撞边界

`.124 m` 目标被 `gripper_right_link/pick_platform` 碰撞拒绝；`.129 m` 可到达但没有
稳定双侧接触；最终冻结 `.127 m`。中央成功使用约 `0.096 m` 抬升和约 `7 s` 保持。
不缩小 footprint、不关闭碰撞、不改摩擦/重力。

## 8. 长 teacher trajectory 与超时

最终抓取前的长 teacher 轨迹实际需要超过 60 s 墙钟，首轮六例在 plan wait 处超时。
不改物理门，而是把该段 wait/Phase 上限调为 90 s；完整 Episode 仍由 120 s sim /
180 s wall 上限约束。

## 9. 历史例外

`tests/integration/test_provenance.py` 对历史 `contacts.py` hash 的既有失败保留；本轮
未修改该文件，未将其重标为通过。
