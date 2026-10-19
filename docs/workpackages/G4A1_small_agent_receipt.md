# G4A-1 小型 agent 回执

日期：2026-10-18。

## 派发

- Agent：`/root/g4a1_fixture`
- 唯一文件范围：`tests/g4a1/fixtures/nearfield_permission_v1.json` 和本回执。
- 任务：创建中心/偏移两个合成权限绑定及一个 stale epoch 拒绝样例。

## 实际结果

Agent 只返回“已加载约束，请发送具体任务”，没有新增或修改任何文件。判定为无有效
产物；按合同不回派同类空任务，由主负责人接管。

主负责人实际创建：

- `tests/g4a1/fixtures/nearfield_permission_v1.json`
- `docs/workpackages/G4A1_small_agent_receipt.md`

Fixture 明确为 `synthetic_fixture` / `NOT_SERVER_EVIDENCE`，不含 world truth、Oracle、
凭据或私有连接信息。它不是地图、物理、采集或训练资格证据。

## 边界

- Agent 未运行 pytest、MuJoCo、渲染或浏览器。
- Agent 未派发嵌套 agent，未提交或推送 Git。
- 后续 A/B/C 只有在主负责人完成该 fixture 的服务器合同检查后才可以扩大。
