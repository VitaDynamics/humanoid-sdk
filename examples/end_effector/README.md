# 统一末端、手指和 OP3 示例

| 文件 | 设备 |
| --- | --- |
| end_effector_external.py | 通用 Info/State/Target 合同 |
| brainco_revo2.py | BrainCo Revo2 |
| linker_l6.py | Linker L6 |
| agx_gripper.py | AGX |
| omnipicker3.py | OmniPicker3 |

```bash
python examples/end_effector/omnipicker3.py --help
python examples/end_effector/omnipicker3.py --dry-run --ee-only --side left --group "$AORTA_GROUP" --timeout 5
```

--dry-run 会连接并只读检查，不请求控制、不发目标或触觉开关。--ee-only 不发隐式 Body，但仍绑定 RobotInfo/runtime。写操作共享 controller，使用 Info 固定 N 轴顺序和 SI 单位；完整 Body 例程与 EE-only 不能混作相同的本体保持行为。

## 目标和反馈

- --left-pos／--right-pos：完整 N 项；--left-unit／--right-unit 显式 m 或 rad。m 为总开口，不是单指 mm。
- --left-current-a／--right-current-a：A，须为本台 Info 允许字段／模式／范围；省略无意图，显式 0 不改成默认电流。
- --left-vel-limit／--right-vel-limit 是非负速度限制，不是有符号速度目标。tau 与 current 不推算。
- 通用例程为支持速度限制的非 OP3 角度端点保留 float32 的 20°/s 默认值，仍须落在各轴真实范围；不合法时显式提供合法数组。总开口的该默认值为 0（保留机器人默认），OP3 不自动注入速度限制。不要把这些例程选择误当 SDK DTO 的通用默认值。
- --left-hz／--right-hz 为提交节奏；--duration 为流送时长，不保证物理采样率或到位。
- AGX 另外要求实际有效配置的 native send_frequency、对应 --left-native-hz／--right-native-hz 及 --native-config；不按客户端 hz 猜它。
- wait_output_ready 只表示本机 Cmd 资源；机器人 ready、output_allowed 和有效新 State 分别检查。终端表格刷新率不改变发送率。

触觉按 Info/区域有效位处理。无能力时例程 --touch off 明确 no-op、不发 RPC、不声称 ACK；--touch on 拒绝，keep 不改现状。Stop、Hold、CANCEL、撤权和释放物体不是一个动作。

OP3 回复无事务号，有界同 key 读重试不能严格分辨迟到回复。软件命令键和 publish 成功不证明某轮设备执行或持物。该例程不是自动总线扫描或原始 CAN 工具。

## 公共前提

使用匹配 bundle 的 CPython 3.10 venv；依赖以根 `compatibility.json` 为准。PC 在创建 Node 前设置私有 `ZENOH_SESSION_CONFIG_URI`，并让 `AORTA_GROUP` 与 `--group` 一致。S100 本机方式先 source `/app/script/env.sh` 再激活 venv。

完整协议、单位、控制时序与图示见[SDK 手册](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh)；新版页面以正式发布回读为准。

运动须单独授权并确认当前现场条件。ACK accepted 不是 RUNNING，publish 返回不是到位。正常退出 CANCEL→PASSIVE 可能使负载下落；异常、超时或退出不完整时不自动重试。
