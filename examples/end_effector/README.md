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

## Trace 记录（开发中）

本 PR 的通用入口和上述四个厂商入口支持同一组 Trace 参数。需要包含
`locomotion_aorta.trace` 及其匹配 native 模块的候选 wheel；不代表 RC1
既有安装包已经包含此功能。

```bash
# 只读示例：不申请控制权，不发送目标或触觉开关。
python examples/end_effector/brainco_revo2.py --dry-run --side left \
  --trace=fbs --trace-output=./readonly.ftrace
```

在原来已获授权的运动命令后增加相同两个参数，即可记录原控制流程。
默认 `--trace=off` 不创建 Trace 会话、队列或文件；文件必须是新的 `.ftrace`，
不会覆盖已有内容。`--trace-compression=raw` 可关闭压缩，默认使用 Zstd。
无需使用者自己为本体、左末端、右末端各写一个 `with`。

一个文件、一个 native writer 记录现有本体读取、保持目标、左右末端目标，
以及实际返回的命令键和所选 SampleRef；不额外读取反馈或创建订阅。
`--ee-only` 仍不发送本体命令。触觉开关继续由原 RPC/State/Touch 流程确认；
Trace 保存确认时的原始区域时间、有效位与负载，不增加连续触觉订阅。

记录的是应用调用入口/返回及已有反馈快照，包含重复或无效样本，不表示
机器人已接纳、CAN 已发送或设备已经执行。内部键分配、取件/解码时间、
连续触觉、MCAP/HTML 分析和整条链路仍未接完。时间均为微秒；原来源时钟
保持区分，不把 Core context stamp 当网络到达，也不直接计算跨时钟延迟。
局部调用区间包含起始 Trace 标记的采集开销，不能冒充纯 SDK 执行耗时。
native、Python 准备快照的总开销和 ARM 安装验证需分别实测。

录制故障不新增 CANCEL、不覆盖原运动异常。原 CANCEL 完成后才关闭采集；
队列满或丢记录会反映在健康账本中。读取到文件末尾后再检查完整性：

```python
from locomotion_aorta.trace.reader import Reader

reader = Reader('readonly.ftrace')
for event in reader:
    print(event['kind'], event['timestamp_us'], event['state_id'])
print(reader.integrity.complete, reader.integrity.issues)
```

`complete` 只说明采集文件的完整性，不表示控制成功或物理设备测试通过。
