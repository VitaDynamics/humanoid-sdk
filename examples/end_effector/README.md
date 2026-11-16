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

`brainco_revo2.py`、`agx_gripper.py`、`linker_l6.py`、`omnipicker3.py`
均调用这个公共入口，直接接受上述参数，不维护另一套厂商专用 Trace 实现。
独立本体归零／返回例程 `examples/external_control/external_control.py`
也支持相同参数。所有 demo 只向 controller 构造函数传入录制选项；原来的
`controller.publish(...)`、`client.publish(...)` 和 `wait_state(...)` 不需要包装。

自己编写程序时同样只需开启一次：

```python
with PullController(trace_output="trial.ftrace") as controller:
    # 保留原发现、能力校验、握手和授权运动流程。
    # EndEffectorClient(connection=controller) 自动共用这个 recorder。
    ...
print(controller.trace_health)
```

SDK 在原 publish 入口、真实键分配完成、返回／失败处记录，已有的 Body、
末端 State/Touch 接收回调也直接采集。`--ee-only` 仍不发送本体命令。
Touch 只记录原客户端已订阅且已接纳的样本，不因录制创建新订阅或打开触觉。
原 RPC 成功不当作新的 State/Touch 样本或物理触觉成功。

一个连接对应一个文件、一个 native writer。调用线程和既有反馈线程使用
预分配的独立通道；通道不足或采集竞争会报告不完整，不阻塞控制线程。
文件记录的是 SDK 调用与原反馈语义，不能冒充 CAN 发送或设备执行证明。
时间均为微秒，原来源时钟保留区分；不把 Core context stamp 当网络到达。
取件/解码细分、模型/WTT、MCAP/HTML 和性能验收仍未完成。

录制故障不新增 CANCEL、不覆盖原运动异常。随 controller 的原退出／关闭流程有界排空采集；
队列满或丢记录会反映在健康账本中。读取到文件末尾后再检查完整性：

```python
from locomotion_aorta.trace.reader import Reader

reader = Reader('readonly.ftrace')
for event in reader:
    print(event['kind'], event['timestamp_us'], event['state_id'])
print(reader.integrity.complete, reader.integrity.issues)
```

`complete` 只说明采集文件的完整性，不表示控制成功或物理设备测试通过。
