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

## OP3 往复模式

OP3 支持 `target`、`sine`、`simultaneous` 和 `sequential`。这些模式在同一个 EXTERNAL 会话内运行，仍使用原始位置反馈作为命令输入，不自动开启触觉。

- `sine` 要求 `--side both`：双手同相正弦，`--sine-min-m`／`--sine-max-m` 为总开口范围，`--period-s` 为周期。先从实测起点插值到中点，结束后返回原始起点；`--duration` 是正弦阶段时长。
- `simultaneous`：所有选中端点从各自 Info 范围的 `--low-frac` 同时到 `--high-frac` 再返回；`sequential` 每次只改变一个端点的一轴。两者可用 `--loop` 持续往复，不以 `--duration` 截止。
- OP3 往复必须显式提供每侧正速度限制；转换成 float32 后也必须大于 0。正弦峰值斜率 `π × (max-min) / period` 不得超过该限制。轨迹斜率限制不代表物理跟踪性能。

例如 10–100 mm、10 秒周期，以下是需要现场运动授权的双夹爪示例；`--ee-only` 不保持本体：

```bash
python examples/end_effector/omnipicker3.py \
  --group "$AORTA_GROUP" --side both --pattern sine \
  --sine-min-m 0.010 --sine-max-m 0.100 --period-s 10 --duration 60 \
  --left-unit m --left-vel-limit 0.04 --left-current-a 3.5 \
  --right-unit m --right-vel-limit 0.04 --right-current-a 3.5 \
  --ee-only --touch keep --confirm-motion
```

## 逐周期 JSONL

给运动命令增加 `--log /absolute/new-run.jsonl` 可启用记录。该路径必须不存在；默认不写日志，`--dry-run` 不接受此选项。复用本体例程的 `policy_support.CycleLog`，记录、flush 和汇总需要 NumPy（`state-arrays` extra）。省略 `--ee-only` 的本体数组反馈读取同样需要 NumPy。

每次单侧实际发布尝试写一条 `cycle`，保留左右独立频率：阶段、请求频率、实际命令键和输入 SampleRef、目标和字段掩码、原始位置 State、最新状态及有效位、本地原始位置接收时间和年龄、相邻同侧发布间隔、publish 调用耗时及返回／异常。Info 的轴顺序、单位域、量程和 runtime/layout 身份在 `end_effector_info` 记录中保存。无效或未选择的非有限数值写为 `null`，必须结合有效位解读。

启用本体保持时，同一个文件还记录每次活动控制帧的 `body_state_read`（读取调用耗时，包括等待、解码及 SDK 校验），以及每次实际 BodyCmd 的 `body_cycle`。后者关联对应读取记录和真实输入 state_id，保存关节测量／目标、原始接收时刻、状态年龄、命令键、发布间隔／耗时及已有应用键。`body_summary` 单独汇总本体读取和发布；本体与左右末端共用原来的一个控制会话和递增命令分配器。纯解码耗时没有单独测量，活动帧之外的启动预检不计入读取统计。

在创建 controller 前设置 `LOCOMOTION_AORTA_PUBLISH_TIMING=1` 可额外记录已有的 BodyCmd 发布分段；末端未提供的分段仍为 `NOT_MEASURED`。使用本地原始接收时刻和发布开始／耗时可计算“SDK 本地接收至该次发布返回”的软件耗时；这包含缓存等待，不是 CAN 往返或电机到位时间。

`S0` 原样采用该位置 State 的发布时间戳；`S5/S6` 是本机 CLOCK_REALTIME 调用边界。发布间隔、调用耗时和反馈年龄使用本机 CLOCK_MONOTONIC。`S1–S4`、底层 publish 分段、固件执行和物理到位时间未观测，写为 `NOT_MEASURED`；不跨时钟域直接相减推断链路延迟。

退出时的 `end_effector_summary` 按左右侧输出尝试／返回次数、实测尝试频率，以及发布间隔、调用耗时、反馈年龄的 p50/p95/p99/max。每周期另有 `logging_overhead`，最终 `logging_summary` 汇总 JSON 序列化／写入／flush 开销。正常或故障退出先由原控制流程尝试 CANCEL，再汇总和关闭日志。同步写日志会影响实际节奏；不补发追赶，统计包含这项影响。长时间记录会占用磁盘及用于精确分位数的内存；保留对应 bundle manifest 和机器人版本信息以供比对。

## 公共前提

使用匹配 bundle 的 CPython 3.10 venv；依赖以根 `compatibility.json` 为准。PC 在创建 Node 前设置私有 `ZENOH_SESSION_CONFIG_URI`，并让 `AORTA_GROUP` 与 `--group` 一致。S100 本机方式先 source `/app/script/env.sh` 再激活 venv。

完整协议、单位、控制时序与图示见[SDK 手册](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh)；新版页面以正式发布回读为准。

运动须单独授权并确认当前现场条件。ACK accepted 不是 RUNNING，publish 返回不是到位。正常退出 CANCEL→PASSIVE 可能使负载下落；异常、超时或退出不完整时不自动重试。
