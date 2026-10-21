# WBC 跟踪模式 demo

对应机器人 action 14/HUMANOID_TRACKING 的模型合同，在 SDK 内推理并以 **EXTERNAL** 输出31槽命令；不触发机器人内部 action 14。模型由使用者提供，需匹配 metadata；直播需要有效 RobotInfo/反馈与现场确认。

```bash
python -m pip install --no-index --find-links ./wheelhouse 'locomotion-aorta[wbc-tracking-demo]==0.2.1'
python examples/wbc_tracking_mode/wbc_tracking_mode.py --help
python examples/wbc_tracking_mode/wbc_tracking_mode.py --model /path/to/actor_g1.onnx --dry-run --steps 100 --log /new/path/wbc-offline.jsonl
```

`--dry-run` 合成反馈，不连接机器人；`--replay-mcap` 使用匹配BFBS的公共 BodyState；`--live --confirm-motion` 才进入控制。`--motion` 为可选命名关节 NPZ，省略时保持平衡站姿参考；`--duration` 限定直播时间。日志 `vita.wbc_tracking.v1` 含推理/观测及实际 session/sequence；没有 `transition_duration` 字段，不把 applied 时间当设备 ACK。

## 0.2.1 已知问题与后续

WBC 当前不会根据驱动关节的 `error_code` 拒绝策略输入。若关节 TIMEOUT/故障后仍有
有限的缓存 q/dq，且整帧时间持续推进并通过已有新鲜度检查，这些数值仍可能进入策略；
日志记录错误码不代表输入已通过关节健康检查。不要把此 demo 用作故障关节的安全过滤器。
0.2.1保留此已知问题，现有 WBC 控制逻辑不变。

hum-230 本轮复测仍有约5.0%的周期超时，瓶颈为观测构造与推理；这是现场配置下的
观测，不是性能保证。日志缩减至约5.4 KB/步、丢弃0并没有消除计算瓶颈。
现有计算/新鲜度截止条件保持不变，0.2.1不再继续优化。WBC demo保留到0.2.1，
计划在0.2.2删除；本分支尚未执行删除。

## 日志容量与完整观测

默认每步记录 `observation_length`、`observation_sha256`，不保存完整 observation。
SHA256 输入是观测按 C 顺序展开后的 little-endian float32 原始字节，保留正负零。
需要逐项离线比对时显式加 `--log-observation`；此时完整 `observation` 与长度、摘要一起保留。
`decoded` 和 `state` 默认仍完整记录，供命令、限位与反馈比对；它们合计约3.6 KB/步，
本次现场 observation 约25.8 KB/步，是主要日志体积来源。实际大小随模型与反馈变化。

控制循环只提交独占的观测/状态快照到有界队列（256条）；后台线程计算摘要、转换数组、
序列化 JSON 并写盘。队列满时丢弃新行并计数，不等待磁盘、不改变原50 Hz节拍。
最终 `summary.logging`（失败时可能是独立 `logging_summary`）报告
`records_submitted/enqueued/written/dropped`、`queue_max_depth/capacity` 和 `writer_error`；
written包含最终summary，enqueued不包含单独保留的summary，不能用日志行数代替策略步数。
日志队列有损，缺失行不能推断为命令未发送。

退出先按原流程尝试 CANCEL→PASSIVE，再排空已接收日志、flush并等待写线程结束。
磁盘或序列化失败会在关闭时报告并令正常运行退出为错误；已有控制异常仍是主异常，
不会被日志关闭异常覆盖。磁盘完全不可写时最终summary也可能无法落盘，此时计数写到stderr。
本修改不调整新鲜度、计算超时、策略频率、增益、目标或运动准入。

## 环境与安全

以下命令从 SDK 仓库或离线发布包根目录运行。安装命令面向含完整 wheelhouse 的离线发布包（含所需 extra 依赖）；缺包时先获取匹配产物，不安装别的消息版本或绕过依赖检查。使用独立 CPython 3.10 venv；依赖版本以根目录 `compatibility.json` 为准。机器人上的 SDK 主机先 source `/app/script/env.sh`，再激活 venv；PC 使用部署方提供的 peer 配置。设置 `ZENOH_SESSION_CONFIG_URI` 和与机器人一致的 `--group`，不复制认证内容。安装与 `--help` 不代表已获得运动许可。

现场运动须单独授权并确认吊装/支撑、活动空间、夹点与可用急停；由现场人员操作。`--confirm-motion` 仅为程序确认参数。异常先停测试，由现场人员使用物理急停，不自动重试、清错或重启。运动例程正常退出走 CANCEL→PASSIVE；PASSIVE 可能使负载下落，客户端崩溃不能保证释放控制权。只读与离线模式不进入 EXTERNAL，不需要或发送 CANCEL。

## 原合同与详细说明

# WBC tracking mode

The SDK `tracking_policy.py` example is now `wbc_tracking_mode.py` and its
policy class is `WbcTrackingPolicy` in `locomotion_aorta.wbc_tracking_policy`.
Profile: `wbc_tracking_asample.json`; log schema: `vita.wbc_tracking.v1`.
Robot action 14/HUMANOID_TRACKING and `groot_wbc.tracking_contract.v1` metadata
are unchanged. This SDK program still requests **EXTERNAL**.

Use matching SDK 0.2.1 and finalized 0.1.5 bindings with `wbc-tracking-demo`.
Model construction lazily loads the built-in C++ ORT 1.20.1 Session, with the
existing `--threads` intra-op setting (default 1) and inter-op=1. There is no
runtime Python ORT dependency or fallback.

```bash
python examples/wbc_tracking_mode/wbc_tracking_mode.py --help
python examples/wbc_tracking_mode/wbc_tracking_mode.py --model /path/to/actor_g1.onnx \
  --dry-run --steps 100 --log /new/path/wbc-offline.jsonl
```

Dry-run uses synthetic public31 feedback and never constructs a node. Model
metadata controls the 28 actor joints, future-frame layout and observation
shape; the input dimension is not hardcoded. NPZ joint-name reordering, 20ms
resampling, terminal velocity, heading, lead-in, recurrent history, clipping,
limits and ankle feedforward retain the source policy's mathematics and checks.
The fixed neck targets remain q=0/kp=20/kd=1. No finger command is sent.

`--replay-mcap FILE` accepts the exact public `/sdk/foot_humanoid/body_state`
FlatBuffer topic with matching embedded BFBS. It does not decode old ROS
44-motor lowstate recordings. The offline profile carries the public A-sample
joint order; use a profile matching the recording's roster. Missing callback
clocks stay unknown rather than being synthesized from MCAP log_time. This
mode does not claim a lease or publish commands.

Live requires `--live --confirm-motion`, an explicitly authorized connection,
the deployment-owned peer session configuration, and confirmation of support,
clearance and emergency stop for the trial. It queries actual RobotInfo names,
uses public PullStateSource snapshots and requests EXTERNAL. Pull snapshot reception
alone is not source-time advancement: the original increasing source-time and
50ms freshness/compute checks remain. The original ownership/reject/drop checks,
50Hz schedule without catch-up bursts, stop-before-entry behavior and
CANCEL/PASSIVE verification remain. Exit to PASSIVE can release loaded limbs.
An exception is not permission for automatic retry or a service restart.
For pull feedback, the 50 ms age bound checks both the original local cache
receipt and `ts_state_pub_us`, the robot's REALTIME publication stamp. Robot and
client REALTIME clocks must be synchronized; missing/future publication stamps
are rejected. Core's receive stamp is a dequeue stamp and does not establish
arrival age. Source-time advancement is still required independently.
Before requesting EXTERNAL, RobotInfo must contain all model joint names and
the fixed neck names, with each of those joints marked populated. Startup
errors list the missing or unpopulated names and do not take ownership.

Body commands/state have 31 slots. q/dq/tau/kp/kd are in the public joint domain;
error_code and sampling clocks are retained. Installation and Mock identity
come from RobotInfo. Trace keys are uint64 session_id/sequence; input_state_id
is uint64 and never wrapped. Inference timing/max_abs_obs remain JSONL evidence,
not invented BodyCmd fields. The flat applied tuple is recorded unchanged;
a zero first-output stamp is not a measured output. Local monotonic durations
and client realtime timestamps are recorded separately from robot clocks.

Tests retain the independent historical C++ two-tick observation golden and
its source/model metadata provenance. `tools/compare_policy_onnx.py --kind wbc`
adds same-version Python/native ORT 1.20.1 exact finite float32 inference
comparison in separate interpreters (+0/-0 equal). Historical model IDs remain
historical, not claims about another supplied actor. Source units, real binding
tests, installed-wheel/native numerical acceptance, ARM execution and hardware
motion are separate evidence stages. The latter stages are not measured here.
