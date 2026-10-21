# 速度模式 demo

参考机器人 action 13/HUMANOID 的模型与 profile，在 SDK 中本地推理后请求 **EXTERNAL**；不是给机器人发送 action 13。`--shadow` 只读控制面，`--live` 会控制本体。需要完整公共31槽配置、已就绪反馈、匹配模型和部署 locomotion 目录。

```bash
python -m pip install --no-index --find-links ./wheelhouse 'locomotion-aorta[velocity-demo]==0.2.0'
python examples/velocity_mode/velocity_mode.py --help
python examples/velocity_mode/velocity_mode.py --shadow --locomotion-dir /path/to/locomotion --log /new/path/velocity-shadow.jsonl --duration 60
```

`--log` 使用新路径；`--duration 0`/省略为不限时。0.2.0 默认 Aorta 方向输入依赖尚未交付的匹配 Joy 绑定，不能靠安装错误消息版本补齐；选择 ROS2 输入也需部署匹配的 ROS2 环境。下文保留实际方向依赖与 S0–S6 时钟说明；日志中的推理耗时、实际 session/sequence、applied 键不等于硬件动作完成。

## 环境与安全

以下命令从 SDK 仓库或离线发布包根目录运行。安装命令面向含完整 wheelhouse 的离线发布包（含所需 extra 依赖）；缺包时先获取匹配产物，不安装别的消息版本或绕过依赖检查。使用独立 CPython 3.10 venv；依赖版本以根目录 `compatibility.json` 为准。机器人上的 SDK 主机先 source `/app/script/env.sh`，再激活 venv；PC 使用部署方提供的 peer 配置。设置 `ZENOH_SESSION_CONFIG_URI` 和与机器人一致的 `--group`，不复制认证内容。安装与 `--help` 不代表已获得运动许可。

现场运动须单独授权并确认吊装/支撑、活动空间、夹点与可用急停；由现场人员操作。`--confirm-motion` 仅为程序确认参数。异常先停测试，由现场人员使用物理急停，不自动重试、清错或重启。运动例程正常退出走 CANCEL→PASSIVE；PASSIVE 可能使负载下落，客户端崩溃不能保证释放控制权。只读与离线模式不进入 EXTERNAL，不需要或发送 CANCEL。

## 原合同与详细说明

# Velocity mode

`action13_external.py` / `Action13Policy` are now `velocity_mode.py` /
`VelocityModePolicy`. Log schema: `vita.velocity_mode_latency.v1`.
The SDK name changed; robot action 13/HUMANOID, protocol enums and model metadata
names did not. This program runs a client policy through **EXTERNAL**, not action 13.

Use a compatible SDK 0.2.0 installation with the finalized 0.1.4 message wheel
and the `velocity-demo` extra. Inference uses its bundled C++ ORT 1.20.1 backend,
loaded only when constructing the model. There is no Python ORT fallback.
Provide the deployment's exported locomotion directory (model, action profile
and main configuration), rather than a robot source checkout. The model/profile
paths remain their original deployment paths. In particular the transition
parameters are read from `config/locomotion_foot_humanoid_asample.yaml`.

```bash
python examples/velocity_mode/velocity_mode.py --help
```

`--shadow` is read-only for control: it never constructs a control publisher or
claims a lease. It queries RobotInfo, subscribes to BodyState/ControlStatus and
uses the configured direction input. It requires a separately authorized
connection and the deployment-owned peer session configuration. `--live` claims
EXTERNAL and moves joints; authorization and support/emergency-stop conditions
must be confirmed for that specific trial. Neither mode changes robot settings.
Live exit uses CANCEL; PASSIVE can let loaded limbs fall.

BodyState and BodyCmd have 31 slots. Model joint names and RobotInfo determine
the mapping, including wrists and the two fixed neck joints. Waist-roll is an
unpopulated logical slot; it is excluded from the transition. Fingers are no
longer written through body commands. Their separate clients are unaffected.
BodyInfo, rather than a NONE error code, indicates installation/Mock status.
Before live entry, RobotInfo must mark the model joints and each public fixed
joint used by the policy as populated. Missing or unpopulated joints are listed
in the startup error; EXTERNAL is not entered.

The model's 28-joint order, history, scaling, clipped-action feedback, target
limits and ankle feedforward remain unchanged. Current velocity asset metadata
is 93 values × 5 history frames; validate the supplied model's actual metadata.
The asymmetric vx/vy/wz ranges, deadbands, 1s input freshness,
50Hz schedule and initial transition retain the source profile's values.
Duration omitted or zero remains unlimited; a positive duration is bounded.
The original finally/CANCEL/close path remains active for SIGINT, SIGTERM and
errors. Scheduling, transition progress and duration use monotonic time;
S0–S6 retain their original REALTIME log clocks. No BodyState advance for the
existing 200 ms live-feedback window stops new targets. Late cycles wait a
full period before resuming; they do not publish catch-up bursts.

## Direction dependency

SDK 0.2.1 uses only Aorta `locomotion/joy`, through the SDK's `JoyReader`.
There is no direction-source CLI option, ROS input subscription or velocity
fallback. Neutral, missing or stale input produces zero direction through the
existing profile. This requires the matching public humanoid **0.1.5** wheel,
Aorta 2026.10.20 and FlatBuffers 25.12.19. The dependency pin and installable
0.2.1 bundle follow generation and integration; the older 0.1.4 public wheel
has no Joy binding and cannot supply this entry point. Do not mix an older
message wheel into the SDK to obtain it.

```python
from locomotion_aorta import JoyReader

with JoyReader(node) as reader:  # reuse the deployment-configured Aorta node
    sample = reader.read()     # call once per policy tick; drains internally
    axes = sample.axes if sample.is_fresh() else ()
    presses = sample.pressed_counts.get(0, 0)
    releases = sample.released_counts.get(0, 0)
```

The reader creates one pull subscription and no Python callback/worker thread.
It reads until the ring is empty on every tick, including the initial
transition. Axes/buttons come from the final frame; press/release counts compare
every adjacent frame (zero versus nonzero), including multiple short presses
within one tick. A successful read consumes the returned increments and keeps
the last state, receipt time and cumulative gap count. Empty reads return zero
increments without refreshing the receipt time. First samples establish a
baseline; adding/removing button indices does not invent edges.

Snapshots are immutable and single-consumer. `received_realtime_ns` is direct
CLOCK_REALTIME immediately after pull returns, not network arrival time;
freshness requires both this receipt and the AortaHeader producer publication
stamp to be within 1s. Robot/client REALTIME clocks must be synchronized;
unknown, future or old source stamps produce neutral direction. Draining an
old backlog cannot refresh its source age. `missing_samples_since_read` and
`missing_samples_total` count forward gaps in the payload AortaHeader's uint64
sequence (zero is valid). Duplicates, resets/reordering, wrap and source changes
are diagnosed separately. Missing header/source identity is unknown, not zero
loss. `event_history_complete` is false for a known gap or ambiguous interval;
it does not recover events missing from those frames. `sequence_diagnostics`
and per-cycle event counts are also included in the velocity log.

Duplicate or older sequences do not overwrite axes, buttons, receipt time or
the sequence baseline. A same-identity sequence reset must catch up, or use a
new publisher identity; uint64 contiguous wrap remains valid. Malformed frames
are skipped and counted by `decode_errors_total` and the `decode_error`
diagnostic, without refreshing freshness or discarding previously counted
edges. Transport failures still propagate.

The default ring depth is 256. Size it from the actual peak input rate with at
least ten 50Hz ticks of headroom: next power of two of
`max(256, ceil(peak_hz * 0.2) + 2)`. The 20Hz app-agent refill timer is not an
upper bound on direct input publication; the actual peak is not measured here.
Ring overflow silently evicts the oldest frame, and the underlying pull
`dropped_count()` stays zero. Sequence gaps can reveal missing frames after a
baseline, but cannot locate the loss or recreate missed presses. The reader
never caps the number drained per tick or claims lossless delivery during an
unbounded stall. Close it before closing its node.

## Records and offline acceptance

S0 is BodyState.ts_state_pub_us; S1 is the Python decoder-entry timestamp and
S2 the completed materialization timestamp. S3–S6 retain their original
observation/inference/publish-call boundaries. These are CLOCK_REALTIME ns;
S1 is not a native receive timestamp. Reusing a state retains its original
S0/S1/S2. SDK-owned client created/send timestamps use the SDK's own clock.
Do not subtract robot and client clocks without independent synchronization.

Publish returns the actual session_id/sequence. Those uint64 keys identify a
successful SDK call; failed calls retain unknown keys and their error record.
The wire has no cmd_id. A shadow candidate_cycle is only a local log number.
BodyState's applied_session_id/applied_sequence/ts_first_output_us are recorded
unchanged. Join only exact session/sequence pairs. Zero output stamps are
unobserved, and a first-output stamp describes the control cycle, not hardware
completion. L1/L2 remain NOT_MEASURED here.

`tests/velocity_mode_parity.py` compares the fixed robot C++ observation/target
oracle with the native-backed demo on 128 inputs. Its robot header/compiler
arguments are **acceptance-only**, not customer dependencies.
`tools/compare_policy_onnx.py --kind velocity` runs a separate Python ORT 1.20.1
oracle on exactly the same float32 tensors. Finite numeric equality is exact;
+0/-0 compare equal. There is no new epsilon. Independent compilation,
installed-wheel, numerical and ARM acceptance are queued separately, not
established by source unit tests or this document.
