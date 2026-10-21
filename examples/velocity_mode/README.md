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
The asymmetric vx/vy/wz ranges, deadbands, twist/joy priority, 1s input freshness,
50Hz schedule and initial transition retain the source profile's values.
Duration omitted or zero remains unlimited; a positive duration is bounded.
The original finally/CANCEL/close path remains active for Ctrl-C and errors.

## Direction dependency

The default remains `--joy-source aorta`, subscribing to `locomotion/joy`.
It additionally needs the deployment-matched generated `locomotion.Joy` Python
binding and a ROS2 environment for the existing `/vel_cmd` subscriber. The
final public SDK wheel set does **not** include that Joy binding. This is an
open packaging dependency, not a reason to install the old rc.3 message wheel
over the new SDK. Until a matching binding is handed off, the default Aorta
input is **not ready for customer use**. No decoder is hand-written here and
no automatic input-source fallback occurs.

`--joy-source ros2` explicitly uses `/joy` and `/vel_cmd` in that deployment's
ROS2 environment. It does not bridge topics, publish joystick data, or change
the profile's axis/range mapping. Selecting it is a user's explicit choice.

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
