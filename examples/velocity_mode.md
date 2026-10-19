# Velocity mode

`action13_external.py` / `Action13Policy` are now `velocity_mode.py` /
`VelocityModePolicy`. Log schema: `vita.velocity_mode_latency.v1`.
The SDK name changed; robot action 13/HUMANOID, protocol enums and model metadata
names did not. This program runs a client policy through **EXTERNAL**, not action 13.

Use a compatible SDK 0.2.0 installation with the finalized 0.1.3 message wheel
and the `velocity-demo` extra. Inference uses its bundled C++ ORT 1.20.1 backend,
loaded only when constructing the model. There is no Python ORT fallback.
Provide the deployment's exported locomotion directory (model, action profile
and main configuration), rather than a robot source checkout. The model/profile
paths remain their original deployment paths. In particular the transition
parameters are read from `config/locomotion_foot_humanoid_asample.yaml`.

```bash
python examples/velocity_mode.py --help
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
