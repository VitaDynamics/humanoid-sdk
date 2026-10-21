# Production-topic Mock (explicit, synthetic self-test)

This optional tool emulates the public humanoid endpoints in a **new Aorta
group and private TCP loopback peer session**. It never opens CAN, sends to the
robot router, changes services or installs a package. It is separate from
`tests/latency`: the original `/bench` benchmark, 11×60 s commands, rate sweep
and gates remain unchanged.

The C++ endpoint provides:

- `/sdk/foot_humanoid/body_state`: BODY2,31 slots, two synthetic IMUs, default
  400 Hz. Joint14 is declared unpopulated in RobotInfo. Provide `--initial-q`
  with31 finite JSON values matching the client's initial-pose guard; the
  default zero pose is a protocol fixture, not a robot standing pose.
  `--initial-imu` supplies two normalized HIP/TRUNK WXYZ JSON quaternions
  matching the client's approved alignment; the default is identity. The tool
  does not normalize a bad fixture or weaken the client's attitude guard.
- `/sdk/foot_humanoid/get_robot_info`: public typed RPC, canonical H1 names,
  synthetic motor identities/ranges and selected optional EE capabilities.
- `/sdk/foot_humanoid/control/{request,ack,status}`: PREPARE/COMMIT/CANCEL,
  request/client/lease identity, idempotent matching requests, busy/conflict/
  identity/lease/version rejections, prepared-entry timeout and command-stream
  TIMEOUT_HOLD. Running ownership has no automatic expiry. This is a public
  protocol test double, not the robot FSM, safety admission or recovery logic.
- `/sdk/foot_humanoid/body_cmd`: validates protocol/action/31 slots, owned
  session/increasing sequence, finite installed-joint values and synthetic
  ranges. Latest accepted command is applied at a configurable **synthetic**
  tick, default500 Hz. Subsequent BodyState echoes `applied_session_id`,
  `applied_sequence` and `ts_first_output_us`. Commands replaced before an
  apply tick remain recorded as accepted but unapplied.

## Build (pkg on shared build machines)

Use the integration revision's normal matching wheelhouse, including real
generated public schema bindings. This script does not download dependencies
or change the lock/environment. Supply the FlatBuffers archive pinned by the
unchanged benchmark builder:

```bash
python tools/production_topic_mock/build.py \
  --build-dir /tmp/new-production-mock-build \
  --flatbuffers-archive /path/to/verified-flatbuffers.tar.gz \
  --run-protocol-checks
```

Targets: `production-topic-mock`, `protocol-checks` and `callback-checks`. The build manifest
records source/core/compiler/BFBS/binary identities. Native protocol checks
cover ownership, idempotency, rejection, timeout, latest-wins application and
CANCEL. Callback checks inject ACK/reply/cleanup errors without creating a
Node; their expected negative cases print named diagnostics. They also test
missing/overlong client IDs, malformed messages and transactional ACK failure.
For an ARM cross-build add `--target-arch aarch64 --cxx <compiler>
--aorta-core <official-ARM-wheel-core>`; compile checks but do not pass
`--run-protocol-checks` on x86. ARM install/build is not ARM execution.

## Run (Linux x86 or separately authorized ARM host)

Run only an architecture-matching artifact and use a new output directory:

```bash
python tools/production_topic_mock/run.py --mode production-topics \
  --build-dir /tmp/new-production-mock-build \
  --output /tmp/new-production-mock-capture --seconds 60
```

This starts only the synthetic source/services. No client, control request or
BodyCmd is automatically created. `client-environment.json` gives `SDK_GROUP`,
`AORTA_GROUP` and the private `ZENOH_SESSION_CONFIG_URI`. The default group is
unique `mock-sdkmain-*`; `default` and other production names are rejected.
Both sides are peers, multicast/gossip/SHM disabled, all endpoints assigned
`127.0.0.1` ports. The C++ endpoint also checks its exact canonical profile and
transport environment before constructing any node. No robot session/profile
is read or altered.

For pkg's explicit real-Aorta protocol/timer smoke, use the fixture client:

```bash
python tools/production_topic_mock/run.py --mode production-topics \
  --build-dir /tmp/new-production-mock-build \
  --output /tmp/new-protocol-smoke --seconds 5 --allow-client --self-test-client
```

It performs RobotInfo/PREPARE/COMMIT, a fixed50 Hz zero-motion SDK stream and
CANCEL, entirely in the private Mock session. It records actual client tick
starts and publication/receipt/applied identities. This exercises the tool and
SDK; it does not run sdkmain, a policy model or a real robot. S2/S3/S4 stay
NOT_MEASURED. The measured400/50 Hz observations are data, not an invented SLA.

To start a **reviewed direct client** in the runner, opt in explicitly:

```bash
python tools/production_topic_mock/run.py --mode production-topics \
  --build-dir /tmp/new-production-mock-build \
  --output /tmp/new-with-client --seconds 60 --allow-client \
  --client-command /path/to/client-venv/bin/python -B /path/to/reviewed-client.py
```

The runner sets group/URI only in its own children and records their actual
return codes. It keeps the Mock alive briefly while a cooperative client
handles SIGTERM with CANCEL. It only stops process groups it created; it never
cancels or stops a real robot process. Preserve the first failed capture; do
not retry it into a PASS. A source-only run has missing client stages and must
not be described as a control-loop performance PASS.

The default group retains the `mock-sdkmain-` prefix and a112-bit random
URL-safe suffix. The complete name fits Aorta2026.10.20's32 UTF-8 byte limit.
Both prechecks use the pinned `MAX_GROUP_LEN` declaration in
`aorta_contract.h`; custom names over the limit report actual and maximum
length before creating a Node. Failed C calls log their function/endpoint,
return code and immediately copied thread-local `aorta_last_error`, including
resource cleanup. Run the additional offline boundary checks with:

```bash
python -m unittest tools.production_topic_mock.test_group_contract -v
```

**Misconnection boundary:** arbitrary client code is not sandboxed. A client
that resets transport environment, opens a second session or calls another
control API can escape a configured session. Use only reviewed entries that
preserve the supplied environment. Do not execute the customer
`start_onboard_vita.sh`: it overwrites `ZENOH_SESSION_CONFIG_URI` with a profile
derived from the robot's real `/app_param` settings. The runner rejects direct
shell launchers but cannot detect every possible nested launcher. Before any
authorized device client run, verify its effective group/peer configuration;
never infer isolation from SSH or a shell variable alone.

### SDK group fixture and sdkmain configuration copy

The pinned Aorta2026.10.20 Core selects the effective group from `AORTA_GROUP`,
ignoring the Node constructor's group argument. The SDK does not read
`SDK_GROUP`; this runner sets both variables in its children. The SDK checks
the requested group against that effective group before creating endpoints.
Clients must pass the runner's group explicitly. Test a matching argument and
rejection of the default/different arguments with the real, installed SDK:

```bash
python tools/production_topic_mock/run.py --mode production-topics \
  --build-dir /tmp/new-production-mock-build \
  --output /tmp/new-sdk-group-fixture --seconds 15 --allow-client \
  --client-command /path/to/sdk-venv/bin/python -B \
  tools/production_topic_mock/sdk_group_fixture.py \
  --output /tmp/new-sdk-group-fixture
```

`sdk-group-fixture.json` records the effective group, typed RobotInfo and real
BodyState for the matching case, and expected rejection errors for the two
mismatched cases. It sends no control requests or BodyCmd; the generic
latency report therefore has no measured control loop.

sdkmain additionally compares the Node's effective group with its own YAML
configuration. Environment variables alone cannot satisfy that guard when
the YAML still says `default`. Make another copy of the reviewed opt021 root,
preserving symlinks and file modes. In
`src/deployment/vita_s100_aorta_30dof/config/vita02a_s100.yaml`, change **only**
`robot.aorta_group` to a newly generated, validated Mock group. Keep every
other configuration value, customer joystick path/thread/mapping/button,
model, calibration and all15 SDK SHA guards unchanged. Reuse that exact group
with the runner's `--group`; do not reuse a group with an abandoned client.

For example, with `MOCK_ROOT` set to that new copy and `MOCK_GROUP` to its YAML
value, the direct client argv is:

```bash
python tools/production_topic_mock/run.py --mode production-topics \
  --build-dir /tmp/new-production-mock-build \
  --output /tmp/new-sdkmain-capture --group "$MOCK_GROUP" \
  --seconds 60 --allow-client \
  --initial-q /path/to/approved-31-slot-initial-q.json \
  --initial-imu /path/to/approved-hip-trunk-wxyz.json \
  --client-command /path/to/matching-sdkmain-venv/bin/python -B \
  "$MOCK_ROOT/scripts/onboard_operator.py" suspended-test \
  --root "$MOCK_ROOT" --controller wifi --motion-group P1 \
  --key-file /path/to/existing-private-xbox.key
```

Set the existing, reviewed Python module/native-library environment before
starting the runner. Its client inherits the runner-assigned `AORTA_GROUP`
and private `ZENOH_SESSION_CONFIG_URI` without a shell launcher. The operator's
existing explicit authorization check, model/native pins, identity,
neutral-input and manual START guards remain in force. A matching SDK wheel
and the separately reviewed opt021 client adaptation are prerequisites.
The Mock neither generates controller input for sdkmain nor bypasses a guard.

The unchanged customer ARM model/preprocessing/Pico libraries cannot run on
x86 through environment variables. An x86 protocol/client fixture measures
SDK/transport/synthetic output only. A CPU or replay policy adapter is a
separate test layer and must not be reported as the ARM customer's native
inference. On ARM use a new tool/client-copy directory after explicit device
authorization; no actual device run is implied by the commands above.

## Optional EE topics and scripted Joy

`--auxiliary-fixture /path/to/fixture.json` starts a separate, main-thread-pull
EE/Joy generator in the same isolated group, with another private loopback
peer. It does not modify SDK hand/gripper/Joy implementations or create Python
RX workers. It supports:

- `/sdk/foot_humanoid/hand/{left,right}/{info,state,touch,cmd}` for BRAINCO_REVO2;
- `/sdk/foot_humanoid/gripper/{left,right}/{info,state,cmd}` for AGX;
- optional `locomotion/joy` script frames, real public0.1.5 generated binding,
  including distinct short press/release frames. Script timing is relative to
  a matched Joy subscriber. The measured publication times are recorded.

Fixture top-level keys are `left`, `right`, `joy` and optional
`command_record_budget`. Each EE entry requires
`kind` (`brainco` or `gripper`), explicit `state_hz`, `info_hz`, a complete
public `info` DTO field mapping (omit side/ts fields), and, for brainco,
explicit `touch_hz`. Use the actual configuration/Info geometry and rates for
a hardware-matched test; there is **no guessed real-device frequency**. This
tool logs requested and measured rates separately. `synthetic_output` defaults
false; set true only to emulate an output tick in this artificial world.
Gripper `sample_q` optionally supplies a virtual sample within the fixture's
Info range. Without it the sample remains absent/invalid. Even with it, motor
ACK/first-feedback timestamps stay zero. A gripper host-target echo does not
prove transmission or hardware completion.

A fixture for a protocol smoke can use one synthetic gripper (values are
deliberate test data, not AGX device calibration):

```json
{"right":{"kind":"gripper","state_hz":50,"info_hz":1,
 "sample_q":0.02,"synthetic_output":true,
 "info":{"gripper_type":1,"model":"SYNTHETIC","firmware":"SYNTHETIC",
 "opening_min_m":0,"opening_max_m":0.08,"max_opening_rate_m_s":0.1,
 "hold_torque_nm":0,"torque_limit_nm":1,"opening_min_source":1,
 "opening_max_source":1,"max_opening_rate_source":1,
 "torque_limit_source":1,"hold_torque_source":1}}}
```

Brainco Info must declare hand_type1, dof6, six names/limits/speeds and explicit
touch geometry/availability/enabled flags. Touch is synthetic zero-valued
fixture data, not measured tactile feedback. `set_touch` RPC is not provided;
use pre-enabled fixtures for receive/clock tests. Device initialization,
touch-enable protocols, current/CRC/torque/firmware behaviour and hardware
fault recovery remain unmeasured. EE commands are validated against fixture
Info and recorded with exact identities. A finite256-entry native pull ring
cannot guarantee lossless observation; sequence gaps are diagnostic, not
reconstructed events, and `dropped_count==0` is not proof of no losses.

Joy syntax is `"joy":[{"at_ms":0,"axes":[0,0],"buttons":[0,0]},
{"at_ms":1,"axes":[0,0],"buttons":[1,0]},
{"at_ms":2,"axes":[0,0],"buttons":[0,0]}]`. Use the final public Joy vector
mapping and enough buttons/axes for the selected consumer. Missing Joy metadata
or a mismatched pin fails explicitly; it never falls back to ROS2 or a private
wire format. Python auxiliary-generation jitter is source jitter, not a test
of a native EE backend. This is not hardware verification.

The1,000,000-record auxiliary capture capacity covers publication and received
command records together. Before startup the runner reserves the bounded
publication/Info/touch/Joy count and a command-record budget (the remaining
capacity by default, or the explicit nonnegative `command_record_budget`).
Accepted and rejected commands both consume that budget. Exceeding it stops
the auxiliary endpoint and marks the report incomplete without overwriting
earlier records. This is a capture limit, not an assumed hardware command rate.

Running control ownership intentionally has no automatic lease expiry, as in
the current robot bridge: a finite requested duration is not a revocation
timer. An abandoned client's TIMEOUT_HOLD still owns the session; use its
cooperative CANCEL or end that isolated Mock and start a new group. The Mock's
COMMIT activates immediately and does not emulate the robot's preparing or
transitioning dwell, admission or hardware activation. Those entry-timeout
and transition races remain outside this fixture's coverage. A same-owner
CANCEL with no acquired lease is rejected while armed until prepare timeout.
Local port selection still has a bind/close race; fail and preserve a capture
if a port is taken. Python Joy dispatch may compress overdue short taps;
recorded publication timing is authoritative.

## Report and evidence boundaries

`states.csv`, `received.csv`, `applied.csv`, `control.csv`, optional
`auxiliary.json`, raw logs, build/request/return codes and SHA256SUMS remain in
the new output. No per-frame filesystem writes occur in the C++ timed path.
Rows/identities are bounded; full buffers fail instead of silently overwriting.

```bash
python tools/production_topic_mock/report.py --output /tmp/new-with-client \
  --client-log /path/to/that-run-client.jsonl
```

Correlation uses `(run directory, session_id, sequence)` and the exact
`input_state_id` **plus source real timestamp**. No nearest-time join or
16-bit diagnostic command ID is used. Without a client trace, report S0→L1,
L1→synthetic G, S0→synthetic G, command arrival periods, observed rates,
applied echo delay and sequence lag. Client cycle-start rate is not inferred
from arrival rate. The source window includes startup/warmup: report actual
window/counts, never assume60 s of active control.

`received_ns` is the callback-entry timestamp after transport dispatch, not a
wire-arrival timestamp. `accepted_ns` follows decode, mutex wait and admission.
G is the first eligible absolute REALTIME synthetic grid tick after admission;
`actual_dispatch_ns` separately records when the loop executed that tick.
`synthetic_tick_dispatch_delay` exposes its scheduling delay. On timeout hold
or CANCEL, the tool clears applied identities/timestamps while holding the
last synthetic joint positions.

The report includes callback parse errors and client identities that were
sent but never recorded as received, separately from received rejections.
Without a client trace, sent/lost counts are NOT_MEASURED. Callback exceptions
retain their text in `mock.log`. Parse/endpoint/cleanup errors, rejections,
capture overflow, missing client identities or clock anomalies mark the
measurement incomplete. Raw anomalous timestamps and unaffected metrics are
retained in JSON; they do not turn a failed run into a PASS. A signal with
active or unknown control ownership is incomplete; even a clean idle shutdown
does not prove the entire requested measurement window completed.

With the client's actual REALTIME trace, add S1/S2/S3/S4/S5/S6. S3 requires
observation-complete/inference-start; whole-policy entry cannot substitute.
Missing or zero points are NOT_MEASURED. Pull receipt is dequeue/assembly time,
not network arrival. S0 is immediately before the C++ publish call, after
synthetic state construction. G is the synthetic first apply tick, not CAN or
motor completion. Local scheduling uses MONOTONIC; cross-process measurements
use the same host's REALTIME, no offsets. Clock reversals, missing IDs,
rejections and accepted-but-unapplied commands are retained. Percentiles are
independent and cannot be added. No new latency/loss/rate acceptance threshold
is introduced.

Local source-only tests:

```bash
python -m unittest discover -s tests -p 'test_production_topic_mock*.py' -v
python -m unittest tools.production_topic_mock.test_group_contract tools.production_topic_mock.test_review -v
```

An additional offline check with real installed public generated EE bindings:

```bash
PYTHONPATH=.:python python tests/test_production_topic_mock_auxiliary.py --real-bindings
```

This packs and decodes five EE packet families with Node construction forbidden;
it is a message-binding check, not real transport or hardware evidence.

These pure-Python tests do not establish C++ compilation, real-Aorta handshake,
400→50 Hz behaviour,500 Hz quantization, ARM execution or device/HIL. Those
checks belong in the fixed-input pkg request and separately authorized runs.
