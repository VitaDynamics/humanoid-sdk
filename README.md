# Humanoid SDK

SDK **0.2.0** is in preparation; packaging and release are pending.
This version provides public body31/EXTERNAL, independent hand/gripper
clients, Aorta 2026.10.12 and an independent read-only BodyStateSource.

This candidate requires public body/EE schema **0.1.3** and BODY protocol 2.
Its 40-byte BodyJointState layout is incompatible with 0.1.2 (56 bytes); use
matching robot software and generated bindings. The 0.1.3 dependency wheel is
a local candidate, not a published asset. `dependency_release_tag` remains
unset until its release is approved; installation requires the complete
matching candidate wheelhouse and hashes in [compatibility.json](compatibility.json).

Python SDK and examples for humanoid locomotion over Aorta. The Python import
name is `locomotion_aorta`. Run the SDK on a Linux x86_64 or aarch64 computer
with **CPython 3.10**, glibc **2.35+** and GLIBCXX **3.4.29+**
(Ubuntu 22.04 or newer). SDK 0.2.0 ships one distribution containing both the
client and the lazy C++ inference backend, as `cp310` wheels for x86_64 and
aarch64 only. There is no pure Python or separate backend wheel for 0.2.0.
Version 0.2.1 is planned to restore general platform support through separate
platform and pure Python wheels under the same SDK package name.

Dependencies are pinned to Aorta SDK and messages `2026.10.12` (ABI 13),
`vbot-humanoid-msgs==0.1.3`, and `flatbuffers==25.9.23`. Verify wheel filenames
and SHA-256s against [compatibility.json](compatibility.json). The prior
dependency release `v0.1.0-rc.4` contains schema 0.1.2 and cannot supply this
candidate's complete dependency set; do not substitute that older schema.

EXTERNAL uses 31-slot `BodyCmd` joints vector, `BodyState`, and typed `get_robot_info`.
The SDK also includes **Velocity mode** (`examples/velocity_mode.py`) and
**WBC tracking mode** (`examples/wbc_tracking_mode.py`). Both use public body31
and the built-in C++ ONNX backend through EXTERNAL. See
[Velocity mode](examples/velocity_mode.md) and [WBC tracking mode](examples/wbc_tracking_mode.md)
for the old-to-new names, optional dependencies and offline entry points.
These additions await independent package/numerical acceptance; a unit test is
not robot-motion evidence.

Match the robot's public schema hashes and roster to [compatibility.json](compatibility.json).
Control requests use protocol 2; ACK/Status remain protocol 1; new body roots use
protocol 2. BodyCmd action is explicitly encoded using the generated EXTERNAL
symbol. Entry remains by action name and lease; the command does not activate
EXTERNAL. Older robots rejecting v2 receive no automatic downgrade or LowCmd
fallback. Discovery preserves partial body/hand results and actual field/limit
validity. The logical waist-roll slot can be unpopulated; it is skipped in
physical validation and output, and never represents a healthy zero sample.

BodyJointState contains three acquisition/ingestion/assembly timestamps,
q, dq, tau_est and the unsigned uint32 JointError error_code. The tuple position is the
slot identity; installation, capabilities and limits come from BodyInfo.
For installed non-MOCK joints, the producer preserves a real nonzero fault
first, otherwise uses TIMEOUT (8) for missing/stale feedback, UNKNOWN (255)
for nonfinite values or ankle FK failure, and NONE (0) otherwise. Uninstalled
and MOCK slots use NONE; NONE alone does not mean a joint is installed.
The 20 JointError values align shared codes with the quadruped contract,
including TIMEOUT=8 and INVALID_PARAM=30; humanoid-only control, motor-phase
and sensor faults use 15/16/17. CAN, enable-state and servo-specific extension
codes are omitted. The SDK imports this generated
enum and preserves received codes, including unknown numeric values, without
synthesizing another status. The full uint32 domain, including unlisted
injection codes >=2^31, is preserved without sign conversion or remapping.
Device TIMEOUT and missing/stale feedback intentionally share TIMEOUT (8);
use the per-joint acquisition/ingestion/assembly timestamps and their clock
domains to assess feedback freshness.
BodyImuSample keeps status and rpy (degrees), and removes temperature,
valid_fields and duplicate device/host timestamps. BodyCmd, BodyState and
BodyImuSample have no deprecated placeholders: IDs are renumbered contiguously
while preserving field order (0..10, 0..12 and 0..6 respectively). There are no
external consumers of 0.1.3 yet; older unpublished candidates cannot be mixed.
Both public and internal IMU locations retain HIP=0/TRUNK=1 (chest), adding
HEAD=2 as a reserved position. The current public list follows the actual
configured HIP and TRUNK locations. Public state_id and BodyCmd.input_state_id
remain full uint64 values.
Command validation uses learned BodyInfo population; until discovery it
validates all 31 command slots. RobotInfo and independent hand/gripper
contracts remain unchanged.

BodyCmd has no cmd_id: correlate BODY commands and logs by session_id and
sequence. ExternalController.publish returns the command with the actual
assigned session/sequence after its transport publish succeeds.
BodyState carries applied_session_id, applied_sequence and ts_first_output_us
instead of a nested 31-joint echo. Only EXTERNAL produces nonzero applied
associations. EXTERNAL session_id and sequence must be nonzero; the robot
EXTERNAL ingress rejects either zero key. All three fields zero means no
EXTERNAL command is affecting the applied output.
The association records the last successful output send/report. TRACKING
output clears all three only after its first successful send/report; pauses,
send failures and the interval before that first success retain the previous
successful EXTERNAL association. It is not cleared by the action name at
publication time, and this does not add a nonzero-key check to TRACKING input.
The timestamp is robot-host Unix epoch microseconds for the control cycle
when interpolation alpha>0 first affects output, latched only after a
successful send. It is not hardware send-completion time.

Documentation: [Humanoid documentation](https://vitadynamics.feishu.cn/wiki/Mu1hw8wcSiKw3ykDGtAcvgXpnAg),
[SDK guide](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh),
and [PC networking and Zenoh configuration](https://vitadynamics.feishu.cn/wiki/M1vEw22vziSNAjkHVCncFlu1nyh).

## Examples

| Main demo | Behavior already implemented |
|---|---|
| `examples/gripper_external.py` | One entry for read-only gripper info/state, single-side target/return and dual-side sine; motion shares body zero/hold and one EXTERNAL lifecycle. |
| `examples/brainco_hand_external.py` | Independent hand info/state/touch and authorized hand motion; `examples/brainco_capture.py` records/exports feedback. |
| `examples/teleop_policy.py` | PC-local ONNX inference and gamepad input; live mode publishes through EXTERNAL, while `--dry-run` processes offline BodyState records without transport. A model must be supplied explicitly. |

Basic examples remain `examples/external_control.py` for body EXTERNAL,
`examples/body_state_subscriber.py` for read-only BodyState, and
`examples/fixed_stand_pose.py` as the robot-profile zero-pose helper.
The policy demo is not limited to offline use; installation or `--help`
does not authorize live motion.

## Offline installation and read-only check

Use a complete, matching offline bundle on your Linux computer. Create a
separate virtual environment to preserve an existing installation. From the
bundle root:

```bash
python3 -m venv venv
source venv/bin/activate
python -m pip install --no-index --find-links wheelhouse locomotion-aorta==0.2.0
python -m pip check
python -c 'import aorta, flatbuffers, locomotion_aorta; import lowlevel.BodyCmd, lowlevel.BodyState, locomotion_sdk.ControlStatus'
```

Copy [config/pc_session_peer.json5](config/pc_session_peer.json5) outside the
checkout or bundle. Fill every placeholder with deployment-provided values:
robot host/port, PC listener host, namespace, authentication user/password and
an absolute dictionary-file path. Keep credentials outside source control and
preserve the peer mode, authentication and matching discovery policy.

```bash
PC_PROFILE="$HOME/.config/locomotion/pc_session_peer.json5"
mkdir -p "$(dirname "$PC_PROFILE")"
test ! -e "$PC_PROFILE" && install -m 600 config/pc_session_peer.json5 "$PC_PROFILE"
${EDITOR:-vi} "$PC_PROFILE"
export ZENOH_SESSION_CONFIG_URI="$PC_PROFILE"
test -r "$ZENOH_SESSION_CONFIG_URI"
python examples/body_state_subscriber.py --group default --timeout 5
```

The last command connects to the robot and reads feedback; run it only with
connection authorization. Use the robot's actual group. A peer can connect to
a router, but the robot-facing address, listener, namespace and authentication
must all match the deployment. Missing authentication is an installation
blocker; do not remove it to connect.

## Motion example

`examples/external_control.py` requests control and moves 30 installed body joints in the public 31 roster.
Review its targets and gains for the specific robot, and obtain explicit
motion authorization, reliable support, an attendant and emergency-stop access.
Installation and read-only checks do not authorize motion.

`ExternalController.enter_external(timeout=5.0)` requests control without an
automatic expiry; `timeout` only bounds the entry wait. Old callers must remove
`lease_duration_s`. `exit_external(timeout=5.0)` sends CANCEL to PASSIVE, which
can let loaded limbs fall. Client failure does not automatically release
ownership. Command watchdogs and validation remain active. Use compatible robot
software supporting unlimited control; an older server rejection is an error.

## Independent Revo2 hands

Use the public EE wheel pinned in [compatibility.json](compatibility.json).
The distribution is `vbot-humanoid-msgs`; generated imports retain `lowlevel`,
`locomotion_sdk`, `aorta.services.humanoid_locomotion` and `*_schema_meta`.
Do not install the full private `vita-msgs` wheel or a temporary HIL wheel.
The pinned public `0.1.2` wheel provides the new hand roots; the older
`0.1.0` public wheel does not.

With connection authorization and the configured peer profile, read both hands:

```bash
python examples/brainco_hand_external.py --dry-run --side both --group default --timeout 5
```

The default entry is also read-only: no lease, hand command or touch write.
`HandClient` reads info/state/touch topics; `wait_info(timeout=5)` waits for the
first info sample or returns its cache. HandInfo is published about once a second;
late subscribers wait for a subsequent sample, with no historical replay promise.
Only the typed `set_touch` control RPC remains. After that control call, the demo
waits for a later info event with the acknowledged `touch_enabled` value;
`ts_info_us` need not change. `wait_info(after_info=client.info)` can request a
later event without clearing the info cache used by command publication.
Left/right wire values are 1/2; they differ from the device register numbering.

HandState uses the T021 public EE layout. It stores the three
sample timestamps once per message and exposes five `ts_cmd_*` stages; zero
future stages remain unmeasured. Touch uses `state_id`, `ts_state_pub_us` and
`raw_words`, with geometry from HandInfo. Old candidate recordings require their
matching SDK; the MCAP exporter refuses BFBS that differ from loaded bindings.
`HandJointState` 的 `*_raw` 字段仅供诊断，含义随设备型号变化，不应依赖。
Examples use normalized feedback and do not depend on these raw fields.

After separate on-site motion authorization, support, an attendant and an
emergency stop, a right-hand sequential round can be run with:

```bash
python examples/brainco_hand_external.py --side right --group default --timeout 5 --body-gains zero --pattern sequential --confirm-motion
```

The default speed is **20 degrees/s**, converted to float32 rad/s and checked
against every joint's advertised `dq_max`. The default fractions remain
25% -> 75% -> 25%; `--low-frac 0 --high-frac 1` selects declared endpoints.
`--pattern sequential` moves one joint at a time, keeping other targets low;
`--loop` repeats until SIGINT/SIGTERM or a safety failure. No automatic retry is
performed.

The body receives measured-pose 31-slot BodyCmd frames; hand commands use only
`/sdk/foot_humanoid/hand/<side>/cmd`. `--body-gains zero` selects kp=kd=dq=tau=0,
with q still measured; it does not support the body or instantly zero gains
during EXTERNAL entry. The default `hold` gains reuse `external_control.py`.
The existing 0.05 rad body drift guard, freshness, ownership, sequence and
hand-limit checks remain active.

Normal exit, faults and SIGINT/SIGTERM stop new targets and attempt CANCEL,
which must confirm PASSIVE. There is no automatic neutral hand trajectory;
PASSIVE can let loaded body limbs fall. A killed client does not guarantee
lease release. Hand session/sequence and body EXTERNAL generation are separate.

`examples/brainco_capture.py` subscribes to independent hand topics plus body
BodyState/ControlStatus (and historical LowCmd/LowState recordings) and exports MCAP with the optional `hil` extra.
Missing or nonfinite hand feedback is retained as JSON null, never a synthetic
zero. Five-stage robot timestamps use Vita host microseconds; zero future stages
are unmeasured. Public HandState carries these stages; no separate public trace
root is required. Host tests and packaging do not establish device HIL.

## Read-only body state

`BodyStateSource` subscribes only to BodyState; it creates no control publisher,
service client or lease. It is independent of `ExternalController`.

```python
from locomotion_aorta import BodyStateSource

source = BodyStateSource(group="default")
try:
    frame = source.snapshot()  # default timeout: 5 seconds
    state = frame.body_state()  # decode this frame once, on first access
    print(state.state_id, frame.received_monotonic_ns, frame.received_realtime_ns)
    next_frame = source.snapshot(timeout=1.0, after=frame)
finally:
    source.close()
```

`snapshot(timeout=5.0, after=None)` uses a finite positive timeout, as the hand
and gripper clients do. It returns the latest cached snapshot, or waits for a
different received object when `after` is given. Repeated or reset producer
state IDs can therefore still produce another snapshot; callers retain their
own producer freshness/validity checks. Timeout raises `TimeoutError`, invalid
timeout raises `ValueError`, and a closed source raises `RuntimeError` and wakes
waiters. Closing takes precedence over cached frames and does not send CANCEL.

Snapshots own their payload bytes and remain usable after later frames or
source close. Decode results, including decode errors, are retained per frame;
errors surface on `body_state()` access, never in the receive callback. Read
state IDs, robot timestamps and validity directly from the returned `BodyState`.
The two receipt timestamps are direct SDK-host nanosecond clock reads:
MONOTONIC for local elapsed time and REALTIME as Unix epoch time. They are not
converted into one another or into robot/device acquisition clock domains.

The single-frame CLI uses this same read-only source:

```bash
python examples/body_state_subscriber.py --group default --timeout 5
```

## Development and offline packaging

Install the four pinned runtime dependencies, not the SDK wheel, into a
separate virtual environment. `make check` sets `PYTHONPATH=python:tests` and
tests the working tree; a pure Python edit does not require a compiler or an
editable SDK install.

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install --no-index --find-links wheelhouse \
  aorta-sdk==2026.10.12 aorta-msgs==2026.10.12 vbot-humanoid-msgs==0.1.3 flatbuffers==25.9.23
python -m pip install mcap==1.3.1
make check
```

The dependency release supplies the Aorta/message/FlatBuffers wheels; it does
not contain this checkout's SDK wheel or the optional MCAP test dependencies.
For a fully offline development environment, obtain MCAP and its dependency
wheels beforehand and add `--no-index --find-links <test-wheelhouse>` to that
MCAP installation. Without the HIL test dependencies, capture tests are skipped;
a complete test result requires them.

Building the SDK wheel or using `pip install -e .` also compiles its native
backend. Install the build tools and explicitly obtain the official ORT archive
first; `fetch_ort.py` uses the current host's URL and SHA from
`demo_onnx/ort-archives.json`. Use a fresh output path or point CMake at an
already verified archive; the helper preserves an existing file.

```bash
python -m pip install build==1.2.2.post1 scikit-build-core==0.11.6 pybind11==2.13.6 ninja==1.11.1.3
python tools/fetch_ort.py --output /absolute/path/to/onnxruntime.tgz
export CMAKE_ARGS='-DORT_ARCHIVE=/absolute/path/to/onnxruntime.tgz'
make build
# Optional editable install; the same archive is required:
python -m pip install --find-links wheelhouse -e .
```

Raw wheels in `dist/raw` are build intermediates. Audit each release architecture
as described in [the build instructions](demo_onnx/README.md), then assemble the
bundle from both audited SDK wheels and the pinned dependency wheelhouse:

```bash
python tools/package_release.py --wheelhouse wheelhouse --wheelhouse dist/audited --output dist/offline-bundle
python tests/offline_install.py dist/offline-bundle
```

The bundle includes the compiled extension and ORT inside each SDK wheel,
examples, source and the public PC configuration template. NumPy and pygame
remain optional demo dependencies. The packager verifies pinned dependency
hashes, requires both SDK architecture filenames, refuses configured credentials
and existing output directories, and writes `MANIFEST.json` with file digests
and source state. Installation does not establish robot connectivity or motion
compatibility.

The `SDK latency report` check exercises Python SDK ↔ C++ Mock communication
at 50–1000 Hz for at least 60 seconds per rate. See the
[benchmark documentation](tests/latency/README.md) for its measurement boundary
and reports. It does not establish robot performance or motion acceptance.

## Independent AGX grippers (in preparation)

The single `examples/gripper_external.py` entry contains the EXTERNAL lifecycle,
body zero/hold, single-side targets/return and optional dual-side sine mode.
`GripperClient` uses `/sdk/foot_humanoid/gripper/<left|right>/{cmd,state,info}`.
It shares the same `GripperInfo` DTO with `get_robot_info`; there is no per-side
info RPC and no gripper slot in the 31-joint BodyCmd. The default example and
`--dry-run` only subscribe, without ownership or command publication:

```bash
python examples/gripper_external.py --side right --dry-run --group default --timeout 5
```

`publish_target(q, dq=0, torque_cmd_nm=None)` uses absolute **total opening in m**
and a host speed cap in **m/s**. It preserves dq=0 for the robot's configured
default; it does not interpolate a second SDK opening trajectory. Values beyond
the advertised ranges are rejected before publishing. Unknown limit sources
remain readable diagnostics and cannot authorize a target.

Torque is a signed software motor-side command in N·m: None uses configured
hold torque; explicit 0 sends 0. Its absolute bound and hold default come from
CONFIG. It is not gripping force in N or a device limit; device interpretation
is governed by the vendor. Acceleration/deceleration are not first-release fields.

After explicit on-site authorization, suspension/support, an attendant and an
emergency stop, choose a target from the actual Info limits and run:

```bash
python examples/gripper_external.py --side right --target-m "$TARGET_OPENING_M" --confirm-motion
```

The example enters EXTERNAL through the existing body preflight/PREPARE/COMMIT,
holds the measured 31-slot body pose and exits through CANCEL to PASSIVE.
In target mode, each leg streams for at least `max(duration, abs(target-start)/rate)`:
`rate` is positive `--speed-m-s`, or Info's configured `max_opening_rate_m_s`
when speed is 0. The segment starts at the robot's published `host_target.q`
when valid, otherwise its measured opening. `--return-to-start` streams one
return to the entry measured opening under the same rule. Sine return also
starts from the latest valid robot host target, with measured fallback.
This budgets command travel time; it does not infer physical device arrival.
`--body-pose zero --body-gains hold` instead ramps to zero using the recorded
robot FIXED_STAND profile, then holds zero in the same EXTERNAL session.
Zero-settle error/speed/dwell and this gripper example's body hold drift are
observation only; they do not trigger CANCEL. Entry stationary, state validity,
freshness, ownership, phase, rejection and CANCEL-confirmation guards remain active.
`BodyState.applied_cmd` is currently always None on the matching robot runtime;
it cannot be used as execution evidence. The gripper sample, host target and
latest-received trace keys stay separate; a host echo is
not proof of device execution or arrival. Zero trace timestamps remain unknown,
including AGX write_ack. No robot connection or motion has been tested here.

After the same on-site authorization, `--side both --pattern sine` uses one
EXTERNAL session for body zero/hold, simultaneous-phase gripper targets,
return to the entry openings, then CANCEL. Defaults are 0.01–0.08 m, a 10 s
period and 30 s of sine streaming; transitions add to the total runtime.
`--speed-m-s 0` keeps the robot configuration default. The two topic writes
share one time phase but do not promise atomic device output or arrival.

```bash
python examples/gripper_external.py --side both --pattern sine --body-pose zero --body-gains hold --period-s 10 --duration 30 --confirm-motion
```

## PC policy example (in preparation)

`examples/teleop_policy.py` subscribes to BodyState, discovers the actual body
roster, runs a local action 17 ONNX policy, and uses the same EXTERNAL lease and
CANCEL exit. Before every live inference it waits up to the existing `--timeout`
for a strictly increasing BodyState `state_id`; unchanged or older feedback is
not reused for another inference or leg command, and timeout attempts CANCEL.
The SDK wheel already contains the C++ ONNX Runtime 1.20.1 CPU backend and
its shared libraries. Only constructing a model loads the bundled inference backend;
core imports and example `--help` do not load it. There is no Python ONNX
Runtime fallback. The optional `teleop-demo` extra installs only NumPy and
pygame from a complete matching demo wheelhouse:

```bash
python -m pip install --no-index --find-links demo-wheelhouse 'locomotion-aorta[teleop-demo]==0.2.0'
python -m pip check
```

The backend intentionally matches the robot's ORT 1.20.1 instead of the
previous Python ORT 1.23.2. Outputs are not bit-identical to the old extra:
82 of 120 values differed in the archived comparison. Acceptance uses a
test-only Python ORT 1.20.1 oracle with exact finite float32 equality.
No compiler or robot filesystem is needed to consume the SDK wheel. Source
build instructions are in `demo_onnx/README.md`; the offline bundle already
contains the compiled backend and ORT inside each SDK wheel. `--model` is required; no
model is bundled or selected automatically.
Use `--help` or `--dry-run --body-state-jsonl <capture>` before live use; dry-run
does not construct SDK transport. The offline validation used a noshoes model;
replace it with the delivered shoes model and recheck metadata/profile parity
when available. The example does not replace the robot's complete action17 FSM,
VR/upper stream or hardware validation. Live motion remains subject to the
existing on-site authorization and emergency-stop requirements above.
