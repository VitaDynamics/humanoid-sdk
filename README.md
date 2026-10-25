# Humanoid SDK

SDK **0.2.1** is in preparation; packaging and release are pending.
This source increment targets **BodyCmd2 / BodyState3** and the new end-effector
roots from schema source `80a1542097384b5d276070cbf3f705f27e9c9d53`.
The actual schema candidate `0.0.0+g80a154209738` wheel, public BFBS and
plain C++ BodyState3 header have been delivered from schema source
`80a1542097384b5d276070cbf3f705f27e9c9d53`. This branch binds those exact
candidate bytes in [compatibility.json](compatibility.json); it is **not an
official schema release or production pin**. Native/installed SDK, full IPC,
final robot producer integration and HIL acceptance remain pending. Generation
and schema-carrier checks do not establish this SDK combination as usable.

Python SDK and examples for humanoid locomotion over Aorta. The Python import
name is `locomotion_aorta`. Run the SDK on a Linux x86_64 or aarch64 computer
with **CPython 3.10**, glibc **2.35+** and GLIBCXX **3.4.29+**
(Ubuntu 22.04 or newer). SDK 0.2.1 ships one distribution containing both the
client and the lazy C++ inference backend, as `cp310` wheels for x86_64 and
aarch64 only. There is no pure Python or separate backend wheel for 0.2.1.

Dependencies are pinned to Aorta SDK and messages `2026.10.20` (ABI 13),
`vbot-humanoid-msgs==0.0.0+g80a154209738`, and `flatbuffers==25.12.19`. Verify wheel filenames
and SHA-256s against [compatibility.json](compatibility.json). The candidate is supplied through the separately
verified offline wheelhouse; it is not in the historical dependency Release.
`dependency_release_tag` is null, so the existing release downloader refuses
before downloading anything. No replacement release, URL or fallback is chosen.


EXTERNAL uses 31-slot `BodyCmd` joints vector, `BodyState`, and typed `get_robot_info`.
The SDK also includes **Velocity mode** (`examples/velocity_mode/velocity_mode.py`) and
**WBC tracking mode** (`examples/wbc_tracking_mode/wbc_tracking_mode.py`). Both use public body31
and the built-in C++ ONNX backend through EXTERNAL. See
[Velocity mode](examples/velocity_mode/README.md) and [WBC tracking mode](examples/wbc_tracking_mode/README.md)
for the old-to-new names, optional dependencies and offline entry points.
These additions await independent package/numerical acceptance; a unit test is
not robot-motion evidence.

The 0.2.1 Joy input candidate adds `JoyReader`: drain Aorta input snapshots in
the policy thread and read the latest axes plus accumulated button edge and
sequence-gap counts. See [the Joy reader contract](examples/velocity_mode/README.md#direction-dependency).
Joy retains its prior public contract. SDK release publication remains pending;
the complete new matching bundle is not yet available.

Match the robot's public schema hashes and roster to [compatibility.json](compatibility.json).
Control requests use protocol 2; ACK/Status remain protocol 1. BodyCmd remains
protocol 2 and BodyState uses protocol 3. BodyCmd action is explicitly encoded using the generated EXTERNAL
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
while preserving field order (0..10 and 0..6 for Cmd and IMU). BodyState3
retains ids0..12 and appends the association fields at ids13..16. There are no
external consumers of 0.1.3 yet; older unpublished candidates cannot be mixed.
Both public and internal IMU locations retain HIP=0/TRUNK=1 (chest), adding
HEAD=2 as a reserved position. The current public list follows the actual
configured HIP and TRUNK locations. Public state_id and BodyCmd.input_state_id
remain full uint64 values.
Command validation uses learned BodyInfo population; until discovery it
validates all 31 command slots. RobotInfo and its nested HandInfo/GripperInfo
descriptions remain frozen; online end-effector commands use the new contract.

BodyCmd has no cmd_id: correlate BODY commands and logs by session_id and
sequence. ExternalController.publish returns the command with the actual
assigned session/sequence after its transport publish succeeds.
BodyState3 retains applied_session_id, applied_sequence and ts_first_output_us,
and adds descriptor_ref, ts_control_us, applied_key_valid and
association_source_type. These identify the actual same-snapshot association
across command sources. A valid key may be zero; validity must not be inferred
from a nonzero number or the current action name. Missing/invalid association
and unobserved timestamps stay explicit. ts_first_output_us remains a
control-cycle first-effect timestamp, not hardware send completion.

The new `EndEffectorClient` borrows the existing controller (or a read-only
`PullStateSource`). Its startup `observation_sources=()` is an ordered immutable
`(stream, side)` tuple; duplicates are errors and empty K remains legal.
Pass the actual used State/Touch `.sample_ref` values through
`publish(..., input_observations=...)` in that order. Reusing the same actual
sample on consecutive calls does not create a new sensor sample. The client
does not fetch a replacement latest sample or synchronize independent streams.
Unknown SDK-to-Vita clock mapping leaves command timestamps invalid/zero.

The development source includes the shared EE client, one motion/read-only CLI,
capture and the original protocol Mock. Legacy HandClient/GripperClient and
their motion entries have been removed; there is no online wire fallback.
The delivered schema candidate is bound below; native builds, final robot
producer integration and HIL acceptance remain required before release.

Documentation: [Humanoid documentation](https://vitadynamics.feishu.cn/wiki/Mu1hw8wcSiKw3ykDGtAcvgXpnAg),
[SDK guide](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh),
and [PC networking and Zenoh configuration](https://vitadynamics.feishu.cn/wiki/M1vEw22vziSNAjkHVCncFlu1nyh).

## Examples

| Main demo | Behavior already implemented |
|---|---|
| `examples/end_effector/end_effector_external.py` | Generic read-only or explicitly authorized motion entry; Body and both endpoints share one EXTERNAL lifecycle and allocator. |
| `examples/brainco_hand/brainco_capture.py` | Read-only new EE/Body capture and explicit offline archive export; no control ownership. |
| `examples/teleop_policy/teleop_policy.py` | PC-local ONNX inference and Aorta Joy input; live mode publishes through EXTERNAL, while `--dry-run` processes offline BodyState records without transport. A model must be supplied explicitly. |

Basic examples remain `examples/external_control/external_control.py` for body EXTERNAL,
`examples/body_state_subscriber/body_state_subscriber.py` for read-only BodyState, and
`examples/fixed_stand_pose/fixed_stand_pose.py` as the robot-profile zero-pose helper.
The policy demo is not limited to offline use; installation or `--help`
does not authorize live motion.

## Offline installation and read-only check

Use a complete, matching offline bundle on your Linux computer. Create a
separate virtual environment to preserve an existing installation. From the
bundle root:

```bash
python3 -m venv venv
source venv/bin/activate
python -m pip install --no-index --find-links wheelhouse 'locomotion-aorta[state-arrays]==0.2.1'
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
python examples/body_state_subscriber/body_state_subscriber.py --group default --timeout 5
```

The last command connects to the robot and reads feedback; run it only with
connection authorization. Use the robot's actual group. A peer can connect to
a router, but the robot-facing address, listener, namespace and authentication
must all match the deployment. Missing authentication is an installation
blocker; do not remove it to connect.

### Effective Aorta group

Pinned Aorta 2026.10.20 Core selects its group from `AORTA_GROUP` (unset means
`default`), even when a different group is passed to `Node`. Before creating
nodes, set `AORTA_GROUP` to the same value as the SDK `group=` / demo `--group`.
The SDK checks `Node.group()` before creating publishers/subscribers and rejects
a mismatch, reporting requested/effective values and the `AORTA_GROUP` setting.
It does not change the process environment. `SDK_GROUP` is only a demo/runner
convention; setting it alone does not configure Core. The production-topic
Mock runner sets both variables in its isolated children; clients must pass
that same group explicitly. This group check does not validate peer routing,
namespace, credentials or permission to move a robot.

## Motion example

`examples/external_control/external_control.py` requests control and moves 30 installed body joints in the public 31 roster.
Review its targets and gains for the specific robot, and obtain explicit
motion authorization, reliable support, an attendant and emergency-stop access.
Installation and read-only checks do not authorize motion.

`ExternalController.enter_external(timeout=5.0)` requests control without an
automatic expiry; `timeout` only bounds the entry wait. Old callers must remove
`lease_duration_s`. `exit_external(timeout=5.0)` sends CANCEL to PASSIVE, which
can let loaded limbs fall. Client failure does not automatically release
ownership. Command watchdogs and validation remain active. Use compatible robot
software supporting unlimited control; an older server rejection is an error.

## End effectors and read-only capture

See [the single EE entry](examples/end_effector/README.md) for explicit per-side
units, rates and actual descriptor limits. Default and `--dry-run` are read-only.
Authorized motion borrows the original Body owner/lease/allocator; left and right
have no private control session. Body freshness, ownership and CANCEL→PASSIVE
remain mandatory. PASSIVE may let unsupported limbs fall.

Angular fraction/sequential patterns keep the 25%/75% fractions and float32
20 deg/s default; LINKER support is determined by its actual descriptor.
AGX target/sine modes use total opening in metres. Missing torque intent,
explicit zero and signed torque are distinct. There is no fabricated numeric
host-target pose or arrival guarantee in new State. See the entry documentation
for migration differences and the separate on-site motion requirements.

`examples/brainco_hand/brainco_capture.py` retains its historical filename but
captures only the new `/sdk/foot_humanoid/end_effector/<side>/{info,state,touch,cmd}`
and current Body/ControlStatus topics. JSONL uses `end_effector_capture_v4`;
kind names are `left_ee_*` / `right_ee_*`. It creates no publisher or service
client. Old Hand/Gripper live decoders are not a fallback.

Touch payload is an exact uint8 list; region stamps, independent sample IDs,
status/direction/validity, full DescriptorRef and command SampleRef/key are
preserved. Missing/nonfinite feedback stays JSON null with its original
validity, never a healthy zero. Queue drops, partial capture and transport loss
remain separate. The optional `hil` extra exports MCAP in storage order with
exact topic/root/BFBS and encoding checks, CRC validation and source/payload
hashes. Historical LowCmd/LowState are explicit offline archive choices only.
Metadata `schema_sha256` binds the new EndEffectorTouch BFBS for touch export.
No time mapping, physical completion or device HIL is inferred by capture.

## Read-only body state

The opt-in `PullStateSource` and `PullController` provide `wait_state(timeout)`
and ControlStatus snapshots using Aorta keep-last rings. Install the
`state-arrays` extra for native NumPy feedback. The arrays own their storage;
joint clocks retain uint64, errors uint32, and positions/velocities/torques
float32. `imu_state_present` distinguishes an absent sample from numeric zeros.
The original callback API remains available. Pull depth 1 intentionally skips
older queued samples; its drop counter is not a packet-loss measurement.

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
python examples/body_state_subscriber/body_state_subscriber.py --group default --timeout 5
```

## Development and offline packaging

For a separately authorized candidate validation, supply the exact verified
offline wheelhouse before installing the four dependencies into a separate
virtual environment. These commands are not an installation or execution GO. `make check` sets `PYTHONPATH=python:tests` and
tests the working tree; a pure Python edit does not require a compiler or an
editable SDK install.

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install --no-index --find-links wheelhouse \
  aorta-sdk==2026.10.20 aorta-msgs==2026.10.20 vbot-humanoid-msgs==0.0.0+g80a154209738 flatbuffers==25.12.19
python -m pip install mcap==1.3.1
make check
```

This candidate requires its actual schema wheel from the fixed validation
handoff, alongside the exact Aorta/message/FlatBuffers wheels. Their names and
SHA-256s are listed in compatibility.json. The existing offline packager accepts
explicit `--wheelhouse` directories and checks every pinned hash. It does not
fetch or synthesize missing artifacts. `tools/fetch_dependencies.py` remains a
published-Release-only downloader and intentionally refuses the null candidate
tag; do not invent a Release to make it proceed. Published-release CI download
jobs cannot validate this unpublished candidate without the authorized offline
inputs. Neither path implies publication or robot acceptance.
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
examples, source and the public PC configuration template. NumPy
remain optional demo dependencies. The packager verifies pinned dependency
hashes, requires both SDK architecture filenames, refuses configured credentials
and existing output directories, and writes `MANIFEST.json` with file digests
and source state. Installation does not establish robot connectivity or motion
compatibility.

### Dependency Release convention

Following SDK #10, fixed `aorta_msgs`, architecture-specific `aorta_sdk`,
`flatbuffers` and `vbot_humanoid_msgs` wheels for CI and offline bundles live
in **this repository's dependency prerelease**. The historical `v0.1.0-rc.4`
("dependencies for #10") does not contain the current schema candidate. CI reads it with this repository's `GITHUB_TOKEN`
as `GH_TOKEN`; it does not read the private schema repository or require a
cross-repository token.

After publishing a schema `humanoid-msgs-<version>` Release:

1. Download its official `vbot_humanoid_msgs` wheel and `.sha256`; verify the
   wheel against the sidecar and the Release asset digest. Mirror both files
   unchanged into the SDK dependency Release, retaining earlier assets.
2. Download and back up that Release's `SHA256SUMS`. Append two lines in its
   existing `SHA256  filename` format, one for the wheel and one for the
   sidecar. Preserve every existing byte and replace only `SHA256SUMS`.
   Download the mirrored files again and verify their hashes against the
   schema Release and the appended entries; keep other Release properties.
3. Pin the exact schema version, official wheel SHA and matching BFBS hashes
   in `compatibility.json`, with this repository as
   `dependency_release_repository` and a nonempty `dependency_release_tag`.
   Synchronize package requirements, release audits, tests and documentation.
   Run the dependency fetcher and relevant checks before pushing the SDK PR.

Never substitute an older schema or skip a missing asset, empty tag or failed
checksum. A dependency mirror is not an SDK release or robot validation.

The `SDK latency report` check exercises Python SDK ↔ C++ Mock communication
at 50–1000 Hz for at least 60 seconds per rate. See the
[benchmark documentation](tests/latency/README.md) for its measurement boundary
and reports. It does not establish robot performance or motion acceptance.

## PC policy example (in preparation)

`examples/teleop_policy/teleop_policy.py` subscribes to BodyState, discovers the actual body
roster, drains Aorta `locomotion/joy` through the same JoyReader as velocity,
runs a local action 17 ONNX policy, and uses the same EXTERNAL lease and
CANCEL exit. Before every live inference it waits up to the existing `--timeout`
for a strictly increasing BodyState `state_id`; unchanged or older feedback is
not reused for another inference or leg command, and timeout attempts CANCEL.
The SDK wheel already contains the C++ ONNX Runtime 1.20.1 CPU backend and
its shared libraries. Only constructing a model loads the bundled inference backend;
core imports and example `--help` do not load it. There is no Python ONNX
Runtime fallback. The optional `teleop-demo` extra installs only NumPy from a complete matching demo wheelhouse:

```bash
python -m pip install --no-index --find-links demo-wheelhouse 'locomotion-aorta[teleop-demo]==0.2.1'
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

## Demo 目录

每个目录包含运行说明、前提与退出方式（从仓库/离线包根运行命令）：

- [velocity_mode](examples/velocity_mode/README.md)
- [wbc_tracking_mode](examples/wbc_tracking_mode/README.md)
- [teleop_policy](examples/teleop_policy/README.md)
- [brainco_hand](examples/brainco_hand/README.md)
- [end_effector](examples/end_effector/README.md)
- [external_control](examples/external_control/README.md)
- [fixed_stand_pose](examples/fixed_stand_pose/README.md)
- [body_state_subscriber](examples/body_state_subscriber/README.md)

Teleop input and the deployed action 17 mapping are documented in
[examples/teleop_policy/README.md](examples/teleop_policy/README.md). Live input comes from
the robot's Aorta Joy publisher, using the same JoyReader as velocity mode.

The generic EE client decodes typed callbacks into a latest immutable cache.
Its positive-timeout waits can require a different full SampleRef; it does not
create a Python polling worker or its own Node. It has no legacy `pull_feedback`
option. User callbacks run on the transport callback path, outside SDK locks,
and must not block; their exceptions are not malformed-wire feedback.
BodyState/BodyCmd retain native codecs and the original pull rings.
`PullController.wait_body_state()` uses native DTOs; `wait_state()` uses native
arrays. `brainco_capture live` retains its bounded callback queue, including
commands, and reports queue drops explicitly. Transport loss remains unmeasured.

Body pull feedback is a latest-state snapshot, not a policy clock. `wait_state(timeout)`
waits for a first frame, then polls the depth=1 ring and otherwise returns the
previous immutable snapshot with unchanged receipt timestamps; timeout=0 does
not block. Live policy demos retain their configured 50 Hz timer, independently
of the BodyState publication rate (including 400 Hz). Core context receive stamps
are dequeue/assembly times, not network-arrival times.

Velocity stops publishing if BodyState does not advance within the existing
200 ms live-feedback window. WBC retains `--max-state-age-ms` (default 50 ms):
pull feedback must pass local feedback receipt age. Producer timestamps remain diagnostic; freshness
does not compare clocks from different machines.
The Core dequeue stamp alone cannot establish arrival age. Policy schedules,
duration and transitions use monotonic time; late cycles re-anchor one full
period ahead instead of sending catch-up bursts. Velocity SIGINT/SIGTERM stop
new targets and run the existing CANCEL/close path.

Pull ControlStatus conversion errors are counted in
`PullTransport.status_decode_errors` and retain the last valid snapshot;
transport failures still propagate. `PullController.wait_body_state()` retains
the callback controller's `ExternalControlTimeoutError` contract. After close,
wait/publish raise `ControllerClosedError`, while `status` returns its last
snapshot. Creating a read-only source does not change Aorta logging environment
variables. Publisher construction validates the native schema/header ABI before
control ownership can be requested.

EE pull clients serialize ring consumption. A concurrent caller that finds a
poll in progress uses already cached Info/state instead of waiting behind that
caller's blocking receive. The existing Info validation and command guards
still apply; this does not create a callback or polling worker.
