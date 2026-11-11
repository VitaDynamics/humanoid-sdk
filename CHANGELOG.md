# Changelog

## 0.3.0 (in preparation) — unified Body and end-effector clients

### Breaking

- Remove the old public Hand/Gripper client and DTO families without aliases or
  wire downgrade. The Owner selected `0.3.0` for this breaking API migration;
  it must not be published under the previous `0.2.1` package version.
- Require the matching native extension for production BodyState as well as
  EndEffectorState/Touch decoding. Private Python decoders remain test oracles,
  never runtime fallbacks. The supported distribution already includes native
  code; source/editable installation requires its build toolchain.

### Changed

- BodyCmd publishing delegates q/dq/tau/kp/kd device-range handling to robot
  EXTERNAL instead of rejecting targets against RobotInfo bounds. Values must
  still be numeric and encode to finite float32; invalid values are rejected
  before command-key allocation. Policy joint discovery no longer requires a
  known position range. Declared targets/fields, MOCK, capabilities and control
  identity remain checked; RobotInfo still exposes its limits for diagnostics
  and application-specific motion planning. No per-call exemption flag is needed.
- The shared end-effector demo checks Body hold drift whenever Body control is
  enabled, including AGX. Its position-error limit is `0.1 rad`, previously
  `0.05 rad` in the strict hand branch; it is an abort threshold, not an arrival
  tolerance or additional motion allowance. The old internal `strict_hold`
  argument is retired. Hand feedback logging still has its separate 1 s minimum.
- The demo's EE position receipt-age ceiling is `1.0 s`, previously the Body
  `0.2 s` window. It is separate because EE and Body have independent acquisition
  and delivery cadences. It does not refresh sample identities or source times,
  override device stale/validity flags, or permit stale Body feedback. Fault,
  readiness, output permission and ownership checks remain in force.
- BrainCo defaults to `--display table` at 5 Hz, throttled by `--display-hz`.
  The positive configured rate has no additional 5 Hz cap. Callback threads store
  observations; only the thread that created the table formats and writes it.
  Incomplete phase/sequence log fields display as unknown instead of raising
  from a callback. `--display log` retains the diagnostic log view.

### Public API migration

| Retired API | Current API and migration |
| --- | --- |
| `HandClient`, `GripperClient` | `EndEffectorClient("left" or "right", connection=controller)`; obtain and validate unified Info before commands. |
| `hand_types`, `gripper_types` | `locomotion_aorta.end_effector_types`; update the individual DTOs below rather than renaming a module import. |
| `HandCmd`, `HandJointCmd`, `GripperCmd` | `EndEffectorTarget`; use fixed axis order, explicit modes/field masks and Info-declared SI units and limits. |
| `HandInfo`, `GripperInfo`, old nested hand/gripper Info | `EndEffectorInfo` for axis capabilities, `is_mock` and Info-only `valid_limits`; GetRobotInfo's endpoint capability retains availability/reason, while dynamic readiness and faults remain State. |
| `HandState`, `HandJointState`, `GripperState`, `GripperSample` | `EndEffectorState`; retain the original sample identity, per-axis validity and measurement times. |
| `GripperHostTarget` | No equivalent device-feedback field. Retain locally submitted intent and its returned key separately from measured State. |
| `HandTouch`, `FingerTouchMeta` | `EndEffectorTouch`, with layout/region descriptions from `EndEffectorInfo`; respect per-region stamps and validity. |
| `HandSide`, `HandType`, `GripperType`, `EndEffectorKind` | Select the client side explicitly; use unified Info's device model, position domains and capabilities. Do not cast old enum integers. |
| Old Hand/Gripper topic prefixes | Let the side-specific `EndEffectorClient` use the unified Info/State/Touch/command interfaces. Old topics are not a control fallback. |
| `BODY_PROTOCOL_VERSION` | `BODY_CMD_PROTOCOL_VERSION` and `BODY_STATE_PROTOCOL_VERSION`; the two contracts are checked independently. |
| String lease identity / lease-duration request | Discover the startup `runtime_id` through GetRobotInfo and use numeric `control_id` on the same connection. Use explicit CANCEL/close; do not expect automatic lease expiry. |

Descriptions remain fixed for a startup runtime. Reconnect and explicitly
discover a new runtime instead of relabelling old commands or observations.

- Support normal and exceptional `with PullStateSource(...)` cleanup without
  publishing commands; reject context entry after closure.
- Keep the Body watchdog active while confirming touch disable before the
  position wait, and preserve detailed original State on output-permission
  failures. Adopt the documented hum-42 example gains and separate Body hold /
  EE freshness defaults.
- Default the BrainCo demo to a throttled live table; keep full diagnostic
  logging available through `--display log` and preserve command scheduling.

Controller and body state source startup now reject a requested SDK group
that differs from the effective `AORTA_GROUP` (or Core's `default`). Programs
that set `AORTA_GROUP` but leave the SDK `group="default"` must pass the matching
group explicitly. Unified end-effector clients and BrainCo capture also reject
mismatches; the body controller and state sources enforce the same rule.

### Carried-forward implementation details

- Use BodyCmd/BodyState protocol **3**, with 31 ordered body axes, and control
  request/ACK/status protocol **4**. Older Body/control wire versions are
  rejected rather than automatically downgraded.
- Bind a connection to the nonzero startup `runtime_id` discovered through
  GetRobotInfo. Use numeric `control_id` and generation/sequence keys; a new
  runtime requires a new connection and clients. Descriptions are static for
  that runtime; runtime hand swapping is unsupported.
- Replace the retired public HandClient/GripperClient and nested HandInfo /
  GripperInfo APIs with EndEffectorClient and unified Info on separate left /
  right topics. There is no legacy alias or wire fallback. Info-only
  `valid_limits`, real capabilities and device availability remain enforced.
- Use end-effector Cmd/State protocol **2**, Info/Touch protocol **1**, and
  set-touch request/response protocol **2**. These versions are independent.
- Add pull BodyState/ControlStatus snapshots, C++ BodyCmd encoding and
  BodyState NumPy decoding, plus C++ EndEffectorState/Touch decoding. Keep the
  existing Python DTOs and State/Touch validation at the client boundary.
- Move examples to latest body feedback while keeping each configured control
  period independent of state publication frequency.
- Add Aorta Joy draining and button-edge diagnostics, and common policy-cycle
  timing/applied-sequence logs. Pin the actual dedicated `gc783514130bb`
  messages wheel and 15 BFBS hashes in compatibility.json. The pinned runtime
  and message wheels are available in dependency Release `v0.1.0-rc.4`; the
  dependency downloader still verifies the exact versions and SHA-256 values.
- Keep the three BrainCo Revo2, AGX and Linker entry points. On an endpoint
  without touch capability, `--touch off` is an explicit no-op with no RPC or
  fabricated confirmation; `--touch on` still rejects before control entry.

Migration from the earlier unpublished candidate requires updating both wire
bindings and public callers; changing a version number or keeping a legacy
hand/gripper wrapper is insufficient. Dependency asset availability and scoped
host/CI results do not establish final robot/SDK compatibility, strict realtime
or device acceptance. The final SDK release remains pending.

## Earlier 0.2.0 development notes — superseded contracts

The following records an earlier unpublished candidate, including Body/control
protocol 2 and the old 0.1.4 dependency. It is historical context, not the
current compatibility or installation contract; use the 0.3.0 entry and
compatibility.json above.

- Use the public 31-slot BodyCmd/BodyState interfaces and EXTERNAL control;
  control requests use protocol 2, without automatic protocol downgrade.
- Provide independent hand and AGX gripper clients and examples. Grippers use
  their own topics and do not occupy BodyCmd slots.
- Pin Aorta SDK/messages 2026.10.20 (ABI 13), public vbot-humanoid-msgs 0.1.4
  and FlatBuffers 25.12.19; pin the official `humanoid-msgs-0.1.4` Release wheel
  SHA and actual BFBS hashes. Mirror that wheel in this repository's
  `v0.1.0-rc.4` dependency Release alongside the pinned runtime wheels, following
  #10; fail on missing tags or checksum mismatches instead of skipping validation.
- Use BODY protocol 2 for BodyCmd/BodyState. BodyJointState changes from 56 to
  40 bytes: slot-ordered three timestamps, q/dq/tau_est and uint32 JointError; preserve unlisted high-bit codes.
  Keep 20 approved error values, aligning shared codes with quadruped:
  TIMEOUT=8, INVALID_PARAM=30, control/motor-phase/sensor faults=15/16/17.
  Omit CAN, enable-state and servo-specific extension codes.
  Remove body_index/status/populated/is_mock/valid_fields/ddq/temperature/lost.
  Preserve real faults first, then missing/stale=TIMEOUT, nonfinite/FK=UNKNOWN,
  otherwise NONE; uninstalled/MOCK use NONE and installation comes from BodyInfo.
  IMU status/rpy remain; temperature/valid_fields/duplicate device and host
  timestamps are removed without placeholders. Compact the three BODY tables
  to contiguous field IDs, preserving order; earlier unpublished 0.1.3
  candidates cannot be mixed. Reserve HEAD=2 alongside
  HIP=0/TRUNK=1; the current configured public list remains HIP and TRUNK.
  Replace the nested applied echo with applied_session_id/applied_sequence and
  control-cycle ts_first_output_us, latched after successful send. Only
  EXTERNAL creates nonzero applied keys; successful TRACKING output clears
  them, while pause/failure retains the previous successful association.
  Remove BodyCmd.cmd_id and use session/sequence in BODY logs. publish returns
  the stamped command. Public state/input-state IDs retain all 64 bits.
  Reject mismatched BODY versions before decoding joints; control/RobotInfo
  versions remain unchanged.
- Switch the optional teleop demo from Python ORT 1.23.2 to an embedded C++
  ORT 1.20.1 CPU backend, matching the robot. This intentionally changes
  numerical outputs: 82/120 values were not bit-identical across versions.
  The Python 1.20.1 test oracle remains exact; no tolerance is introduced.
  The SDK wheel contains the backend, while core import leaves it unloaded.
  See README for the current support scope and the planned 0.3.0 packaging.
- Add an independent, read-only BodyStateSource with frame-owned snapshots,
  whole BodyState access with one lazy decode per received frame and direct
  CLOCK_REALTIME/MONOTONIC receipt timestamps. It creates no control endpoints
  or lease; ExternalController retains its existing state subscription.

Packaging and release are pending; local checks do not establish aarch64
execution, robot connectivity or motion acceptance.
