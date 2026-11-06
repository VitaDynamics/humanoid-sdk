# Changelog

## Unreleased — unified Body and end-effector clients

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

## 0.2.1 (in preparation)

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
current compatibility or installation contract; use the 0.2.1 entry and
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
  See README for the current support scope and the planned 0.2.1 packaging.
- Add an independent, read-only BodyStateSource with frame-owned snapshots,
  whole BodyState access with one lazy decode per received frame and direct
  CLOCK_REALTIME/MONOTONIC receipt timestamps. It creates no control endpoints
  or lease; ExternalController retains its existing state subscription.

Packaging and release are pending; local checks do not establish aarch64
execution, robot connectivity or motion acceptance.
