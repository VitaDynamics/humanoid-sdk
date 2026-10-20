# Changelog

## 0.2.0 (in preparation)

- Use the public 31-slot BodyCmd/BodyState interfaces and EXTERNAL control;
  control requests use protocol 2, without automatic protocol downgrade.
- Provide independent hand and AGX gripper clients and examples. Grippers use
  their own topics and do not occupy BodyCmd slots.
- Pin Aorta SDK/messages 2026.10.12 (ABI 13), public vbot-humanoid-msgs 0.1.3
  and FlatBuffers 25.9.23; pin the official `humanoid-msgs-0.1.3` Release wheel
  SHA and actual BFBS hashes. Fetch schema and runtime wheels from their
  respective Releases; fail on missing tags or checksums instead of skipping
  validation.
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
