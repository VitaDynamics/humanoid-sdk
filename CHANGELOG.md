# Changelog

## 0.2.0 (in preparation)

- Use the public 31-slot BodyCmd/BodyState interfaces and EXTERNAL control;
  control requests use protocol 2, without automatic protocol downgrade.
- Provide independent hand and AGX gripper clients and examples. Grippers use
  their own topics and do not occupy BodyCmd slots.
- Pin Aorta SDK/messages 2026.10.12 (ABI 13), public vbot-humanoid-msgs 0.1.2
  and FlatBuffers 25.9.23; dependency wheel hashes remain unchanged.
- Switch the optional teleop demo from Python ORT 1.23.2 to an embedded C++
  ORT 1.20.1 CPU backend, matching the robot. This intentionally changes
  numerical outputs: 82/120 values were not bit-identical across versions.
  The Python 1.20.1 test oracle remains exact; no tolerance is introduced.
  SDK 0.2.0 ships only two cp310 wheels, x86_64 and aarch64, for Ubuntu 22.04+
  (glibc 2.35 / GLIBCXX 3.4.29+). Each SDK wheel includes the extension and ORT;
  there is no separate backend package or pure Python wheel. Core import does
  not load the backend. The teleop extra contains only NumPy and pygame.
  Version 0.2.1 is planned to restore generic platform support using platform
  and pure Python wheels under the same SDK distribution name.
- Add an independent, read-only BodyStateSource with frame-owned snapshots,
  local receive tokens, metadata, whole BodyState access and direct
  CLOCK_REALTIME/MONOTONIC receipt timestamps. It creates no control endpoints
  or lease; ExternalController retains its existing state subscription.

Field-selection APIs and controller borrowing are excluded. No state decoding
speedup is claimed. Packaging and release are pending; local checks do not
establish aarch64 execution, robot connectivity or motion acceptance.
