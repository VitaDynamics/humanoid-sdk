# Changelog

## 0.2.0 (in preparation)

- Use the public 31-slot BodyCmd/BodyState interfaces and EXTERNAL control;
  control requests use protocol 2, without automatic protocol downgrade.
- Provide independent hand and AGX gripper clients and examples. Grippers use
  their own topics and do not occupy BodyCmd slots.
- Pin Aorta SDK/messages 2026.10.12 (ABI 13), public vbot-humanoid-msgs 0.1.2
  and FlatBuffers 25.9.23; dependency wheel hashes remain unchanged.
- Add an independent, read-only BodyStateSource with frame-owned snapshots,
  local receive tokens, metadata, whole BodyState access and direct
  CLOCK_REALTIME/MONOTONIC receipt timestamps. It creates no control endpoints
  or lease; ExternalController retains its existing state subscription.

Field-selection APIs and controller borrowing are excluded. No state decoding
speedup is claimed. Packaging and release are pending; local checks do not
establish aarch64 execution, robot connectivity or motion acceptance.
