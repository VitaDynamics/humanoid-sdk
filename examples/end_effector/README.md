# Generic end-effector example (development source)

This source targets the R3 static-descriptor schema candidate. Its actual
generated bindings, final robot integration, packaging and HIL remain pending.
Do not substitute an older wheel for the matching generated dependency.

`end_effector_external.py` uses one shared controller for Body and both selected
end effectors. The default and `--dry-run` use a read-only source: no control
control request, command publisher or touch switch. `--side none` reads/controls Body only;
`--ee-only` still queries RobotInfo for the shared startup identity but omits Body
motion preflight and Body commands. Each endpoint uses its own
explicit `--left-hz`/`--right-hz`; missed deadlines are not replayed. A combined
Body/EE call is not an atomic frame.

AGX motion additionally requires each selected side's `--left-native-hz` or
`--right-native-hz`, taken from that arm group's effective deployed
`send_frequency` after overrides, and `--native-config` naming that effective
configuration. These are operator-supplied inputs, not automatic verification
of the robot. They are distinct from the PC publish rate; no hardware period is
assumed. Read-only use and the BrainCo/Linker paths do not require them.

Motion requires `--confirm-motion` plus separate current on-site authorization.
Use full-axis targets in actual Info order with `--left-pos`/`--right-pos` and
explicit `--left-unit`/`--right-unit` (`m` total opening or `rad` joint angle).
`--*-vel-limit` is a nonnegative magnitude, not signed velocity. Zero preserves
the robot default and does not claim that default was measured. Omitted torque
has no intent bit; explicit zero and signed torque remain distinct. Limits and
units come from the actual descriptor; targets are never clipped. OP3 remains
unsupported.

`--pattern sine` requires two actual one-axis AGX endpoints and validates the
peak command slope. Angular `simultaneous`/`sequential` patterns require six-axis
angular capability and a proven positive slope limit. The BrainCo example keeps
the default fractions 25%/75%, with 20 deg/s expressed as float32 radians;
an actual lower limit rejects that default. Current LINKER_L6 declares POS only,
so its vendor entry accepts `target` only, without borrowing BrainCo velocity.
Sequential first moves all selected axes to low, then varies one axis while
continuing POS commands on the other axes. These trajectories are SDK intent; backend output,
physical arrival and complete progress cannot be inferred from it.

Full-N target, sequential and `--return-to-start` are not first single-axis HIL.
For a separately authorized single-axis check, the existing direct
`EndEffectorTarget.command_fields` API distinguishes the selected POS axis from
non-target mask 0; use its established safety/control procedure. These file CLIs
do not expose that mask selection, fill missing positions with zero or silently
copy measurements into other axes and call them uncommanded. Even explicitly
submitting the measured position is still a command on every selected field.

State does not confirm individual command admission, physical output or arrival.
Target-mode `--duration` bounds local streaming only. Entry
interpolation for trajectory patterns starts from a current valid same-descriptor State after ready and
touch waits; the saved pre-entry measurement remains the separate return target.
Body and each endpoint retain their absolute send deadlines across all phases;
the loop wakes at the earliest deadline without replaying missed sends. Trajectory entry
and return use declared command slope limits. For AGX, target, sine and return
are explicit source steps bounded by the effective command rate and the smaller
of the actual source interval and one native period. A late wakeup never banks
extra distance or replays missed sends. Float32 rounding is kept within that
budget. A finite timeout ends the example if the final source point cannot be
submitted; this is not a guarantee that native accepted any particular point.
Each command references the actual
endpoint State used for its safety checks (fixed K=1). Diagnostic timestamps are not permission gates and
are not subtracted from the SDK clock.

For a requested nonzero AGX movement the example also waits a bounded time for
fresh feedback showing some movement after the first nonzero local submission,
relative to the position in that submission's input sample. Its observation threshold is at most one
micrometre, the AGX wire opening representation; this does not claim device
accuracy or provide per-command admission, output or arrival confirmation. A
silent loss, busy path or stalled device may therefore end as unconfirmed
failure. Stop, fault, permission loss and the original CANCEL still take
precedence over producing the final point.

`--touch on/off` makes one existing typed RPC under the shared owner. The main
thread continues the original Body watchdog while one service-only worker
waits for a new valid State; enabling additionally requires a new Touch sample.
Info is never used as dynamic touch confirmation. The synchronous API separately
bounds service matching and RPC; the observation budget is not a strict total
RPC deadline. On failure or interrupt the original lifecycle attempts CANCEL
once before joining the service worker. No retry, second lease, autonomous
heartbeat or neutral end-effector target is sent. An unconfirmed CANCEL is an
error; PASSIVE may let unsupported limbs fall.

The old HandClient/GripperClient, their online DTOs/codecs and separate motion
entries are removed. RobotInfo's nested HandInfo/GripperInfo and the explicit
LowState offline path remain. Migrate callers to EndEffectorClient with the
existing connection; legacy private sequence counters, numeric host-target
poses and `pull_feedback` are not adapters to the new contract.

The common streaming duration defaults to 1 second; select `--duration 30
--period-s 10` explicitly for three sine periods (transitions add time).
Unlike the retired gripper entry, target-mode duration does not infer travel
from a numeric host target. Retained historic capture/trajectory fixtures do
not assert old wire parity for the new protocol. A later valid Info with the
same immutable descriptor is not a touch-state acknowledgement; touch changes
are confirmed by State and, when enabled, an actual Touch sample.

Closing an endpoint attempts every owned handle and preserves the shared
connection; it does not send cancellation or retry failed cleanup. The enclosing
motion lifecycle remains responsible for CANCEL and for joining its service
worker before closing the connection. Historical recordings require their
explicit matching archive path; they are never decoded as live fallback data.

## Robot-local TELEOP balance with remote EE input (source candidate)

The Python controller also exposes `enter_control("HUMANOID_TELEOP", timeout)`
and `exit_control(timeout)`. These use the same request/lease/COMMIT connection
as EXTERNAL. Entry returns only after both accepted COMMIT and a fresh matching
RUNNING status with a valid generation; ACK and status can arrive in either
order. An already running robot action does not authorize this connection.
`enter_external(timeout)` still selects EXTERNAL, and `exit_external(timeout)`
remains a release wrapper for existing callers.

For an independently authorized motion session with a matching robot build:

1. Create one `PullController`, and lend it to both `EndEffectorClient` objects
   using `connection=controller`. Supply the actual startup observation-source
   tuples. `wait_info` accepts one immutable startup Info for the connection's
   nonzero runtime; it does not query RobotInfo. A runtime change invalidates
   the client, so reconnect and reacquire control before creating a new client.
   Do not infer observations or ownership from Info alone.
2. Call `active = controller.enter_control("HUMANOID_TELEOP", timeout)` and
   wait for it to return before sending EE input. The robot runs its local Body
   balance model. The SDK must not send BodyCmd PD, empty Body commands or a
   separate Body heartbeat; `controller.publish(BodyCmd(...))` is rejected
   before allocating a sequence in this mode.
3. Send targets with the existing `left.publish(target,
   input_observations=actual_refs)` / right equivalent. Both sides use the one
   controller's sequence and confirmed remote generation. Preserve the real
   descriptor, sample identities, limits and timeout rules. A returned
   submission identifies network input, not completed native output. It is not
   the robot's local inference/output-generation key.
4. Use the existing `set_touch` only under that same confirmed owner. It neither
   consumes a motion sequence nor renews a heartbeat. On stop/error, attempt
   `controller.exit_control(timeout)` before joining any existing service work;
   then close the EE clients and connection. An exit timeout is unconfirmed,
   and `close()` alone is only best-effort CANCEL, not proof of robot cleanup.

The CLI above, including `--ee-only`, still selects EXTERNAL. The separate
`teleop_policy.py` demo still runs a client-side Body policy through EXTERNAL;
neither entry silently changes into robot-local TELEOP. This Python addition
requires the corresponding robot action binding and regenerated static-Ref
bindings; final candidate packaging, combined execution and HIL are pending.

## Three vendor file entries

Run these files from the SDK checkout or its core bundle using the matched
installed SDK. Each is a thin call to the same `main`, `EndEffectorClient` and
EXTERNAL owner/loop. Before any control, every selected endpoint must have the
actual public side, model, axis count and quantity domain below. `--side both`
requires both to match; on a mixed robot choose only the actual vendor side.
The generic entry above retains `--side none`; vendor entries reject it.

| File | Actual descriptor and units | Available patterns |
| --- | --- | --- |
| `brainco_revo2.py` | BRAINCO_REVO2, six JOINT_ANGLE axes; rad and supported rad/s speed limits | target, simultaneous, sequential |
| `agx_gripper.py` | AGX, one TOTAL_OPENING axis; total opening in m, speed limit m/s | target, sine (both actual AGX sides) |
| `linker_l6.py` | LINKER_L6, six JOINT_ANGLE axes; rad, POS-only | target only |

Axis names/order, per-axis valid limits and their sources come from actual Info;
there is no universal six-finger ordering or CAN-ID assumption. BrainCo has
POS/VEL_LIMIT; LINKER_L6 has no velocity/torque capability to invent. AGX's
optional N·m column has VENDOR_UNCONFIRMED physical domain: it is not gripping
force in N, and AGX State has no measured velocity. Initial examples omit torque.
An unsupported explicit field still fails through the original codec. Six-axis
positions or speed limits require all six values, with no scalar broadcast.

After the normal peer profile/group setup and permission to connect, inspect
one actual side without taking control. `GROUP` is the deployment group;
`right` below is an example selection, not a statement about your robot:

```bash
python examples/end_effector/brainco_revo2.py --group "${GROUP:?}" --side right --dry-run
python examples/end_effector/agx_gripper.py --group "${GROUP:?}" --side right --dry-run
python examples/end_effector/linker_l6.py --group "${GROUP:?}" --side right --dry-run
```

Default invocation is also read-only. Check printed Info/State and valid bits;
missing motion limits or a dynamic fault remain observable rather than being
invented as healthy values. Motion still rejects absent/unproven limits.

Only after separate current motion authorization, support, clear pinch/movement
zones, attendant and emergency stop, the following full-axis templates may be
filled with explicitly chosen actual values. They are not single-axis HIL or
permission to operate. `--ee-only` omits Body commands, not EXTERNAL ownership
or the effect of CANCEL to PASSIVE on unsupported limbs.

```bash
# BrainCo: full-hand fraction sequence, not an isolated one-axis command.
# The original float32 20deg/s default must fit each actual speed limit;
# otherwise supply six explicit legal --right-vel-limit values.
python examples/end_effector/brainco_revo2.py --group "${GROUP:?}" --side right --ee-only \
  --pattern sequential --right-unit rad --low-frac "${LOW_FRAC:?}" --high-frac "${HIGH_FRAC:?}" --confirm-motion

# AGX: one explicitly selected total opening, never per-finger mm or force.
python examples/end_effector/agx_gripper.py --group "${GROUP:?}" --side right --ee-only \
  --pattern target --right-unit m --right-pos "${OPENING_M:?}" \
  --right-vel-limit "${OPENING_RATE_M_S:?}" --duration "${STREAM_S:?}" \
  --right-native-hz "${DEPLOYED_RIGHT_ARM_SEND_HZ:?}" \
  --native-config "${EFFECTIVE_ARM_CONFIG_REFERENCE:?}" --confirm-motion

# L6: six explicit rad values in actual Info order. No inferred velocity.
python examples/end_effector/linker_l6.py --group "${GROUP:?}" --side right --ee-only \
  --pattern target --right-unit rad --right-pos "${Q1_RAD:?}" "${Q2_RAD:?}" "${Q3_RAD:?}" \
  "${Q4_RAD:?}" "${Q5_RAD:?}" "${Q6_RAD:?}" --duration "${STREAM_S:?}" --confirm-motion
```

L6 target duration only bounds repeated input submission: no interpolation,
backend speed or arrival is inferred. The default 50 Hz per-side setting is PC
submission cadence, not a vendor speed or a measured bus guarantee. Body hold,
if selected instead of `--ee-only`, uses the same original owner/allocator and
guards. Keep Touch unchanged unless its actual capability and separate operation
are confirmed; Info alone never acknowledges a touch switch.

Ctrl-C/TERM/error stops target submission and attempts one original CANCEL before
joining any service wait. No zero tail target or automatic retry is added.
Check the actual PASSIVE confirmation; unconfirmed CANCEL remains an error,
and `close()` is not proof of release. A runtime/immutable descriptor change
requires a new connection, control context and client, not a cache clear.
