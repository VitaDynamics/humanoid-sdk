# Generic end-effector example (development source)

This entry requires the matching generated schema and native BodyState3 codec.
The repository's old released pin is not a runnable combination with this
source increment. No device/HIL result is implied.

`end_effector_external.py` uses one shared controller for Body and both selected
end effectors. The default and `--dry-run` use a read-only source: no control
lease, command publisher or touch switch. `--side none` reads/controls Body only;
`--ee-only` omits Body discovery and Body commands. Each endpoint uses its own
explicit `--left-hz`/`--right-hz`; missed deadlines are not replayed. A combined
Body/EE call is not an atomic frame.

Motion requires `--confirm-motion` plus separate current on-site authorization.
Use full-axis targets in actual Info order with `--left-pos`/`--right-pos` and
explicit `--left-unit`/`--right-unit` (`m` total opening or `rad` joint angle).
`--*-vel-limit` is a nonnegative magnitude, not signed velocity. Zero preserves
the robot default and does not claim that default was measured. Omitted torque
has no intent bit; explicit zero and signed torque remain distinct. Limits and
units come from the actual descriptor; targets are never clipped. OP3 remains
unsupported.

`--pattern sine` requires two actual one-axis AGX endpoints and validates the
peak command slope. Angular `simultaneous`/`sequential` patterns require actual
six-axis BrainCo or Linker capabilities; the default fractions are 25%/75%, with
20 deg/s expressed as float32 radians. Sequential movement selects one axis
across the selected hands. These trajectories are SDK intent; backend output,
physical arrival and complete progress cannot be inferred from it.

The new State has an admission key, not the legacy numeric host-target pose.
Target-mode `--duration` bounds streaming only. It does not reuse a feedback-
applied target as the current host target, or claim an arrival duration. Entry
interpolation starts from a current valid same-descriptor State after ready and
touch waits; the saved pre-entry measurement remains the separate return target.
Body and each endpoint retain their absolute send deadlines across all phases;
the loop wakes at the earliest deadline without replaying missed sends. Entry
and return use declared command slope limits. Each command references the actual
endpoint State used for its safety checks (fixed K=1). Diagnostic timestamps are not permission gates and
are not subtracted from the SDK clock.

`--touch on/off` makes one existing typed RPC under the shared owner. The main
thread continues the original Body watchdog while one service-only worker
waits for a new valid State; enabling additionally requires a new Touch sample.
Info is never used as dynamic touch confirmation. The synchronous API separately
bounds service matching and RPC; the observation budget is not a strict total
RPC deadline. On failure or interrupt the original lifecycle attempts CANCEL
once before joining the service worker. No retry, second lease, autonomous
heartbeat or neutral end-effector target is sent. An unconfirmed CANCEL is an
error; PASSIVE may let unsupported limbs fall.

Old online callers remain temporarily while their safety regressions are
migrated. They are not a wire fallback for this entry, and will be removed in
the final source closure. Historical capture files are handled only by the
explicit offline capture/archive path.
