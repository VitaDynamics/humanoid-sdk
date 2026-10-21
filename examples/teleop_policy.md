# Action 17 teleop policy input

Live input uses Aorta `locomotion/joy`, through the same SDK JoyReader and
DirectionInput as velocity mode. It drains pending Joy samples each policy
tick, uses the latest axes, and records button-edge/gap diagnostics. A sample
is fresh for 1 s using its original local REALTIME dequeue stamp; reading a
cached sample does not refresh that stamp. Missing/stale/future-stamped input
supplies zero velocity axes, followed by the profile's normal deceleration.

The source contract is robot `a0ecd6e208a092e7eb55aa6bf96336e5f40c133e`:

- `humanoid/io/humanoid_aorta_command_ingress.cpp:51` copies Joy axes/buttons
  unchanged; the ingress subscribes to `locomotion/joy` at line 291.
- `humanoid/actions/rl_business_logic/humanoid_business_logic.cpp:5398`
  maps the configured Joy axes; the final range clamp is at line 5589.
- `humanoid/actions/policy_profiles/vt_human_v2_teleop_asample*.yaml` has the
  same command block for the base, none, BrainCo and gripper variants.
  The exact profile path/SHA checked on the robot is recorded in
  [action17_contract.json](action17_contract.json).

| Command | Joy axis | Range | Units |
|---|---:|---|---|
| vx | 1 | -0.4 to 0.6 | m/s |
| vy | 0 | -0.5 to 0.5 | m/s |
| wz | 3 | -0.8 to 0.8 | rad/s |
| height | disabled (-1) | default 0.855; clamp 0.375 to 0.855 | m |

For each velocity axis, `abs(v)<0.05` gives zero; otherwise positive values
scale by the upper range and negative values by the magnitude of the lower
range, then clamp. The raw arrays are not reordered or sign-flipped. Buttons
are logged as input/edge counters; this demo assigns them no motion actions.
The robot profile also supports Twist; this SDK demo selects only the Joy
branch as specified by the input contract.

Velocity smoothing uses LPF alpha 0.4 followed by a slew cap: xy acceleration/
deceleration 1.0/2.0 m/s², wz 1.3/2.6 rad/s². Height retains its configured
0.25/0.15 m/s down/up rates and alpha 1.0. These steps run at the configured
50 Hz policy frequency, independent of BodyState or Joy publication frequency.
Neither a local HID input option nor a fixed neutral-input option is exposed.

The action 17 model is separately supplied and must match its 492-input/
12-output metadata. A neutral Joy input retains the height/training pose; it
does not mean a zero BodyCmd. Live execution still requires authorization for
the specific supported robot, an attendant and physical emergency stop.
