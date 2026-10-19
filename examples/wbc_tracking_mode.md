# WBC tracking mode

The SDK `tracking_policy.py` example is now `wbc_tracking_mode.py` and its
policy class is `WbcTrackingPolicy` in `locomotion_aorta.wbc_tracking_policy`.
Profile: `wbc_tracking_asample.json`; log schema: `vita.wbc_tracking.v1`.
Robot action 14/HUMANOID_TRACKING and `groot_wbc.tracking_contract.v1` metadata
are unchanged. This SDK program still requests **EXTERNAL**.

Use a matching SDK 0.2.0 and finalized 0.1.3 bindings with `wbc-tracking-demo`.
Model construction lazily loads the built-in C++ ORT 1.20.1 Session, with the
existing `--threads` intra-op setting (default 1) and inter-op=1. There is no
runtime Python ORT dependency or fallback.

```bash
python examples/wbc_tracking_mode.py --help
python examples/wbc_tracking_mode.py --model /path/to/actor_g1.onnx \
  --dry-run --steps 100 --log /new/path/wbc-offline.jsonl
```

Dry-run uses synthetic public31 feedback and never constructs a node. Model
metadata controls the 28 actor joints, future-frame layout and observation
shape; the input dimension is not hardcoded. NPZ joint-name reordering, 20ms
resampling, terminal velocity, heading, lead-in, recurrent history, clipping,
limits and ankle feedforward retain the source policy's mathematics and checks.
The fixed neck targets remain q=0/kp=20/kd=1. No finger command is sent.

`--replay-mcap FILE` accepts the exact public `/sdk/foot_humanoid/body_state`
FlatBuffer topic with matching embedded BFBS. It does not decode old ROS
44-motor lowstate recordings. The offline profile carries the public A-sample
joint order; use a profile matching the recording's roster. Missing callback
clocks stay unknown rather than being synthesized from MCAP log_time. This
mode does not claim a lease or publish commands.

Live requires `--live --confirm-motion`, an explicitly authorized connection,
the deployment-owned peer session configuration, and confirmation of support,
clearance and emergency stop for the trial. It queries actual RobotInfo names,
uses public BodyStateSource snapshots and requests EXTERNAL. Snapshot reception
alone is not source-time advancement: the original increasing source-time and
50ms freshness/compute checks remain. The original ownership/reject/drop checks,
50Hz schedule without catch-up bursts, stop-before-entry behavior and
CANCEL/PASSIVE verification remain. Exit to PASSIVE can release loaded limbs.
An exception is not permission for automatic retry or a service restart.
Before requesting EXTERNAL, RobotInfo must contain all model joint names and
the fixed neck names, with each of those joints marked populated. Startup
errors list the missing or unpopulated names and do not take ownership.

Body commands/state have 31 slots. q/dq/tau/kp/kd are in the public joint domain;
error_code and sampling clocks are retained. Installation and Mock identity
come from RobotInfo. Trace keys are uint64 session_id/sequence; input_state_id
is uint64 and never wrapped. Inference timing/max_abs_obs remain JSONL evidence,
not invented BodyCmd fields. The flat applied tuple is recorded unchanged;
a zero first-output stamp is not a measured output. Local monotonic durations
and client realtime timestamps are recorded separately from robot clocks.

Tests retain the independent historical C++ two-tick observation golden and
its source/model metadata provenance. `tools/compare_policy_onnx.py --kind wbc`
adds same-version Python/native ORT 1.20.1 exact finite float32 inference
comparison in separate interpreters (+0/-0 equal). Historical model IDs remain
historical, not claims about another supplied actor. Source units, real binding
tests, installed-wheel/native numerical acceptance, ARM execution and hardware
motion are separate evidence stages. The latter stages are not measured here.
