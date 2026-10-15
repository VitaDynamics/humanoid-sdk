# Humanoid SDK

This D2 successor uses the released public body/EE schema `0.1.2` from
S2 daadf8f7449e. Its official CI-built wheel is pinned in the manifest and
available with its SHA-256 sidecar in dependency release `v0.1.0-rc.4`.
Its RobotInfo/get_robot_info layout removes the end-effector info-service
endpoint; use matching generated bindings and robot software. Earlier 6520
pre-release RobotInfo buffers are incompatible with this checkout.

Python SDK and examples for humanoid locomotion over Aorta. The Python import
name is `locomotion_aorta`. Run the SDK on a Linux x86_64 or aarch64 computer
with Python 3.10 or newer.

Dependencies are pinned to Aorta SDK and messages `2026.10.12` (ABI 13),
`vbot-humanoid-msgs==0.1.2`, and `flatbuffers==25.9.23`. Obtain the matching
wheels from this repository's [Releases](https://github.com/VitaDynamics/humanoid-sdk/releases),
using the `dependency_release_tag` in [compatibility.json](compatibility.json)
(`v0.1.0-rc.4`). Verify wheel filenames and SHA-256s against that manifest;
do not substitute older wheels if an asset is missing. The public body/EE
`0.1.2` wheel and sidecar are included in that dependency release; use the
public wheel rather than a private/HIL wheel.

EXTERNAL uses 31-slot `BodyCmd` joints vector, `BodyState`, and typed `get_robot_info`.
Match the robot's public schema hashes and roster to [compatibility.json](compatibility.json).
Control requests use protocol 2; ACK/Status remain protocol 1; new body roots use
protocol 1. BodyCmd action is explicitly encoded using the generated EXTERNAL
symbol. Entry remains by action name and lease; the command does not activate
EXTERNAL. Older robots rejecting v2 receive no automatic downgrade or LowCmd
fallback. Discovery preserves partial body/hand results and actual field/limit
validity. The logical waist-roll slot can be unpopulated; it is skipped in
physical validation and output, and never represents a healthy zero sample.

Documentation: [Humanoid documentation](https://vitadynamics.feishu.cn/wiki/Mu1hw8wcSiKw3ykDGtAcvgXpnAg),
[SDK guide](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh),
and [PC networking and Zenoh configuration](https://vitadynamics.feishu.cn/wiki/M1vEw22vziSNAjkHVCncFlu1nyh).

## Examples

| Main demo | Behavior already implemented |
|---|---|
| `examples/gripper_external.py` | One entry for read-only gripper info/state, single-side target/return and dual-side sine; motion shares body zero/hold and one EXTERNAL lifecycle. |
| `examples/brainco_hand_external.py` | Independent hand info/state/touch and authorized hand motion; `examples/brainco_capture.py` records/exports feedback. |
| `examples/teleop_policy.py` | PC-local ONNX inference and gamepad input; live mode publishes through EXTERNAL, while `--dry-run` processes offline BodyState records without transport. A model must be supplied explicitly. |

Basic examples remain `examples/external_control.py` for body EXTERNAL,
`examples/body_state_subscriber.py` for read-only BodyState, and
`examples/fixed_stand_pose.py` as the robot-profile zero-pose helper.
The policy demo is not limited to offline use; installation or `--help`
does not authorize live motion.

## Offline installation and read-only check

Use a complete, matching offline bundle on your Linux computer. Create a
separate virtual environment to preserve an existing installation. From the
bundle root:

```bash
python3 -m venv venv
source venv/bin/activate
python -m pip install --no-index --find-links wheelhouse locomotion-aorta==0.1.0
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
python examples/body_state_subscriber.py --group default --timeout 5
```

The last command connects to the robot and reads feedback; run it only with
connection authorization. Use the robot's actual group. A peer can connect to
a router, but the robot-facing address, listener, namespace and authentication
must all match the deployment. Missing authentication is an installation
blocker; do not remove it to connect.

## Motion example

`examples/external_control.py` requests control and moves 30 installed body joints in the public 31 roster.
Review its targets and gains for the specific robot, and obtain explicit
motion authorization, reliable support, an attendant and emergency-stop access.
Installation and read-only checks do not authorize motion.

`ExternalController.enter_external(timeout=5.0)` requests control without an
automatic expiry; `timeout` only bounds the entry wait. Old callers must remove
`lease_duration_s`. `exit_external(timeout=5.0)` sends CANCEL to PASSIVE, which
can let loaded limbs fall. Client failure does not automatically release
ownership. Command watchdogs and validation remain active. Use compatible robot
software supporting unlimited control; an older server rejection is an error.

## Independent Revo2 hands

Use the public EE wheel pinned in [compatibility.json](compatibility.json).
The distribution is `vbot-humanoid-msgs`; generated imports retain `lowlevel`,
`locomotion_sdk`, `aorta.services.humanoid_locomotion` and `*_schema_meta`.
Do not install the full private `vita-msgs` wheel or a temporary HIL wheel.
The pinned public `0.1.2` wheel provides the new hand roots; the older
`0.1.0` public wheel does not.

With connection authorization and the configured peer profile, read both hands:

```bash
python examples/brainco_hand_external.py --dry-run --side both --group default --timeout 5
```

The default entry is also read-only: no lease, hand command or touch write.
`HandClient` reads info/state/touch topics; `wait_info(timeout=5)` waits for the
first info sample or returns its cache. HandInfo is published about once a second;
late subscribers wait for a subsequent sample, with no historical replay promise.
Only the typed `set_touch` control RPC remains. After that control call, the demo
waits for a later info event with the acknowledged `touch_enabled` value;
`ts_info_us` need not change. `wait_info(after_info=client.info)` can request a
later event without clearing the info cache used by command publication.
Left/right wire values are 1/2; they differ from the device register numbering.

HandState uses the T021 public EE layout. It stores the three
sample timestamps once per message and exposes five `ts_cmd_*` stages; zero
future stages remain unmeasured. Touch uses `state_id`, `ts_state_pub_us` and
`raw_words`, with geometry from HandInfo. Old candidate recordings require their
matching SDK; the MCAP exporter refuses BFBS that differ from loaded bindings.
`HandJointState` 的 `*_raw` 字段仅供诊断，含义随设备型号变化，不应依赖。
Examples use normalized feedback and do not depend on these raw fields.

After separate on-site motion authorization, support, an attendant and an
emergency stop, a right-hand sequential round can be run with:

```bash
python examples/brainco_hand_external.py --side right --group default --timeout 5 --body-gains zero --pattern sequential --confirm-motion
```

The default speed is **20 degrees/s**, converted to float32 rad/s and checked
against every joint's advertised `dq_max`. The default fractions remain
25% -> 75% -> 25%; `--low-frac 0 --high-frac 1` selects declared endpoints.
`--pattern sequential` moves one joint at a time, keeping other targets low;
`--loop` repeats until SIGINT/SIGTERM or a safety failure. No automatic retry is
performed.

The body receives measured-pose 31-slot BodyCmd frames; hand commands use only
`/sdk/foot_humanoid/hand/<side>/cmd`. `--body-gains zero` selects kp=kd=dq=tau=0,
with q still measured; it does not support the body or instantly zero gains
during EXTERNAL entry. The default `hold` gains reuse `external_control.py`.
The existing 0.05 rad body drift guard, freshness, ownership, sequence and
hand-limit checks remain active.

Normal exit, faults and SIGINT/SIGTERM stop new targets and attempt CANCEL,
which must confirm PASSIVE. There is no automatic neutral hand trajectory;
PASSIVE can let loaded body limbs fall. A killed client does not guarantee
lease release. Hand session/sequence and body EXTERNAL generation are separate.

`examples/brainco_capture.py` subscribes to independent hand topics plus body
BodyState/ControlStatus (and historical LowCmd/LowState recordings) and exports MCAP with the optional `hil` extra.
Missing or nonfinite hand feedback is retained as JSON null, never a synthetic
zero. Five-stage robot timestamps use Vita host microseconds; zero future stages
are unmeasured. Public HandState carries these stages; no separate public trace
root is required. Host tests and packaging do not establish device HIL.

## Development and offline packaging

Install the matching dependency wheels into a separate virtual environment:

```bash
python -m pip install --find-links wheelhouse -e . build
make check
make build
python tools/package_release.py --wheelhouse wheelhouse --output dist/offline-bundle
python tests/offline_install.py dist/offline-bundle
```

The dependency wheelhouse contains both Linux Aorta SDK architectures,
aorta-msgs, vbot-humanoid-msgs and FlatBuffers. The bundle adds the locomotion
SDK wheel, examples, source and public PC configuration template. The packager
verifies dependency hashes, refuses configured credentials and existing output
directories, and writes `MANIFEST.json` with file digests and source state.
Local installation and native imports do not establish robot connectivity or
motion compatibility on either architecture.

The `SDK latency report` check exercises Python SDK ↔ C++ Mock communication
at 50–1000 Hz for at least 60 seconds per rate. See the
[benchmark documentation](tests/latency/README.md) for its measurement boundary
and reports. It does not establish robot performance or motion acceptance.

## Independent AGX grippers (in preparation)

The single `examples/gripper_external.py` entry contains the EXTERNAL lifecycle,
body zero/hold, single-side targets/return and optional dual-side sine mode.
`GripperClient` uses `/sdk/foot_humanoid/gripper/<left|right>/{cmd,state,info}`.
It shares the same `GripperInfo` DTO with `get_robot_info`; there is no per-side
info RPC and no gripper slot in the 31-joint BodyCmd. The default example and
`--dry-run` only subscribe, without ownership or command publication:

```bash
python examples/gripper_external.py --side right --dry-run --group default --timeout 5
```

`publish_target(q, dq=0, torque_cmd_nm=None)` uses absolute **total opening in m**
and a host speed cap in **m/s**. It preserves dq=0 for the robot's configured
default; it does not interpolate a second SDK opening trajectory. Values beyond
the advertised ranges are rejected before publishing. Unknown limit sources
remain readable diagnostics and cannot authorize a target.

Torque is a signed software motor-side command in N·m: None uses configured
hold torque; explicit 0 sends 0. Its absolute bound and hold default come from
CONFIG. It is not gripping force in N or a device limit; device interpretation
is governed by the vendor. Acceleration/deceleration are not first-release fields.

After explicit on-site authorization, suspension/support, an attendant and an
emergency stop, choose a target from the actual Info limits and run:

```bash
python examples/gripper_external.py --side right --target-m "$TARGET_OPENING_M" --confirm-motion
```

The example enters EXTERNAL through the existing body preflight/PREPARE/COMMIT,
holds the measured 31-slot body pose and exits through CANCEL to PASSIVE.
`--body-pose zero --body-gains hold` instead ramps to zero using the recorded
robot FIXED_STAND profile, then holds zero in the same EXTERNAL session.
Zero-settle error/speed/dwell and this gripper example's body hold drift are
observation only; they do not trigger CANCEL. Entry stationary, state validity,
freshness, ownership, phase, rejection and CANCEL-confirmation guards remain active.
`BodyState.applied_cmd` is currently always None on the matching robot runtime;
it cannot be used as execution evidence. The gripper sample, host target and
latest-received trace keys stay separate; a host echo is
not proof of device execution or arrival. Zero trace timestamps remain unknown,
including AGX write_ack. No robot connection or motion has been tested here.

After the same on-site authorization, `--side both --pattern sine` uses one
EXTERNAL session for body zero/hold, simultaneous-phase gripper targets,
return to the entry openings, then CANCEL. Defaults are 0.01–0.08 m, a 10 s
period and 30 s of sine streaming; transitions add to the total runtime.
`--speed-m-s 0` keeps the robot configuration default. The two topic writes
share one time phase but do not promise atomic device output or arrival.

```bash
python examples/gripper_external.py --side both --pattern sine --body-pose zero --body-gains hold --period-s 10 --duration 30 --confirm-motion
```

## PC policy example (in preparation)

`examples/teleop_policy.py` subscribes to BodyState, discovers the actual body
roster, runs a local action 17 ONNX policy, and uses the same EXTERNAL lease and
CANCEL exit. Before every live inference it waits up to the existing `--timeout`
for a strictly increasing BodyState `state_id`; unchanged or older feedback is
not reused for another inference or leg command, and timeout attempts CANCEL.
Install the optional `teleop-demo` extra for ONNX Runtime and gamepad input. `--model` is required; no model is bundled or selected automatically.
Use `--help` or `--dry-run --body-state-jsonl <capture>` before live use; dry-run
does not construct SDK transport. The offline validation used a noshoes model;
replace it with the delivered shoes model and recheck metadata/profile parity
when available. The example does not replace the robot's complete action17 FSM,
VR/upper stream or hardware validation. Live motion remains subject to the
existing on-site authorization and emergency-stop requirements above.
