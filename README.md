# Humanoid SDK

Python SDK and examples for humanoid locomotion over Aorta. The Python import
name is `locomotion_aorta`. Run the SDK on a Linux x86_64 or aarch64 computer
with Python 3.10 or newer.

Dependencies are pinned to Aorta SDK and messages `2026.10.12` (ABI 13),
`vbot-humanoid-msgs==0.1.2`, and `flatbuffers==25.9.23`. Obtain the matching
wheels from this repository's [Releases](https://github.com/VitaDynamics/humanoid-sdk/releases),
using the `dependency_release_tag` in [compatibility.json](compatibility.json)
(`v0.1.0-rc.4`). Verify wheel filenames and SHA-256s against that manifest;
do not substitute older wheels if an asset is missing. The public body/EE `0.1.2` release
assets are being prepared; until they are available, use only the reviewed
public candidate bundle for host validation, not a private/HIL wheel.

EXTERNAL uses fixed31 `HumCmd`, `BodyState`, and typed `get_robot_info`.
Match the robot's public schema hashes and roster to [compatibility.json](compatibility.json).
Control requests use protocol2; ACK/Status remain protocol1; new body roots use
protocol1. HumCmd action is explicitly encoded using the generated EXTERNAL
symbol. Entry remains by action name and lease; the command does not activate
EXTERNAL. Older robots rejecting v2 receive no automatic downgrade or LowCmd
fallback. Discovery preserves partial body/hand results and actual field/limit
validity. The logical waist-roll slot can be unpopulated; it is skipped in
physical validation and output, and never represents a healthy zero sample.

Documentation: [Humanoid documentation](https://vitadynamics.feishu.cn/wiki/Mu1hw8wcSiKw3ykDGtAcvgXpnAg),
[SDK guide](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh),
and [PC networking and Zenoh configuration](https://vitadynamics.feishu.cn/wiki/M1vEw22vziSNAjkHVCncFlu1nyh).

## Offline installation and read-only check

Use a complete, matching offline bundle on your Linux computer. Create a
separate virtual environment to preserve an existing installation. From the
bundle root:

```bash
python3 -m venv venv
source venv/bin/activate
python -m pip install --no-index --find-links wheelhouse locomotion-aorta==0.1.0
python -m pip check
python -c 'import aorta, flatbuffers, locomotion_aorta; import lowlevel.HumCmd, lowlevel.BodyState, locomotion_sdk.ControlStatus'
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
python examples/lowstate_subscriber.py --group default --timeout 5
```

The last command connects to the robot and reads feedback; run it only with
connection authorization. Use the robot's actual group. A peer can connect to
a router, but the robot-facing address, listener, namespace and authentication
must all match the deployment. Missing authentication is an installation
blocker; do not remove it to connect.

## Motion example

`examples/external_control.py` requests control and moves30 installed body joints in the public31 roster.
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
This branch requires the EE candidate and its matching release assets; the
older `0.1.0` public wheel does not provide the new hand roots.

With connection authorization and the configured peer profile, read both hands:

```bash
python examples/brainco_hand_external.py --dry-run --side both --group default --timeout 5
```

The default entry is also read-only: no lease, hand command or touch write.
`HandClient` reads info/state/touch and supports typed `get_info`/`set_touch` RPCs.
Left/right wire values are 1/2; they differ from the device register numbering.

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
performed. The legacy `brainco_hand_control.py` forwards to this entry.

The body receives measured-pose31-slot HumCmd frames; hand commands use only
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
zero. Seven-stage robot timestamps use VitaNow microseconds; zero future stages
are unmeasured. Robot `cmd_trace` is enabled by its deployment-owned environment,
not the PC's environment. Host tests and packaging do not establish device HIL.

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

## PC policy example (in preparation)

`examples/teleop_policy.py` subscribes to BodyState, discovers the actual body
roster, runs a local action17 ONNX policy, and uses the same EXTERNAL lease and
CANCEL exit. Install the optional `teleop-demo` extra for ONNX Runtime and gamepad
input. `--model` is required; no model is bundled or selected automatically.
Use `--help` or `--dry-run --body-state-jsonl <capture>` before live use; dry-run
does not construct SDK transport. The offline validation used a noshoes model;
replace it with the delivered shoes model and recheck metadata/profile parity
when available. The example does not replace the robot's complete action17 FSM,
VR/upper stream or hardware validation. Live motion remains subject to the
existing on-site authorization and emergency-stop requirements above.
