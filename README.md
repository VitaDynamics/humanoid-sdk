# Humanoid SDK

Python SDK and examples for humanoid locomotion over Aorta. The Python import
name is `locomotion_aorta`. Run the SDK on a Linux x86_64 or aarch64 computer
with Python 3.10 or newer.

Dependencies are pinned to Aorta SDK and messages `2026.10.12` (ABI 13),
`vbot-humanoid-msgs==0.1.0`, and `flatbuffers==25.9.23`. Obtain the matching
wheels from this repository's [Releases](https://github.com/VitaDynamics/humanoid-sdk/releases),
using the `dependency_release_tag` in [compatibility.json](compatibility.json)
(`v0.1.0-rc.4`). Verify wheel filenames and SHA-256s against that manifest;
do not substitute older wheels if an asset is missing.

The HumanoidLowState wire layout differs from rc.2/rc.3, including the
`motor_states` field ID. LowCmd and the control messages retain their layouts.
The SDK and robot must use matching schema hashes. Public DTOs, topics, QoS,
control semantics and joint order are unchanged.

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
python -c 'import aorta, flatbuffers, locomotion_aorta; import lowlevel.LowCmd, locomotion_sdk.ControlStatus'
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

`examples/external_control.py` requests control and moves 42 command slots.
Review its targets and gains for the specific robot, and obtain explicit
motion authorization, reliable support, an attendant and emergency-stop access.
Installation and read-only checks do not authorize motion.

`ExternalController.enter_external(timeout=5.0)` requests control without an
automatic expiry; `timeout` only bounds the entry wait. Old callers must remove
`lease_duration_s`. `exit_external(timeout=5.0)` sends CANCEL to PASSIVE, which
can let loaded limbs fall. Client failure does not automatically release
ownership. Command watchdogs and validation remain active. Use compatible robot
software supporting unlimited control; an older server rejection is an error.

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
