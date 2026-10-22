# Production-topic robot Mock

This standalone C++ Mock exercises SDK functionality on private loopback peers.
It does not connect to a robot or simulate actuator dynamics. Use `tests/latency`
for transport latency measurements; this Mock produces counters, not latency
reports or per-frame evidence.

Build with the pinned runtime wheels installed and the official FlatBuffers
25.12.19 source archive available (the builder verifies its pinned SHA):

```sh
python tools/production_topic_mock/build.py --build-dir /tmp/sdk-mock-build \
  --flatbuffers-archive /absolute/path/flatbuffers-25.12.19.tar.gz \
  --run-protocol-checks
```

The default target is the host architecture. Cross builds accept `--target-arch`,
`--cxx` and `--aorta-core`; never execute ARM output on an x86 host.

Run only through the isolated runner, using a new output directory:

```sh
python -m tools.production_topic_mock.run --mode production-topics \
  --build-dir /tmp/sdk-mock-build --output /tmp/sdk-mock-run --seconds 60
```

The runner chooses a private `mock-sdkmain-*` group (maximum 32 UTF-8 bytes),
uses only `127.0.0.1` peers, disables multicast/gossip/shared memory, and strips
inherited Aorta/Zenoh overrides from its children. A client profile listens on
`:0`, allowing multiple Nodes in the same client process. It connects only to
the assigned Mock and optional auxiliary peer. `client-environment.json` contains
the group/profile settings for an independently started client.

To have the runner launch a reviewed client, add `--allow-client` and finish the
command with `--client-command python /path/client.py ...`. This runs arbitrary
client code with your user permissions; it is not a sandbox. The client must
honor the assigned group/profile and avoid all hardware interfaces. At the end
of the requested duration it receives SIGTERM while the Mock stays alive for
cooperative CANCEL; remaining owned children are then terminated and reaped.

## Robot behavior

- BodyState defaults to 400 Hz, ControlStatus to 50 Hz, apply ticks to 500 Hz.
  `--state-hz` and `--apply-hz` select synthetic rates; they are not timing SLAs.
- The body has 31 slots; slot 14 is uninstalled. `get_robot_info` describes the
  synthetic roster and limits. `--initial-q` accepts 31 finite JSON values.
- `--initial-imu` accepts two normalized WXYZ quaternions for HIP and TRUNK.
  These are constant synthetic attitudes, not reconstructed sensor data.
- PREPARE/COMMIT/CANCEL use the public protocol, with identity/lease/version
  checks, command validation, increasing sequence and watchdog TIMEOUT_HOLD.
  COMMIT takes effect immediately here; real locomotion may transition later.
- The latest accepted BodyCmd is applied at a synthetic tick and its identity
  is echoed in BodyState. There is no CAN, device ACK or physical output.
- `--reject-operation 1|2|3` scripts PREPARE/COMMIT/CANCEL rejection.
- `--auxiliary-fixture` retains explicit hand/gripper Info, publication rates,
  command echo and timed Joy frames. No physical EE feedback is synthesized
  without an explicit fixture. Auxiliary and body clocks are host clocks.

`mock-result.json` reports accepted/rejected/applied commands, protocol request
and parse/endpoint error counts. `auxiliary.json` reports auxiliary counts and
pull-ring drops. No CSV, S0–S6 report or build/output SHA manifest is produced.

## Tests

`make check` includes both Mock directory unit suites. The native-wheel CI job
compiles the C++ Mock with warnings as errors, runs its protocol checks, and
runs a real SDK IPC smoke: two Nodes, RobotInfo, PREPARE/COMMIT, three BodyCmd
frames, applied identity echo and CANCEL/PASSIVE. Run that smoke locally with:

```sh
MOCK_BUILD_DIR=/tmp/sdk-mock-build \
  python -m unittest tools.production_topic_mock.test_ipc -v
```

This is host-only functional coverage. ARM execution, hardware timing, physical
stability and robot motion are not measured.
