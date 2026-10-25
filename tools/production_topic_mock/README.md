# Production-topic robot Mock

This standalone C++ Mock exercises SDK functionality on private loopback peers.
It does not connect to a robot or simulate actuator dynamics. Use `tests/latency`
for transport latency measurements; this Mock produces counters, not latency
reports or per-frame evidence.

This source increment requires actual schema1322 generated bindings for the
committed RobotInfo Ref and the matching BodyState3 native codec/pins. These
new generated inputs are pending; the fixed schema80 package is insufficient;
the commands below are not evidence of a completed build or run.

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
- The Mock returns its fixed runtime/epoch1 committed Ref after every configured
  EE Info has actually been published successfully; it does not model recovery.
  The no-EE roster needs no EE publication. This is synthetic IPC evidence, not
  proof of the robot's descriptor commit transaction.
- `--initial-imu` accepts two normalized WXYZ quaternions for HIP and TRUNK.
  These are constant synthetic attitudes, not reconstructed sensor data.
- PREPARE/COMMIT/CANCEL use the public protocol, with identity/lease/version
  checks, command validation, increasing sequence and watchdog TIMEOUT_HOLD.
  COMMIT takes effect immediately here; real locomotion may transition later.
- The latest accepted BodyCmd2 is applied at a synthetic tick. BodyState3 uses
  this Mock's startup DescriptorRef and actual synthetic tick's control time,
  key-valid flag and EXTERNAL source; it does not infer validity from a key.
  There is no CAN, device ACK or physical output.
- `--reject-operation 1|2|3` scripts PREPARE/COMMIT/CANCEL rejection.
- `--ee-fixture` supplies explicit generic EE fixtures to the **main process**.
  Its existing Protocol owns the only PREPARE/COMMIT/CANCEL authority for Body
  and both endpoints. EE and typed `set_touch` never refresh Body's watchdog or
  reserve a separate lease. Body TIMEOUT_HOLD retains the existing owner;
  CANCEL clears pending EE commands and valid output keys without moving to zero.
- `--auxiliary-fixture` accepts only timed Joy frames. Old Hand/Gripper auxiliary
  endpoints and their independent session table are removed. Move EE fixtures
  to the main process; there is no old-wire fallback.

The EE JSON is keyed by `left`/`right`. Each selected endpoint supplies `info`
(an actual EndEffectorInfo FlatBuffer file), `initial_pos` in Info axis order,
`state_hz`, `info_hz`, and the fixed ordered `observation_sources` pairs
`[stream, side]`. Touch-capable Info also requires `touch` (an actual matching
EndEffectorTouch packet) and `touch_hz`. Paths are relative to that JSON file
unless absolute. No frequencies, positions or permission flags are inferred.
Stream/side meanings are the actual schema enums; unsupported OP3 is rejected.
For example, a right endpoint consuming its own State uses `[[2, 2]]`.

The runner validates actual Info/Touch shape, limits and identity before any
child starts. The main process assigns one new startup runtime/epoch to Body
and every EE stream. Each endpoint has one pending command and independent
receive sequence checks because separate topics may arrive out of order.
State contains synthetic position and actual simulated output keys; other
measurements remain invalid, no feedback-carried applied key or complete
progress is invented. Touch retains the fixture byte payload and clears
hardware times/sequence validity. Typed `set_touch` changes that same endpoint
snapshot before replying; a failed reply is not claimed to undo the change.

`mock-result.json` reports accepted/rejected/applied commands, protocol request
and parse/endpoint error counts, plus EE accepted/rejected/applied and typed
touch request counts. `auxiliary.json` reports Joy publication counts only.

## Tests

`make check` includes the Mock group-contract unit suite. The native-wheel CI job
compiles the C++ Mock with warnings as errors, runs its protocol checks, and
runs a real SDK IPC smoke: two Nodes, RobotInfo, PREPARE/COMMIT, three BodyCmd
frames, applied identity echo and CANCEL/PASSIVE. The new source adds a main-EE
IPC case using real generated fixtures, one Body owner/allocator, BodyState3
association, per-axis EE keys, typed touch with State/Touch confirmation, and
CANCEL permission removal. Neither new case nor native checks have been run
as part of this source increment. Run that smoke only under the authorized gate:

```sh
MOCK_BUILD_DIR=/tmp/sdk-mock-build \
  python -m unittest tools.production_topic_mock.test_ipc -v
```

This is host-only functional coverage. ARM execution, hardware timing, physical
stability and robot motion are not measured.

`test_ipc` includes three synthetic service cases from `test_touch_wait_ipc.py` on the same actual SDK
connection: deferred reply after CANCEL, no reply until native timeout, and no
provider during matching. It invokes the real CLI, executor, typed RPC and C-ABI;
only observation wrappers record calls. The original compiled Mock owns Body
and EXTERNAL. The test peer supplies explicitly synthetic EE descriptions/state
and retains native service responder handles; it adds no production fault flag.

The test requires Body publications and distinct Body samples during the native
wait, CANCEL before that wait completes, and no EE motion. A late successful
reply must be rejected by the SDK's current ownership guard. The per-call RPC
client is freed by `set_touch` after its call ends; the real worker then joins
before EE client and shared connection close. JSON timing/order evidence goes
into the captured test log. These are regression sources until separately run;
no strict total timeout/free bound, hardware response or HIL result is implied.
