# Body31 explicit state source (API v1.1)

The default ExternalController still subscribes to and returns eager BodyState DTOs.
An explicit source can share one state subscription with a controller:

```python
from locomotion_aorta import BodyStateSource, ExternalController

source = BodyStateSource(group="default", conversion="on_demand")
try:
    frame = source.snapshot(timeout=1.0)
    print(frame.metadata.state_id, frame.received_realtime_ns)
    controller = ExternalController(group="default", state_source=source)
    try:
        state = controller.wait_body_state(timeout=1.0)
    finally:
        controller.close()
finally:
    source.close()
```

The source creates only one state subscriber. The caller owns it; closing a borrowing
controller does not close the source. Source close is not CANCEL. Closing either object
wakes its waiting calls. A closed source raises explicitly, even if a DTO was cached.
Existing snapshots retain their immutable payload and remain readable after close.

snapshot(0) polls: cached qualifying frame or immediate StateSourceTimeoutError.
Negative/NaN timeout raises ValueError; positive infinity explicitly waits without a
deadline. after_token waits for a local receive event on this source. Tokens do not
establish producer freshness and do not replace state_id, timestamps or policy checks.
wait_body_state retains its original finite positive timeout contract.

The callback reads time.monotonic_ns() and time.time_ns() directly. Use received_realtime_ns
with CLOCK_REALTIME points in another process. MONOTONIC may measure elapsed time within
one process. No offset conversion between clocks is used.

Eager mode converts every received frame; on_demand converts only when requested.
Full DTO conversion retains the original codec, errors, NaN values and metadata.
No decoding is performed while holding the controller ACK/status condition lock.
No command encoder, schema, default rate, timeout, or example policy guard is changed.

## Selected columns (U2)

```python
selection = frame.select(joint_indices=(30, 0, 14), fields=("q", "dq"), imu_location=1)
columns = selection.columns()
q = columns.values["q"]  # read-only native float32 memoryview backed by immutable bytes
imu = selection.imu_sample()  # explicit location, no coordinate transform
echo = frame.applied_cmd()   # independent source key preserved
```

Indices and fields preserve the requested order. slot14 is never filtered; NaN and
populated/is_mock/status/valid_fields/clock columns retain source values. A missing joint
vector yields empty columns, matching the existing empty DTO tuple; it never yields zeros.
Each memoryview access gets its own wrapper, so release() cannot invalidate another reader.
Successful conversion results are cached per frame/request; a failed conversion is not
cached as success. No cross-frame storage is reused. IMU absence/null state is retained;
duplicate requested locations raise ValueError only when imu_sample() is requested.
Selected success validates the requested component, not unread IMU/applied regions.
No NumPy dependency, role/limit interpretation, freshness bypass or policy transform is added.

A malformed frame received by an explicit eager source retains its new frame token.
Its conversion raises the original codec error in the callback and on later body_state()
access, so readers cannot silently obtain the previous successful DTO as its replacement.
The legacy controller/default typed callback path remains unchanged.
