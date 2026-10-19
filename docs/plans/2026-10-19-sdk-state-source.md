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
