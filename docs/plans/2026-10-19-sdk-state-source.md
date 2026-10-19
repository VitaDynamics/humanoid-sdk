# Independent body31 state source

Owner's 2026-10-19 scope reduction supersedes API v1/v1.1 and removes field-selection
APIs and controller borrowing. The remaining public additions are BodyStateSource,
BodyStateSnapshot, BodyStateMetadata, FrameToken, StateSourceClosedError and
StateSourceTimeoutError. ExternalController remains byte-identical to b697f8b.

```python
from locomotion_aorta import BodyStateSource

source = BodyStateSource(group="default", conversion="on_demand")
try:
    frame = source.snapshot(timeout=1.0)
    print(frame.metadata.state_id, frame.received_realtime_ns)
    state = frame.body_state()  # whole existing BodyState DTO
finally:
    source.close()
```

The source creates only a state subscriber, no control endpoint, service client or
lease. It is independent of ExternalController. Each immutable snapshot owns its
payload. Whole DTO and metadata conversions are cached per frame; old snapshots
remain usable after later frames and source close. No decoder speedup is claimed.

snapshot(0) polls: return a qualifying cached frame or immediately raise
StateSourceTimeoutError. Negative/NaN/bool timeouts raise ValueError; positive inf
explicitly waits without a deadline. Large finite waits are chunked at the platform
wait ceiling. close wakes waiters and takes precedence over cached frames.

after_token belongs to one source instance and identifies a later local receive event,
not producer freshness. It does not replace state_id/source timestamp/validity checks.
The callback directly reads time.monotonic_ns() and time.time_ns(); no offset conversion
is used. Cross-process points use REALTIME, same-process elapsed may use MONOTONIC.

eager mode attempts a whole DTO conversion per received frame; on_demand defers it
until body_state(). Malformed eager frames retain their token and original decode
error, which is raised in the callback and on access. No failed conversion is replaced
by an older successful DTO or zeros. Controller policy consumption remains through
the existing wait_body_state() interface. The old default codec/control path is unchanged.
