"""Caller-thread, keep-last end-effector reception; no Python worker."""
import threading


class FeedbackPulls:
    def __init__(self, node, receive, failed):
        self.node, self.receive, self.failed = node, receive, failed
        self.rings = {}
        self._lock = threading.RLock()

    def add(self, kind, decoder, topic, qos):
        ring = self.node.create_subscriber_pull(topic, depth=1, qos=qos)
        self.rings[kind] = ring, decoder
        return ring

    def poll(self, wait_kind=None, timeout=0):
        # One latest sample per topic, not an unbounded drain driven by state Hz.
        # A bounded wait also lets close/other topic events be noticed promptly.
        with self._lock:
            for kind in sorted(self.rings, key=lambda k: k != 'info'):
                ring, decoder = self.rings[kind]
                try:
                    sample = ring.try_recv()
                    if sample is None and kind == wait_kind and timeout > 0:
                        sample = ring.recv(min(timeout, .05))
                    if sample is not None:
                        self.receive(kind, decoder(sample.payload))
                except Exception as error:
                    self.failed(error)
