"""Shared Aorta Joy adapter for the velocity and teleop demos."""
from .joy import JoyReader
from ._demo_latency import realtime_ns


class DirectionInput:
    """Profile adapter; JoyReader owns draining, edge counts and receipt time."""

    def __init__(self, transport):
        self.reader = JoyReader(transport.node)
        self.last = None

    def snapshot(self):
        self.last = self.reader.read()
        return self.last.axes if self.last.is_fresh(realtime_ns()) else ()

    def statistics(self):
        if self.last is None:
            return dict(joy_messages=0, last_joy_ns=None, last_joy_axes=())
        sample = self.last
        return dict(joy_messages=sample.samples_total,
                    last_joy_ns=sample.received_realtime_ns, last_joy_axes=sample.axes,
                    last_joy_buttons=sample.buttons,
                    joy_pressed_counts=dict(sample.pressed_counts),
                    joy_released_counts=dict(sample.released_counts),
                    joy_missing_since_read=sample.missing_samples_since_read,
                    joy_missing_total=sample.missing_samples_total,
                    joy_sequence_available=sample.sequence_available,
                    joy_event_history_complete=sample.event_history_complete,
                    joy_sequence_diagnostics=dict(sample.sequence_diagnostics))

    def close(self):
        self.reader.close()
