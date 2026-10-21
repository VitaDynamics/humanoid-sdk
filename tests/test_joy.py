"""Real generated Joy payloads over an in-memory pull ring; no transport."""

from collections import deque
from dataclasses import FrozenInstanceError
from types import SimpleNamespace
import unittest
import sys
from unittest.mock import Mock, patch

import flatbuffers
import aorta.sys.AortaHeader as header_wire
import locomotion.Joy as joy_wire

from locomotion_aorta import JoyReader, JOY_PULL_DEPTH, JOY_TOPIC


def payload(sequence=0, *, axes=(0.25, -0.5), buttons=(0,), publisher="gamepad",
            header=True):
    builder = flatbuffers.Builder(128)
    header_offset = None
    if header:
        node = builder.CreateString(publisher) if publisher is not None else None
        header_wire.AortaHeaderStart(builder)
        header_wire.AortaHeaderAddSequence(builder, sequence)
        if node is not None:
            header_wire.AortaHeaderAddPublisherNode(builder, node)
        header_offset = header_wire.AortaHeaderEnd(builder)
    joy_wire.JoyStartAxesVector(builder, len(axes))
    for value in reversed(axes):
        builder.PrependFloat32(value)
    axes_offset = builder.EndVector()
    joy_wire.JoyStartButtonsVector(builder, len(buttons))
    for value in reversed(buttons):
        builder.PrependInt32(value)
    buttons_offset = builder.EndVector()
    joy_wire.JoyStart(builder)
    joy_wire.JoyAddAxes(builder, axes_offset)
    joy_wire.JoyAddButtons(builder, buttons_offset)
    if header_offset is not None:
        joy_wire.JoyAddAortaHeader(builder, header_offset)
    builder.Finish(joy_wire.JoyEnd(builder))
    return bytes(builder.Output())


class PullRing:
    def __init__(self, depth):
        self.queue = deque(maxlen=depth)
        self.close = Mock()
        self.empty_reads = 0

    def put(self, *frames):
        self.queue.extend(SimpleNamespace(payload=frame) for frame in frames)

    def try_recv(self):
        if not self.queue:
            self.empty_reads += 1
            return None
        item = self.queue.popleft()
        if isinstance(item, BaseException):
            raise item
        return item


class JoyReaderTest(unittest.TestCase):
    def setUp(self):
        self.ring = PullRing(JOY_PULL_DEPTH)
        self.node = SimpleNamespace(create_subscriber_pull=Mock(return_value=self.ring))
        self.clock = patch("locomotion_aorta.joy.time.time_ns", return_value=1_000_000_000)
        self.time_ns = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.reader = JoyReader(self.node)
        self.addCleanup(self.reader.close)

    def read(self, *frames):
        self.ring.put(*frames)
        return self.reader.read()

    def test_only_pull_subscription_no_callback_or_worker(self):
        self.node.create_subscriber_pull.assert_called_once()
        args, kwargs = self.node.create_subscriber_pull.call_args
        self.assertEqual(args, (JOY_TOPIC,))
        self.assertEqual(kwargs["depth"], 256)
        self.assertNotIn("callback", kwargs)
        self.assertNotIn("thread", vars(self.reader))

    def test_empty_initial_read_is_unavailable_and_stale(self):
        sample = self.read()
        self.assertEqual((sample.axes, sample.buttons), ((), ()))
        self.assertIsNone(sample.received_realtime_ns)
        self.assertFalse(sample.sequence_available)
        self.assertFalse(sample.event_history_complete)
        self.assertFalse(sample.is_fresh())
        self.time_ns.assert_not_called()

    def test_first_held_button_establishes_baseline(self):
        sample = self.read(payload(0, buttons=(1,)))
        self.assertEqual(sample.buttons, (1,))
        self.assertEqual(dict(sample.pressed_counts), {})
        self.assertFalse(sample.event_history_complete)
        self.assertTrue(sample.sequence_available)  # sequence 0 is valid

    def test_drain_short_double_press_and_return_last_axes(self):
        self.read(payload(0))
        frames = [payload(n, axes=(float(n),), buttons=(n % 2,)) for n in range(1, 5)]
        sample = self.read(*frames)
        self.assertEqual(sample.axes, (4.0,))
        self.assertEqual(sample.buttons, (0,))
        self.assertEqual(dict(sample.pressed_counts), {0: 2})
        self.assertEqual(dict(sample.released_counts), {0: 2})
        self.assertTrue(sample.event_history_complete)
        self.assertEqual((sample.samples_since_read, sample.samples_total), (4, 5))
        self.assertFalse(self.ring.queue)
        self.assertEqual(self.ring.empty_reads, 2)

    def test_read_consumes_increments_but_keeps_state_and_baseline(self):
        self.read(payload(0))
        first = self.read(payload(1, buttons=(1,)))
        self.assertEqual(dict(first.pressed_counts), {0: 1})
        empty = self.read()
        self.assertEqual(dict(empty.pressed_counts), {})
        self.assertEqual(empty.axes, first.axes)
        released = self.read(payload(2))
        self.assertEqual(dict(released.released_counts), {0: 1})
        self.assertEqual(dict(first.pressed_counts), {0: 1})

    def test_nonzero_button_values_are_pressed_without_retrigger(self):
        self.read(payload(0, buttons=(0, 0)))
        sample = self.read(payload(1, buttons=(-1, 2)), payload(2, buttons=(2, -5)),
                           payload(3, buttons=(0, 0)))
        self.assertEqual(dict(sample.pressed_counts), {0: 1, 1: 1})
        self.assertEqual(dict(sample.released_counts), {0: 1, 1: 1})

    def test_periodic_duplicate_state_has_no_edges(self):
        self.read(payload(0, buttons=(1,)))
        sample = self.read(payload(1, buttons=(1,)), payload(2, buttons=(1,)))
        self.assertFalse(sample.pressed_counts or sample.released_counts)
        self.assertTrue(sample.event_history_complete)

    def test_empty_read_does_not_refresh_receipt_or_freshness(self):
        initial = self.read(payload())
        self.time_ns.return_value = 3_000_000_001
        self.time_ns.reset_mock()
        empty = self.read()
        self.time_ns.assert_not_called()
        self.assertEqual(empty.received_realtime_ns, initial.received_realtime_ns)
        self.assertFalse(empty.is_fresh())
        self.assertTrue(initial.is_fresh(2_000_000_000))
        self.assertFalse(initial.is_fresh(2_000_000_001))
        self.assertFalse(initial.is_fresh(999_999_999))

    def test_receipt_uses_direct_realtime_for_each_pulled_sample(self):
        self.time_ns.side_effect = [100, 200, 300]
        sample = self.read(payload(0), payload(1), payload(2))
        self.assertEqual(sample.received_realtime_ns, 300)
        self.assertEqual(self.time_ns.call_count, 3)

    def test_forward_gap_reports_delta_and_total_without_inventing_edges(self):
        self.read(payload(10))
        gap = self.read(payload(14))
        self.assertEqual((gap.missing_samples_since_read, gap.missing_samples_total), (3, 3))
        self.assertFalse(gap.event_history_complete)
        self.assertFalse(gap.pressed_counts or gap.released_counts)
        empty = self.read()
        self.assertEqual((empty.missing_samples_since_read, empty.missing_samples_total), (0, 3))
        self.assertFalse(empty.event_history_complete)
        self.assertTrue(self.read(payload(15)).event_history_complete)

    def test_first_sequence_does_not_count_prehistory(self):
        self.assertEqual(self.read(payload(12345)).missing_samples_total, 0)

    def test_duplicate_sequence_does_not_add_missing_or_edges(self):
        self.read(payload(4, buttons=(1,)))
        sample = self.read(payload(4, buttons=(1,)))
        self.assertEqual(sample.missing_samples_total, 0)
        self.assertEqual(sample.sequence_diagnostics["duplicate"], 1)
        self.assertFalse(sample.pressed_counts or sample.released_counts)

    def test_uint64_contiguous_wrap_is_not_a_gap(self):
        self.read(payload((1 << 64) - 1))
        sample = self.read(payload(0, buttons=(1,)))
        self.assertEqual(sample.missing_samples_total, 0)
        self.assertEqual(dict(sample.pressed_counts), {0: 1})
        self.assertTrue(sample.event_history_complete)

    def test_reset_or_reorder_rebases_without_uint64_huge_gap(self):
        self.read(payload(100, buttons=(1,)))
        sample = self.read(payload(0))
        self.assertEqual(sample.missing_samples_total, 0)
        self.assertEqual(sample.sequence_diagnostics["reset_or_reorder"], 1)
        self.assertFalse(sample.released_counts)
        self.assertFalse(sample.event_history_complete)
        self.assertTrue(self.read(payload(1)).event_history_complete)

    def test_source_change_rebases_buttons_and_does_not_cross_subtract(self):
        self.read(payload(100, buttons=(1,), publisher="a"))
        sample = self.read(payload(900, publisher="b"))
        self.assertEqual(sample.missing_samples_total, 0)
        self.assertFalse(sample.released_counts)
        self.assertEqual(sample.sequence_diagnostics["source_changed"], 1)

    def test_missing_header_or_source_is_unknown_not_zero_loss_evidence(self):
        self.read(payload(0))
        for frame in (payload(header=False), payload(5, publisher=None)):
            with self.subTest(frame=frame):
                sample = self.read(frame)
                self.assertFalse(sample.sequence_available)
                self.assertFalse(sample.event_history_complete)
                self.assertEqual(sample.missing_samples_total, 0)
        self.assertEqual(self.read(payload(20)).missing_samples_total, 0)

    def test_length_change_does_not_synthesize_release_for_missing_button(self):
        self.read(payload(0, buttons=(0, 1)))
        shrunk = self.read(payload(1, buttons=(0,)))
        self.assertFalse(shrunk.released_counts)
        expanded = self.read(payload(2, buttons=(0, 1)))
        self.assertFalse(expanded.pressed_counts)
        self.assertFalse(expanded.event_history_complete)

    def test_bounded_ring_eviction_is_detected_only_after_known_baseline(self):
        self.read(payload(0))
        self.ring.queue = deque(maxlen=3)
        sample = self.read(*(payload(n) for n in range(1, 7)))
        self.assertEqual(sample.samples_since_read, 3)
        self.assertEqual(sample.missing_samples_since_read, 3)
        self.assertFalse(sample.event_history_complete)

    def test_failure_keeps_pending_edges_for_next_successful_read(self):
        self.read(payload(0))
        self.ring.put(payload(1, buttons=(1,)))
        self.ring.queue.append(RuntimeError("disconnected"))
        with self.assertRaisesRegex(RuntimeError, "disconnected"):
            self.reader.read()
        sample = self.read()
        self.assertEqual(dict(sample.pressed_counts), {0: 1})
        self.assertEqual(sample.samples_since_read, 1)

    def test_bad_frame_does_not_partially_commit_or_clear_prior_counts(self):
        self.read(payload(0))
        self.ring.put(payload(1, buttons=(1,)), b"bad")
        with self.assertRaises(Exception):
            self.reader.read()
        sample = self.read()
        self.assertEqual(dict(sample.pressed_counts), {0: 1})
        self.assertEqual(sample.samples_total, 2)

    def test_snapshots_and_count_maps_are_immutable(self):
        sample = self.read(payload())
        with self.assertRaises(FrozenInstanceError):
            sample.axes = ()
        with self.assertRaises(TypeError):
            sample.pressed_counts[0] = 1

    def test_close_is_idempotent_and_read_after_close_fails(self):
        self.reader.close()
        self.reader.close()
        self.ring.close.assert_called_once_with()
        with self.assertRaisesRegex(RuntimeError, "closed"):
            self.reader.read()

    def test_invalid_depth_fails_before_declaring_subscription(self):
        self.node.create_subscriber_pull.reset_mock()
        for depth in (0, -1, True, 1.5):
            with self.subTest(depth=depth), self.assertRaises(ValueError):
                JoyReader(self.node, depth=depth)
        self.node.create_subscriber_pull.assert_not_called()

    def test_missing_generated_binding_fails_without_fallback(self):
        from locomotion_aorta import AortaBindingsUnavailableError
        self.node.create_subscriber_pull.reset_mock()
        with patch.dict(sys.modules, {"locomotion.Joy": None}), \
             self.assertRaisesRegex(AortaBindingsUnavailableError, "0.1.5"):
            JoyReader(self.node)
        self.node.create_subscriber_pull.assert_not_called()
