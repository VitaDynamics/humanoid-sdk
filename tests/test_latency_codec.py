"""Real latency encoder boundary and installed-wheel provenance checks."""
import importlib.util
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('latency_client_codec', Path(__file__).parent / 'latency/client.py')
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)


class LatencyCodecTest(unittest.TestCase):
    def test_encoder_arguments_result_and_stage_stamps(self):
        timing = threading.local()
        seen = []
        def encode(*args, **kwargs):
            self.assertEqual(timing.fill_ns, 10)
            self.assertFalse(hasattr(timing, 'filled_ns'))
            seen.append((args, kwargs))
            return b'real-encoded-buffer', None
        with patch.object(client.time, 'monotonic_ns', side_effect=[10, 20]), \
             patch.object(client.time, 'time_ns', side_effect=[100, 200]):
            result = client.encode_with_timing(encode, timing, 'command', 1, measure=True)
        self.assertEqual(result, (b'real-encoded-buffer', None))
        self.assertEqual(seen, [(('command', 1), {'measure': True})])
        self.assertEqual((timing.fill_ns, timing.filled_ns), (10, 20))
        self.assertEqual((timing.fill_realtime_ns, timing.filled_realtime_ns), (100, 200))

    def test_encoding_failure_is_not_reported_as_completed_stage(self):
        timing = threading.local()
        def fail():
            raise ValueError('invalid command')
        with self.assertRaisesRegex(ValueError, 'invalid command'):
            client.encode_with_timing(fail, timing)
        self.assertFalse(hasattr(timing, 'filled_ns'))

    def test_both_python_sdk_and_codec_must_come_from_install(self):
        installed = SimpleNamespace(__file__=str(Path(sys.prefix) / 'lib/sdk.so'))
        source = SimpleNamespace(__file__='/outside-venv/python/locomotion_aorta/transport.py')
        client.require_installed_codec(installed, installed)
        for modules in ((source, installed), (installed, source)):
            with self.assertRaisesRegex(RuntimeError, 'installed SDK wheel'):
                client.require_installed_codec(*modules)
