"""Real generated RawAudio decoding and WAV output; no network connection."""

import argparse
from contextlib import redirect_stdout
import importlib.util
import io
import os
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import wave

from examples.raw_audio import raw_audio_subscriber as example


@unittest.skipUnless(importlib.util.find_spec("foxglove"), "requires real aorta-msgs wheel")
class RawAudioExampleTest(unittest.TestCase):
    def payload(self, pcm, *, rate=16000, channels=6, format_name="pcm-s16"):
        import flatbuffers
        import foxglove.RawAudio as schema

        builder = flatbuffers.Builder(0)
        data = builder.CreateByteVector(pcm)
        encoding = builder.CreateString(format_name)
        schema.RawAudioStart(builder)
        schema.RawAudioAddData(builder, data)
        schema.RawAudioAddFormat(builder, encoding)
        schema.RawAudioAddSampleRate(builder, rate)
        schema.RawAudioAddNumberOfChannels(builder, channels)
        builder.Finish(schema.RawAudioEnd(builder))
        return bytes(builder.Output())

    def test_six_channel_block_and_channel_extraction(self):
        pcm = struct.pack("<6h", -32768, -1, 0, 1, 2, 32767) * 512
        payload = self.payload(pcm)
        self.assertEqual(example.decode_pcm(payload), (16000, 6, 512, pcm))
        self.assertEqual(example.decode_pcm(payload, 1)[3], struct.pack("<h", -1) * 512)
        self.assertEqual(example.decode_pcm(payload, 5)[3], struct.pack("<h", 32767) * 512)

    def test_invalid_blocks_rejected(self):
        for kwargs, pcm, channel in [
            ({"format_name": "adpcm"}, b"\0" * 12, None),
            ({"rate": 0}, b"\0" * 12, None),
            ({"channels": 0}, b"\0" * 12, None),
            ({}, b"\0" * 11, None),
            ({}, b"", None),
            ({}, b"\0" * 12, 6),
        ]:
            with self.subTest(kwargs=kwargs, size=len(pcm), channel=channel):
                with self.assertRaises(ValueError):
                    example.decode_pcm(self.payload(pcm, **kwargs), channel)

    def test_bulk_copy_preserves_bytes_without_scalar_accessors(self):
        from foxglove.RawAudio import RawAudio

        pcm = struct.pack("<6h", -32768, -1, 0, 1, 2, 32767) * 256
        payload = bytearray(self.payload(pcm))
        with patch.object(RawAudio, "Data", side_effect=AssertionError("scalar PCM read")):
            decoded = example.decode_pcm(payload)
            mono = example.decode_pcm(payload, 5)
        payload[:] = b"\0" * len(payload)
        self.assertEqual(decoded, (16000, 6, 256, pcm))
        self.assertEqual(mono, (16000, 6, 256, struct.pack("<h", 32767) * 256))

    def test_record_finalizes_wav_and_inherits_group(self):
        pcm = struct.pack("<6h", 1, 2, 3, 4, 5, 6) * 512
        sub = Mock()
        sub.recv.side_effect = [SimpleNamespace(payload=self.payload(pcm)), KeyboardInterrupt]
        node = Mock()
        node.create_subscriber_pull.return_value = sub
        aorta = SimpleNamespace(Node=Mock(return_value=node), QoS=Mock())
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture.wav"
            args = argparse.Namespace(group=None, config=None, topic="/raw_audio_dump",
                                      output=output, duration=10, timeout=5, channel=2)
            with patch.dict(sys.modules, {"aorta": aorta}), \
                    patch.dict(os.environ, {"AORTA_GROUP": "deployment_group"}), \
                    redirect_stdout(io.StringIO()):
                example.record(args)
            aorta.Node.assert_called_once_with("raw_audio_python", group="deployment_group")
            with wave.open(str(output), "rb") as recording:
                self.assertEqual((recording.getnchannels(), recording.getsampwidth(),
                                  recording.getframerate(), recording.getnframes()),
                                 (1, 2, 16000, 512))
                self.assertEqual(recording.readframes(512), struct.pack("<h", 3) * 512)
            sub.close.assert_called_once_with()
            node.close.assert_called_once_with()

    def run_record(self, directory, samples, *, captured=None, **options):
        node, sub = Mock(), Mock()
        node.create_subscriber_pull.return_value = sub
        sub.recv.side_effect = samples
        observed = []

        def create_node(name, *, group):
            observed.append((os.environ.get("AORTA_GROUP"),
                             os.environ.get("ZENOH_SESSION_CONFIG_URI")))
            return node

        aorta = SimpleNamespace(Node=Mock(side_effect=create_node), QoS=Mock(), observed=observed)
        args = argparse.Namespace(group=None, config=None, topic="/raw_audio_dump",
                                  output=Path(directory) / "capture.wav",
                                  duration=10, timeout=5, channel=None)
        for key, value in options.items():
            setattr(args, key, value)
        stdout = captured if captured is not None else io.StringIO()
        with patch.dict(sys.modules, {"aorta": aorta}), redirect_stdout(stdout):
            try:
                example.record(args)
            finally:
                sub.close.assert_called_once_with()
                node.close.assert_called_once_with()
        return args.output, stdout.getvalue(), aorta

    def test_no_audio_times_out_without_creating_file(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(example.time, "monotonic", side_effect=[0, 0, 6]):
                with self.assertRaises(TimeoutError):
                    self.run_record(directory, [None])
            self.assertFalse((Path(directory) / "capture.wav").exists())

    def test_existing_file_is_unchanged_and_not_reported_saved(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture.wav"
            output.write_bytes(b"existing recording")
            stdout = io.StringIO()
            with redirect_stdout(stdout), self.assertRaises(FileExistsError):
                self.run_record(directory, [SimpleNamespace(payload=self.payload(b"\0" * 12))],
                                captured=stdout)
            self.assertEqual(output.read_bytes(), b"existing recording")
            self.assertNotIn("Saved", stdout.getvalue())

    def test_format_change_preserves_a_valid_partial_wav(self):
        pcm = struct.pack("<6h", 1, 2, 3, 4, 5, 6) * 512
        samples = [SimpleNamespace(payload=self.payload(pcm)),
                   SimpleNamespace(payload=self.payload(pcm, rate=48000))]
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "format changed"):
                self.run_record(directory, samples)
            with wave.open(str(Path(directory) / "capture.wav"), "rb") as recording:
                self.assertEqual(recording.getframerate(), 16000)
                self.assertEqual(recording.getnframes(), 512)
                self.assertEqual(recording.readframes(512), pcm)

    def test_explicit_group_and_config_are_set_before_node_creation(self):
        pcm = b"\0" * 12
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "peer.json5"
            with patch.dict(os.environ, {"AORTA_GROUP": "deployment_group"}):
                output, stdout, aorta = self.run_record(
                    directory, [SimpleNamespace(payload=self.payload(pcm)), KeyboardInterrupt],
                    group="explicit_group", config=config)
                self.assertEqual(os.environ["AORTA_GROUP"], "explicit_group")
                self.assertEqual(os.environ["ZENOH_SESSION_CONFIG_URI"], str(config.resolve()))
                aorta.Node.assert_called_once_with("raw_audio_python", group="explicit_group")
                self.assertEqual(aorta.observed, [("explicit_group", str(config.resolve()))])
                self.assertIn("Saved", stdout)

    def test_invalid_durations_are_rejected(self):
        for value in ["0", "-1", "nan", "inf"]:
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                example.positive_seconds(value)


if __name__ == "__main__":
    unittest.main()
