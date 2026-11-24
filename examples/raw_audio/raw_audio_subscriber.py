#!/usr/bin/env python3
"""Subscribe to /raw_audio_dump and save received PCM blocks as a WAV file."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import math
import os
from pathlib import Path
import time
import wave


def decode_pcm(payload, channel=None):
    """Return rate, input channels, frames and interleaved pcm-s16 bytes.

    channel is a zero-based input channel index; None preserves all channels.
    """
    from foxglove.RawAudio import RawAudio

    message = RawAudio.GetRootAs(payload, 0)
    if message.Format() != b"pcm-s16":
        raise ValueError(f"unsupported audio format: {message.Format()!r}")
    rate = message.SampleRate()
    channels = message.NumberOfChannels()
    size = message.DataLength()
    if not rate or not channels or not size or size % (2 * channels):
        raise ValueError("invalid sample rate, channels or PCM block length")
    if channel is not None and not 0 <= channel < channels:
        raise ValueError(f"channel {channel} outside 0..{channels - 1}")
    pcm = bytes(message.Data(i) for i in range(size))
    frames = size // (2 * channels)
    if channel is not None:
        # Each frame contains one little-endian int16 per input channel.
        pcm = b"".join(pcm[i:i + 2] for i in range(2 * channel, size, 2 * channels))
    return rate, channels, frames, pcm


def record(args):
    # The current runtime reads the effective group from the environment too.
    group = args.group if args.group is not None else os.environ.get("AORTA_GROUP", "default")
    os.environ["AORTA_GROUP"] = group
    if args.config:
        os.environ["ZENOH_SESSION_CONFIG_URI"] = str(args.config.resolve())

    import aorta

    blocks = total_frames = 0
    audio_format = None
    writer = None
    started = None
    with ExitStack() as stack:
        node = aorta.Node("raw_audio_python", group=group)
        stack.callback(node.close)
        subscriber = node.create_subscriber_pull(
            args.topic, depth=256, qos=aorta.QoS.sensor_data()
        )
        stack.callback(subscriber.close)
        last_received = time.monotonic()
        last_report = last_received
        print(f"Listening: {args.topic}, group={group}, output={args.output}", flush=True)
        try:
            while True:
                now = time.monotonic()
                if started is not None and now - started >= args.duration:
                    break
                if now - last_received >= args.timeout:
                    raise TimeoutError(f"no audio received for {args.timeout:g}s on {args.topic}")
                sample = subscriber.recv(timeout=0.1)
                if sample is None:
                    continue
                rate, channels, frames, pcm = decode_pcm(sample.payload, args.channel)
                if audio_format is None:
                    audio_format = rate, channels
                    # Exclusive creation protects previous recordings.
                    output = stack.enter_context(args.output.open("xb"))
                    writer = stack.enter_context(wave.open(output, "wb"))
                    writer.setparams((1 if args.channel is not None else channels,
                                      2, rate, 0, "NONE", "not compressed"))
                    started = time.monotonic()
                    print(f"pcm-s16: {rate} Hz, {channels} input channels, "
                          f"first block={frames} frames ({1000 * frames / rate:.2f} ms)",
                          flush=True)
                elif audio_format != (rate, channels):
                    raise ValueError("audio format changed during recording; start a new WAV")
                writer.writeframesraw(pcm)
                blocks += 1
                total_frames += frames
                last_received = time.monotonic()
                if last_received - last_report >= 1:
                    print(f"blocks={blocks}, saved_audio={total_frames / rate:.3f}s", flush=True)
                    last_report = last_received
        except KeyboardInterrupt:
            pass
    if not blocks:
        raise RuntimeError("no audio captured")
    print(f"Saved {args.output}: {blocks} blocks, "
          f"{total_frames / audio_format[0]:.3f}s of received audio", flush=True)


def positive_seconds(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("raw_audio.wav"))
    parser.add_argument("--duration", type=positive_seconds, default=10,
                        help="wall-clock recording seconds after first block (default: 10)")
    parser.add_argument("--timeout", type=positive_seconds, default=5,
                        help="initial / idle receive timeout in seconds (default: 5)")
    parser.add_argument("--channel", type=int,
                        help="save one zero-based channel; default: all input channels")
    parser.add_argument("--topic", default="/raw_audio_dump")
    parser.add_argument("--group", help="explicit group overrides AORTA_GROUP; fallback: default")
    parser.add_argument("--config", type=Path, help="deployment-provided local session profile")
    args = parser.parse_args()
    if args.channel is not None and args.channel < 0:
        parser.error("--channel must be nonnegative")
    try:
        record(args)
    except (ImportError, OSError, ValueError, RuntimeError, TimeoutError) as error:
        parser.exit(1, f"error: {error}\n")


if __name__ == "__main__":
    main()
