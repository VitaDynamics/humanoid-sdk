#!/usr/bin/env python3
"""Take over the humanoid display, show images/animations/text, or return to RCP.

Requires the SDK and generated bindings matching the robot's schema release.
See README.md for installation, connection configuration and examples.
"""

from __future__ import annotations

import argparse
import importlib
import io
import os
from pathlib import Path
import sys
import time


def _schema(name):
    return importlib.import_module(f"aorta.services.peripheral.{name}")


def _table(schema, builder, **fields):
    """Use generated builders, without embedding schema field IDs."""
    name = schema.__name__.rsplit(".", 1)[-1]
    getattr(schema, name + "Start")(builder)
    for field, value in fields.items():
        getattr(schema, name + "Add" + field)(builder, value)
    return getattr(schema, name + "End")(builder)


def _string_vector(schema, field, builder, values):
    offsets = [builder.CreateString(value) for value in values]
    name = schema.__name__.rsplit(".", 1)[-1]
    getattr(schema, name + "Start" + field + "Vector")(builder, len(offsets))
    for offset in reversed(offsets):
        builder.PrependUOffsetTRelative(offset)
    return builder.EndVector()


class Screen:
    """An external display client; closing it does not release the switch."""

    def __init__(self, *, config=None, group="default", timeout_ms=5000):
        # Set connection options before the SDK creates its first session.
        if config:
            os.environ["ZENOH_SESSION_CONFIG_URI"] = str(Path(config).resolve())
        os.environ["AORTA_GROUP"] = group
        self.aorta = importlib.import_module("aorta")
        self.timeout_ms = timeout_ms
        self.node = self.aorta.Node("external_display_python", group=group)

    def close(self):
        self.node.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def _call(self, endpoint, request_name, response_name, fill):
        response_module = _schema(response_name)
        response_type = getattr(response_module, response_name)
        options = self.aorta.ClientOptions(timeout_ms=self.timeout_ms)
        with self.node.create_client_typed(
            "/display_node/" + endpoint,
            lambda payload: response_type.GetRootAs(payload, 0),
            options=options,
        ) as client:
            if not client.raw().wait_for_matching(self.timeout_ms):
                raise RuntimeError(f"service unavailable: /display_node/{endpoint}")
            response = client.call(
                lambda builder, header: fill(_schema(request_name), builder, header)
            ).decode()
        status_type = importlib.import_module(
            "aorta.services.common.ServiceStatus"
        ).ServiceStatus
        message = (response.Message() or b"").decode("utf-8", errors="replace")
        if response.Status() != status_type.SUCCESS:
            raise RuntimeError(f"{endpoint}: status={response.Status()}: {message}")
        return response

    def external_control(self):
        response = self._call(
            "external_control_state",
            "QueryStateRequest",
            "QueryStateResponse",
            lambda schema, builder, header: _table(schema, builder, AortaHeader=header),
        )
        return response.Enable()

    def _set_bool(self, endpoint, enabled):
        self._call(
            endpoint,
            "SetBoolRequest",
            "SetBoolResponse",
            lambda schema, builder, header: _table(
                schema, builder, AortaHeader=header, Enable=enabled
            ),
        )

    def set_external_control(self, enabled):
        self._set_bool("external_control", enabled)

    def _show(self, operation):
        previous = self.external_control()
        try:
            self.set_external_control(True)
            operation()
        except BaseException:
            try:
                self.set_external_control(previous)
            except Exception as error:
                print(f"Could not restore external control: {error}", file=sys.stderr)
            raise

    def show_text(self, items, *, section="CUSTOM"):
        """Publish resident ASCII key/value fields on the system info page."""
        if not items or not section.strip() or section.lower() in {"x5", "s100"}:
            raise ValueError("provide fields and a non-reserved section name")
        for value in [section, *items.keys(), *items.values()]:
            if not value.isascii() or any(
                ord(char) < 32 or ord(char) > 126 for char in value
            ):
                raise ValueError(
                    "key/value text supports printable ASCII; use an image for Chinese"
                )
        schema = importlib.import_module("aorta.topic.peripheral.DisplayInfo")
        metadata = importlib.import_module("display_info_schema_meta")
        with self.node.create_publisher_typed(metadata, "/display_info") as publisher:
            if not publisher.wait_for_matching(self.timeout_ms):
                raise RuntimeError("/display_info subscriber unavailable")

            def fill(builder, header):
                title = builder.CreateString(section)
                keys = _string_vector(schema, "Key", builder, items.keys())
                values = _string_vector(schema, "Info", builder, items.values())
                return _table(
                    schema,
                    builder,
                    AortaHeader=header,
                    Type=title,
                    Key=keys,
                    Info=values,
                    Resident=True,
                )

            def display():
                self._set_bool("enable_show_info", True)
                # Matching precedes publishing; repeat briefly before closing
                # this one-shot publisher. resident keeps the fields afterward.
                for _ in range(3):
                    publisher.publish_typed(fill)
                    time.sleep(0.2)

            self._show(display)

    @staticmethod
    def _encode_image(image, *, landscape=False):
        from PIL import Image, ImageOps

        image = ImageOps.exif_transpose(image).convert("RGBA")
        background = Image.new("RGBA", image.size, "black")
        image = Image.alpha_composite(background, image).convert("RGB")
        size = (854, 480) if landscape else (480, 854)
        image = ImageOps.pad(image, size, color="black")
        if landscape:
            image = image.transpose(Image.Transpose.ROTATE_90)
        output = io.BytesIO()
        image.save(output, format="JPEG", quality=85, optimize=True)
        return output.getvalue()

    @staticmethod
    def _image_vector(schema, builder, frames):
        frame_schema = _schema("ShowImg")
        offsets = []
        for jpeg in frames:
            data = builder.CreateByteVector(jpeg)
            encoding = builder.CreateString("jpeg")
            offsets.append(
                _table(
                    frame_schema,
                    builder,
                    Width=480,
                    Height=854,
                    Data=data,
                    Encoding=encoding,
                )
            )
        name = schema.__name__.rsplit(".", 1)[-1]
        getattr(schema, name + "StartImagesVector")(builder, len(offsets))
        for offset in reversed(offsets):
            builder.PrependUOffsetTRelative(offset)
        return builder.EndVector()

    def _display_inline(self, frames, durations, *, loop=True):
        if not frames or len(frames) != len(durations) or len(frames) > 4096:
            raise ValueError("provide 1..4096 frames and one duration per frame")
        if any(not frame or len(frame) > 8 * 1024 * 1024 for frame in frames):
            raise ValueError("each JPEG must contain 1 byte to 8 MiB")
        if sum(map(len, frames)) > 32 * 1024 * 1024:
            raise ValueError("inline sequence exceeds the 32 MiB JPEG limit")
        if any(not 1 <= duration <= 0xFFFFFFFF for duration in durations):
            raise ValueError("frame durations must be positive uint32 milliseconds")

        def fill(schema, builder, header):
            images = self._image_vector(schema, builder, frames)
            schema.DisplayImgsRequestStartDurationMsVector(builder, len(durations))
            for duration in reversed(durations):
                builder.PrependUint32(duration)
            times = builder.EndVector()
            return _table(
                schema,
                builder,
                AortaHeader=header,
                Images=images,
                DurationMs=times,
                Loop=loop,
            )

        self._show(
            lambda: self._call(
                "display_imgs", "DisplayImgsRequest", "DisplayImgsResponse", fill
            )
        )

    def show_image(self, path, *, name=None, landscape=False):
        """Show an image without registration; name opts into persistent storage."""
        from PIL import Image

        with Image.open(path) as original:
            jpeg = self._encode_image(original, landscape=landscape)
        if name is None:
            self._display_inline([jpeg], [1000])
            return
        if len(jpeg) > 4 * 1024 * 1024:
            raise ValueError("JPEG exceeds the 4 MiB registration request limit")

        def register(schema, builder, header):
            images = self._image_vector(schema, builder, [jpeg])
            title = builder.CreateString(name)
            return _table(
                schema,
                builder,
                AortaHeader=header,
                CustomImagesName=title,
                Images=images,
                Fps=1,
            )

        response = self._call(
            "add_images", "AddImagesRequest", "AddImagesResponse", register
        )
        if not response.Complete():
            raise RuntimeError("image registration incomplete")

        def play(schema, builder, header):
            title = builder.CreateString(name)
            return _table(
                schema,
                builder,
                AortaHeader=header,
                TargetState=1,
                CustomImagesName=title,
                DurationMs=0,
            )

        self._show(
            lambda: self._call(
                "play_emotion", "PlayEmotionRequest", "PlayEmotionResponse", play
            )
        )
        return name

    def show_animation(self, path, *, landscape=False, fps=None, loop=True):
        """Send GIF/animated WebP frames once; the device runs the loop."""
        from PIL import Image

        if fps is not None and not 1 <= fps <= 60:
            raise ValueError("fps must be between 1 and 60")
        frames, durations = [], []
        total_bytes = 0
        with Image.open(path) as animation:
            count = getattr(animation, "n_frames", 1)
            if not 2 <= count <= 4096:
                raise ValueError("animation must contain 2..4096 frames")
            for index in range(count):
                animation.seek(index)
                # Pillow composites GIF/WebP disposal and partial-frame updates.
                jpeg = self._encode_image(animation, landscape=landscape)
                total_bytes += len(jpeg)
                if total_bytes > 32 * 1024 * 1024:
                    raise ValueError("animation exceeds the 32 MiB JPEG limit")
                frames.append(jpeg)
                duration = int(animation.info.get("duration", 100) or 100)
                durations.append(
                    round(1000 / fps)
                    if fps is not None
                    else (duration if duration > 0 else 100)
                )
        self._display_inline(frames, durations, loop=loop)
        return len(frames)

    def show_message(self, message, *, font, font_size=48, landscape=False):
        """Render full-screen Unicode text into an image, without registration."""
        from PIL import Image, ImageDraw, ImageFont

        if not message or font_size <= 0:
            raise ValueError("provide nonempty text and a positive font size")
        canvas = Image.new("RGB", (854, 480) if landscape else (480, 854), "black")
        draw = ImageDraw.Draw(canvas)
        face = ImageFont.truetype(str(font), font_size)
        box = draw.multiline_textbbox((0, 0), message, font=face, spacing=8)
        width, height = box[2] - box[0], box[3] - box[1]
        if width > canvas.width - 64 or height > canvas.height - 64:
            raise ValueError(
                "text exceeds the safe area; use a smaller font or line breaks"
            )
        position = (
            (canvas.width - width) / 2 - box[0],
            (canvas.height - height) / 2 - box[1],
        )
        draw.multiline_text(
            position, message, font=face, fill="white", spacing=8, align="center"
        )
        self._display_inline([self._encode_image(canvas, landscape=landscape)], [1000])

    def restore_rcp(self):
        """End the external picture/info page, restore default, then release."""
        try:
            self._call(
                "play_emotion",
                "PlayEmotionRequest",
                "PlayEmotionResponse",
                lambda schema, builder, header: _table(
                    schema, builder, AortaHeader=header, TargetState=0
                ),
            )
        finally:
            self.set_external_control(False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", help="authorized Zenoh session config JSON5")
    parser.add_argument("--group", default="default")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("takeover", help="enable external display control")
    image = commands.add_parser("image", help="show an image without registration")
    image.add_argument("path", type=Path)
    image.add_argument(
        "--name", help="opt into registration under this persistent name"
    )
    image.add_argument(
        "--landscape", action="store_true", help="854x480 landscape layout"
    )
    animation = commands.add_parser(
        "animation",
        aliases=["animate"],
        help="play a GIF/animated WebP without registration",
    )
    animation.add_argument("path", type=Path)
    animation.add_argument("--landscape", action="store_true")
    animation.add_argument(
        "--fps", type=int, help="override frame delays with 1..60 FPS"
    )
    animation.add_argument(
        "--once", action="store_true", help="play once, then restore default"
    )
    text = commands.add_parser("text", help="show key/value fields or rendered text")
    content = text.add_mutually_exclusive_group(required=True)
    content.add_argument("--item", action="append", metavar="KEY=VALUE")
    content.add_argument(
        "--message", help="full-screen text; supports Unicode with a suitable font"
    )
    text.add_argument("--section", default="CUSTOM")
    text.add_argument("--font", type=Path, help="TTF/OTF/TTC font file for --message")
    text.add_argument("--font-size", type=int, default=48)
    text.add_argument("--landscape", action="store_true", help="layout for --message")
    commands.add_parser("state", help="query external control")
    commands.add_parser(
        "rcp", aliases=["release"], help="end external display and return to RCP"
    )
    args = parser.parse_args(argv)
    items = {}
    if args.command == "text":
        if args.message is not None and args.font is None:
            parser.error("--message requires --font with the needed glyphs")
        for item in args.item or []:
            key, separator, value = item.partition("=")
            if not separator or not key or key in items:
                parser.error("--item requires a unique, nonempty KEY=VALUE")
            items[key] = value
    try:
        with Screen(config=args.config, group=args.group) as screen:
            if args.command == "takeover":
                screen.set_external_control(True)
                print("External display control enabled.")
            elif args.command == "image":
                name = screen.show_image(
                    args.path, name=args.name, landscape=args.landscape
                )
                print(
                    f"Displaying {name or 'inline image'}; external control remains enabled."
                )
            elif args.command in {"animation", "animate"}:
                count = screen.show_animation(
                    args.path,
                    landscape=args.landscape,
                    fps=args.fps,
                    loop=not args.once,
                )
                print(
                    f"Playing {count} frames, loop={not args.once}; external control remains enabled."
                )
            elif args.command == "text":
                if args.message is not None:
                    screen.show_message(
                        args.message,
                        font=args.font,
                        font_size=args.font_size,
                        landscape=args.landscape,
                    )
                else:
                    screen.show_text(items, section=args.section)
                print("Text sent; external control remains enabled.")
            elif args.command in {"rcp", "release"}:
                screen.restore_rcp()
                print("External display ended; RCP control restored.")
            else:
                print(f"external_control={screen.external_control()}")
    except ImportError as error:
        parser.exit(
            1, f"Missing Python dependency/binding: {error}. See scripts/README.md.\n"
        )
    except (OSError, RuntimeError, ValueError) as error:
        parser.exit(1, f"Display request failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
