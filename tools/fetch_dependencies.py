"""Verify pinned native inputs from an official Release or explicit local files."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import shutil

MANIFEST = Path(__file__).resolve().parents[1] / "compatibility.json"


def required_text(manifest: dict, key: str) -> str:
    value = manifest.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(
            f"{key} must be a nonempty string; pinned release validation cannot be skipped. "
            "See README#dependency-release-convention for the pending Release; "
            "verified local inputs require explicit --source-dir and --native-header."
        )
    return value


def fetch(output: Path, arch: str, *, source: Path | None = None,
          native_header: Path | None = None) -> Path:
    manifest = json.loads(MANIFEST.read_text())
    # A local input is explicit, never a fallback after a failed Release download.
    if source is None:
        release_tag = required_text(manifest, "dependency_release_tag")
        release_repo = required_text(manifest, "dependency_release_repository")
        if native_header is not None:
            raise ValueError("--native-header requires --source-dir")
    names = [name for name in manifest["wheel_sha256"]
             if name.endswith((f"-linux_{arch}.whl", "-any.whl"))]
    if sorted(name.split("-")[0] for name in names) != [
        "aorta_msgs", "aorta_sdk", "flatbuffers", "vbot_humanoid_msgs"
    ]:
        raise ValueError("incomplete pinned wheelhouse")
    expected = {name: manifest["wheel_sha256"][name] for name in names}
    expected["aorta.h"] = required_text(manifest, "aorta_native_header_sha256")
    if source is not None:
        inputs = {name: source / name for name in names}
        inputs["aorta.h"] = native_header if native_header is not None else source / "aorta.h"
        # Verify the whole local set before creating or copying an output.
        for name, path in inputs.items():
            if hashlib.sha256(path.read_bytes()).hexdigest() != expected[name]:
                raise ValueError(f"SHA-256 mismatch: {name}; source retained at {path}")
    output.mkdir(parents=True, exist_ok=True)
    for name, digest in expected.items():
        if source is not None:
            with inputs[name].open("rb") as incoming, (output / name).open("xb") as destination:
                shutil.copyfileobj(incoming, destination)
        else:
            try:
                subprocess.run([
                    "gh", "release", "download", release_tag,
                    "--repo", release_repo,
                    "--dir", str(output), "--pattern", name,
                ], check=True)
            except subprocess.CalledProcessError as error:
                raise RuntimeError(
                    f"Cannot download pinned input {name} from {release_repo}@{release_tag}; "
                    "if the asset is missing, mirror the official release as documented in "
                    "https://github.com/VitaDynamics/humanoid-sdk#dependency-release-convention"
                ) from error
        if hashlib.sha256((output / name).read_bytes()).hexdigest() != digest:
            raise ValueError(f"SHA-256 mismatch: {name}; downloaded bytes retained at {output}")
    return output.resolve()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--arch", choices=("x86_64", "aarch64"), default="x86_64")
    parser.add_argument("--source-dir", type=Path,
                        help="Explicit directory of already received pinned wheels; no network fallback")
    parser.add_argument("--native-header", type=Path,
                        help="Matching generated aorta.h for --source-dir; defaults to SOURCE_DIR/aorta.h")
    args = parser.parse_args()
    try:
        print(fetch(args.output, args.arch, source=args.source_dir,
                    native_header=args.native_header))
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
