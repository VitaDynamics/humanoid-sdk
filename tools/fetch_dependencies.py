"""Download the pinned runtime wheels from their official GitHub Releases."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

MANIFEST = Path(__file__).resolve().parents[1] / "compatibility.json"


def required_text(manifest: dict, key: str) -> str:
    value = manifest.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be a nonempty string; pinned release validation cannot be skipped")
    return value


def fetch(output: Path, arch: str) -> Path:
    manifest = json.loads(MANIFEST.read_text())
    # Validate the fixed release before downloading anything. Never fall back to latest.
    release_tag = required_text(manifest, "dependency_release_tag")
    release_repo = required_text(manifest, "dependency_release_repository")
    names = [name for name in manifest["wheel_sha256"]
             if name.endswith((f"-linux_{arch}.whl", "-any.whl"))]
    if sorted(name.split("-")[0] for name in names) != [
        "aorta_msgs", "aorta_sdk", "flatbuffers", "vbot_humanoid_msgs"
    ]:
        raise ValueError("incomplete pinned wheelhouse")
    output.mkdir(parents=True, exist_ok=True)
    for name in names:
        try:
            subprocess.run([
                "gh", "release", "download", release_tag,
                "--repo", release_repo,
                "--dir", str(output), "--pattern", name,
            ], check=True)
        except subprocess.CalledProcessError as error:
            raise RuntimeError(
                f"Cannot download pinned wheel {name} from {release_repo}@{release_tag}; "
                "if the asset is missing, mirror the official release as documented in "
                "https://github.com/VitaDynamics/humanoid-sdk#dependency-release-convention"
            ) from error
        if hashlib.sha256((output / name).read_bytes()).hexdigest() != manifest["wheel_sha256"][name]:
            raise ValueError(f"SHA-256 mismatch: {name}; downloaded bytes retained at {output}")
    return output.resolve()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--arch", choices=("x86_64", "aarch64"), default="x86_64")
    args = parser.parse_args()
    try:
        print(fetch(args.output, args.arch))
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
