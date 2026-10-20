"""Download the pinned runtime wheels from their official GitHub Releases."""
import argparse
import hashlib
import json
import os
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
    # Validate both sources before downloading anything. Never fall back to latest.
    schema_tag = required_text(manifest, "dependency_release_tag")
    schema_repo = required_text(manifest, "dependency_release_repository")
    runtime_tag = required_text(manifest, "runtime_release_tag")
    runtime_repo = required_text(manifest, "runtime_release_repository")
    names = [name for name in manifest["wheel_sha256"]
             if name.endswith((f"-linux_{arch}.whl", "-any.whl"))]
    if sorted(name.split("-")[0] for name in names) != [
        "aorta_msgs", "aorta_sdk", "flatbuffers", "vbot_humanoid_msgs"
    ]:
        raise ValueError("incomplete pinned wheelhouse")
    schema_token = os.environ.get("SCHEMA_RELEASE_TOKEN", "")
    if not schema_token.strip():
        raise ValueError("SCHEMA_RELEASE_TOKEN is required to read the private schema release")
    output.mkdir(parents=True, exist_ok=True)
    for name in names:
        schema = name.startswith("vbot_humanoid_msgs-")
        env = os.environ.copy()
        if schema:
            env["GH_TOKEN"] = schema_token
        subprocess.run([
            "gh", "release", "download", schema_tag if schema else runtime_tag,
            "--repo", schema_repo if schema else runtime_repo,
            "--dir", str(output), "--pattern", name,
        ], check=True, env=env)
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
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
