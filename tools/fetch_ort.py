"""Download the current host's official ORT archive and verify its pinned SHA."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import urllib.request

MANIFEST = Path(__file__).resolve().parents[1] / "demo_onnx/ort-archives.json"


def fetch(output: Path) -> Path:
    manifest = json.loads(MANIFEST.read_text())
    archive = manifest[platform.machine()]
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    release = manifest.get("mirror_release")
    if release is not None:
        # Explicit pinned source, not a fallback after an upstream failure.
        subprocess.run([
            "gh", "release", "download", release["tag"], "--repo", release["repository"],
            "--pattern", archive["directory"] + ".tgz", "--output", str(output),
        ], check=True)
    else:
        with urllib.request.urlopen(archive["url"]) as response:
            output.write_bytes(response.read())
    if hashlib.sha256(output.read_bytes()).hexdigest() != archive["sha256"]:
        raise ValueError(f"ORT SHA-256 mismatch; downloaded bytes retained at {output}")
    return output.resolve()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        print(fetch(args.output))
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
