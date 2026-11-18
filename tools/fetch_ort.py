"""Download the pinned ORT mirror archive and verify its original SHA."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import tempfile

MANIFEST = Path(__file__).resolve().parents[1] / "demo_onnx/ort-archives.json"


def fetch(output: Path) -> Path:
    manifest = json.loads(MANIFEST.read_text())
    archive = manifest[platform.machine()]
    release = manifest["mirror_release"]
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    # Failed downloads stay here for diagnosis without occupying the final name.
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}-", dir=output.parent))
    pending = staging / output.name
    subprocess.run([
        "gh", "release", "download", release["tag"], "--repo", release["repository"],
        "--pattern", archive["directory"] + ".tgz", "--output", str(pending),
    ], check=True)
    if hashlib.sha256(pending.read_bytes()).hexdigest() != archive["sha256"]:
        raise ValueError(f"ORT SHA-256 mismatch; downloaded bytes retained at {pending}")
    # Same-filesystem link is atomic and refuses an output created during download.
    os.link(pending, output)
    pending.unlink()
    staging.rmdir()
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
