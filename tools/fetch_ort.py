"""Download the current host's official ORT archive and verify its pinned SHA."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import urllib.request

MANIFEST = Path(__file__).resolve().parents[1] / "demo_onnx/ort-archives.json"


def fetch(output: Path) -> Path:
    archive = json.loads(MANIFEST.read_text())[platform.machine()]
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
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
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
