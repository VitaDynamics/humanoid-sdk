"""Assemble a self-contained offline bundle; never download or publish it."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def package(wheelhouses: list[Path], output: Path) -> None:
    compatibility = json.loads((ROOT / "compatibility.json").read_text())
    wheels = []
    for name, expected in compatibility["wheel_sha256"].items():
        matches = [directory / name for directory in wheelhouses
                   if (directory / name).is_file()]
        if not matches:
            raise ValueError(f"missing wheel: {name}")
        if any(digest(path) != expected for path in matches):
            raise ValueError(f"SHA-256 mismatch: {name}")
        wheels.append(matches[0])
    sdk_wheel = ROOT / "dist" / (
        f"locomotion_aorta-{compatibility['sdk_version']}-py3-none-any.whl"
    )
    if not sdk_wheel.is_file():
        raise ValueError("missing SDK wheel; run make build first")
    wheels.append(sdk_wheel)
    link = ROOT / "CLAUDE.md"
    if not link.is_symlink() or str(link.readlink()) != "AGENTS.md":
        raise ValueError("CLAUDE.md must be a relative symlink to AGENTS.md")
    # Refuse overwrites, including partially assembled bundles from a prior run.
    if output.exists():
        raise ValueError(f"output already exists: {output}")
    revision = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip()
    dirty = bool(subprocess.check_output(
        ["git", "status", "--porcelain"], cwd=ROOT, text=True
    ).strip())
    output.mkdir(parents=True)
    (output / "wheelhouse").mkdir()
    for wheel in wheels:
        shutil.copyfile(wheel, output / "wheelhouse" / wheel.name)
    for name in ("README.md", "AGENTS.md", "compatibility.json"):
        shutil.copyfile(ROOT / name, output / name)
    (output / "CLAUDE.md").symlink_to("AGENTS.md")
    shutil.copytree(ROOT / "examples", output / "examples",
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    shutil.copytree(ROOT / "python/locomotion_aorta", output / "source/locomotion_aorta",
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    manifest = dict(compatibility, sdk_repository_head=revision, source_dirty=dirty)
    manifest["files"] = {
        path.relative_to(output).as_posix(): digest(path)
        for path in sorted(output.rglob("*")) if path.is_file()
    }
    manifest["symlinks"] = {"CLAUDE.md": "AGENTS.md"}
    (output / "MANIFEST.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n"
    )
    print(json.dumps({"output": str(output), "source_dirty": dirty,
                      "wheels": len(wheels)}))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wheelhouse", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        package(args.wheelhouse, args.output)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
