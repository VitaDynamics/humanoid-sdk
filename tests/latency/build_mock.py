"""Build a standalone C++ endpoint against the pinned wheel's real Aorta core."""
import argparse
import hashlib
import importlib
import importlib.metadata
import json
from pathlib import Path
import platform
import shutil
import subprocess
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
FB_URL = "https://codeload.github.com/google/flatbuffers/tar.gz/refs/tags/v25.9.23"
FB_SHA = "9102253214dea6ae10c2ac966ea1ed2155d22202390b532d1dea64935c518ada"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(*args):
    print("+", *map(str, args), flush=True)
    subprocess.run(list(map(str, args)), check=True)


def extract_source(archive, directory):
    """Restore compiler inputs from the pinned archive, never trust a reused tree."""
    if digest(archive) != FB_SHA:
        raise RuntimeError("FlatBuffers archive checksum mismatch")
    source = directory / "flatbuffers-25.9.23"
    if source.is_symlink():
        raise RuntimeError("FlatBuffers source directory must not be a symlink")
    with tarfile.open(archive) as tar:
        # Java/TS links are irrelevant to flatc. Only extract the expected subtree.
        members = [m for m in tar.getmembers() if m.isfile() or m.isdir()]
        for member in members:
            path = Path(member.name)
            if not path.parts or path.is_absolute() or ".." in path.parts or path.parts[0] != source.name:
                raise RuntimeError("unsafe archive member")
        if source.exists():
            shutil.rmtree(source)  # only this task's extracted dependency, not its build cache
        tar.extractall(directory, members=members)
    return source


def build(directory):
    directory = directory.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((ROOT / "compatibility.json").read_text())
    if platform.machine() != "x86_64":
        raise RuntimeError("this CI benchmark is Linux x86_64 only")
    versions = {p: importlib.metadata.version(p) for p in
                ("aorta-sdk", "aorta-msgs", "flatbuffers")}
    for name, version in versions.items():
        expected = manifest["flatbuffers_version" if name == "flatbuffers" else "aorta_version"]
        if version != expected:
            raise RuntimeError(f"{name}: {version} != pinned {expected}")
    archive = directory / "flatbuffers.tar.gz"
    if not archive.exists():
        with urllib.request.urlopen(FB_URL, timeout=120) as response:
            archive.write_bytes(response.read())
    source = extract_source(archive, directory)
    cmake = directory / "flatbuffers-build"
    run("cmake", "-S", source, "-B", cmake, "-DCMAKE_BUILD_TYPE=Release",
        "-DFLATBUFFERS_BUILD_TESTS=OFF", "-DFLATBUFFERS_BUILD_FLATLIB=OFF")
    run("cmake", "--build", cmake, "--target", "flatc", "--parallel", "2")
    generated = directory / "generated"
    generated.mkdir(exist_ok=True)
    schemas = {}
    for name in ("humanoid_low_state", "low_cmd"):
        meta = importlib.import_module(name + "_schema_meta")
        bfbs = generated / (name + ".bfbs")
        bfbs.write_bytes(meta.SCHEMA_BFBS)
        schemas[name] = digest(bfbs)
        run(cmake / "flatc", "--cpp", "--gen-object-api", "--gen-all",
            "-o", generated, bfbs)
    import aorta
    core = Path(aorta.__file__).parent / "libaorta_core.so"
    run("g++", "-std=c++17", "-O2", "-pthread", "-Wall", "-Wextra", "-Werror",
        "-I" + str(source / "include"), "-I" + str(generated),
        ROOT / "tests/latency/mock.cpp", ROOT / "tests/latency/state.cpp", core,
        "-Wl,-rpath," + str(core.parent), "-o", directory / "mock")
    result = {"versions": versions, "aorta_revision": manifest["aorta_revision"],
              "core_sha256": digest(core), "flatbuffers_source_sha256": FB_SHA,
              "bfbs_sha256": schemas, "mock_sha256": digest(directory / "mock"),
              "compiler": subprocess.check_output(["g++", "--version"], text=True).splitlines()[0]}
    (directory / "build.json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", type=Path, required=True)
    build(parser.parse_args().build_dir)
