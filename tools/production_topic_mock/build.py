"""Build only on explicit request; use pkg for shared-workspace compilation."""
import argparse
import hashlib
import importlib
import importlib.metadata
import importlib.util
import json
from pathlib import Path
import platform
import subprocess

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
SCHEMAS = ("body_state", "body_cmd", "get_robot_info", "control_request", "control_ack", "control_status")


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def run(argv):
    print("+", *map(str, argv), flush=True)
    subprocess.run(list(map(str, argv)), check=True)


def build(args):
    manifest = json.loads((ROOT / "compatibility.json").read_text())
    versions = {name: importlib.metadata.version(name) for name in
                ("aorta-sdk", "aorta-msgs", "vbot-humanoid-msgs", "flatbuffers")}
    for name, version in versions.items():
        key = {"vbot-humanoid-msgs": "humanoid_msgs_version", "flatbuffers": "flatbuffers_version"}.get(name, "aorta_version")
        if version != manifest[key]:
            raise ValueError(f"{name} {version} != pinned {manifest[key]}")
    directory = args.build_dir.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    # Reuse the unchanged benchmark's archive verifier, not its x86-only builder.
    spec = importlib.util.spec_from_file_location("original_latency_build", ROOT / "tests/latency/build_mock.py")
    original = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(original)
    archive = args.flatbuffers_archive.resolve()
    if digest(archive) != original.FB_SHA:
        raise ValueError("FlatBuffers archive differs from pinned build input")
    source = original.extract_source(archive, directory)
    cmake = directory / "flatbuffers-build"
    run(["cmake", "-S", source, "-B", cmake, "-DCMAKE_BUILD_TYPE=Release",
         "-DFLATBUFFERS_BUILD_TESTS=OFF", "-DFLATBUFFERS_BUILD_FLATLIB=OFF"])
    run(["cmake", "--build", cmake, "--target", "flatc", "--parallel", "2"])
    generated = directory / "generated"
    generated.mkdir()
    for name in SCHEMAS:
        meta = importlib.import_module(name + "_schema_meta")
        data = meta.SCHEMA_BFBS
        sha = hashlib.sha256(data).hexdigest()
        if sha != manifest["schema_sha256"][name] or sha != meta.SCHEMA_HASH_HEX:
            raise ValueError("BFBS mismatch: " + name)
        bfbs = generated / (name + ".bfbs")
        bfbs.write_bytes(data)
        if name == "get_robot_info":
            (generated / (name + ".sha256")).write_text(sha)
        run([cmake / "flatc", "--cpp", "--gen-object-api", "--gen-all", "--gen-mutable", "-o", generated, bfbs])
    if args.aorta_core:
        core = args.aorta_core.resolve()
    else:
        import aorta
        core = Path(aorta.__file__).parent / "libaorta_core.so"
    machine_text = subprocess.check_output(["readelf", "-h", core], text=True)
    expected_machine = {"x86_64": "Advanced Micro Devices X86-64", "aarch64": "AArch64"}[args.target_arch]
    if expected_machine not in machine_text:
        raise ValueError("Aorta core is for the wrong target architecture")
    core_link = directory / f"libaorta_core.so.{manifest['aorta_abi_version']}"
    core_link.symlink_to(core)
    flags = [args.cxx, "-std=c++17", "-O2", "-g", "-pthread", "-Wall", "-Wextra", "-Werror",
             "-DAORTA_EXPECTED_ABI=" + str(manifest["aorta_abi_version"]),
             "-I" + str(HERE), "-I" + str(ROOT / "tests/latency"),
             "-I" + str(source / "include"), "-I" + str(generated)]
    objects = []
    for name, path, macro in [
        ("main", HERE / "mock.cpp", None), ("state", HERE / "state.cpp", None),
        ("protocol", HERE / "protocol.cpp", None),
        *[(key, HERE / "protocol.cpp", "PRODUCTION_MOCK_" + key.upper() + "_CODEC")
          for key in ("request", "ack", "status", "info")],
    ]:
        obj = directory / (name + ".o")
        run([*flags, *(["-D" + macro] if macro else []), "-c", path, "-o", obj])
        objects.append(obj)
    binary = directory / "production-topic-mock"
    run([args.cxx, "-pthread", *objects, core, "-Wl,-rpath," + str(directory), "-o", binary])
    checks = HERE / "protocol_checks.cpp"
    run([*flags, checks, directory / "protocol.o", directory / "request.o",
         "-o", directory / "protocol-checks"])
    if args.run_protocol_checks:
        if args.target_arch != platform.machine():
            raise ValueError("never execute cross-built ARM checks on the build host")
        run([directory / "protocol-checks"])
    result = {"mode": "production-topics", "target_arch": args.target_arch,
              "core_path": str(core), "aorta_abi": manifest["aorta_abi_version"]}
    (directory / "build.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--flatbuffers-archive", type=Path, required=True)
    parser.add_argument("--target-arch", choices=("x86_64", "aarch64"), default=platform.machine())
    parser.add_argument("--aorta-core", type=Path, help="Architecture-matching extracted official wheel core for cross-builds")
    parser.add_argument("--cxx", default="g++")
    parser.add_argument("--run-protocol-checks", action="store_true", help="pkg: execute same-architecture protocol checks after building")
    build(parser.parse_args())
