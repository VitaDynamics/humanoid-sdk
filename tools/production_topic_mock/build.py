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
PROTOCOL_CHECKS = r'''
#include "protocol.h"
#include <cassert>
#include <limits>
using namespace production_mock;
int main() {
  Protocol p(10,100,50);
  Request r{2,1,"req","client","","EXTERNAL"};
  auto prep=p.RequestControl(r,1000); assert(prep.accepted && !prep.lease_id.empty());
  assert(p.status().phase==2);
  auto duplicate=p.RequestControl(r,1001); assert(duplicate.lease_id==prep.lease_id);
  auto other=r; other.client_id="other"; assert(p.RequestControl(other,1002).error==1008);
  other=r; other.request_id="other"; assert(p.RequestControl(other,1002).error==1004);
  r.operation=2; r.lease_id="wrong"; assert(p.RequestControl(r,1003).error==1006);
  r.lease_id=prep.lease_id; assert(p.RequestControl(r,1004).accepted);
  auto generation=p.status().generation; assert(generation && p.status().phase==4);
  assert(p.RequestControl(r,1005).accepted && p.status().generation==generation);
  Command cmd; cmd.session=generation; cmd.sequence=1; cmd.input_state_real_us=1;
  cmd.kp.fill(1); cmd.kd.fill(1);
  assert(p.Accept(cmd,1010)); assert(!p.Accept(cmd,1011));
  assert(p.Tick(1012,2000)); assert(p.output().sequence==1 && p.output().first_output_us==2000);
  assert(!p.Tick(1013,2001));
  cmd.sequence=2; cmd.q[0]=std::numeric_limits<float>::quiet_NaN(); assert(!p.Accept(cmd,1020));
  cmd.q[0]=0; cmd.session=0; assert(!p.Accept(cmd,1021)); cmd.session=generation;
  assert(p.Accept(cmd,1022)); cmd.sequence=3; assert(p.Accept(cmd,1023));
  assert(p.overwritten()==1); assert(p.Tick(1024,2002)); assert(p.output().sequence==3);
  p.Watchdog(1123); assert(p.status().phase==5);
  cmd.sequence=4; assert(p.Accept(cmd,1124) && p.status().phase==4);
  r.operation=3; assert(p.RequestControl(r,1125).accepted); assert(p.status().phase==0);
  assert(!p.Tick(1126,2003)); assert(!p.Accept(cmd,1127));
  assert(p.RequestControl(r,1128).accepted);
  r.operation=1; r.lease_id=""; r.version=1; assert(p.RequestControl(r,1130).error==1001);
  r.version=2; r.action="OTHER"; assert(p.RequestControl(r,1131).error==1003);
  r.action="EXTERNAL"; assert(p.RequestControl(r,1140).accepted);
  p.Watchdog(1190); assert(p.status().phase==0);
  Protocol rejected(20,100,50,1); assert(!rejected.RequestControl(r,1200).accepted);
  Protocol exhausted(std::numeric_limits<uint64_t>::max(),100,50);
  assert(exhausted.RequestControl(r,1200).error==1009);
}
'''


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
    schema_hashes = {}
    for name in SCHEMAS:
        meta = importlib.import_module(name + "_schema_meta")
        data = meta.SCHEMA_BFBS
        sha = hashlib.sha256(data).hexdigest()
        if sha != manifest["schema_sha256"][name] or sha != meta.SCHEMA_HASH_HEX:
            raise ValueError("BFBS mismatch: " + name)
        bfbs = generated / (name + ".bfbs")
        bfbs.write_bytes(data)
        (generated / (name + ".sha256")).write_text(sha)
        schema_hashes[name] = sha
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
    checks = directory / "protocol-checks.cpp"
    checks.write_text(PROTOCOL_CHECKS)
    run([*flags, checks, directory / "protocol.o", "-o", directory / "protocol-checks"])
    if args.run_protocol_checks:
        if args.target_arch != platform.machine():
            raise ValueError("never execute cross-built ARM checks on the build host")
        run([directory / "protocol-checks"])
    result = {"mode": "production-topics", "target_arch": args.target_arch, "versions": versions,
              "schemas": schema_hashes, "aorta_abi": manifest["aorta_abi_version"],
              "flatbuffers_archive_sha256": original.FB_SHA, "compiler": subprocess.check_output([args.cxx, "--version"], text=True).splitlines()[0],
              "source_sha256": {str(p.relative_to(ROOT)): digest(p) for p in sorted(HERE.iterdir()) if p.is_file()},
              "original_benchmark_builder_sha256": digest(ROOT / "tests/latency/build_mock.py"),
              "binary_sha256": digest(binary), "core_path": str(core), "core_sha256": digest(core),
              "protocol_checks_sha256": digest(directory / "protocol-checks"),
              "protocol_checks_executed": args.run_protocol_checks,
              "generated_sha256": {str(p.relative_to(directory)): digest(p) for p in sorted(generated.iterdir()) if p.is_file()},
              "aorta_ipc_or_device_validation": False}
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
