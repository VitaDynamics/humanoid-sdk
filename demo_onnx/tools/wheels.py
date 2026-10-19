"""Audit/repair and check the owner-approved cp310 two-architecture wheel set."""
import argparse
import email
import hashlib
import json
import logging
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import zipfile

from packaging.requirements import Requirement

ARCHES = {"x86_64": 62, "aarch64": 183}


def inspect(path):
    with zipfile.ZipFile(path) as wheel:
        info = next(n.rsplit("/", 1)[0] for n in wheel.namelist() if n.endswith(".dist-info/WHEEL"))
        tags = email.message_from_bytes(wheel.read(info + "/WHEEL")).get_all("Tag", [])
        arch = next((a for a in ARCHES if tags == ["cp310-cp310-manylinux_2_35_" + a]), None)
        if arch is None:
            raise ValueError("expected one audited cp310/manylinux_2_35 x86_64 or aarch64 tag: " + str(tags))
        if not path.name.endswith("-cp310-cp310-manylinux_2_35_" + arch + ".whl"):
            raise ValueError("filename and WHEEL tags differ")
        metadata = email.message_from_bytes(wheel.read(info + "/METADATA"))
        if metadata["Name"] != "locomotion-aorta" or metadata["Version"] != "0.2.0":
            raise ValueError("unexpected distribution/version")
        core = {"aorta-sdk": "==2026.10.12", "aorta-msgs": "==2026.10.12",
                "vbot-humanoid-msgs": "==0.1.2", "flatbuffers": "==25.9.23"}
        requirements = [Requirement(value) for value in metadata.get_all("Requires-Dist", [])]
        actual = {req.name: str(req.specifier) for req in requirements if req.marker is None}
        if metadata["Requires-Python"] != "==3.10.*" or actual != core:
            raise ValueError("unexpected SDK Python/core dependencies")
        licenses = metadata.get_all("License-File", [])
        if len(licenses) != 3 or any(info + "/licenses/" + n not in wheel.namelist() for n in licenses):
            raise ValueError("three declared license files must exist in dist-info")
        module = next(n for n in wheel.namelist()
                      if n.startswith("locomotion_aorta/_onnx/_native.cpython-310-") and n.endswith(".so"))
        for name in ["locomotion_aorta/__init__.py", "locomotion_aorta/_onnx/__init__.py",
                     "locomotion_aorta/_onnx/.libs/libonnxruntime.so.1",
                     "locomotion_aorta/_onnx/.libs/libonnxruntime_providers_shared.so"]:
            if name not in wheel.namelist():
                raise ValueError("SDK wheel is missing bundled payload: " + name)
        raw = wheel.read(module)
        if raw[:6] != b"\x7fELF\x02\x01" or struct.unpack_from("<H", raw, 18)[0] != ARCHES[arch]:
            raise ValueError("ELF machine does not match the wheel tag")
        if not module.endswith(".cpython-310-" + arch + "-linux-gnu.so"):
            raise ValueError("extension SOABI does not match the target")
        with tempfile.TemporaryDirectory(prefix="native-wheel-audit-") as directory:
            elf = Path(directory) / "native.so"
            elf.write_bytes(raw)
            dynamic = subprocess.check_output(["readelf", "-d", str(elf)], text=True)
            sections = subprocess.check_output(["readelf", "-S", str(elf)], text=True)
        if "(RPATH)" not in dynamic or "$ORIGIN/.libs" not in dynamic or "(RUNPATH)" in dynamic:
            raise ValueError("extension must use bundled-library RPATH, not RUNPATH")
        if ".debug_info" not in sections or ".symtab" not in sections:
            raise ValueError("extension must retain debug info and symbols")
    return dict(path=str(path), arch=arch, tags=tags, license_files=licenses,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    repair = sub.add_parser("repair")
    repair.add_argument("--wheel", type=Path, required=True)
    repair.add_argument("--arch", choices=ARCHES, required=True)
    repair.add_argument("--output", type=Path, required=True)
    check = sub.add_parser("check-release")
    check.add_argument("--wheelhouse", type=Path, required=True)
    check.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "repair":
        if args.output.exists():
            raise SystemExit("use a new output directory; do not mix previous wheels")
        subprocess.run([sys.executable, "-m", "auditwheel", "show", str(args.wheel)], check=True)
        # auditwheel 6.4.2's CLI choices list only the host architecture.
        # Its target-aware repair entrypoint performs the actual wheel/ELF,
        # symbol-version, ISA and policy checks for either architecture.
        from auditwheel.main_repair import execute
        logging.basicConfig(level=logging.INFO)
        options = argparse.Namespace(
            EXCLUDE=[], WHEEL_DIR=args.output, WHEEL_FILE=[args.wheel],
            PLAT="manylinux_2_35_" + args.arch, ONLY_PLAT=True,
            LIB_SDIR=".libs", UPDATE_TAGS=True, STRIP=False,
            ZIP_COMPRESSION_LEVEL=-1, DISABLE_ISA_EXT_CHECK=False)
        rc = execute(options, parser)
        if rc:
            raise SystemExit(rc)
        result = inspect(next(args.output.glob("*.whl")))
        print(json.dumps(result, indent=2))
        return
    report = dict(result="FAIL", wheels=[], missing_arches=[], errors=[])
    for path in sorted(args.wheelhouse.glob("locomotion_aorta-*.whl")):
        try:
            report["wheels"].append(inspect(path))
        except (ValueError, KeyError, StopIteration, zipfile.BadZipFile, subprocess.CalledProcessError) as error:
            report["errors"].append(dict(path=str(path), error=str(error)))
    found = [w["arch"] for w in report["wheels"]]
    report["missing_arches"] = sorted(set(ARCHES) - set(found))
    if len(found) != len(set(found)):
        report["errors"].append(dict(error="duplicate architecture wheels"))
    if not report["missing_arches"] and not report["errors"]:
        report["result"] = "PASS"
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if report["result"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
