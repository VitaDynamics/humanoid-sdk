"""Packaging negative controls: raw Linux tags and incomplete release sets."""
import importlib.util
import json
from pathlib import Path
import subprocess
import struct
import sys
import tempfile
import unittest
from unittest import mock
import zipfile

TOOL = Path(__file__).resolve().parents[1] / "tools/wheels.py"
spec = importlib.util.spec_from_file_location("native_wheel_tool", TOOL)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReleaseGateTest(unittest.TestCase):
    def test_generic_linux_wheel_is_not_a_release_asset(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "locomotion_aorta-0.2.1-cp310-cp310-linux_x86_64.whl"
            with zipfile.ZipFile(path, "w") as wheel:
                wheel.writestr("locomotion_aorta-0.2.1.dist-info/WHEEL",
                               "Wheel-Version: 1.0\nTag: cp310-cp310-linux_x86_64\n")
            with self.assertRaisesRegex(ValueError, "audited cp310"):
                module.inspect(path)

    def test_release_requires_exact_current_public_message_version(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "locomotion_aorta-0.2.1-cp310-cp310-manylinux_2_35_x86_64.whl"
            info = "locomotion_aorta-0.2.1.dist-info"
            elf = bytearray(20)
            elf[:6] = b"\x7fELF\x02\x01"
            struct.pack_into("<H", elf, 18, 62)
            for version in ("0.1.5", "0.1.4"):
                with self.subTest(humanoid_msgs=version):
                    with zipfile.ZipFile(path, "w") as wheel:
                        wheel.writestr(info + "/WHEEL",
                                       "Wheel-Version: 1.0\nTag: cp310-cp310-manylinux_2_35_x86_64\n")
                        wheel.writestr(info + "/METADATA", (
                            "Name: locomotion-aorta\nVersion: 0.2.1\nRequires-Python: ==3.10.*\n"
                            "Requires-Dist: aorta-sdk==2026.10.20\n"
                            "Requires-Dist: aorta-msgs==2026.10.20\n"
                            f"Requires-Dist: vbot-humanoid-msgs=={version}\n"
                            "Requires-Dist: flatbuffers==25.12.19\n"
                            "License-File: LICENSE\nLicense-File: NOTICE\nLicense-File: ORT_LICENSE\nLicense-File: FLATBUFFERS_LICENSE\n"))
                        for name in ("LICENSE", "NOTICE", "ORT_LICENSE", "FLATBUFFERS_LICENSE"):
                            wheel.writestr(info + "/licenses/" + name, "fixture")
                        for name in ("locomotion_aorta/__init__.py",
                                     "locomotion_aorta/_onnx/__init__.py",
                                     "locomotion_aorta/_onnx/.libs/libonnxruntime.so.1",
                                     "locomotion_aorta/_onnx/.libs/libonnxruntime_providers_shared.so"):
                            wheel.writestr(name, b"")
                        wheel.writestr("locomotion_aorta/_onnx/_native.cpython-310-x86_64-linux-gnu.so", elf)
                        wheel.writestr("locomotion_aorta/_body_cmd_native.cpython-310-x86_64-linux-gnu.so", elf)
                        wheel.writestr("locomotion_aorta/licenses/flatbuffers/LICENSE", "fixture")
                    with mock.patch.object(module.subprocess, "check_output", side_effect=[
                        "(RPATH) Library rpath: [$ORIGIN/.libs]", ".debug_info .symtab", ".debug_info .symtab"
                    ]) as readelf:
                        if version == "0.1.5":
                            self.assertEqual(module.inspect(path)["arch"], "x86_64")
                            self.assertEqual(readelf.call_count, 3)
                        else:
                            with self.assertRaisesRegex(ValueError, "unexpected SDK Python/core dependencies"):
                                module.inspect(path)
                            readelf.assert_not_called()

    def test_body_codec_is_required_and_wrong_elf_arch_is_rejected(self):
        for present in (False, True):
            with self.subTest(present=present), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'codec.whl'
                with zipfile.ZipFile(path, 'w') as wheel:
                    wheel.writestr('locomotion_aorta/licenses/flatbuffers/LICENSE', 'fixture')
                    if present:
                        elf = bytearray(20)
                        elf[:6] = b'\x7fELF\x02\x01'
                        struct.pack_into('<H', elf, 18, 183)
                        wheel.writestr('locomotion_aorta/_body_cmd_native.cpython-310-x86_64-linux-gnu.so', elf)
                with zipfile.ZipFile(path) as wheel, self.assertRaises(ValueError):
                    module.inspect_body_codec(wheel, 'x86_64')

    def test_release_cannot_pass_without_both_architectures(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report = root / "report.json"
            result = subprocess.run([sys.executable, str(TOOL), "check-release", "--wheelhouse",
                                     str(root), "--report", str(report)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            data = json.loads(report.read_text())
            self.assertEqual(data["result"], "FAIL")
            self.assertEqual(set(data["missing_arches"]), {"x86_64", "aarch64"})


if __name__ == "__main__":
    unittest.main()
