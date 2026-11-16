import shutil
import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class CalibrationStatusFirmwareTests(unittest.TestCase):
    def test_firmware_status_transitions_and_error_mapping(self):
        compiler = shutil.which("g++")
        if compiler is None:
            self.skipTest("g++ is required to exercise the firmware status helper")

        executable = Path(__file__).with_suffix(".status-test-bin")
        source = ROOT / "tests" / "calibration_status_harness.cpp"
        try:
            subprocess.run(
                [
                    compiler,
                    "-std=c++11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-I",
                    str(ROOT / "src"),
                    str(source),
                    "-o",
                    str(executable),
                ],
                check=True,
                cwd=ROOT,
            )
            subprocess.run([str(executable)], check=True, cwd=ROOT)
        finally:
            executable.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
