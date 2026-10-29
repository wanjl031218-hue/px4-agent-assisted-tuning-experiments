"""Archive regression checks; fixtures do not represent valid flight/ULog data."""
from pathlib import Path
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).with_name("finalize_px4_run.sh")
ROS_NAMES = ("samples.csv", "events.csv", "result.txt", "parameters.yaml", "metadata.txt")
PX4_NAMES = ("px4_params.txt", "px4_version.txt", "snapshot_time.txt", "flight.ulg")


class ArchiveTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.run = self.root / "archive"
        self.run.mkdir()
        self.px4 = self.root / "px4"
        self.px4.mkdir()
        for name in PX4_NAMES:
            (self.px4 / name).write_text("fixture " + name)
        self.ros = self.root / "ros" / "single_axis_123" / "allfile_single_axis_123"
        self.ros.mkdir(parents=True)
        for name in ROS_NAMES:
            (self.ros / name).write_text("fixture " + name)

    def invoke(self, ros=None):
        args = ["bash", str(SCRIPT), str(self.run)]
        args += [str(self.px4 / name) for name in PX4_NAMES]
        if ros is not None:
            args.append(str(ros))
        return subprocess.run(args, text=True, capture_output=True)

    def test_px4_only_cannot_report_complete(self):
        result = self.invoke()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("ROS 记录缺失", result.stderr)
        self.assertFalse((self.run / "flight.ulg").exists())

    def test_explicit_v3_full_directory_and_repeat(self):
        for _ in range(2):
            result = self.invoke(self.ros)
            self.assertEqual(result.returncode, 0, result.stderr)
        for name in ROS_NAMES + PX4_NAMES:
            self.assertTrue((self.run / name).stat().st_size)
        self.assertEqual((self.run / "metadata.txt").read_text(), "fixture metadata.txt")

    def test_missing_final_result_is_not_complete(self):
        (self.ros / "result.txt").unlink()
        self.assertNotEqual(self.invoke(self.ros).returncode, 0)
        self.assertEqual(list(self.run.iterdir()), [])

    def test_conflict_does_not_overwrite_or_partially_copy(self):
        (self.run / "metadata.txt").write_text("existing metadata")
        result = self.invoke(self.ros)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual((self.run / "metadata.txt").read_text(), "existing metadata")
        self.assertFalse((self.run / "flight.ulg").exists())

    def test_existing_flat_layout_supported(self):
        for name in ROS_NAMES:
            (self.run / name).write_text("fixture " + name)
        self.assertEqual(self.invoke().returncode, 0)


if __name__ == "__main__":
    unittest.main()
