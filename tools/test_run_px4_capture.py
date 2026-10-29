"""File-workflow tests only: no ROS node or aircraft is started."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location("capture", Path(__file__).with_name("run_px4_capture.py"))
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.logs = self.root / "px4_log"
        self.logs.mkdir()
        self.old_log = self.logs / "previous.ulg"
        self.old_log.write_bytes(b"ULog\x01\x12\x35previous")
        self.before = capture.inventory(self.logs)
        self.run = self.root / "experiment"
        self.ros = self.run / "ros" / "single_axis_123" / "allfile_single_axis_123"
        self.ros.mkdir(parents=True)
        for name in capture.ROS_FILES:
            (self.ros / name).write_text("fixture\n")
        (self.ros / "result.txt").write_text("state=IDLE\nflight_success=true\n")
        (self.ros / "samples.csv").write_text("arming_state,landed\n1,1\n")

    def test_ab_mission_command_uses_ab_package_and_log_root(self):
        command = capture.scenario_command("ab_mission", self.run)
        self.assertEqual(command[:4], ["ros2", "run", "px4_ab_mission", "ab_mission"])
        self.assertIn(f"log_root:={self.run / 'ros'}", command)
        self.assertIn("a_x:=2.0", command)
        self.assertNotIn("--params-file", command)

    def test_existing_control_scenario_command_is_unchanged(self):
        command = capture.scenario_command("single_axis", self.run, "/opt/share/px4_control_experiment")
        self.assertEqual(command[:4], ["ros2", "run", "px4_control_experiment", "control_experiment"])
        self.assertIn("--params-file", command)
        self.assertTrue(any(item.endswith("single_axis.yaml") for item in command))

    def collect(self, busy=False):
        with patch.object(capture, "open_by_px4", return_value=busy):
            return capture.collect(self.run, self.logs, self.before, wait_seconds=0)

    def test_same_run_ros_and_new_ulog_are_collected(self):
        log = self.logs / "current.ulg"
        log.write_bytes(b"ULog\x01\x12\x35current")
        manifest = self.collect()
        for name in capture.ROS_FILES:
            self.assertEqual((self.run / name).read_bytes(), (self.ros / name).read_bytes())
        self.assertEqual((self.run / "flight.ulg").read_bytes(), log.read_bytes())
        self.assertTrue(log.exists())
        self.assertEqual(manifest["ulog_source"], str(log.resolve()))

    def test_never_uses_old_latest_log_when_no_log_was_recorded(self):
        with self.assertRaisesRegex(RuntimeError, "等待 ULog 超时"):
            self.collect()
        self.assertFalse((self.run / "flight.ulg").exists())
        self.assertTrue((self.run / "samples.csv").exists())

    def test_updated_continuous_log_is_identified_as_preexisting(self):
        self.old_log.write_bytes(self.old_log.read_bytes() + b"new-flight")
        self.assertTrue(self.collect()["ulog_existed_before_launch"])

    def test_multiple_logs_do_not_get_arbitrarily_matched(self):
        for name in ("a.ulg", "b.ulg"):
            (self.logs / name).write_bytes(b"ULog\x01\x12\x35data")
        with self.assertRaisesRegex(RuntimeError, "多个 ULog 更新"):
            self.collect()
        self.assertFalse((self.run / "flight.ulg").exists())

    def test_open_log_cannot_be_marked_complete(self):
        (self.logs / "current.ulg").write_bytes(b"ULog\x01\x12\x35data")
        with self.assertRaisesRegex(RuntimeError, "等待 ULog 超时"):
            self.collect(busy=True)
        self.assertFalse((self.run / "flight.ulg").exists())

    def test_missing_result_is_preserved_as_incomplete(self):
        (self.ros / "result.txt").unlink()
        with self.assertRaisesRegex(RuntimeError, "result.txt"):
            self.collect()
        self.assertTrue((self.ros / "samples.csv").exists())

    def test_final_transition_sample_is_allowed_when_result_is_idle(self):
        (self.ros / "samples.csv").write_text("arming_state,landed\n2,0\n")
        log = self.logs / "current.ulg"
        log.write_bytes(b"ULog\x01\x12\x35data")
        self.collect()

    def test_linux_descriptor_check(self):
        proc = self.root / "proc"
        process = proc / "42"
        (process / "fd").mkdir(parents=True)
        (process / "cmdline").write_bytes(b"/some/build/bin/px4\0-d\0")
        descriptor = process / "fd" / "8"
        descriptor.symlink_to(self.old_log)
        self.assertTrue(capture.open_by_px4(self.old_log, proc))
        descriptor.unlink()
        self.assertFalse(capture.open_by_px4(self.old_log, proc))


if __name__ == "__main__":
    unittest.main()
