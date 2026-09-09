import json
from pathlib import Path
import unittest

import ros_test_runner


class RosTestRunnerArgumentsTest(unittest.TestCase):
    def setUp(self):
        self.parser = ros_test_runner.create_argument_parser()
        self.repo = Path(__file__).resolve().parents[1]

    def test_defaults_select_sih_firmware_and_suites(self):
        args = self.parser.parse_args([])
        self.assertEqual(args.config_file, "test/ros_tests/config-sih.json")
        self.assertEqual(args.build_dir, "build/px4_sitl_sih/")
        self.assertEqual(args.model, "all")
        config = json.loads((self.repo / args.config_file).read_text())
        self.assertEqual(config["simulator"], "sih")
        self.assertEqual(config["model_prefix"], "sihsim_")
        self.assertEqual(
            {test["model"] for test in config["tests"]}, {"quadx"}
        )
        self.assertEqual(len(config["tests"]), 3)

    def test_classic_remains_an_explicit_option(self):
        args = self.parser.parse_args([
            "--config-file", "test/ros_tests/config.json",
            "--build-dir", "build/px4_sitl_default/",
            "--model", "iris", "--gui",
        ])
        config = json.loads((self.repo / args.config_file).read_text())
        self.assertEqual(config["simulator"], "gazebo")
        self.assertEqual(config["model_prefix"], "gazebo-classic_")
        self.assertEqual(args.build_dir, "build/px4_sitl_default/")
        self.assertEqual(args.model, "iris")
        self.assertTrue(args.gui)

    def test_case_filter_and_custom_build_directory(self):
        args = self.parser.parse_args([
            "--case", "ModesTest.*", "--build-dir", "/tmp/custom-sih",
        ])
        self.assertEqual(args.case, "ModesTest.*")
        self.assertEqual(args.build_dir, "/tmp/custom-sih")
        self.assertEqual(args.config_file, "test/ros_tests/config-sih.json")


if __name__ == "__main__":
    unittest.main()
