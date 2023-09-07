#!/usr/bin/env python3

import argparse
import json
import os
import psutil  # type: ignore
import signal
import subprocess
import sys
from mavsdk_tests.integration_test_runner import test_runner, process_helper as ph, logger_helper
from typing import Any, Dict, List, NoReturn


class MicroXrceAgent:
    def __init__(self, verbose: bool):
        self._verbose = verbose
        # Potential binary names to check for
        self._binary_names = ['MicroXRCEAgent', 'micro-xrce-dds-agent']
        self._proc = None

    def is_running(self) -> bool:
        return any(is_running(name) for name in self._binary_names)

    def start_process(self):
        if self._verbose:
            print('Starting micro-xrce-dds-agent')
        assert self._proc is None

        for name in self._binary_names:
            try:
                self._proc = subprocess.Popen([name] + 'udp4 -p 8888'.split())
            except FileNotFoundError:
                pass
            else:  # Process was started
                break
        if self._proc is None:
            raise RuntimeError(f'Failed to start agent, tried these binaries: {self._binary_names}.\n'
                               'Make sure it is installed and available. '
                               'You can also run it manually before running this script')

    def stop_process_if_started(self):
        if self._proc is None:
            return

        if self._verbose:
            print('Stopping micro-xrce-dds-agent')
        self._proc.kill()
        self._proc = None


class TesterInterfaceRos(test_runner.TesterInterface):

    def __init__(self, ros_package_build_dir: str):
        self._ros_package_build_dir = ros_package_build_dir

    def query_test_cases(self, build_dir: str, filter: str) -> List[str]:
        cmd = os.path.join(self._ros_package_build_dir, 'integration_tests')
        args = ["--gtest_list_tests", "--gtest_filter=" + filter]
        # Example output:
        # Running main() from ros2_install/install/gtest_vendor/src/gtest_vendor/src/gtest_main.cc
        # Tester.
        #   runModeTests
        p = subprocess.Popen(
            [cmd] + args,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT)
        assert p.stdout is not None
        lines = str(p.stdout.read().decode("utf-8")).strip().split('\n')
        if lines[0].startswith('Running main'):
            del lines[0]
        cases = []
        test_suite_name = None
        for line in lines:
            if line.startswith(' '):
                assert test_suite_name is not None
                cases.append(test_suite_name + line.strip())
            else:
                test_suite_name = line
        return cases

    def create_test_runner(self,
                           workspace_dir: str,
                           log_dir: str,
                           model: str,
                           case: str,
                           config: Dict[str, Any],
                           speed_factor: float,
                           verbose: bool,
                           build_dir: str) -> ph.Runner:
        return ph.TestRunnerRos(
            workspace_dir,
            log_dir,
            model,
            case,
            verbose,
            self._ros_package_build_dir)

    def rootfs_base_dirname(self) -> str:
        return "tmp_ros_tests"


def main() -> NoReturn:
    parser = argparse.ArgumentParser()
    parser.add_argument("--list-cases", action='store_true',
                        help="List available test cases")
    parser.add_argument("--log-dir",
                        help="Directory for log files", default="logs")
    parser.add_argument("--iterations", type=int, default=1,
                        help="how often to run all tests")
    parser.add_argument("--abort-early", action='store_true',
                        help="abort on first unsuccessful test")
    parser.add_argument("--gui", default=False, action='store_true',
                        help="display the visualization for a simulation")
    parser.add_argument("--model", type=str, default='all',
                        help="only run tests for one model")
    parser.add_argument("--case", type=str, default='all',
                        help="only run tests for one case "
                             "(or multiple cases with wildcard '*')")
    parser.add_argument("--debugger", default="",
                        help="choice from valgrind, callgrind, gdb, lldb")
    parser.add_argument("--upload", default=False, action='store_true',
                        help="Upload logs to logs.px4.io")
    parser.add_argument("--force-color", default=False, action='store_true',
                        help="Force colorized output")
    parser.add_argument("--verbose", default=False, action='store_true',
                        help="enable more verbose output")
    parser.add_argument("--config-file", help="JSON config file to use",
                        default="test/ros_tests/config.json")
    parser.add_argument("--build-dir", type=str,
                        default='build/px4_sitl_default/',
                        help="relative path where the built files are stored")
    parser.add_argument("--px4-ros2-interface-lib-build-dir", type=str,
                        default=None,
                        help="path which contains the integration_tests binary. "
                             "If not provided, it is determined via 'ros2 pkg'")
    args = parser.parse_args()

    if args.force_color:
        logger_helper.force_color = True

    ros_package_build_dir = args.px4_ros2_interface_lib_build_dir
    if ros_package_build_dir is None:
        ros_package_build_dir = lookup_px4_ros2_cpp_build_dir()
        if args.verbose:
            print(f'px4_ros2_cpp build dir: {ros_package_build_dir}')

    tester_interface = TesterInterfaceRos(ros_package_build_dir)

    if args.list_cases:
        for case in tester_interface.query_test_cases(args.build_dir, '*'):
            print(f'{case}')
        sys.exit(0)

    with open(args.config_file) as json_file:
        config = json.load(json_file)

    if config["mode"] != "sitl" and args.gui:
        print("--gui is not compatible with the mode '{}'"
              .format(config["mode"]))
        sys.exit(1)

    if not is_everything_ready(config, args.build_dir):
        sys.exit(1)

    if args.verbose:
        print("Creating directory: {}".format(args.log_dir))
    os.makedirs(args.log_dir, exist_ok=True)

    speed_factor = 1  # Not (yet) supported
    tester = test_runner.Tester(
        config,
        args.iterations,
        args.abort_early,
        speed_factor,
        args.model,
        args.case,
        args.debugger,
        args.log_dir,
        args.gui,
        args.verbose,
        args.upload,
        args.build_dir,
        tester_interface
    )
    signal.signal(signal.SIGINT, tester.sigint_handler)

    # Automatically start & stop the XRCE Agent if not running already
    micro_xrce_agent = MicroXrceAgent(args.verbose)
    if not micro_xrce_agent.is_running():
        micro_xrce_agent.start_process()

    try:
        result = tester.run()
    finally:
        micro_xrce_agent.stop_process_if_started()

    sys.exit(0 if result else 1)


def is_running(process_name: str) -> bool:
    for proc in psutil.process_iter(attrs=['name']):
        if proc.info['name'] == process_name:
            return True
    return False


def is_everything_ready(config: Dict[str, str], build_dir: str) -> bool:
    result = True

    if config['mode'] == 'sitl':
        if is_running('px4'):
            print("px4 process already running\n"
                  "run `killall px4` and try again")
            result = False
        if not os.path.isfile(os.path.join(build_dir, 'bin/px4')):
            print("PX4 SITL is not built\n"
                  "run `DONT_RUN=1 make px4_sitl gazebo` or "
                  "`DONT_RUN=1 make px4_sitl_default gazebo`")
            result = False
        if config['simulator'] == 'gazebo':
            if is_running('gzserver'):
                print("gzserver process already running\n"
                      "run `killall gzserver` and try again")
                result = False
            if is_running('gzclient'):
                print("gzclient process already running\n"
                      "run `killall gzclient` and try again")
                result = False

    return result


def lookup_px4_ros2_cpp_build_dir():
    """Get the ROS2 build directory for px4_ros2_cpp"""
    try:
        p = subprocess.Popen(
            'ros2 pkg prefix px4_ros2_cpp'.split(),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT)
    except FileNotFoundError:
        print('Error: command "ros2" not found. Did you source the ros workspace?')
        sys.exit(1)
    assert p.stdout is not None
    lines = str(p.stdout.read().decode("utf-8")).strip().split('\n')
    assert len(lines) == 1
    return os.path.join(lines[0], '../../build/px4_ros2_cpp')


if __name__ == '__main__':
    main()
