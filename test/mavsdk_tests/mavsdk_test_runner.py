#!/usr/bin/env python3

import argparse
import json
import os
import psutil  # type: ignore
import signal
import subprocess
import sys
from integration_test_runner import test_runner, logger_helper
import integration_test_runner.process_helper as ph
from typing import Any, Dict, List, NoReturn


class TesterInterfaceMavsdk(test_runner.TesterInterface):

    def query_test_cases(self, build_dir: str, filter: str) -> List[str]:
        cmd = os.path.join(build_dir, 'mavsdk_tests/mavsdk_tests')
        args = ["--list-test-names-only", filter]
        p = subprocess.Popen(
            [cmd] + args,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT)
        assert p.stdout is not None
        cases = str(p.stdout.read().decode("utf-8")).strip().split('\n')
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
        return ph.TestRunnerMavsdk(
            workspace_dir,
            log_dir,
            model,
            case,
            config['mavlink_connection'],
            speed_factor,
            verbose,
            build_dir)

    def rootfs_base_dirname(self) -> str:
        return "tmp_mavsdk_tests"


def main() -> NoReturn:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir",
                        help="Directory for log files", default="logs")
    parser.add_argument("--speed-factor", default=1,
                        help="how fast to run the simulation")
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
    parser.add_argument("config_file", help="JSON config file to use")
    parser.add_argument("--build-dir", type=str,
                        default='build/px4_sitl_default/',
                        help="relative path where the built files are stored")
    args = parser.parse_args()

    if args.force_color:
        logger_helper.force_color = True

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

    tester_interface = TesterInterfaceMavsdk()
    tester = test_runner.Tester(
        config,
        args.iterations,
        args.abort_early,
        args.speed_factor,
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

    sys.exit(0 if tester.run() else 1)


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
                  "run `DONT_RUN=1 "
                  "make px4_sitl gazebo mavsdk_tests` or "
                  "`DONT_RUN=1 make px4_sitl_default gazebo mavsdk_tests`")
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

    if not os.path.isfile(os.path.join(build_dir,
                                       'mavsdk_tests/mavsdk_tests')):
        print("Test runner is not built\n"
              "run `DONT_RUN=1 "
              "make px4_sitl gazebo mavsdk_tests` or "
              "`DONT_RUN=1 make px4_sitl_default gazebo mavsdk_tests`")
        result = False

    return result


if __name__ == '__main__':
    main()
