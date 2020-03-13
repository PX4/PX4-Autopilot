#!/usr/bin/env python3

import argparse
import json
import os
import psutil  # type: ignore
import signal
import subprocess
import sys
from logger_helper import color, colorize
import process_helper as ph


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir",
                        help="Directory for log files", default="logs")
    parser.add_argument("--speed-factor", default=1,
                        help="How fast to run the simulation")
    parser.add_argument("--iterations", type=int, default=1,
                        help="How often to run all tests")
    parser.add_argument("--abort-early", action='store_true',
                        help="Abort on first unsuccessful test")
    parser.add_argument("--gui", default=False, action='store_true',
                        help="Display gzclient with simulation")
    parser.add_argument("--model", type=str, default='all',
                        help="Specify which model to run")
    parser.add_argument("--debugger", default="",
                        help="valgrind callgrind gdb lldb")
    parser.add_argument("--verbose", default=False, action='store_true',
                        help="Enable more verbose output")
    parser.add_argument("config_file", help="JSON config file to use")
    args = parser.parse_args()

    with open(args.config_file) as json_file:
        config = json.load(json_file)

    if config["mode"] != "sitl" and args.gui:
        print("--gui is not compatible with the mode '{}'"
              .format(config["mode"]))
        sys.exit(1)

    if not is_everything_ready(config):
        sys.exit(1)

    if args.verbose:
        print("Creating directory: {}".format(args.log_dir))
    os.makedirs(args.log_dir, exist_ok=True)

    tester = Tester(
        config,
        args.iterations,
        args.abort_early,
        args.speed_factor,
        args.model,
        args.debugger,
        args.log_dir,
        args.gui,
        args.verbose
    )
    signal.signal(signal.SIGINT, tester.sigint_handler)

    sys.exit(tester.run())


def determine_tests(filter):
    cmd = os.getcwd() + "/build/px4_sitl_default/mavsdk_tests/mavsdk_tests"
    args = ["--list-test-names-only", filter]
    p = subprocess.Popen(
        [cmd] + args,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT)
    assert p.stdout is not None
    tests = str(p.stdout.read().decode("utf-8")).strip().split('\n')
    return tests


def is_running(process_name):
    for proc in psutil.process_iter(attrs=['name']):
        if proc.info['name'] == process_name:
            return True
    return False


def is_everything_ready(config):
    result = True

    if config['mode'] == 'sitl':
        if is_running('px4'):
            print("px4 process already running\n"
                  "run `killall px4` and try again")
            result = False
        if not os.path.isfile('build/px4_sitl_default/bin/px4'):
            print("PX4 SITL is not built\n"
                  "run `DONT_RUN=1 "
                  "make px4_sitl gazebo mavsdk_tests`")
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

    if not os.path.isfile('build/px4_sitl_default/mavsdk_tests/mavsdk_tests'):
        print("Test runner is not built\n"
              "run `DONT_RUN=1 "
              "make px4_sitl gazebo mavsdk_tests`")
        result = False

    return result


class Tester:
    def __init__(self,
                 config,
                 iterations,
                 abort_early,
                 speed_factor,
                 model,
                 debugger,
                 log_dir,
                 gui,
                 verbose):
        self.active_runners = []
        self.iterations = iterations
        self.abort_early = abort_early
        self.config = config
        self.speed_factor = speed_factor
        self.model = model
        self.debugger = debugger
        self.log_dir = log_dir
        self.gui = gui
        self.verbose = verbose

    def run(self):
        overall_success = True

        for iteration in range(self.iterations):
            if self.iterations > 1:
                print("Test iteration: {} / {}".
                      format(iteration + 1, self.iterations))

            was_success = self.run_test_group()

            if not was_success:
                overall_success = False

            if self.iterations > 1 and not was_success and self.abort_early:
                print("Aborting with a failure in test run {}/{}".
                      format(iteration + 1, self.iterations))
                break

        if overall_success:
            print(colorize(colorize(
                "Overall result: success!", color.GREEN), color.BOLD))
            return 0
        else:
            print(colorize(colorize(
                "Overall result: failure!", color.RED), color.BOLD))
            return 1

    def run_test_group(self):
        overall_success = True

        tests = self.config["tests"]

        if self.model == 'all':
            models = tests
        else:
            found = False
            for elem in tests:
                if elem['model'] == self.model:
                    models = [elem]
                    found = True
            if not found:
                print("Specified model is not defined")
                models = []

        for group in models:
            print(colorize(
                "==> Running tests for '{}' with filter '{}'"
                .format(group['model'], group['test_filter']),
                color.BOLD))

            tests = determine_tests(group['test_filter'])

            for i, test in enumerate(tests):
                print("--> Test {} of {}: '{}' running ...".
                      format(i+1, len(tests), test))
                was_success = self.run_test(test, group)
                if was_success:
                    print(colorize(
                        "--- Test {} of {}: '{}' succeeded."
                        .format(i+1, len(tests), test),
                        color.GREEN))
                else:
                    print(colorize(
                        "--- Test {} of {}: '{}' failed."
                        .format(i+1, len(tests), test),
                        color.RED))
                    # TODO: now print the test output

                if not was_success:
                    overall_success = False

                if not was_success and self.abort_early:
                    print("Aborting early")
                    return False

        return overall_success

    def run_test(self, test, group):
        self.active_runners = []

        speed_factor = self.speed_factor
        if "max_speed_factor" in group:
            speed_factor = min(int(speed_factor), group["max_speed_factor"])

        if self.config['mode'] == 'sitl':
            px4_runner = ph.Px4Runner(
                os.getcwd(),
                self.log_dir,
                group,
                speed_factor,
                self.debugger,
                self.verbose)
            px4_runner.start()
            self.active_runners.append(px4_runner)

            if self.config['simulator'] == 'gazebo':
                gzserver_runner = ph.GzserverRunner(
                    os.getcwd(),
                    self.log_dir,
                    group,
                    self.speed_factor,
                    self.verbose)
                gzserver_runner.start()
                self.active_runners.append(gzserver_runner)

                if self.gui:
                    gzclient_runner = ph.GzclientRunner(
                        os.getcwd(),
                        self.log_dir,
                        group,
                        self.verbose)
                    gzclient_runner.start()
                    self.active_runners.append(gzclient_runner)

        test_runner = ph.TestRunner(
            os.getcwd(),
            self.log_dir,
            group,
            test,
            self.config['mavlink_connection'],
            self.verbose)

        test_runner.start()
        self.active_runners.append(test_runner)

        while test_runner.time_elapsed_s() < group['timeout_min']*60:
            returncode = test_runner.poll()
            if returncode is not None:
                is_success = (returncode == 0)
                break

            if self.verbose:
                for runner in self.active_runners:
                    runner.print_output()

        else:
            print(colorize(
                "Test timeout of {} mins triggered!".
                format(group['timeout_min']),
                color.BOLD))
            is_success = False

        for runner in self.active_runners:
            runner.stop()

        return is_success

    def sigint_handler(self, sig, frame):
        print("Received SIGINT")
        print("Stopping all processes ...")
        for runner in self.active_runners:
            runner.stop()
        print("Stopping all processes done.")
        # TODO: print interim results
        sys.exit(-sig)


if __name__ == '__main__':
    main()
