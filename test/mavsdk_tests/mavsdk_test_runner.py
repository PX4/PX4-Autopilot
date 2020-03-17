#!/usr/bin/env python3

import argparse
import datetime
import json
import math
import os
import psutil  # type: ignore
import signal
import subprocess
import sys
from logger_helper import color, colorize
import process_helper as ph
from typing import Any, Dict, List, NoReturn, TextIO, Optional
from types import FrameType


def main() -> NoReturn:

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

    sys.exit(0 if tester.run() else 1)


def is_running(process_name: str) -> bool:
    for proc in psutil.process_iter(attrs=['name']):
        if proc.info['name'] == process_name:
            return True
    return False


def is_everything_ready(config: Dict[str, str]) -> bool:
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
                 config: Dict[str, Any],
                 iterations: int,
                 abort_early: bool,
                 speed_factor: float,
                 model: str,
                 debugger: str,
                 log_dir: str,
                 gui: bool,
                 verbose: bool):
        self.config = config
        self.active_runners: List[ph.Runner]
        self.iterations = iterations
        self.abort_early = abort_early
        self.tests = self.determine_tests(config['tests'], model)
        self.speed_factor = speed_factor
        self.debugger = debugger
        self.log_dir = log_dir
        self.gui = gui
        self.verbose = verbose
        self.start_time = datetime.datetime.now()
        self.combined_log_fd = TextIO

    @classmethod
    def determine_tests(cls, tests: List[Dict[str, Any]], model: str) \
            -> List[Dict[str, Any]]:
        for test in tests:
            test['selected'] = (model == 'all') or model == test['model']
            test['cases'] = dict.fromkeys(
                cls.determine_test_cases(test['test_filter']))

        return tests

    @staticmethod
    def determine_test_cases(filter: str) -> List[str]:
        cmd = os.getcwd() + "/build/px4_sitl_default/mavsdk_tests/mavsdk_tests"
        args = ["--list-test-names-only", filter]
        p = subprocess.Popen(
            [cmd] + args,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT)
        assert p.stdout is not None
        cases = str(p.stdout.read().decode("utf-8")).strip().split('\n')
        return cases

    @staticmethod
    def plural_s(n_items: int) -> str:
        if n_items > 1:
            return "s"
        else:
            return ""

    def run(self) -> bool:
        self.show_plans()
        self.prepare_for_results()
        self.run_tests()
        self.show_detailed_results()
        self.show_overall_result()
        return self.was_overall_pass()

    def show_plans(self) -> None:
        n_models = sum(map(
            lambda test: 1 if test['selected'] else 0,
            self.tests))
        n_cases = sum(map(
            lambda test: len(test['cases']) if test['selected'] else 0,
            self.tests))

        print()
        print(colorize(
            "About to run {} test case{} for {} selected model{} "
            "({} iteration{}):".
            format(n_cases, self.plural_s(n_cases),
                   n_models, self.plural_s(n_models),
                   self.iterations, self.plural_s(self.iterations)),
            color.BOLD))

        for test in self.tests:
            if not test['selected']:
                print(colorize(colorize(
                    "  - {} (not selected)".format(test['model']),
                    color.BOLD), color.GRAY))
                continue
            print(colorize("  - {}:".format(test['model']), color.BOLD))
            for case in test['cases']:
                print("    - '{}'".format(case))
        print()

    def prepare_for_results(self) -> None:
        for test in self.tests:
            if not test['selected']:
                continue
            for key in test['cases'].keys():
                test['cases'][key] = {'results': []}

    def run_tests(self) -> None:
        for iteration in range(self.iterations):
            if self.iterations > 1:
                print(colorize("%%% Test iteration: {} / {}".
                      format(iteration + 1, self.iterations), color.BOLD))

            success = self.run_iteration(iteration)
            if not success:
                if self.abort_early:
                    break

    def run_iteration(self, iteration: int) -> bool:
        iteration_success = True
        for test in self.tests:
            if not test['selected']:
                continue

            print(colorize(
                "==> Running tests for {}".format(test['model']),
                color.BOLD))

            for i, case in enumerate(test['cases']):
                print("--> Test case {} of {}: '{}' running ...".
                      format(i+1, len(test['cases']), case))

                log_dir = self.get_log_dir(iteration, test['model'], case)
                if self.verbose:
                    print("creating log directory: {}"
                          .format(log_dir))
                os.makedirs(log_dir, exist_ok=True)

                was_success = self.run_test_case(test, case, log_dir)

                print("--- Test case {} of {}: '{}' {}."
                      .format(i+1,
                              len(test['cases']),
                              case,
                              colorize("succeeded", color.GREEN)
                              if was_success
                              else colorize("failed", color.RED)))

                if not was_success:
                    iteration_success = False

                if not was_success and self.abort_early:
                    print("Aborting early")
                    return False

        return iteration_success

    def get_log_dir(self, iteration: int, model: str, case: str) -> str:
        date_and_time = self.start_time.strftime("%Y-%m-%dT%H-%M-%SZ")
        foldername = os.path.join(self.log_dir, date_and_time)

        if self.iterations > 1:
            # Use as many zeros in foldername as required.
            digits = math.floor(math.log10(self.iterations)) + 1
            foldername = os.path.join(foldername,
                                      str(iteration + 1).zfill(digits))

        foldername = os.path.join(foldername, model)
        foldername = os.path.join(foldername, case.replace(" ", "_"))

        return foldername

    def run_test_case(self, test: Dict[str, Any],
                      case: str, log_dir: str) -> bool:
        self.active_runners = []

        speed_factor = self.speed_factor
        if "max_speed_factor" in test:
            speed_factor = min(float(speed_factor), test["max_speed_factor"])

        if self.config['mode'] == 'sitl':
            px4_runner = ph.Px4Runner(
                os.getcwd(),
                log_dir,
                test['model'],
                case,
                speed_factor,
                self.debugger,
                self.verbose)
            self.active_runners.append(px4_runner)

            if self.config['simulator'] == 'gazebo':
                gzserver_runner = ph.GzserverRunner(
                    os.getcwd(),
                    log_dir,
                    test['model'],
                    case,
                    self.speed_factor,
                    self.verbose)
                self.active_runners.append(gzserver_runner)

                if self.gui:
                    gzclient_runner = ph.GzclientRunner(
                        os.getcwd(),
                        log_dir,
                        test['model'],
                        case,
                        self.verbose)
                    self.active_runners.append(gzclient_runner)

        test_runner = ph.TestRunner(
            os.getcwd(),
            log_dir,
            test['model'],
            case,
            self.config['mavlink_connection'],
            self.verbose)

        self.active_runners.append(test_runner)

        for runner in self.active_runners:
            runner.set_log_filename(
                self.determine_logfile_path(log_dir, runner.name))
            runner.start()

        max_name = max(len(runner.name) for runner in self.active_runners)
        logfile_path = self.determine_logfile_path(log_dir, 'combined')

        self.start_combined_log(logfile_path)

        while test_runner.time_elapsed_s() < test['timeout_min']*60:
            returncode = test_runner.poll()
            if returncode is not None:
                is_success = (returncode == 0)
                break

            for runner in self.active_runners:
                output = runner.get_output()
                if not output:
                    continue

                output = self.add_name_prefix(max_name, runner.name, output)
                self.add_to_combined_log(output)
                if self.verbose:
                    print(output)

        else:
            print(colorize(
                "Test timeout of {} mins triggered!".
                format(test['timeout_min']),
                color.BOLD))
            is_success = False

        for runner in self.active_runners:
            runner.stop()
        self.stop_combined_log()

        result = {'success': is_success,
                  'logfiles': [runner.get_log_filename()
                               for runner in self.active_runners]}
        test['cases'][case]['results'].append(result)

        if not is_success:
            print(self.get_combined_log(logfile_path))
            print("Logfiles: ")
            print("  - {}".format(logfile_path))
            for runner in self.active_runners:
                print("  - {}".format(runner.get_log_filename()))
        return is_success

    def start_combined_log(self, filename: str) -> None:
        self.log_fd = open(filename, 'w')

    def stop_combined_log(self) -> None:
        self.log_fd.close()

    def add_to_combined_log(self, output: str) -> None:
        self.log_fd.write(output)

    def get_combined_log(self, filename: str) -> str:
        with open(filename, 'r') as f:
            return f.read()

    @staticmethod
    def add_name_prefix(width: int, name: str, text: str) -> str:
        return colorize("[" + name.ljust(width) + "] " + text, color.RESET)

    @staticmethod
    def determine_logfile_path(log_dir: str, desc: str) -> str:
        return os.path.join(log_dir, "log-{}.log".format(desc))

    def show_detailed_results(self) -> None:
        print()
        print(colorize("Results:", color.BOLD))

        for test in self.tests:
            if not test['selected']:
                print(colorize(colorize(
                    "  - {} (not selected)".
                    format(test['model']), color.BOLD), color.GRAY))
                continue
            else:
                print(colorize("  - {}:".format(test['model']), color.BOLD))

            for name, case in test['cases'].items():
                print("     - '{}': ".format(name), end="")

                n_succeeded = [result['success']
                               for result in case['results']].count(True)
                n_failed = [result['success']
                            for result in case['results']].count(False)
                n_cancelled = self.iterations - len(case['results'])

                notes = [
                    self.note_if_any(
                        colorize("{}succeeded", color.GREEN),
                        n_succeeded, self.iterations),
                    self.note_if_any(
                        colorize("{}failed", color.RED),
                        n_failed, self.iterations),
                    self.note_if_any(
                        colorize("{}cancelled", color.GRAY),
                        n_cancelled, self.iterations)]
                notes_without_none = list(filter(None, notes))
                print(", ".join(notes_without_none))

    def was_overall_pass(self) -> bool:
        for test in self.tests:
            if not test['selected']:
                continue

            for case in test['cases'].values():
                for result in case['results']:
                    if result['success'] is not True:
                        return False
        return True

    def show_overall_result(self) -> None:
        print(colorize(
            "Overall result: {}".format(
             colorize("PASS", color.GREEN)
             if self.was_overall_pass()
             else colorize("FAIL", color.RED)), color.BOLD))

    @staticmethod
    def note_if_any(text_to_format: str, n: int, total: int) -> Optional[str]:
        assert not n < 0
        if n == 0:
            return None
        elif n == 1 and total == 1:
            return text_to_format.format("")
        else:
            return text_to_format.format(str(n) + " ")

    def sigint_handler(self, sig: signal.Signals, frame: FrameType) \
            -> NoReturn:
        print("Received SIGINT")
        print("Stopping all processes ...")
        for runner in self.active_runners:
            runner.stop()
        self.stop_combined_log()
        print("Stopping all processes done.")
        self.show_detailed_results()
        sys.exit(-sig)


if __name__ == '__main__':
    main()
