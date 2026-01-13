import datetime
import fnmatch
import math
import os
import re
import sys
import time
from typing import Any, Dict, List, NoReturn, TextIO, Optional
from types import FrameType
from . import process_helper as ph
from .logger_helper import color, colorize

try:
    import requests
except ImportError as e:
    print("Failed to import requests: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user requests")
    print("")
    sys.exit(1)


class TesterInterface:

    def query_test_cases(self, build_dir: str, filter: str) -> List[str]:
        """Get a list of test cases that match a given filter"""
        raise NotImplementedError

    def create_test_runner(self,
                           workspace_dir: str,
                           log_dir: str,
                           model: str,
                           case: str,
                           config: Dict[str, Any],
                           speed_factor: float,
                           verbose: bool,
                           build_dir: str) -> ph.Runner:
        """Instantiate a Runner that starts the test executable"""
        raise NotImplementedError

    def rootfs_base_dirname(self) -> str:
        """Get a directory name to be used as rootfs dir in PX4 inside the build directory"""
        raise NotImplementedError


class Tester:
    def __init__(self,
                 config: Dict[str, Any],
                 iterations: int,
                 abort_early: bool,
                 speed_factor: float,
                 model: str,
                 case: str,
                 debugger: str,
                 log_dir: str,
                 gui: bool,
                 verbose: bool,
                 upload: bool,
                 build_dir: str,
                 tester_interface: TesterInterface):
        self.config = config
        self.build_dir = build_dir
        self.active_runners: List[ph.Runner]
        self.iterations = iterations
        self.abort_early = abort_early
        self.speed_factor = speed_factor
        self.debugger = debugger
        self.log_dir = log_dir
        self.gui = gui
        self.verbose = verbose
        self.upload = upload
        self.start_time = datetime.datetime.now()
        self.log_fd: Any[TextIO] = None
        self.tester_interface = tester_interface
        self.tests = self.determine_tests(config['tests'], model, case)
        self.active_runners = []

    @staticmethod
    def wildcard_match(pattern: str, potential_match: str) -> bool:
        return fnmatch.fnmatchcase(potential_match, pattern)

    @staticmethod
    def plural_s(n_items: int) -> str:
        if n_items > 1:
            return "s"
        else:
            return ""

    def determine_tests(self,
                        tests: List[Dict[str, Any]],
                        model: str,
                        case: str) -> List[Dict[str, Any]]:
        for test in tests:
            test['selected'] = (model == 'all' or model == test['model'])
            test['cases'] = dict.fromkeys(
                self.tester_interface.query_test_cases(self.build_dir, test['test_filter']))
            for key in test['cases'].keys():
                test['cases'][key] = {
                    'selected': (test['selected'] and
                                 (case == 'all' or
                                  self.wildcard_match(case, key)))}
        return tests

    def run(self) -> bool:
        self.show_plans()
        self.prepare_for_results()
        self.run_tests()
        self.show_detailed_results()
        self.show_overall_result()
        return self.was_overall_pass()

    def show_plans(self) -> None:
        print()
        print(colorize(
            "About to run {} test case{} for {} selected model{} "
            "({} iteration{}):".
            format(self.num_cases(), self.plural_s(self.num_cases()),
                   self.num_models(), self.plural_s(self.num_models()),
                   self.iterations, self.plural_s(self.iterations)),
            color.BOLD))

        for test in self.tests:
            if not test['selected']:
                print(colorize(colorize(
                    "  - {} (not selected)".format(test['model']),
                    color.BOLD), color.GRAY))
                continue
            print(colorize("  - {}:".format(test['model']), color.BOLD))
            for key, case in test['cases'].items():
                if case['selected']:
                    print("    - '{}'".format(key))
                else:
                    print(colorize("    - '{}' (not selected)".format(key),
                                   color.GRAY))
        print()

    def num_models(self) -> int:
        return sum(map(
            lambda test: 1 if test['selected'] else 0,
            self.tests))

    def num_cases(self) -> int:
        n_cases = 0
        for test in self.tests:
            if not test['selected']:
                continue
            n_cases += self.num_cases_for_test(test)

        return n_cases

    def num_cases_for_test(self, test: Dict[str, Any]) -> int:
        n_cases = 0
        for case in test['cases'].values():
            if not case['selected']:
                continue
            n_cases += 1
        return n_cases

    def prepare_for_results(self) -> None:
        for test in self.tests:
            if not test['selected']:
                continue
            for key, value in test['cases'].items():
                if not value['selected']:
                    continue
                value['results'] = []

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

            test_i = 0
            for key, case_value in test['cases'].items():
                if not case_value['selected']:
                    continue

                print("--> Test case {} of {}: '{}' running ...".
                      format(test_i + 1,
                             self.num_cases_for_test(test),
                             key))

                log_dir = self.get_log_dir(iteration, test['model'], key)
                if self.verbose:
                    print("Creating log directory: {}"
                          .format(log_dir))
                os.makedirs(log_dir, exist_ok=True)

                was_success = self.run_test_case(test, key, log_dir)

                print("--- Test case {} of {}: '{}' {}."
                      .format(test_i + 1,
                              self.num_cases_for_test(test),
                              key,
                              colorize("succeeded", color.GREEN)
                              if was_success
                              else colorize("failed", color.RED)))

                if not was_success:
                    iteration_success = False

                if not was_success and self.abort_early:
                    print("Aborting early")
                    return False

                test_i += 1

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

    def get_max_speed_factor(self, test: Dict[str, Any]) -> float:
        speed_factor = self.speed_factor
        if "max_speed_factor" in test:
            speed_factor = min(float(speed_factor), test["max_speed_factor"])
        return speed_factor

    def run_test_case(self, test: Dict[str, Any],
                      case: str, log_dir: str) -> bool:

        logfile_path = self.determine_logfile_path(log_dir, 'combined')
        self.start_combined_log(logfile_path)

        self.start_runners(log_dir, test, case)

        test_timeout_s = test['timeout_min'] * 60
        while self.active_runners[-1].time_elapsed_s() < test_timeout_s:
            returncode = self.active_runners[-1].poll()

            self.collect_runner_output()

            if returncode is not None:
                is_success = (returncode == 0)
                break

        else:
            print(colorize(
                "Test timeout of {} mins triggered!".
                format(test['timeout_min']),
                color.BOLD))
            is_success = False

        self.stop_runners()
        # Collect what was left in output buffers.
        self.collect_runner_output()
        self.stop_combined_log()

        result = {'success': is_success,
                  'logfiles': [runner.get_log_filename()
                               for runner in self.active_runners]}
        test['cases'][case]['results'].append(result)

        if not is_success:
            if not self.verbose:
                print(self.get_combined_log(logfile_path))
            print("Logfiles: ")
            print("  - {}".format(logfile_path))
            for runner in self.active_runners:
                print("  - {}".format(runner.get_log_filename()))

        if self.upload:
            ulog_file = self.parse_for_ulog_file(
                self.get_combined_log(logfile_path))
            self.upload_log(ulog_file, test['model'], case, is_success)

        return is_success

    def start_runners(self,
                      log_dir: str,
                      test: Dict[str, Any],
                      case: str) -> None:
        self.active_runners = []

        if self.config['mode'] == 'sitl':
            if self.config['simulator'] == 'gazebo':
                # Use RegEx to extract worldname.world from case name
                match = re.search(r'\((.*?\.world)\)', case)
                if match:
                    world_name = match.group(1)
                else:
                    world_name = 'empty.world'

                gzserver_runner = ph.GzserverRunner(
                    os.getcwd(),
                    log_dir,
                    test['vehicle'],
                    case,
                    self.get_max_speed_factor(test),
                    self.verbose,
                    self.build_dir,
                    world_name)
                self.active_runners.append(gzserver_runner)

                gzmodelspawn_runner = ph.GzmodelspawnRunner(
                    os.getcwd(),
                    log_dir,
                    test['vehicle'],
                    case,
                    self.verbose,
                    self.build_dir)
                self.active_runners.append(gzmodelspawn_runner)

                if self.gui:
                    gzclient_runner = ph.GzclientRunner(
                        os.getcwd(),
                        log_dir,
                        test['model'],
                        case,
                        self.verbose)
                    self.active_runners.append(gzclient_runner)

                # We must start the PX4 instance at the end, as starting
                # it in the beginning, then connecting Gazebo server freaks
                # out the PX4 (it needs to have data coming in when started),
                # and can lead to EKF to freak out, or the instance itself
                # to die unexpectedly.
                px4_runner = ph.Px4Runner(
                    os.getcwd(),
                    log_dir,
                    test['model'],
                    case,
                    self.get_max_speed_factor(test),
                    self.debugger,
                    self.verbose,
                    self.build_dir,
                    self.tester_interface.rootfs_base_dirname())
                for env_key in test.get('env', []):
                    px4_runner.env[env_key] = str(test['env'][env_key])
                self.active_runners.append(px4_runner)

        self.active_runners.append(self.tester_interface.create_test_runner(
            os.getcwd(),
            log_dir,
            test['model'],
            case,
            self.config,
            self.speed_factor,
            self.verbose,
            self.build_dir
        ))

        abort = False
        for runner in self.active_runners:
            runner.set_log_filename(
                self.determine_logfile_path(log_dir, runner.name))

            if not self.try_to_run_several_times(runner):
                abort = True
                break

        if abort:
            print("Could not start runner: {}".format(runner.name))
            self.collect_runner_output()
            self.stop_combined_log()
            self.stop_runners()
            sys.exit(1)

    def try_to_run_several_times(self, runner: ph.Runner) -> bool:
        for _ in range(3):
            runner.start()

            if runner.has_started_ok():
                return True

            else:
                runner.stop()
                time.sleep(1)

        return False

    def stop_runners(self) -> None:
        for runner in self.active_runners:
            runner.stop()

    def collect_runner_output(self) -> None:
        for runner in self.active_runners:
            while True:
                line = runner.get_output_line()
                if not line:
                    break

                self.add_to_combined_log(line)
                if self.verbose:
                    print(line, end="")

    def parse_for_ulog_file(self, log: str) -> Optional[str]:

        match = "[logger] Opened full log file: ./"
        for line in log.splitlines():
            found = line.find(match)
            if found != -1:
                return os.path.join(os.getcwd(), self.build_dir,
                                    self.tester_interface.rootfs_base_dirname(),
                                    "rootfs",
                                    line[found + len(match):])
        return None

    def upload_log(self, ulog_path: Optional[str],
                   model: str, case: str, success: bool) -> None:
        if not ulog_path:
            print("    Could not find ulog log file to upload")
            return

        if not os.getenv('GITHUB_RUN_ID'):
            print("    Upload only implemented for GitHub Actions CI")
            return

        print("    Uploading logfile '{}' ...".format(ulog_path))

        server = "https://logs.px4.io"

        result_str = "passing" if success else "failing"

        payload = {
            "type": "flightreport",
            "description": "SITL integration test - {}: '{}' -> {}"
            .format(model, case, result_str),
            "feedback":
                "{}/{}/actions/runs/{}"
                .format(os.getenv("GITHUB_SERVER_URL"),
                        os.getenv("GITHUB_REPOSITORY"),
                        os.getenv("GITHUB_RUN_ID")),
            "email": "",
            "source": "CI",
            "videoUrl": "",
            "rating": "notset",
            "windSpeed": -1,
            "public": "true"
        }

        with open(ulog_path, 'rb') as f:
            r = requests.post(server + "/upload",
                              data=payload,
                              files={'filearg': f},
                              allow_redirects=False)
            if r.status_code == 302:  # redirect
                if 'Location' in r.headers:
                    plot_url = r.headers['Location']
                    if len(plot_url) > 0 and plot_url[0] == '/':
                        plot_url = server + plot_url
                    print("    Uploaded to: " + plot_url)
            else:
                print("    Upload failed with status_code: {}"
                      .format(r.status_code))

    def start_combined_log(self, filename: str) -> None:
        self.log_fd = open(filename, 'w')

    def stop_combined_log(self) -> None:
        if self.log_fd:
            self.log_fd.close()

    def add_to_combined_log(self, output: str) -> None:
        self.log_fd.write(output)

    def get_combined_log(self, filename: str) -> str:
        with open(filename, 'r') as f:
            return f.read()

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
                if not case['selected']:
                    continue
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
                if not case['selected']:
                    continue
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

    def sigint_handler(self, sig: int, frame: Optional[FrameType]) \
            -> NoReturn:
        print("Received SIGINT")
        print("Stopping all processes ...")
        self.stop_runners()
        self.stop_combined_log()
        print("Stopping all processes done.")
        self.show_detailed_results()
        sys.exit(-sig)
