#!/usr/bin/env python3

import argparse
import atexit
import datetime
import errno
import json
import os
import psutil
import re
import signal
import subprocess
import sys
import threading
import time
import queue


def supports_color():
    """Returns True if the running system's terminal supports color.

    From https://stackoverflow.com/a/22254892/8548472
    """
    plat = sys.platform
    supported_platform = plat != 'Pocket PC' and (plat != 'win32' or
                                                  'ANSICON' in os.environ)
    # isatty is not always implemented, #6223.
    is_a_tty = hasattr(sys.stdout, 'isatty') and sys.stdout.isatty()
    return supported_platform and is_a_tty


if supports_color():
    class color:
        PURPLE = '\033[95m'
        CYAN = '\033[96m'
        DARKCYAN = '\033[36m'
        BLUE = '\033[94m'
        GREEN = '\033[92m'
        YELLOW = '\033[93m'
        RED = '\033[91m'
        BOLD = '\033[1m'
        UNDERLINE = '\033[4m'
        END = '\033[0m'
else:
    class color:
        PURPLE = ''
        CYAN = ''
        DARKCYAN = ''
        BLUE = ''
        GREEN = ''
        YELLOW = ''
        RED = ''
        BOLD = ''
        UNDERLINE = ''
        END = ''


def remove_color(text):
    """Remove ANSI and xterm256 color codes.

    From https://stackoverflow.com/a/30500866/8548472
    """
    return re.sub(r'\x1b(\[.*?[@-~]|\].*?(\x07|\x1b\\))', '', text)


class Runner:
    def __init__(self, log_dir, verbose):
        self.name = ""
        self.cmd = ""
        self.cwd = None
        self.args = []
        self.env = {}
        self.log_dir = log_dir
        self.log_filename = ""
        self.log_fd = None
        self.verbose = verbose
        self.output_queue = queue.Queue()
        self.start_time = time.time()

    def create_log_filename(self, model, test_filter):
        return self.log_dir + os.path.sep + \
            "log-{}-{}-{}-{}.log".format(
                self.name,
                model,
                test_filter,
                datetime.datetime.now().strftime("%Y-%m-%dT%H-%M-%SZ"))

    def start(self, config):
        if self.verbose:
            print("Running: {}".format(" ".join([self.cmd] + self.args)))

        atexit.register(self.stop)

        self.log_filename = self.create_log_filename(
            config['model'], config['test_filter'])
        self.log_fd = open(self.log_filename, 'w')

        self.process = subprocess.Popen(
            [self.cmd] + self.args,
            cwd=self.cwd,
            env=self.env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )

        self.stop_thread = threading.Event()
        self.thread = threading.Thread(target=self.process_output)
        self.thread.start()

    def process_output(self):
        while not self.stop_thread.is_set():
            line = self.process.stdout.readline()
            if line == "\n":
                continue
            self.output_queue.put(line)
            self.log_fd.write(line)
            self.log_fd.flush()

    def poll(self):
        return self.process.poll()

    def wait(self, timeout_min):
        try:
            return self.process.wait(timeout=timeout_min*60)
        except subprocess.TimeoutExpired:
            print("Timeout of {} min{} reached, stopping...".
                  format(timeout_min, "s" if timeout_min > 1 else ""))
            self.stop()
            print("stopped.")
            return errno.ETIMEDOUT

    def get_output(self):
        try:
            output = self.output_queue.get(block=True, timeout=0.1)
            if supports_color():
                return output
            else:
                return remove_color(output)
        except queue.Empty:
            return None

    def print_output(self):
        output = self.get_output()
        if not output:
            return
        print(color.END +
              "[" + self.name.ljust(11) + "] " +
              output +
              color.END, end="")

    def stop(self):
        atexit.unregister(self.stop)

        self.stop_thread.set()

        returncode = self.process.poll()
        if returncode is None:

            if self.verbose:
                print("Terminating {}".format(self.cmd))
            self.process.terminate()

            try:
                returncode = self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass

            if returncode is None:
                if self.verbose:
                    print("Killing {}".format(self.cmd))
                self.process.kill()
                returncode = self.process.poll()

        if self.verbose:
            print("{} exited with {}".format(
                self.cmd, self.process.returncode))

        self.thread.join()

        self.log_fd.close()

        return self.process.returncode

    def time_elapsed_s(self):
        return time.time() - self.start_time


class Px4Runner(Runner):
    def __init__(self, model, workspace_dir, log_dir, speed_factor, debugger,
                 verbose):
        super().__init__(log_dir, verbose)
        self.name = "px4"
        self.cmd = workspace_dir + "/build/px4_sitl_default/bin/px4"
        self.cwd = workspace_dir + "/build/px4_sitl_default/tmp/rootfs"
        self.args = [
                workspace_dir + "/ROMFS/px4fmu_common",
                "-s",
                "etc/init.d-posix/rcS",
                "-t",
                workspace_dir + "/test_data",
                "-d"
            ]
        self.env = {"PATH": os.environ['PATH'],
                    "PX4_SIM_MODEL": model,
                    "PX4_SIM_SPEED_FACTOR": str(speed_factor)}
        self.name = "px4"
        self.debugger = debugger

        if not self.debugger:
            pass
        elif self.debugger == "valgrind":
            self.args = ["--track-origins=yes", "--leak-check=full", "-v",
                         self.cmd] + self.args
            self.cmd = "valgrind"
        elif self.debugger == "callgrind":
            self.args = ["--tool=callgrind", "-v", self.cmd] + self.args
            self.cmd = "valgrind"
        elif self.debugger == "gdb":
            self.args = ["--args", self.cmd] + self.args
            self.cmd = "gdb"
        else:
            print("Using custom debugger " + self.debugger)
            self.args = [self.cmd] + self.args
            self.cmd = self.debugger


class GzserverRunner(Runner):
    def __init__(self, model, workspace_dir, log_dir, speed_factor, verbose):
        super().__init__(log_dir, verbose)
        self.name = "gzserver"
        self.env = {"PATH": os.environ['PATH'],
                    "HOME": os.environ['HOME'],
                    "GAZEBO_PLUGIN_PATH":
                    workspace_dir + "/build/px4_sitl_default/build_gazebo",
                    "GAZEBO_MODEL_PATH":
                    workspace_dir + "/Tools/sitl_gazebo/models",
                    "PX4_SIM_SPEED_FACTOR": str(speed_factor),
                    "DISPLAY": os.environ['DISPLAY']}
        self.cmd = "gzserver"
        self.args = ["--verbose",
                     workspace_dir + "/Tools/sitl_gazebo/worlds/" +
                     model + ".world"]


class GzclientRunner(Runner):
    def __init__(self, workspace_dir, log_dir, verbose):
        super().__init__(log_dir, verbose)
        self.name = "gzclient"
        self.env = {"PATH": os.environ['PATH'],
                    "HOME": os.environ['HOME'],
                    # "GAZEBO_PLUGIN_PATH":
                    # workspace_dir + "/build/px4_sitl_default/build_gazebo",
                    "GAZEBO_MODEL_PATH":
                    workspace_dir + "/Tools/sitl_gazebo/models",
                    "DISPLAY": os.environ['DISPLAY']}
        self.cmd = "gzclient"
        self.args = ["--verbose"]


class TestRunner(Runner):
    def __init__(self, workspace_dir, log_dir, config, test,
                 mavlink_connection, verbose):
        super().__init__(log_dir, verbose)
        self.name = "test_runner"
        self.env = {"PATH": os.environ['PATH']}
        self.cmd = workspace_dir + \
            "/build/px4_sitl_default/mavsdk_tests/mavsdk_tests"
        self.args = ["--url", mavlink_connection, test]


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
                print("Test iteration: {}".
                      format(iteration + 1, self.iterations))

            was_success = self.run_test_group()

            if not was_success:
                overall_success = False

            if self.iterations > 1 and not was_success and self.abort_early:
                print("Aborting with a failure in test run {}/{}".
                      format(iteration + 1, self.iterations))
                break

        if overall_success:
            print(color.GREEN + "Overall result: success!" + color.END)
            return 0
        else:
            print(color.RED + "Overall result: failure!" + color.END)
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
            print(color.BOLD + "==> Running tests for '{}' with filter '{}'"
                  .format(group['model'], group['test_filter']) + color.END)

            tests = determine_tests(group['test_filter'])

            for i, test in enumerate(tests):
                print("--> Test {} of {}: '{}' running ...".
                      format(i+1, len(tests), test))
                was_success = self.run_test(test, group)
                if was_success:
                    print(color.GREEN + "--- Test {} of {}: '{}' succeeded.".
                          format(i+1, len(tests), test) + color.END)
                else:
                    print(color.RED + "--- Test {} of {}: '{}' failed.".
                          format(i+1, len(tests), test) + color.END)

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
            px4_runner = Px4Runner(
                group['model'], os.getcwd(), self.log_dir, speed_factor,
                self.debugger, self.verbose)
            px4_runner.start(group)
            self.active_runners.append(px4_runner)

            if self.config['simulator'] == 'gazebo':
                gzserver_runner = GzserverRunner(
                    group['model'],
                    os.getcwd(),
                    self.log_dir,
                    self.speed_factor,
                    self.verbose)
                gzserver_runner.start(group)
                self.active_runners.append(gzserver_runner)

                if self.gui:
                    gzclient_runner = GzclientRunner(
                        os.getcwd(), self.log_dir, self.verbose)
                    gzclient_runner.start(group)
                    self.active_runners.append(gzclient_runner)

        test_runner = TestRunner(
            os.getcwd(),
            self.log_dir,
            group,
            test,
            self.config['mavlink_connection'],
            self.verbose)

        test_runner.start(group)
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
            print(color.BOLD + "Test timeout of {} mins triggered!".
                  format(group['timeout_min']) + color.END)
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
        sys.exit(-sig)


if __name__ == '__main__':
    main()
