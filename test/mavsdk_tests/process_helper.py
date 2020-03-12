#!/usr/bin/env python3

import queue
import time
import os
import datetime
import atexit
import subprocess
import threading
import errno
from logger_helper import color, maybe_strip_color, colorize


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
            return maybe_strip_color(output)
        except queue.Empty:
            return None

    def print_output(self):
        output = self.get_output()
        if not output:
            return
        print(colorize("[" + self.name.ljust(11) + "] " + output, color.RESET),
              end="")

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
