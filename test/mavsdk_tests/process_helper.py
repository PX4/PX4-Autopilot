#!/usr/bin/env python3

import queue
import time
import os
import atexit
import subprocess
import shutil
import threading
import errno
from typing import Any, Dict, List, TextIO, Optional


class Runner:
    def __init__(self,
                 log_dir: str,
                 model: str,
                 case: str,
                 verbose: bool):
        self.name = ""
        self.cmd = ""
        self.cwd = ""
        self.args: List[str]
        self.env: Dict[str, str] = os.environ.copy()
        self.model = model
        self.case = case
        self.log_filename = ""
        self.log_fd: TextIO
        self.verbose = verbose
        self.output_queue: queue.Queue[str] = queue.Queue()
        self.start_time = time.time()
        self.log_dir = log_dir
        self.log_filename = ""
        self.stop_thread: Any[threading.Event] = None

    def set_log_filename(self, log_filename: str) -> None:
        self.log_filename = log_filename

    def get_log_filename(self) -> str:
        return self.log_filename

    def start(self) -> None:
        if self.verbose:
            print("Running: {}".format(" ".join([self.cmd] + self.args)))

        atexit.register(self.stop)

        if self.verbose:
            print("Logging to {}".format(self.log_filename))
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

    def has_started_ok(self) -> bool:
        return True

    def process_output(self) -> None:
        assert self.process.stdout is not None
        while True:
            line = self.process.stdout.readline()
            if not line and \
                    (self.stop_thread.is_set() or self.poll is not None):
                break
            if not line or line == "\n":
                continue
            line = self.add_prefix(10, self.name, line)
            self.output_queue.put(line)
            self.log_fd.write(line)
            self.log_fd.flush()

    def add_prefix(self, width: int, name: str, text: str) -> str:
        return "[" + self.seconds() + "|" + name.ljust(width) + "] " + text

    def seconds(self) -> str:
        dt = time.time() - self.start_time
        return "{: 8.03f}".format(dt)

    def poll(self) -> Optional[int]:
        return self.process.poll()

    def wait(self, timeout_min: float) -> Optional[int]:
        try:
            return self.process.wait(timeout=timeout_min*60)
        except subprocess.TimeoutExpired:
            print("Timeout of {} min{} reached, stopping...".
                  format(timeout_min, "s" if timeout_min > 1 else ""))
            self.stop()
            print("stopped.")
            return errno.ETIMEDOUT

    def get_output_line(self) -> Optional[str]:
        while True:
            try:
                return self.output_queue.get(block=True, timeout=0.1)
            except queue.Empty:
                return None

    def stop(self) -> int:
        atexit.unregister(self.stop)

        if not self.stop_thread:
            return 0

        returncode = self.process.poll()
        if returncode is None:

            if self.verbose:
                print("Terminating {}".format(self.name))
            self.process.terminate()

            try:
                returncode = self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass

            if returncode is None:
                if self.verbose:
                    print("Killing {}".format(self.name))
                self.process.kill()
                returncode = self.process.poll()

        if self.verbose:
            print("{} exited with {}".format(
                self.name, self.process.returncode))

        self.stop_thread.set()
        self.thread.join()
        self.log_fd.close()

        return self.process.returncode

    def time_elapsed_s(self) -> float:
        return time.time() - self.start_time


class Px4Runner(Runner):
    def __init__(self, workspace_dir: str, log_dir: str,
                 model: str, case: str, speed_factor: float,
                 debugger: str, verbose: bool, build_dir: str):
        super().__init__(log_dir, model, case, verbose)
        self.name = "px4"
        self.cmd = os.path.join(workspace_dir, build_dir, "bin/px4")
        self.cwd = os.path.join(workspace_dir, build_dir,
                                "tmp_mavsdk_tests/rootfs")
        self.args = [
                os.path.join(workspace_dir, build_dir, "etc"),
                "-s",
                "etc/init.d-posix/rcS",
                "-t",
                os.path.join(workspace_dir, "test_data"),
                "-d"
            ]
        self.env["PX4_SIM_MODEL"] = self.model
        self.env["PX4_SIM_SPEED_FACTOR"] = str(speed_factor)
        self.debugger = debugger
        self.clear_rootfs()
        self.create_rootfs()

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

    def clear_rootfs(self) -> None:
        rootfs_path = self.cwd
        if self.verbose:
            print("Clearing rootfs (except logs): {}".format(rootfs_path))
        if os.path.isdir(rootfs_path):
            for item in os.listdir(rootfs_path):
                if item == 'log':
                    continue
                path = os.path.join(rootfs_path, item)
                if os.path.isfile(path) or os.path.islink(path):
                    os.remove(path)
                else:
                    shutil.rmtree(path)

    def create_rootfs(self) -> None:
        rootfs_path = self.cwd
        if self.verbose:
            print("Creating rootfs: {}".format(rootfs_path))
        try:
            os.makedirs(rootfs_path)
        except FileExistsError:
            pass


class GzserverRunner(Runner):
    def __init__(self,
                 workspace_dir: str,
                 log_dir: str,
                 model: str,
                 case: str,
                 speed_factor: float,
                 verbose: bool,
                 build_dir: str):
        super().__init__(log_dir, model, case, verbose)
        self.name = "gzserver"
        self.cwd = workspace_dir
        self.env["GAZEBO_PLUGIN_PATH"] = \
            os.path.join(workspace_dir, build_dir, "build_gazebo")
        self.env["GAZEBO_MODEL_PATH"] = \
            os.path.join(workspace_dir, "Tools/sitl_gazebo/models")
        self.env["PX4_SIM_SPEED_FACTOR"] = str(speed_factor)
        self.cmd = "stdbuf"
        self.args = ["-o0", "-e0", "gzserver", "--verbose",
                     os.path.join(workspace_dir,
                                  "Tools/sitl_gazebo/worlds",
                                  "empty.world")]

    def has_started_ok(self) -> bool:
        # Wait until gzerver has started and connected to gazebo master.
        timeout_s = 20
        steps = 10
        for step in range(steps):
            with open(self.log_filename, 'r') as f:
                for line in f.readlines():
                    if 'Connected to gazebo master' in line:
                        return True
            time.sleep(float(timeout_s)/float(steps))

        print("gzserver did not connect within {}s"
              .format(timeout_s))
        return False


class GzmodelspawnRunner(Runner):
    def __init__(self,
                 workspace_dir: str,
                 log_dir: str,
                 model: str,
                 case: str,
                 verbose: bool,
                 build_dir: str):
        super().__init__(log_dir, model, case, verbose)
        self.name = "gzmodelspawn"
        self.cwd = workspace_dir
        self.env["GAZEBO_PLUGIN_PATH"] = \
            os.path.join(workspace_dir, build_dir, "build_gazebo")
        self.env["GAZEBO_MODEL_PATH"] = \
            os.path.join(workspace_dir, "Tools/sitl_gazebo/models")
        self.cmd = "gz"

        if os.path.isfile(os.path.join(workspace_dir,
                                       "Tools/sitl_gazebo/models",
                                       self.model, self.model + ".sdf")):
            model_path = os.path.join(workspace_dir,
                                      "Tools/sitl_gazebo/models",
                                      self.model, self.model + ".sdf")
        elif os.path.isfile(os.path.join(workspace_dir,
                                         "Tools/sitl_gazebo/models",
                                         self.model, self.model + "-gen.sdf")):
            model_path = os.path.join(workspace_dir,
                                      "Tools/sitl_gazebo/models",
                                      self.model, self.model + "-gen.sdf")
        else:
            raise Exception("Model not found")

        self.cmd = "stdbuf"
        self.args = ["-o0", "-e0",
                     "gz", "model",
                     "--verbose",
                     "--spawn-file", model_path,
                     "--model-name", self.model,
                     "-x", "1.01", "-y", "0.98", "-z", "0.83"]

    def has_started_ok(self) -> bool:
        # The problem is that sometimes gzserver does not seem to start
        # quickly enough and gz model spawn fails with the error:
        # "An instance of Gazebo is not running." but still returns 0
        # as a result.
        # We work around this by trying to start and then check whether
        # using has_started_ok() whether it was successful or not.
        timeout_s = 20
        steps = 10
        for _ in range(steps):
            returncode = self.process.poll()
            if returncode is None:
                time.sleep(float(timeout_s)/float(steps))
                continue

            with open(self.log_filename, 'r') as f:
                for line in f.readlines():
                    if 'An instance of Gazebo is not running' in line:
                        return False
                else:
                    return True

        print("gzmodelspawn did not return within {}s"
              .format(timeout_s))
        return False


class GzclientRunner(Runner):
    def __init__(self,
                 workspace_dir: str,
                 log_dir: str,
                 model: str,
                 case: str,
                 verbose: bool):
        super().__init__(log_dir, model, case, verbose)
        self.name = "gzclient"
        self.cwd = workspace_dir
        self.env = dict(os.environ, **{
            "GAZEBO_MODEL_PATH": os.path.join(workspace_dir,
                                              "Tools/sitl_gazebo/models")})
        self.cmd = "gzclient"
        self.args = ["--verbose"]


class TestRunner(Runner):
    def __init__(self,
                 workspace_dir: str,
                 log_dir: str,
                 model: str,
                 case: str,
                 mavlink_connection: str,
                 speed_factor: float,
                 verbose: bool,
                 build_dir: str):
        super().__init__(log_dir, model, case, verbose)
        self.name = "mavsdk_tests"
        self.cwd = workspace_dir
        self.cmd = "nice"
        self.args = ["-5",
                     os.path.join(
                         workspace_dir,
                         build_dir,
                         "mavsdk_tests/mavsdk_tests"),
                     "--url", mavlink_connection,
                     "--speed-factor", str(speed_factor),
                     case]
