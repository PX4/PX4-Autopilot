#!/usr/bin/env python3

import argparse
import datetime
import os
import subprocess
import sys


test_matrix = [
    {
        "model": "iris",
        "test_filter": "[multicopter]",
        "timeout_min": 10,
    },
    # {
    #     "model": "standard_vtol",
    #     "test_filter": "[vtol]",
    #     "timeout_min": 10,
    # }
]


class Runner:
    def __init__(self, log_dir):
        self.cmd = ""
        self.cwd = None
        self.args = []
        self.env = {}
        self.log_prefix = ""
        self.log_dir = log_dir

    def start(self, config):
        if self.log_dir:
            f = open(self.log_dir + os.path.sep +
                     "log-{}-{}-{}-{}.txt".format(
                         self.log_prefix,
                         config['model'],
                         config['test_filter'],
                         datetime.datetime.now().strftime("%Y-%m-%dT%H-%M-%SZ")
                        ), 'w')
        else:
            f = sys.stdout

        print("Running: {}".format(" ".join([self.cmd] + self.args)))

        self.process = subprocess.Popen(
            [self.cmd] + self.args,
            cwd=self.cwd,
            env=self.env,
            stdout=f, stderr=f
        )

    def wait(self, timeout_s):
        try:
            return self.process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            self.stop()

    def stop(self):
        returncode = self.process.poll()
        if returncode is not None:
            return returncode

        print("Sending terminate to {}".format(self.process.pid))
        self.process.terminate()

        try:
            return self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            print("Sending kill to {}".format(self.process.pid))
            self.process.kill()
            return self.process.returncode


class Px4Runner(Runner):
    def __init__(self, workspace_dir, log_dir):
        super().__init__(log_dir)
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
                    "PX4_SIM_MODEL": "iris"}
        self.log_prefix = "px4"


class GazeboRunner(Runner):
    def __init__(self, workspace_dir, log_dir):
        super().__init__(log_dir)
        self.env = {"PATH": os.environ['PATH'],
                    "HOME": os.environ['HOME'],
                    "GAZEBO_PLUGIN_PATH":
                    workspace_dir + "/build/px4_sitl_default/build_gazebo",
                    "GAZEBO_MODEL_PATH":
                    workspace_dir + "/Tools/sitl_gazebo/models",
                    "PX4_SIM_SPEED_FACTOR": "5"}
        self.cmd = "gzserver"
        self.args = ["--verbose",
                     workspace_dir + "/Tools/sitl_gazebo/worlds/iris.world"]
        self.log_prefix = "gazebo"


class TestRunner(Runner):
    def __init__(self, workspace_dir, log_dir, config):
        super().__init__(log_dir)
        self.env = {"PATH": os.environ['PATH']}
        self.cmd = workspace_dir + \
            "/build/px4_sitl_default/test_mission_multicopter"
        self.args = [config['test_filter']]
        self.log_prefix = "test_runner"


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir",
                        help="Directory for log files, stdout if not provided")
    args = parser.parse_args()

    for group in test_matrix:
        print("Running test group for '{}' with filter '{}'"
              .format(group['model'], group['test_filter']))

        px4_runner = Px4Runner(os.getcwd(), args.log_dir)
        px4_runner.start(group)

        gazebo_runner = GazeboRunner(os.getcwd(), args.log_dir)
        gazebo_runner.start(group)

        test_runner = TestRunner(os.getcwd(), args.log_dir, group)
        test_runner.start(group)

        returncode = test_runner.wait(group['timeout_min']*60)
        print("Test exited with {}".format(returncode))

        returncode = gazebo_runner.stop()
        print("Gazebo exited with {}".format(returncode))

        px4_runner.stop()
        print("PX4 exited with {}".format(returncode))


if __name__ == '__main__':
    main()
