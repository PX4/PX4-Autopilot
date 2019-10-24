#!/usr/bin/env python3

import datetime
import os
import subprocess
import time

test_matrix = [
    {
        "model": "iris",
        "test_filter": "[multicopter]",
        "timeout_min": 10,
    },
    {
        "model": "standard_vtol",
        "test_filter": "[vtol]",
        "timeout_min": 10,
    }
]


class Runner:
    def __init__(self):
        self.cmd = ""
        self.cwd = None
        self.args = []
        self.env = {}
        self.log_prefix = ""

    def run(self, group):
        with open(
            "log-{}-{}-{}-{}.txt".format(
                self.log_prefix,
                group['model'],
                group['test_filter'],
                datetime.datetime.now().strftime("%Y-%m-%dT%H-%M-%SZ")
                ), 'w') as f:

            print("Running: {}".format(" ".join([self.cmd] + self.args)))
            print("Raw: {}".format([self.cmd] + self.args))

            self.process = subprocess.Popen(
                [self.cmd] + self.args,
                cwd=self.cwd,
                env=self.env,
                stdout=f, stderr=f
            )

            # self.process.wait(timeout=None)
            # print("Result is {}".format(self.process.returncode))


class Px4Runner(Runner):
    def __init__(self, workspace_dir):
        super().__init__()
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
    def __init__(self, workspace_dir):
        super().__init__()
        self.env = {"PATH": os.environ['PATH'],
                    "HOME": os.environ['HOME'],
                    "GAZEBO_PLUGIN_PATH":
                    workspace_dir + "/build/px4_sitl_default/build_gazebo",
                    "GAZEBO_MODEL_PATH":
                    workspace_dir + "/Tools/sitl_gazebo/models",
                    "PX4_SIM_SPEED_FACTOR": "1"}
        self.cmd = "gzserver"
        self.args = ["--verbose",
                     workspace_dir + "/Tools/sitl_gazebo/worlds/iris.world"]
        self.log_prefix = "gazebo"


def main():
    for group in test_matrix:
        print("Running test group for '{}' with filter '{}'"
              .format(group['model'], group['test_filter']))

        px4_runner = Px4Runner(os.getcwd())
        px4_runner.run(group)

        gazebo_runner = GazeboRunner(os.getcwd())
        gazebo_runner.run(group)

        # Run test here
        time.sleep(30)


if __name__ == '__main__':
    main()
