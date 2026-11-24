#!/usr/bin/env python3
"""Tests of copy_to_ros_ws.sh with an external modules location."""

import os
import shutil
import subprocess
import tempfile
import unittest

TOOLS = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(TOOLS, "copy_to_ros_ws.sh")
MSG = os.path.join(TOOLS, "..", "msg")
VERSIONED = os.path.join(MSG, "versioned")


def messages(directory):
    return sorted(n for n in os.listdir(directory) if n.endswith(".msg"))


class CopyToRosWsTest(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)
        self.ws = os.path.join(self.tmp, "ws")
        # an existing px4_msgs keeps the script from cloning it
        self.px4_msgs = os.path.join(self.ws, "src", "px4_msgs")
        for directory in ("msg", "srv"):
            os.makedirs(os.path.join(self.px4_msgs, directory))
        self.px4 = set(messages(MSG)) | set(messages(VERSIONED))

    def location(self, *files):
        path = tempfile.mkdtemp(dir=self.tmp)
        for name in files:
            os.makedirs(os.path.join(path, "msg"), exist_ok=True)
            with open(os.path.join(path, "msg", name), "w") as f:
                f.write("uint64 timestamp\n")
        return path

    def copy(self, *location):
        return subprocess.run(["bash", SCRIPT, self.ws, *location],
                              capture_output=True, text=True)

    def copied(self):
        return set(os.listdir(os.path.join(self.px4_msgs, "msg")))

    def test_px4_messages_without_a_location(self):
        result = self.copy()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.copied(), self.px4)

    def test_a_location_adds_its_messages(self):
        result = self.copy(self.location("MyMessage.msg"))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.copied(), self.px4 | {"MyMessage.msg"})

    def test_a_location_without_messages_adds_none(self):
        result = self.copy(self.location("dds_topics.yaml"))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.copied(), self.px4)

    def test_a_px4_message_name_is_refused_before_copying(self):
        for name in (messages(MSG)[0], messages(VERSIONED)[0]):
            with self.subTest(name=name):
                result = self.copy(self.location("MyMessage.msg", name))
                self.assertEqual(result.returncode, 1)
                self.assertIn(f"{name} has the name of a PX4 message",
                              result.stdout)
                self.assertEqual(os.listdir(os.path.join(self.ws, "src")),
                                 ["px4_msgs"])
                self.assertEqual(self.copied(), set())

    def test_a_location_without_a_msg_directory_is_refused(self):
        result = self.copy(self.location())
        self.assertEqual(result.returncode, 1)
        self.assertIn("'msg' directory not found", result.stdout)
        self.assertEqual(os.listdir(os.path.join(self.ws, "src")),
                         ["px4_msgs"])


if __name__ == "__main__":
    unittest.main()
