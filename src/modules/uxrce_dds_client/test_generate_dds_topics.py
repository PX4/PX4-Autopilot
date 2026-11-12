#!/usr/bin/env python3
"""Tests of generate_dds_topics.py with more than one topics file."""

import os
import subprocess
import sys
import tempfile
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
GENERATOR = os.path.join(HERE, "generate_dds_topics.py")
TEMPLATE = os.path.join(HERE, "dds_topics.h.em")
PX4_TOPICS = os.path.join(HERE, "dds_topics.yaml")
LISTS = ("publications", "subscriptions", "subscriptions_multi")
OUT_OF_TREE = {
    "publications": [
        {"topic": "/fmu/out/my_message", "type": "px4_msgs::msg::MyMessage"},
    ],
    "subscriptions": [
        {"topic": "/fmu/in/my_command", "type": "px4_msgs::msg::MyCommand"},
    ],
}


class GenerateDdsTopicsTest(unittest.TestCase):

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = directory.name
        with open(PX4_TOPICS) as f:
            self.px4 = yaml.safe_load(f)

    def write(self, topics):
        handle, path = tempfile.mkstemp(suffix=".yaml", dir=self.directory)
        with os.fdopen(handle, "w") as f:
            if isinstance(topics, str):
                f.write(topics)
            else:
                yaml.safe_dump(topics, f, sort_keys=False)
        return path

    def generate(self, *topics_files):
        out = tempfile.mkdtemp(dir=self.directory)
        command = [sys.executable, GENERATOR, "--client-outdir", out,
                   "--template_file", TEMPLATE]
        for topics_file in topics_files:
            command += ["--dds-topics-file", topics_file]
        result = subprocess.run(command, capture_output=True, text=True)
        header = os.path.join(out, "dds_topics.h")
        if not os.path.exists(header):
            return result, None
        with open(header) as f:
            return result, f.read()

    def test_a_second_file_appends_its_topics_to_each_list(self):
        joined = {key: (self.px4.get(key) or []) + OUT_OF_TREE.get(key, [])
                  for key in LISTS}
        _, expected = self.generate(self.write(joined))
        result, header = self.generate(PX4_TOPICS, self.write(OUT_OF_TREE))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('"/fmu/out/my_message"', header)
        self.assertIn('"/fmu/in/my_command"', header)
        self.assertEqual(header, expected)

    def test_lists_split_across_files_generate_the_same_header(self):
        publications = {"publications": self.px4["publications"]}
        subscriptions = {key: self.px4[key] for key in LISTS[1:]}
        _, expected = self.generate(PX4_TOPICS)
        result, header = self.generate(self.write(publications),
                                       self.write(subscriptions))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(header, expected)

    def test_files_without_topics_add_nothing(self):
        _, expected = self.generate(PX4_TOPICS)
        result, header = self.generate(PX4_TOPICS, self.write(""),
                                       self.write("publications:\n"))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(header, expected)

    def test_a_topic_declared_twice_stops_the_generator(self):
        again = {"publications": self.px4["publications"][:1]}
        topic = again["publications"][0]["topic"]
        result, header = self.generate(PX4_TOPICS, self.write(again))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(f"publications: {topic} declared more than once",
                      result.stderr)
        self.assertIsNone(header)


if __name__ == "__main__":
    unittest.main()
