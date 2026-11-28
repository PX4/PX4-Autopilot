#!/usr/bin/env python3
"""Unit tests for check_param_sources.py."""

import unittest
from typing import Dict, List

from check_param_sources import EXCLUDES_FILE, find_problems, parse_excludes

PARAM_YAML = "module_name: x\nparameters:\n- group: X\n"
OUTPUT_YAML = "module_name: x\nactuator_output:\n  output_groups: []\n"
OTHER_YAML = "topics:\n- topic: /fmu/out/x\n"
PARAM_C = "PARAM_DEFINE_INT32(X_PARAM, 0);\n"
OTHER_C = "int x = 0;\n"


def check(files: Dict[str, str], excludes: List[str]) -> List[str]:
    return find_problems(files, files.__getitem__, excludes,
                         files.__contains__)


class ParseExcludesTest(unittest.TestCase):
    def test_strips_comments_and_blank_lines(self) -> None:
        text = ("# header\n\n"
                "src/a/module.yaml  # reason\n"
                "  boards/b/params.yaml\n")
        self.assertEqual(parse_excludes(text),
                         ["src/a/module.yaml", "boards/b/params.yaml"])


class FindProblemsTest(unittest.TestCase):
    def test_yaml_under_src_is_read(self) -> None:
        self.assertEqual(check({"src/modules/a/module.yaml": PARAM_YAML},
                               []), [])

    def test_yaml_outside_src_must_be_excluded(self) -> None:
        files = {"boards/v/b/x/parameters.yaml": OUTPUT_YAML}
        problems = check(files, [])
        self.assertEqual(len(problems), 1)
        self.assertIn("boards/v/b/x/parameters.yaml", problems[0])
        self.assertEqual(check(files, ["boards/v/b/x/parameters.yaml"]), [])

    def test_yml_under_src_is_not_read(self) -> None:
        files = {"src/modules/a/module.yml": PARAM_YAML}
        problems = check(files, [])
        self.assertEqual(len(problems), 1)
        self.assertIn("src/modules/a/module.yml", problems[0])
        self.assertIn("only .yaml files", problems[0])
        self.assertEqual(check(files, ["src/modules/a/module.yml"]), [])

    def test_yaml_without_parameters_is_ignored(self) -> None:
        self.assertEqual(check({"boards/v/b/topics.yaml": OTHER_YAML}, []),
                         [])

    def test_missing_exclude_fails(self) -> None:
        problems = check({}, ["src/gone/module.yaml"])
        self.assertEqual(len(problems), 1)
        self.assertIn(EXCLUDES_FILE, problems[0])
        self.assertIn("src/gone/module.yaml", problems[0])

    def test_only_yaml_can_be_excluded(self) -> None:
        problems = check({"src/a/params.c": PARAM_C}, ["src/a/params.c"])
        self.assertEqual(len(problems), 1)
        self.assertIn("only yaml", problems[0])

    def test_c_file_depth(self) -> None:
        self.assertEqual(check({"src/a/b/c/d/params.c": PARAM_C}, []), [])
        problems = check({"src/a/b/c/d/e/params.c": PARAM_C}, [])
        self.assertEqual(len(problems), 1)
        self.assertIn("src/a/b/c/d/e/params.c", problems[0])

    def test_c_file_outside_src(self) -> None:
        self.assertEqual(len(check({"boards/v/b/src/params.c": PARAM_C},
                                   [])), 1)
        self.assertEqual(check({"boards/v/b/src/board.c": OTHER_C}, []), [])


if __name__ == "__main__":
    unittest.main()
