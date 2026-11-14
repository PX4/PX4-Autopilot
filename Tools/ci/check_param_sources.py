#!/usr/bin/env python3
"""Check that every parameter definition reaches the parameter reference.

`make parameters_metadata` (cmake/metadata.cmake) builds the full parameter
reference from:
- every .yaml file under src/, except the ones listed in
  cmake/metadata_param_excludes.txt, and
- the .c and .h files in the directories that `find src -maxdepth 4`
  lists (px_process_params.py does not recurse into subdirectories).

A file outside those sets is left out of the reference without any error.
This check fails when a tracked file in src/ or boards/ defines parameters
but is neither read nor excluded, or when the exclude list names a file
that does not exist.

Usage:
  check_param_sources.py [--root <repository root>]

No dependencies beyond the standard library and git.
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path
from typing import Callable, Iterable, List

EXCLUDES_FILE = "cmake/metadata_param_excludes.txt"

# Must match the `find ${PX4_SOURCE_DIR}/src -maxdepth 4 -type d` calls in
# cmake/metadata.cmake.
C_SCAN_MAX_DEPTH = 4

# Top-level keys that make Tools/module_config/generate_params.py or
# Tools/serial/generate_config.py generate parameters.
YAML_PARAM_RE = re.compile(
    r'^(parameters|actuator_output|serial_config)\s*:', re.MULTILINE
)

# Same definitions src/lib/parameters/px4params/srcparser.py reads.
C_PARAM_RE = re.compile(
    r'(PX4_)?PARAM_DEFINE_[A-Z_][A-Z0-9_]*\s*\([^;]*\)\s*;'
)

SCANNED_DIRS = ("src", "boards")


def parse_excludes(text: str) -> List[str]:
    """Return the paths in an exclude list, without comments."""
    paths = []
    for line in text.splitlines():
        path = line.split("#", 1)[0].strip()
        if path:
            paths.append(path)
    return paths


def is_yaml(path: str) -> bool:
    return path.endswith((".yaml", ".yml"))


def is_c_source(path: str) -> bool:
    return path.endswith((".c", ".h"))


def c_file_is_scanned(path: str) -> bool:
    parts = path.split("/")
    # parts[0] is "src", parts[-1] the file name
    return parts[0] == "src" and len(parts) - 2 <= C_SCAN_MAX_DEPTH


def find_problems(files: Iterable[str],
                  read_text: Callable[[str], str],
                  excludes: List[str],
                  exists: Callable[[str], bool]) -> List[str]:
    """Return one message per problem; an empty list means the check passes.

    files: repository-relative paths, with forward slashes.
    """
    problems = []
    excluded = set(excludes)

    for path in excludes:
        if not exists(path):
            problems.append(
                f"{EXCLUDES_FILE}: {path} does not exist, remove it")
        elif not is_yaml(path):
            problems.append(
                f"{EXCLUDES_FILE}: {path} is not a yaml file; only yaml "
                "files can be excluded")

    for path in sorted(files):
        if is_yaml(path):
            if path in excluded:
                continue
            if not YAML_PARAM_RE.search(read_text(path)):
                continue
            # file(GLOB_RECURSE ... src/*.yaml) reads only .yaml under src/
            if not (path.startswith("src/") and path.endswith(".yaml")):
                problems.append(
                    f"{path} defines parameters, but only .yaml files "
                    f"under src/ are read: move or rename it, or add it to "
                    f"{EXCLUDES_FILE} with the reason")
        elif is_c_source(path):
            if c_file_is_scanned(path):
                continue
            if not C_PARAM_RE.search(read_text(path)):
                continue
            problems.append(
                f"{path} defines parameters, but only .c and .h files "
                f"at most {C_SCAN_MAX_DEPTH} directories below src/ are "
                "read: move it")

    return problems


def tracked_files(root: Path) -> List[str]:
    out = subprocess.run(
        ["git", "ls-files", "-z", "--", *SCANNED_DIRS],
        cwd=root, check=True, stdout=subprocess.PIPE,
    ).stdout.decode("utf-8")
    return [f for f in out.split("\0") if f]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--root", type=Path,
        default=Path(__file__).resolve().parents[2],
        help="repository root (default: this script's repository)")
    args = parser.parse_args()
    root: Path = args.root

    excludes = parse_excludes(
        (root / EXCLUDES_FILE).read_text(encoding="utf-8"))
    files = tracked_files(root)

    def read_text(path: str) -> str:
        return (root / path).read_text(encoding="utf-8", errors="replace")

    def exists(path: str) -> bool:
        return (root / path).is_file()

    problems = find_problems(files, read_text, excludes, exists)
    for problem in problems:
        print(problem)
    if problems:
        print(f"{len(problems)} problem(s) found")
        return 1
    print(f"OK: {len(files)} files in {', '.join(SCANNED_DIRS)} checked, "
          f"{len(excludes)} excluded")
    return 0


if __name__ == "__main__":
    sys.exit(main())
