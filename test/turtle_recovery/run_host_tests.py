#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Run turtle recovery host unit tests with UndefinedBehaviorSanitizer."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
TESTS = Path(__file__).resolve().parent
SUITES = ("rc", "core", "fault_matrix", "mapping", "interlock", "lifecycle")


def main() -> None:
    compiler = os.environ.get("CXX", "g++")
    if not shutil.which(compiler):
        raise SystemExit(f"C++ compiler not found: {compiler}")

    with tempfile.TemporaryDirectory(prefix="px4-turtle-tests-") as directory:
        for suite in SUITES:
            executable = Path(directory) / suite
            sources = [TESTS / f"test_{suite}.cpp"]
            if suite == "interlock":
                sources.append(TESTS / "interlock_other.cpp")

            command = [
                compiler, "-std=c++14", "-O2", "-Wall", "-Wextra", "-Werror",
                "-UNDEBUG", "-fsanitize=undefined",
                "-fno-sanitize-recover=all",
                "-pthread", f"-I{ROOT / 'src/drivers/dshot'}",
                f"-I{ROOT / 'src'}",
                *(str(source) for source in sources), "-o", str(executable),
            ]
            subprocess.run(command, check=True)
            subprocess.run([str(executable)], check=True)

    print("PASS: all turtle_recovery host unit tests passed (UBSan)")


if __name__ == "__main__":
    main()
