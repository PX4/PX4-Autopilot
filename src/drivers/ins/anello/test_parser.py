#!/usr/bin/env python3
"""Compile and run the standalone decoder tests using a configured PX4 build."""
import argparse
import re
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--build", default="build/px4_sitl_default")
args = parser.parse_args()
root = Path(__file__).resolve().parents[4]
build = root / args.build
# Use the root dialect: inherited command enums may be emitted only there.
dialect = re.search(r'#define CONFIG_MAVLINK_DIALECT "([^"]+)"',
                    (build / "px4_boardconfig.h").read_text()).group(1)
header = build / "mavlink" / dialect / "mavlink.h"
if not header.exists():
    raise SystemExit("Build PX4 first to generate the MAVLink headers")
with tempfile.TemporaryDirectory(prefix="anello-test-") as directory:
    binary = Path(directory) / "anello_parser_test"
    subprocess.run([
        "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
        "-Wno-address-of-packed-member", "-fsanitize=address,undefined",
        "-fno-omit-frame-pointer", "-g",
        "-D__PX4_POSIX", "-D__PX4_LINUX",
        "-I" + str(root / "src/lib"),
        "-I" + str(root / "platforms/common/include"),
        "-I" + str(build),
        "-I" + str(header.parent),
        str(Path(__file__).with_name("AnelloParserTest.cpp")),
        "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
