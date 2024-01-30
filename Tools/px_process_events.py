#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2020 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#
# PX4 events processor (main executable file)
#
# This tool scans the PX4 source code for definitions of events.
#

import sys
import os
import argparse
from px4events import srcscanner, srcparser, jsonout

import re
import codecs


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Process events definitions.")
    parser.add_argument("-s", "--src-path",
                        default=["../src"],
                        metavar="PATH",
                        nargs='*',
                        help="one or more paths/files to source files to scan for events")
    parser.add_argument("-b", "--base-path",
                        default="",
                        metavar="PATH",
                        help="path prefix for everything passed with --src-path")
    parser.add_argument("-j", "--json",
                        nargs='?',
                        const="events.json",
                        metavar="FILENAME",
                        help="Create Json output file"
                             " (default FILENAME: events.json)")
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        help="verbose output")

    args = parser.parse_args()

    # Check for valid command
    if not (args.json):
        print("Error: You need to specify at least one output method!")
        parser.print_usage()
        sys.exit(1)

    # Initialize source scanner and parser
    scanner = srcscanner.SourceScanner()
    parser = srcparser.SourceParser()

    # Scan directories, and parse the files
    if args.verbose:
        print("Scanning source path/files " + str(args.src_path))

    # canonicalize + remove duplicates
    src_paths = set()
    for path in args.src_path:
        src_paths.add(os.path.realpath(os.path.join(args.base_path, path)))

    if not scanner.ScanDir(src_paths, parser):
        sys.exit(1)

    events = parser.events

    # Output to JSON file
    if args.json:
        if args.verbose: print("Creating Json file " + args.json)
        cur_dir = os.path.dirname(os.path.realpath(__file__))
        out = jsonout.JsonOutput(events)
        out.save(args.json)

if __name__ == "__main__":
    main()
