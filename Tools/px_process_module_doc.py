#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2017 PX4 Development Team. All rights reserved.
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
# PX4 module documentation processor (main executable file)
#
# This tool scans the PX4 source code for declarations of module documentations
# in the form PRINT_MODULE_* and converts them into Mardown output
#

from __future__ import print_function
import sys
import os
import argparse
from px4moduledoc import srcscanner, srcparser, markdownout

import re
import json
import codecs

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Process module documentation.")
    parser.add_argument("-s", "--src-path",
                        default=["../src"],
                        metavar="PATH",
                        nargs='*',
                        help="one or more paths to source files to scan for parameters")
    parser.add_argument("-m", "--markdown",
                        nargs='?',
                        const=".",
                        metavar="DIRECTORY",
                        help="Markdown output directory"
                             " (default DIRECTORY: .)")
    parser.add_argument('--no-validation', action='store_true', help="do not fail if consistency checks fail")
    parser.add_argument('-v', '--verbose', action='store_true', help="verbose output")

    args = parser.parse_args()

    # Check for valid command
    if not (args.markdown):
        print("Error: You need to specify at least one output method!")
        parser.print_usage()
        sys.exit(1)

    # Initialize source scanner and parser
    scanner = srcscanner.SourceScanner()
    parser = srcparser.SourceParser()

    # Scan directories, and parse the files
    if (args.verbose): print("Scanning source path " + str(args.src_path))

    if not scanner.ScanDir(args.src_path, parser):
        sys.exit(1)

    if not args.no_validation and parser.HasValidationFailure():
        print("Error: validation failed")
        sys.exit(1)

    module_groups = parser.GetModuleGroups()


    # Output to Markdown/HTML tables
    if args.markdown:
        if args.verbose: print("Creating markdown output to directory " + str(args.markdown))
        if not os.path.exists(args.markdown):
            os.makedirs(args.markdown)
        out = markdownout.MarkdownOutput(module_groups)
        out.Save(args.markdown)


if __name__ == "__main__":
    main()
