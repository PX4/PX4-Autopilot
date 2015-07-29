#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2013-2015 PX4 Development Team. All rights reserved.
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
# PX4 airframe config processor (main executable file)
#
# This tool scans the PX4 source code for declarations of airframes
#
# Currently supported formats are:
#   * XML for the parametric UI generator
#
#

from __future__ import print_function
import sys
import os
import argparse
from px4airframes import srcscanner, srcparser, xmlout

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Process airframe documentation.")
    parser.add_argument("-a", "--airframes-path",
                        default="../ROMFS/px4fmu_common",
                        metavar="PATH",
                        help="path to source files to scan for parameters")
    parser.add_argument("-x", "--xml",
                        nargs='?',
                        const="airframes.xml",
                        metavar="FILENAME",
                        help="Create XML file"
                             " (default FILENAME: airframes.xml)")
    parser.add_argument("-b", "--board",
                         nargs='?',
                         const="",
                         metavar="BOARD",
                         help="Board to create airframes xml for")
    args = parser.parse_args()

    # Check for valid command
    if not (args.xml):
        print("Error: You need to specify at least one output method!\n")
        parser.print_usage()
        sys.exit(1)

    # Initialize source scanner and parser
    scanner = srcscanner.SourceScanner()
    parser = srcparser.SourceParser()

    # Scan directories, and parse the files
    print("Scanning source path " + args.airframes_path)
    if not scanner.ScanDir(args.airframes_path, parser):
        sys.exit(1)
    # We can't validate yet
    # if not parser.Validate():
    #     sys.exit(1)
    param_groups = parser.GetParamGroups()

    # Output to XML file
    if args.xml:
        print("Creating XML file " + args.xml)
        out = xmlout.XMLOutput(param_groups, args.board)
        out.Save(args.xml)

    print("All done!")


if __name__ == "__main__":
    main()
