#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2013-2017 PX4 Development Team. All rights reserved.
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
# This tool scans the PX4 ROMFS directory for declarations of airframes
#
# Currently supported output formats are:
#   * XML for the parametric UI generator (Used in QGC)
#   * Markdown for the PX4 User guide (https://github.com/PX4/PX4-user_guide)
#

from __future__ import print_function
import sys
import os
import argparse
from px4airframes import srcscanner, srcparser, xmlout, rcout, markdownout

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
    parser.add_argument("-m", "--markdown",
                        nargs='?',
                        const="airframes.md",
                        metavar="FILENAME",
                        help="Create Markdown file"
                             " (default FILENAME: airframes.md)")
    default_image_path = '../../assets/airframes/types'
    parser.add_argument("-i", "--image-path",
                        default=default_image_path,
                        metavar="IMAGEPATH",
                        help="HTML image path for Markdown (containing the airframe svg files)"
                             " (default IMAGEPATH: "+default_image_path+")")
    parser.add_argument("-s", "--start-script",
                        nargs='?',
                        const="rc.autostart",
                        metavar="FILENAME",
                        help="Create start script file")
    parser.add_argument("-b", "--board",
                         nargs='?',
                         const="",
                         metavar="BOARD",
                         help="Board to create airframes xml for")
    parser.add_argument('-v', '--verbose', action='store_true', help="verbose output")
    args = parser.parse_args()

    # Check for valid command
    if not (args.xml) and not (args.start_script) and not args.markdown:
        print("Error: You need to specify at least one output method!\n")
        parser.print_usage()
        sys.exit(1)

    # Initialize source scanner and parser
    scanner = srcscanner.SourceScanner()
    parser = srcparser.SourceParser()

    # Scan directories, and parse the files
    if args.verbose: print("Scanning source path " + args.airframes_path)
    if not scanner.ScanDir(args.airframes_path, parser):
        sys.exit(1)
    # We can't validate yet
    # if not parser.Validate():
    #     sys.exit(1)
    airframe_groups = parser.GetAirframeGroups()

    # Output to XML file
    if args.xml:
        if args.verbose: print("Creating XML file " + args.xml)
        out = xmlout.XMLOutput(airframe_groups, args.board)
        out.Save(args.xml)

    # Output to markdown file
    if args.markdown:
        if args.verbose: print("Creating markdown file " + args.markdown)
        out = markdownout.MarkdownTablesOutput(airframe_groups, args.board, args.image_path)
        out.Save(args.markdown)

    # Output to start scripts
    if args.start_script:
        # Airframe start script
        if args.verbose: print("Creating start script " + args.start_script)
        out = rcout.RCOutput(airframe_groups, args.board)
        out.Save(args.start_script)

        # Airframe post-start script
        post_start_script = args.start_script + '.post'
        if args.verbose: print("Creating post-start script " + post_start_script)
        out_post = rcout.RCOutput(airframe_groups, args.board, post_start=True)
        out_post.Save(post_start_script)

    if (args.verbose): print("All done!")


if __name__ == "__main__":
    main()
