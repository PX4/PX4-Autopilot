#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2014 PX4 Development Team. All rights reserved.
#   Author: Julian Oes <joes@student.ethz.ch>

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


"""
px_romfs_pruner.py:
Delete all comments and newlines before ROMFS is converted to an image
"""

from __future__ import print_function
import argparse
import os


def main():

        # Parse commandline arguments
        parser = argparse.ArgumentParser(description="ROMFS pruner.")
        parser.add_argument('--folder', action="store", help="ROMFS scratch folder.")
        args = parser.parse_args()

        print("Pruning ROMFS files.")

        # go through
        for (root, dirs, files) in os.walk(args.folder):
                for file in files:
                        # only prune text files
                        if ".zip" in file or ".bin" in file or ".swp" in file or ".data" in file:
                                continue

                        file_path = os.path.join(root, file)

                        # read file line by line
                        pruned_content = ""
                        with open(file_path, "r") as f:
                                for line in f:

                                        # handle mixer files differently than startup files
                                        if file_path.endswith(".mix"):
                                                if line.startswith(("Z:", "M:", "R: ", "O:", "S:")):
                                                        pruned_content += line
                                        else:
                                                if not line.isspace() and not line.strip().startswith("#"):
                                                        pruned_content += line

                        # overwrite old scratch file
                        with open(file_path, "w") as f:
                                f.write(pruned_content)


if __name__ == '__main__':
        main()
