#!/usr/bin/env python
from __future__ import print_function

import sys
import subprocess

filename = sys.argv[1]

try:
    fp_header = open(filename, 'r')
    old_header = fp_header.read()
except:
    old_header = ''

git_tag = subprocess.check_output('git describe --always --tags'.split(),
                                  stderr=subprocess.STDOUT).decode('utf-8').strip()
git_version = subprocess.check_output('git rev-parse --verify HEAD'.split(),
                                      stderr=subprocess.STDOUT).decode('utf-8').strip()
git_version_short = git_version[0:16]

# Generate the header file content
header = """
/* Auto Magically Generated file */
/* Do not edit! */
#pragma once
#define PX4_GIT_VERSION_STR  "{git_version}"
#define PX4_GIT_VERSION_BINARY 0x{git_version_short}
#define PX4_GIT_TAG_STR  "{git_tag}"
""".format(git_tag=git_tag,
           git_version=git_version,
           git_version_short=git_version_short)

if old_header != header:
    print('Updating header {}'.format(sys.argv[1]))
    fp_header = open(filename, 'w')
    fp_header.write(header)
