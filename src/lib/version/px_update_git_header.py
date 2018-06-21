#!/usr/bin/env python
from __future__ import print_function

import os
import sys
import subprocess
import re

filename = sys.argv[1]

try:
    fp_header = open(filename, 'r')
    old_header = fp_header.read()
except:
    old_header = ''


# Generate the header file content
header = """
/* Auto Magically Generated file */
/* Do not edit! */
#pragma once
"""


# PX4
git_tag = subprocess.check_output('git describe --always --tags --dirty'.split(),
                                  stderr=subprocess.STDOUT).decode('utf-8').strip()
git_version = subprocess.check_output('git rev-parse --verify HEAD'.split(),
                                      stderr=subprocess.STDOUT).decode('utf-8').strip()
try:
    git_branch_name = subprocess.check_output('git symbolic-ref -q --short HEAD'.split(),
                                          stderr=subprocess.STDOUT).decode('utf-8').strip()
except:
    git_branch_name = ''
git_version_short = git_version[0:16]

header += """
#define PX4_GIT_VERSION_STR  "{git_version}"
#define PX4_GIT_VERSION_BINARY 0x{git_version_short}
#define PX4_GIT_TAG_STR  "{git_tag}"
#define PX4_GIT_BRANCH_NAME  "{git_branch_name}"
""".format(git_tag=git_tag,
           git_version=git_version,
           git_version_short=git_version_short,
           git_branch_name=git_branch_name)


# ECL
if (os.path.exists('src/lib/ecl/.git')):
    ecl_git_tag = subprocess.check_output('git describe --always --tags --dirty'.split(),
                                  cwd='src/lib/ecl', stderr=subprocess.STDOUT).decode('utf-8')

    ecl_git_version = subprocess.check_output('git rev-parse --verify HEAD'.split(),
                                      cwd='src/lib/ecl', stderr=subprocess.STDOUT).decode('utf-8').strip()
    ecl_git_version_short = ecl_git_version[0:16]

    header += """
#define ECL_LIB_GIT_VERSION_STR  "{ecl_git_version}"
#define ECL_LIB_GIT_VERSION_BINARY 0x{ecl_git_version_short}
""".format(ecl_git_version=ecl_git_version,
           ecl_git_version_short=ecl_git_version_short)


# Mavlink
if (os.path.exists('mavlink/include/mavlink/v2.0/.git')):
    mavlink_git_version = subprocess.check_output('git rev-parse --verify HEAD'.split(),
                                      cwd='mavlink/include/mavlink/v2.0', stderr=subprocess.STDOUT).decode('utf-8').strip()
    mavlink_git_version_short = mavlink_git_version[0:16]

    header += """
#define MAVLINK_LIB_GIT_VERSION_STR  "{mavlink_git_version}"
#define MAVLINK_LIB_GIT_VERSION_BINARY 0x{mavlink_git_version_short}
""".format(mavlink_git_version=mavlink_git_version,
           mavlink_git_version_short=mavlink_git_version_short)


# NuttX
if (os.path.exists('platforms/nuttx/NuttX/nuttx/.git')):
    nuttx_git_tag = subprocess.check_output('git describe --always --tags --match nuttx-*  --dirty'.split(),
                                  cwd='platforms/nuttx/NuttX/nuttx', stderr=subprocess.STDOUT).decode('utf-8').strip().replace("nuttx-","v")
    nuttx_git_tag = re.sub('-.*','.0',nuttx_git_tag)
    nuttx_git_version = subprocess.check_output('git rev-parse --verify HEAD'.split(),
                                      cwd='platforms/nuttx/NuttX/nuttx', stderr=subprocess.STDOUT).decode('utf-8').strip()
    nuttx_git_version_short = nuttx_git_version[0:16]

    header += """
#define NUTTX_GIT_VERSION_STR  "{nuttx_git_version}"
#define NUTTX_GIT_VERSION_BINARY 0x{nuttx_git_version_short}
#define NUTTX_GIT_TAG_STR  "{nuttx_git_tag}"
""".format(nuttx_git_version=nuttx_git_version,
           nuttx_git_version_short=nuttx_git_version_short,
           nuttx_git_tag=nuttx_git_tag)


if old_header != header:
    print('Updating header {}'.format(sys.argv[1]))
    fp_header = open(filename, 'w')
    fp_header.write(header)
