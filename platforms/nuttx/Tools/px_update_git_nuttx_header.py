#!/usr/bin/env python
from __future__ import print_function

import sys
import subprocess
import re

filename = sys.argv[1]
px4_source_dir = sys.argv[2] 

try:
    fp_header = open(filename, 'r')
    old_header = fp_header.read()
except:
    old_header = ''

nuttx_git_tag = subprocess.check_output('git describe --always --tags --match nuttx-*  --dirty'.split(),
                                  cwd=px4_source_dir+'/platforms/nuttx/NuttX/nuttx', stderr=subprocess.STDOUT).decode('utf-8').strip().replace("nuttx-","v")
nuttx_git_tag = re.sub('-.*','.0',nuttx_git_tag)
nuttx_git_version = subprocess.check_output('git rev-parse --verify HEAD'.split(),
                                      cwd=px4_source_dir+'/platforms/nuttx/NuttX/nuttx', stderr=subprocess.STDOUT).decode('utf-8').strip()
nuttx_git_version_short = nuttx_git_version[0:16]

# Generate the header file content
header = """
/* Auto Magically Generated file */
/* Do not edit! */
#pragma once
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
