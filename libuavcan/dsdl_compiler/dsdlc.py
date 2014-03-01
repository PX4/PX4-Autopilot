#!/usr/bin/env python3
#
# UAVCAN DSDL compiler for libuavcan
#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

import sys, os

RUNNING_FROM_SRC_DIR = os.path.abspath(__file__).endswith(os.path.join('libuavcan', 'dsdl_compiler', 'dsdlc.py'))
if RUNNING_FROM_SRC_DIR:
    print('Running from the source directory')
    scriptdir = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(os.path.join(scriptdir, '..', '..', 'pyuavcan'))

from pyuavcan import dsdl

print('Hello')
