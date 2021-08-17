#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
############################################################################
#
#   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

"""

import os
import glob
import kconfiglib
import tempfile
import sys

import diffconfig
import merge_config

px4_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))

for name in glob.glob(px4_dir + '/boards/*/*/default.px4board'):
    kconf = kconfiglib.Kconfig()
    kconf.load_config(name)
    print(kconf.write_min_config(name))

for name in glob.glob(px4_dir + '/boards/*/*/bootloader.px4board'):
    kconf = kconfiglib.Kconfig()
    kconf.load_config(name)
    print(kconf.write_min_config(name))

for name in glob.glob(px4_dir + '/boards/*/*/*.px4board'):
    if(os.path.basename(name) != "default.px4board" and os.path.basename(name) != "bootloader.px4board"):
        board_default = os.path.dirname(name) + "/default.px4board";

        # Merge with default config
        kconf = merge_config.main(px4_dir + "/Kconfig", board_default, name)
        tf = tempfile.NamedTemporaryFile()

        # Save minconfig
        kconf.write_min_config(tf.name)

        # Diff with default config and save to label.px4board
        stdoutpipe = sys.stdout
        sys.stdout = open(name, "w")
        diffconfig.main(1, board_default, tf.name)
        sys.stdout.close()
        sys.stdout = stdoutpipe
