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
import tempfile
import sys
from pathlib import Path

import diffconfig
import loadconfig
import merge_config

px4_dir = loadconfig.PX4_ROOT

loadconfig.ensure_env()
targets = list(loadconfig.enumerate_targets())

for target in targets:
    if target.label != 'default':
        continue
    kconf = loadconfig.load_kconfig()
    kconf.load_config(target.path)
    print(kconf.write_min_config(target.path))

    defconfig_path = Path(target.path).parent / "nuttx-config" / "nsh" / "defconfig"

    if os.path.exists(defconfig_path):
        # Merge NuttX with default config
        kconf = merge_config.main(px4_dir + "/Kconfig", target.path, defconfig_path)
        print(kconf.write_min_config(target.path))

for target in targets:
    if target.label != 'bootloader':
        continue
    kconf = loadconfig.load_kconfig()
    kconf.load_config(target.path)
    print(kconf.write_min_config(target.path))

for target in targets:
    if target.label != 'canbootloader':
        continue
    kconf = loadconfig.load_kconfig()
    kconf.load_config(target.path)
    print(kconf.write_min_config(target.path))

for target in targets:
    if target.label in ('default', 'bootloader', 'canbootloader'):
        continue
    board_default = os.path.join(os.path.dirname(target.path), 'default.px4board')

    # Merge with default config
    kconf = merge_config.main(px4_dir + "/Kconfig", board_default, target.path)
    tf = tempfile.NamedTemporaryFile()

    # Save minconfig
    kconf.write_min_config(tf.name)

    # Diff with default config and save to label.px4board
    stdoutpipe = sys.stdout
    sys.stdout = open(target.path, "w")
    diffconfig.main(1, board_default, tf.name)
    sys.stdout.close()
    sys.stdout = stdoutpipe
