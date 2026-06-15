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
from pathlib import Path

import diffconfig
import merge_config

px4_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))

CLASS_NAMES = {'copter', 'fixedwing', 'vtol', 'rover', 'uuv', 'spacecraft', 'airship',
               'cannode', 'linux', 'sitl', 'io', 'ros2', 'voxl2'}
target_classes_dir = os.path.join(px4_dir, 'target_classes')


def board_sole_class(board_dir):
    classes = [os.path.basename(f)[:-len('.px4board')]
               for f in glob.glob(board_dir + '/*.px4board')
               if os.path.basename(f)[:-len('.px4board')] in CLASS_NAMES]
    return classes[0] if len(classes) == 1 else None


def is_standalone(basename):
    # base is the non-buildable foundation; bootloader/canbootloader/performance-test
    # are complete savedefconfig labels (no class merge).
    return (basename == 'base.px4board' or basename == 'performance-test.px4board'
            or basename.startswith('bootloader') or basename.startswith('canbootloader'))


# Normalize each board's base.px4board (and fold in its NuttX nsh defconfig).
for name in glob.glob(px4_dir + '/boards/*/*/base.px4board'):
    kconf = kconfiglib.Kconfig()
    kconf.load_config(name)
    print(kconf.write_min_config(name))

    defconfig_path = Path(name).parent / "nuttx-config" / "nsh" / "defconfig"
    if os.path.exists(defconfig_path):
        # Merge NuttX with the base config
        kconf = merge_config.main(px4_dir + "/Kconfig", [name, str(defconfig_path)])
        print(kconf.write_min_config(name))

# Standalone savedefconfig labels carry a complete config; just normalize them.
for pattern in ('bootloader*.px4board', 'canbootloader*.px4board', 'performance-test.px4board'):
    for name in glob.glob(px4_dir + '/boards/*/*/' + pattern):
        kconf = kconfiglib.Kconfig()
        kconf.load_config(name)
        print(kconf.write_min_config(name))

# Class/variant labels: re-save each as a minimal delta over its class baseline
# (base.px4board -> target_classes/<class>.px4board -> optional board <class> overlay).
for name in glob.glob(px4_dir + '/boards/*/*/*.px4board'):
    if is_standalone(os.path.basename(name)):
        continue

    label = os.path.basename(name)[:-len('.px4board')]
    board_dir = os.path.dirname(name)
    if '.' in label:
        target_class = label.split('.', 1)[0]
    elif label in CLASS_NAMES:
        target_class = label
    else:
        target_class = board_sole_class(board_dir)

    lower = [board_dir + '/base.px4board', os.path.join(target_classes_dir, target_class + '.px4board')]
    overlay = board_dir + '/' + target_class + '.px4board'
    if os.path.abspath(overlay) != os.path.abspath(name) and os.path.exists(overlay):
        lower.append(overlay)

    # Baseline = the layers beneath the label; full = baseline + the label fragment.
    baseline_tf = tempfile.NamedTemporaryFile()
    merge_config.main(px4_dir + "/Kconfig", lower).write_min_config(baseline_tf.name)
    full_tf = tempfile.NamedTemporaryFile()
    merge_config.main(px4_dir + "/Kconfig", lower + [name]).write_min_config(full_tf.name)

    # Diff the full config against the baseline and save the delta back to the label.
    stdoutpipe = sys.stdout
    sys.stdout = open(name, "w")
    diffconfig.main(1, baseline_tf.name, full_tf.name)
    sys.stdout.close()
    sys.stdout = stdoutpipe
