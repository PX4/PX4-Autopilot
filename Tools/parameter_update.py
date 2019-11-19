#!/usr/bin/env python
# -*- coding: utf-8 -*-

############################################################################
#
#   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
File: parameter_update.py
Description:
    This script searches for declarations of BlockParam, creates a variable name of the
    form "_param_name_of_parameter" and replaces the old variable with the new one.

Example:
        (ParamExtFloat<px4::params::EKF2_GYR_NOISE>) _gyroNoise,
        [...]
        _gyroNoise.set(1.f);

    is replaced by

        (ParamExtFloat<px4::params::EKF2_GYR_NOISE>) _param_ekf2_gyr_noise,
        [...]
        _param_ekf2_gyr_noise.set(1.f);

Usage:
    The script needs a list of files as an input. To quickly find and feed all the file
    that it needs to inspect, one can simply pipe the result of a ripgrep -l command as follows:

        rg -l '\.[gs]et\(|px4::params::' -tcpp | python parameter_update.py
"""

import re
import fileinput
import sys

filenames = []
for filename in sys.stdin:
    filenames.append(filename.rstrip())

pattern = r'^.*<px4::params::([A-Za-z0-9_]+)>\)\s*([A-Za-z0-9_]+)[,\s]?.*$'
regex = re.compile(pattern, re.MULTILINE)

replace_dict = dict()

print("Searching for parameters...")
for filename in filenames:
    print(filename)
    with open(filename, 'r') as f:
        text = f.read()
        for match in regex.finditer(text):
            replace_dict[match.group(2)] = "_param_" + match.group(1).strip().lower()

print("The following variables will be changed")
for old_var in replace_dict:
    print("{} -> {}".format(old_var, replace_dict[old_var]))

for filename in filenames:
    print(filename)
    for old_var in replace_dict:
        for line in fileinput.input(filename, inplace=1):
            line = re.sub(r'(>\)\s*)\b' + old_var + r'\b', r'\1' + replace_dict[old_var], line.rstrip()) # replace the declaration
            line = re.sub(r'(^\s*)\b' + old_var + r'\b', r'\1' + replace_dict[old_var], line.rstrip()) # replace the 2 liner declaration and when the variable is used a the beginning of a new line
            line = re.sub(r'\b' + old_var + r'\b(\.([gs]et|commit)\()',replace_dict[old_var] + r'\1' , line.rstrip()) # replace the usages (with get/set/commit)
            line = re.sub(r'(\()\b' + old_var + r'\b', r'\1' + replace_dict[old_var], line.rstrip()) # replace the 2 liner declaration
            print(line)
