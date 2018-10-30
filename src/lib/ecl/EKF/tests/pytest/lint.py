#!/usr/bin/env python3
###############################################################################
#
# Copyright (c) 2017 Estimation and Control Library (ECL). All rights reserved.
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
# 3. Neither the name ECL nor the names of its contributors may be
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
###############################################################################
"""Runs code checkers on ecl pytest code
@author: Peter Dürr <Peter.Duerr@sony.com>
"""
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

from subprocess import Popen, PIPE
import sys
from datetime import datetime

CHECKERS = [
    ['pep8', '.'],
    ['pep8', './ekf_test.py'],
    ['pep8', './lint.py'],
    ['pylint', './lint.py', '--reports=n', '--score=n'],
    ['pylint', './ekf_test.py', '--reports=n', '--score=n'],
    ['pylint', './plot_utils.py', '--reports=n', '--score=n'],
    ['pylint', './test_altitude.py', '--reports=n', '--score=n'],
    ['pylint', './test_basics.py', '--reports=n', '--score=n'],
    ['pylint', './test_sampling.py', '--reports=n', '--score=n'],
    ['pylint', './test_utils.py', '--reports=n', '--score=n'],
]
"""List of checkers to run and their arguments
"""

if __name__ == "__main__":
    try:
        TIC = datetime.now()
        CHECKERS_RUN = 0
        ERROR = False
        for checker in CHECKERS:
            process = Popen(checker, stdout=PIPE)
            sys.stdout.write('.')
            sys.stdout.flush()
            CHECKERS_RUN += 1
            header = "Output from %s" % " ".join(checker)
            output = process.communicate()
            message = ""
            for elem in output:
                if elem is not None and elem:
                    message += elem.decode('utf-8')
                    ERROR = True
            if not message == "":
                print(header)
                print("=" * len(header))
                print(message)
                print("=" * len(header))

        TOC = datetime.now()
        print("\n" + ''.join(['-']*70))
        print("Ran %i checkers in %fs\n" % (CHECKERS_RUN,
                                            (TOC - TIC).total_seconds()))

        if not ERROR:
            exit(0)
        else:
            exit(1)
    except KeyboardInterrupt:
        exit(2)
