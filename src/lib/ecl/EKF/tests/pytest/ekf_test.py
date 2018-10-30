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

"""Tests for the ecl library (using it's Python bindings via SWIG)

In order to run the tests, make sure to compile the ecl library including the
SWIG bindings, e.g. by running

    cmake -DEKF_PYTHON_TESTS=ON .. && make ecl_EKF_pytest-quick

from your build directory.

This script can also be run directly (e.g., by using `make pytest`), or you can
run pytest on the test folder.

Running the script directly provides some useful flags, in particular one can
enable plots (using `-p`) to visualize the test results. For example, the
following command will run all test that include the string "baro" in the test
name and plot the results:

    PYTHONPATH='/path/to/ecl' ekf_test -p "baro"


@author: Peter Dürr <Peter.Duerr@sony.com>
"""
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import os
from datetime import datetime
import argparse

import pytest
from hypothesis import settings

START_TIME = datetime.now()
"""Record when the script was loaded
"""

# Register hypothesis profiles
settings.register_profile("plots", settings(max_examples=0))
settings.register_profile("quick", settings(max_examples=20))


def main():
    """Called when the script is run standalone
    """

    parser = argparse.ArgumentParser(description="Tests ecl's EKF",
                                     usage=(__file__ +
                                            " [OPTIONS] [EXPRESSION]"))
    parser.add_argument('-q', '--quick',
                        action='store_true',
                        help='Run only the quick tests')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        help='Print verbose output')
    parser.add_argument('-s', '--stdout',
                        action='store_true',
                        help='Print output to stdout during tests')
    parser.add_argument('-b', '--benchmark',
                        action='store_true',
                        help='Run the benchmark tests')
    parser.add_argument('-p', '--plots',
                        action='store_true',
                        help='Create plots during the tests '
                        '(requires matplotlib and seaborn)')
    parser.add_argument('tests', type=str, nargs='?',
                        help='Run only tests which match '
                        'the given substring expression')
    args = parser.parse_args()
    pytest_arguments = [os.path.dirname(os.path.realpath(__file__))]
    pytest_plugins = []

    if args.quick:
        pytest_arguments.append('--hypothesis-profile=plots')
    if args.verbose:
        pytest_arguments.append('-v')
        pytest_arguments.append('-l')
        pytest_arguments.append('--hypothesis-show-statistics')
    if args.stdout:
        pytest_arguments.append('-s')
    if args.benchmark:
        pytest_arguments.append('-m benchmark')
        pytest_arguments.append('--benchmark-verbose')
    else:
        pytest_arguments.append('--benchmark-disable')

    if args.tests:
        pytest_arguments.append('-k ' + args.tests)

    # To control plotting, monkeypatch a flag onto pytest (cannot just use a
    # global variable due to pytest's discovery)
    pytest.ENABLE_PLOTTING = args.plots
    if args.plots:
        pytest_arguments.append('--hypothesis-profile=plots')

    # Run pytest
    result = pytest.main(pytest_arguments, plugins=pytest_plugins)
    exit(result)


if __name__ == '__main__':
    main()
