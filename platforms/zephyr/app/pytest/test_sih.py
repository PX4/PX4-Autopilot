############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
Fly the SIH quadcopter from the Zephyr shell: take off, hover and land.
"""

import re
import time

from twister_harness import Shell


def px4(shell: Shell, command: str) -> list[str]:
    try:
        return shell.exec_command(f'px4 {command}')
    except AssertionError:
        # a PX4 message was printed over the echo of the command
        return []


def listener_value(shell: Shell, topic: str, field: str) -> float | None:
    for line in px4(shell, f'listener {topic}'):
        # a log message printed in the middle of a line leaves it without a value
        match = re.match(rf'\s+{field}: (-?[0-9.]+(?:e[-+]?[0-9]+)?)\s*$', line)

        if match:
            return float(match.group(1))

    return None


def wait_until(condition, timeout: float, what: str):
    end = time.time() + timeout

    while time.time() < end:
        if condition():
            return

        time.sleep(1)

    raise AssertionError(f'timeout waiting for {what}')


def commander_output(shell: Shell, command: str) -> str:
    return '\n'.join(px4(shell, f'commander {command}'))


def test_takeoff_and_land(shell: Shell):
    wait_until(lambda: 'Preflight check: OK' in commander_output(shell, 'check'), 300, 'the preflight checks')

    commander_output(shell, 'takeoff')

    def climbed():
        z = listener_value(shell, 'vehicle_local_position', 'z')
        return z is not None and z < -2.0

    wait_until(climbed, 120, 'the takeoff')

    commander_output(shell, 'land')
    wait_until(lambda: 'INFO  [commander] Disarmed' in commander_output(shell, 'status'), 120, 'the landing')
