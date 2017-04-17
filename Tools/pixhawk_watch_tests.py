#! /usr/bin/env python

"""
This is a simple helper for running pixhawk onboard unit tests.

The script returns 0 if the tests successfully pass, or -1 otherwise.
"""

from __future__ import print_function

import serial
import sys

def main(port):
    ser = serial.Serial(port, 57600, timeout=20)
    rc = 1
    line = 1
    while line:
        line = ser.readline()
        print(line, end="")
        if "All tests passed" in line:
            rc = 0
        if "NuttShell (NSH)" in line:
            return rc
    return rc

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Error, specify serial port", file=sys.stderr)
        sys.exit(1)

    sys.exit(main(sys.argv[1]))

