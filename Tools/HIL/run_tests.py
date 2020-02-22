#! /usr/bin/python

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re
import unittest
import os

def do_test(port, baudrate, test_name):
    databits = serial.EIGHTBITS
    stopbits = serial.STOPBITS_ONE
    parity = serial.PARITY_NONE
    ser = serial.Serial(port, baudrate, databits, parity, stopbits, timeout=10)

    ser.write('\n\n')

    finished = 0
    success = False

    timeout = 10  # 10 seconds
    timeout_start = time.time()

    while finished == 0:
        serial_line = ser.readline()
        print(serial_line.replace('\n',''))

        if "nsh>" in serial_line:
            finished = 1

        if time.time() > timeout_start + timeout:
            print("Error, timeout")
            finished = 1
            break


    # run test
    ser.write('tests ' + test_name + '\n')
    time.sleep(0.05)

    finished = 0
    timeout = 300  # 5 minutes
    timeout_start = time.time()
    timeout_newline = time.time()

    while finished == 0:
        serial_line = ser.readline()
        print(serial_line.replace('\n',''))

        if test_name + " PASSED" in serial_line:
            finished = 1
            success = True
        elif test_name + " FAILED" in serial_line:
            finished = 1
            success = False

        if time.time() > timeout_start + timeout:
            print("Error, timeout")
            print(test_name + " FAILED")
            finished = 1
            success = False
            break

        # newline every 30 seconds if still running
        if time.time() - timeout_newline > 30:
            ser.write('\n')
            timeout_newline = time.time()

    ser.close()

    return success

class TestHadrwareMethods(unittest.TestCase):
    TEST_DEVICE = 0
    TEST_BAUDRATE = 0

    def test_autodeclination(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "autodeclination"))

    def test_bezier(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "bezier"))

    def test_bson(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "bson"))

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default = None, help='')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()

    TestHadrwareMethods.TEST_DEVICE = args.device
    TestHadrwareMethods.TEST_BAUDRATE = 57600

    unittest.main(__name__, argv=['main'], exit=False)

if __name__ == "__main__":
   main()
