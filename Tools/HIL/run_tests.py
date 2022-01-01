#! /usr/bin/env python3

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re
import unittest
import os
import sys
import datetime
import serial.tools.list_ports as list_ports

COLOR_RED    = "\x1b[31m"
COLOR_GREEN  = "\x1b[32m"
COLOR_YELLOW = "\x1b[33m"
COLOR_WHITE  = "\x1b[37m"
COLOR_RESET  = "\x1b[0m"

def print_line(line):
    if "WARNING" in line:
        line = line.replace("WARNING", f"{COLOR_YELLOW}WARNING{COLOR_RESET}", 1)
    elif "WARN" in line:
        line = line.replace("WARN", f"{COLOR_YELLOW}WARN{COLOR_RESET}", 1)
    elif "ERROR" in line:
        line = line.replace("ERROR", f"{COLOR_RED}ERROR{COLOR_RESET}", 1)
    elif "INFO" in line:
        line = line.replace("INFO", f"{COLOR_WHITE}INFO{COLOR_RESET}", 1)

    if "PASSED" in line:
        line = line.replace("PASSED", f"{COLOR_GREEN}PASSED{COLOR_RESET}", 1)

    if "FAILED" in line:
        line = line.replace("FAILED", f"{COLOR_RED}FAILED{COLOR_RESET}", 1)

    if "\n" in line:
        current_time = datetime.datetime.now()
        print('[{0}] {1}'.format(current_time.isoformat(timespec='milliseconds'), line), end='')
    else:
        print('{0}'.format(line), end='')


def do_test(port, baudrate, test_name):
    ser = serial.Serial(port, baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1, xonxoff=False, rtscts=False, dsrdtr=False)

    timeout_start = time.monotonic()
    timeout = 30  # 30 seconds

    # wait for nsh prompt
    while True:
        ser.write("\n".encode("ascii"))
        ser.flush()

        serial_line = ser.readline().decode("ascii", errors='ignore')

        if "nsh>" in serial_line:
            break
        else:
            if len(serial_line) > 0:
                print(serial_line, end='')

        if time.monotonic() > timeout_start + timeout:
            print("Error, timeout waiting for prompt")
            return False

    # clear
    ser.reset_input_buffer()

    success = False

    # run test cmd
    print('\n|======================================================================')
    cmd = 'tests ' + test_name
    print("| Running:", cmd)
    print('|======================================================================')

    timeout_start = time.monotonic()
    timeout = 2  # 2 seconds

    # wait for command echo
    print("Running command: \'{0}\'".format(cmd))
    serial_cmd = '{0}\n'.format(cmd)
    ser.write(serial_cmd.encode("ascii"))
    ser.flush()
    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')

        if cmd in serial_line:
            break
        else:
            if len(serial_line) > 0:
                print_line(serial_line)

        if time.monotonic() > timeout_start + timeout:
            print("Error, timeout waiting for command echo")
            break


    # print results, wait for final result (PASSED or FAILED)
    timeout = 300  # 5 minutes
    timeout_start = time.monotonic()
    timeout_newline = timeout_start

    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')

        if len(serial_line) > 0:
            print_line(serial_line)

        if test_name + " PASSED" in serial_line:
            success = True
            break
        elif test_name + " FAILED" in serial_line:
            success = False
            break

        if time.monotonic() > timeout_start + timeout:
            print("Error, timeout")
            print(test_name + f" {COLOR_RED}FAILED{COLOR_RESET}")
            success = False
            break

        # newline every 10 seconds if still running
        if (len(serial_line) <= 0) and (time.monotonic() - timeout_newline > 10):
            ser.write("\n".encode("ascii"))
            timeout_newline = time.monotonic()

    ser.close()

    return success

class TestHardwareMethods(unittest.TestCase):
    TEST_DEVICE = 0
    TEST_BAUDRATE = 0

    def test_atomic_bitset(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "atomic_bitset"))

    def test_bezier(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "bezier"))

    def test_bitset(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "bitset"))

    def test_bson(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "bson"))

    def test_dataman(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "dataman"))

    # def test_file(self):
    #     self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "file"))

    def test_file2(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "file2"))

    def test_float(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "float"))

    def test_hrt(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "hrt"))

    def test_int(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "int"))

    def test_i2c_spi_cli(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "i2c_spi_cli"))

    def test_IntrusiveQueue(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "IntrusiveQueue"))

    def test_IntrusiveSortedList(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "IntrusiveSortedList"))

    def test_List(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "List"))

    def test_mathlib(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "mathlib"))

    def test_matrix(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "matrix"))

    # def test_mixer(self):
    #     self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "mixer"))

    def test_param(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "param"))

    def test_parameters(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "parameters"))

    def test_perf(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "perf"))

    # def test_rc(self):
    #     self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "rc"))

    def test_search_min(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "search_min"))

    def test_sleep(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "sleep"))

    def test_time(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "time"))

    def test_versioning(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "versioning"))

def main():

    default_device = None
    device_required = True

    # select USB UART as default if there's only 1
    ports = list(serial.tools.list_ports.grep('USB UART'))

    if (len(ports) == 1):
        default_device = ports[0].device
        device_required = False

        print("Default USB UART port: {0}".format(ports[0].name))
        print(" device: {0}".format(ports[0].device))
        print(" description: \"{0}\" ".format(ports[0].description))
        print(" hwid: {0}".format(ports[0].hwid))
        #print(" vid: {0}, pid: {1}".format(ports[0].vid, ports[0].pid))
        #print(" serial_number: {0}".format(ports[0].serial_number))
        #print(" location: {0}".format(ports[0].location))
        print(" manufacturer: {0}".format(ports[0].manufacturer))
        #print(" product: {0}".format(ports[0].product))
        #print(" interface: {0}".format(ports[0].interface))

    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default=default_device, help='', required=device_required)
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()

    TestHardwareMethods.TEST_DEVICE = args.device
    TestHardwareMethods.TEST_BAUDRATE = args.baudrate

    unittest.main(__name__, failfast=True, verbosity=0, argv=['main'])

if __name__ == "__main__":
   main()
