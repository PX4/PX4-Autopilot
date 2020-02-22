#! /usr/bin/python

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re
import unittest

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
        elif "PANIC!!!" in serial_line:
            print("Error, Hardfault!!")
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

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default = None, help='')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()
    success = True

    success = success and do_test(args.device, args.baudrate, "autodeclination")
    success = success and do_test(args.device, args.baudrate, "bezier")
    success = success and do_test(args.device, args.baudrate, "bson")
    success = success and do_test(args.device, args.baudrate, "commander")
    success = success and do_test(args.device, args.baudrate, "controllib")
    success = success and do_test(args.device, args.baudrate, "conv")
    #success = success and do_test(args.device, args.baudrate, "dataman")
    success = success and do_test(args.device, args.baudrate, "float")
    success = success and do_test(args.device, args.baudrate, "hrt")
    success = success and do_test(args.device, args.baudrate, "int")
    success = success and do_test(args.device, args.baudrate, "IntrusiveQueue")
    success = success and do_test(args.device, args.baudrate, "List")
    success = success and do_test(args.device, args.baudrate, "mathlib")
    success = success and do_test(args.device, args.baudrate, "matrix")
    success = success and do_test(args.device, args.baudrate, "microbench_hrt")
    success = success and do_test(args.device, args.baudrate, "microbench_math")
    success = success and do_test(args.device, args.baudrate, "microbench_matrix")
    success = success and do_test(args.device, args.baudrate, "microbench_uorb")
    #success = success and do_test(args.device, args.baudrate, "mixer")
    success = success and do_test(args.device, args.baudrate, "param")
    success = success and do_test(args.device, args.baudrate, "parameters")
    success = success and do_test(args.device, args.baudrate, "perf")
    success = success and do_test(args.device, args.baudrate, "search_min")
    success = success and do_test(args.device, args.baudrate, "sleep")
    success = success and do_test(args.device, args.baudrate, "smoothz")
    success = success and do_test(args.device, args.baudrate, "time")
    success = success and do_test(args.device, args.baudrate, "uorb")
    success = success and do_test(args.device, args.baudrate, "versioning")

    if success:
        print("all run_test.py passed");
    else:
        print("some run_test.py Tests failed"); #TODO: show which test pass or failed

    assertFalse(success)

    return success

if __name__ == "__main__":
   main()
