#! /usr/bin/env python3

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re
import sys
import datetime

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

def reboot(port, baudrate):
    ser = serial.Serial(port, baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1, xonxoff=False, rtscts=False, dsrdtr=False)

    # clear
    ser.reset_input_buffer()

    time_start = time.monotonic()
    ser.write("\n".encode("ascii"))
    ser.write("reboot\n".encode("ascii"))
    time_reboot_cmd = time_start

    timeout_reboot_cmd = 90
    timeout = 300  # 5 minutes

    return_code = 0

    while True:
        if time.monotonic() > time_reboot_cmd + timeout_reboot_cmd:
            time_reboot_cmd = time.monotonic()
            print("sending reboot cmd again")
            ser.write("reboot\n".encode("ascii"))
            time.sleep(0.5)

        serial_line = ser.readline().decode("ascii", errors='ignore')

        if len(serial_line) > 0:
            if "ERROR" in serial_line:
                return_code = -1

            print_line(serial_line)

        if "NuttShell (NSH)" in serial_line:
            sys.exit(return_code)

        if time.monotonic() > time_start + timeout:
            print("Error, timeout")
            sys.exit(-1)

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default=None, help='', required=True)
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()

    reboot(args.device, args.baudrate)

if __name__ == "__main__":
   main()
