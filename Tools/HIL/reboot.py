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

def monitor_firmware_upload(port, baudrate):
    ser = serial.Serial(port, baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1, xonxoff=True, rtscts=False, dsrdtr=False)

    # clear
    ser.readlines()

    ser.write("\nreboot\n".encode("ascii"))
    ser.flush()

    timeout_reboot_cmd = 30
    timeout = 300  # 5 minutes
    timeout_start = time.monotonic()
    timeout_newline = time.monotonic()
    time_success = 0

    return_code = 0

    while True:
        if time.monotonic() > timeout_start + timeout_reboot_cmd:
            ser.write("reboot\n".encode("ascii"))

        serial_line = ser.readline().decode("ascii", errors='ignore')

        if len(serial_line) > 0:
            if "ERROR" in serial_line:
                return_code = -1

            print_line(serial_line)

        if "NuttShell (NSH)" in serial_line:
            time_success = time.monotonic()

        # wait at least 2 seconds after seeing prompt to catch potential errors
        if time_success > 0 and time.monotonic() > time_success + 2:
            sys.exit(return_code)

        if time.monotonic() > timeout_start + timeout:
            print("Error, timeout")
            sys.exit(-1)

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default=None, help='', required=True)
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()

    monitor_firmware_upload(args.device, args.baudrate)

if __name__ == "__main__":
   main()
