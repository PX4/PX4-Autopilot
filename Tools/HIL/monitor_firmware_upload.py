#! /usr/bin/env python3

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re
import sys

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

    print(line, end='')

def monitor_firmware_upload(port, baudrate):
    ser = serial.Serial(port, baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1, xonxoff=True, rtscts=False, dsrdtr=False)

    timeout = 180  # 3 minutes
    timeout_start = time.time()
    timeout_newline = time.time()

    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')

        if "NuttShell (NSH)" in serial_line:
            sys.exit(0)
        elif "nsh>" in serial_line:
            sys.exit(0)
        else:
            if len(serial_line) > 0:
                print_line(serial_line)

        if time.time() > timeout_start + timeout:
            print("Error, timeout")
            sys.exit(-1)

        # newline every 10 seconds if still running
        if time.time() - timeout_newline > 10:
            timeout_newline = time.time()
            ser.write("\n".encode("ascii"))
            ser.flush()

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default=None, help='', required=True)
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()

    monitor_firmware_upload(args.device, args.baudrate)

if __name__ == "__main__":
   main()
