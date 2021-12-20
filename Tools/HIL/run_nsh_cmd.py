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


def do_nsh_cmd(port, baudrate, cmd):
    ser = serial.Serial(port, baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.2, xonxoff=True, rtscts=False, dsrdtr=False)

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
                print_line(serial_line)

        if time.monotonic() > timeout_start + timeout:
            print("Error, timeout waiting for prompt")
            sys.exit(1)

    # clear
    ser.readlines()

    # run command
    timeout_start = time.monotonic()
    timeout = 1  # 1 second

    success_cmd = "cmd succeeded!"

    # wait for command echo
    print("Running command: \'{0}\'".format(cmd))
    serial_cmd = '{0}; echo "{1}"; echo "{2}";\r\n'.format(cmd, success_cmd, success_cmd)
    ser.write(serial_cmd.encode("ascii"))
    ser.flush()
    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')

        if cmd in serial_line:
            break
        elif serial_line.startswith(success_cmd) and len(serial_line) <= len(success_cmd) + 2:
            print_line(serial_line)
            # we missed the echo, but command ran and succeeded
            sys.exit(0)
        else:
            if len(serial_line) > 0:
                print_line(serial_line)

        if (len(serial_line) <= 0) and (time.monotonic() > timeout_start + timeout):
            print("Error, timeout waiting for command echo")
            break


    timeout_start = time.monotonic()
    timeout = 240 # 4 minutes

    return_code = 0

    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')

        if success_cmd in serial_line:
            sys.exit(return_code)
            break
        else:
            if len(serial_line) > 0:
                if "ERROR " in serial_line:
                    return_code = -1

                print_line(serial_line)

            if "nsh>" in serial_line:
                sys.exit(1) # error, command didn't complete successfully
            elif "NuttShell (NSH)" in serial_line:
                sys.exit(1) # error, command didn't complete successfully

        if (len(serial_line) <= 0) and (time.monotonic() > timeout_start + timeout):
            print("Error, timeout")
            sys.exit(-1)

        if len(serial_line) <= 0:
            ser.write("\r\n".encode("ascii"))
            ser.flush()
            time.sleep(0.2)

    ser.close()

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default=None, help='', required=True)
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    parser.add_argument("--cmd", "-c", dest="cmd", help="Command to run")
    args = parser.parse_args()

    do_nsh_cmd(args.device, args.baudrate, args.cmd)

if __name__ == "__main__":
   main()
