#! /usr/bin/env python3

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re
import sys

def do_nsh_cmd(port, baudrate, cmd):
    ser = serial.Serial(port, baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.1, xonxoff=True, rtscts=False, dsrdtr=False)

    timeout_start = time.time()
    timeout = 10  # 10 seconds

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

        if time.time() > timeout_start + timeout:
            print("Error, timeout waiting for prompt")
            sys.exit(1)

    # run command
    timeout_start = time.time()
    timeout = 10  # 10 seconds

    success_cmd = "cmd succeeded!"

    # wait for command echo
    serial_cmd = '{0}; echo "{1}"\r\n'.format(cmd, success_cmd)
    ser.write(serial_cmd.encode("ascii"))
    ser.flush()
    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')

        if cmd in serial_line:
            break
        else:
            if len(serial_line) > 0:
                print(serial_line, end='')

        if time.time() > timeout_start + timeout:
            print("Error, timeout waiting for command echo")
            break


    timeout_start = time.time()
    timeout = 30  # 30 seconds

    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')

        if success_cmd in serial_line:
            break
        else:
            if len(serial_line) > 0:
                print(serial_line, end='')

            if "nsh>" in serial_line:
                sys.exit(1) # error, command didn't complete successfully
            elif "NuttShell (NSH)" in serial_line:
                sys.exit(1) # error, command didn't complete successfully

        if len(serial_line) <= 0:
            ser.write("\r\n".encode("ascii"))
            ser.flush()

        if time.time() > timeout_start + timeout:
            print("Error, timeout")
            sys.exit(-1)

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
