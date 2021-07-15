#! /usr/bin/env python3

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re
import sys

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
                print(serial_line, end='')

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
