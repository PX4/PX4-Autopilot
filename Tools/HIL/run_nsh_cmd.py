#! /usr/bin/python

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re

def do_nsh_cmd(port, baudrate, cmd):
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

    # run command
    ser.write(cmd + '\n')
    time.sleep(0.05)
    ser.write('\n')

    finished = 0
    timeout = 30  # 30 seconds
    timeout_start = time.time()
    timeout_newline = time.time()

    while finished == 0:
        serial_line = ser.readline()
        print(serial_line.replace('\n',''))

        if cmd in serial_line:
            continue
        elif "nsh>" in serial_line:
            finished = 1
            break

        if time.time() > timeout_start + timeout:
            print("Error, timeout")
            finished = 1
            break

        # newline every 10 seconds if still running
        if time.time() - timeout_newline > 10:
            ser.write('\n')
            timeout_newline = time.time()

    ser.close()

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default = None, help='')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    parser.add_argument("--cmd", "-c", dest="cmd", help="Command to run")
    args = parser.parse_args()

    do_nsh_cmd(args.device, args.baudrate, args.cmd)

if __name__ == "__main__":
   main()
