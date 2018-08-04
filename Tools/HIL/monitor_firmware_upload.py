#! /usr/bin/python

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re

def monitor_firmware_upload(port, baudrate):     
    databits = serial.EIGHTBITS
    stopbits = serial.STOPBITS_ONE
    parity = serial.PARITY_NONE
    ser = serial.Serial(port, baudrate, databits, parity, stopbits, 100)

    finished = 0

    while finished == 0:
        serial_line = ser.readline()
        print(serial_line.replace('\n','')) 

        if "NuttShell (NSH)" in serial_line:
            finished = 1
        time.sleep(0.05)
    ser.close()

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default = None, help='')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()

    monitor_firmware_upload(args.device, args.baudrate)

if __name__ == "__main__":
   main()
