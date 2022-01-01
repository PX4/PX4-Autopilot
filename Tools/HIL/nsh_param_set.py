#! /usr/bin/env python3

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re
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


def do_param_set_cmd(port, baudrate, param_name, param_value):
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
                print_line(serial_line)

        if time.monotonic() > timeout_start + timeout:
            print("Error, timeout waiting for prompt")
            sys.exit(1)

    # clear
    ser.reset_input_buffer()

    # run command
    timeout_start = time.monotonic()
    timeout = 10  # 10 seconds

    cmd = "param set " + param_name + " " + param_value

    # write command (param set) and wait for command echo
    print("Running command: \'{0}\'".format(cmd))
    serial_cmd = '{0}\r\n'.format(cmd)
    ser.write(serial_cmd.encode("ascii"))
    ser.flush()
    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')

        if cmd in serial_line:
            print_line(serial_line)
            break
        else:
            if len(serial_line) > 0:
                print_line(serial_line)

        if time.monotonic() > timeout_start + timeout:
            print("Error, timeout waiting for command echo")
            break

    # verify param value
    cmd = "param show " + param_name
    serial_cmd = '{0}\r\n'.format(cmd)
    ser.write(serial_cmd.encode("ascii"))
    ser.flush()

    param_show_response = param_name + " ["

    timeout_start = time.monotonic()
    timeout = 3  # 3 seconds

    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')

        if param_show_response in serial_line:
            print_line(serial_line)
            current_param_value = serial_line.split(":")[-1].strip()

            if (current_param_value == param_value):
                sys.exit(0)
            else:
                sys.exit(1)
        else:
            if len(serial_line) > 0:
                print_line(serial_line)

            if time.monotonic() > timeout_start + timeout:
                if "nsh>" in serial_line:
                    sys.exit(1) # error, command didn't complete successfully
                elif "NuttShell (NSH)" in serial_line:
                    sys.exit(1) # error, command didn't complete successfully

        if time.monotonic() > timeout_start + timeout:
            print("Error, timeout")
            sys.exit(-1)

        if len(serial_line) <= 0:
            ser.write("\r\n".encode("ascii"))
            ser.flush()
            time.sleep(0.2)

    ser.close()

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
    parser.add_argument("--name", "-p", dest="param_name", help="Parameter name")
    parser.add_argument("--value", "-v", dest="param_value", help="Parameter value")
    args = parser.parse_args()

    do_param_set_cmd(args.device, args.baudrate, args.param_name, args.param_value)

if __name__ == "__main__":
   main()
