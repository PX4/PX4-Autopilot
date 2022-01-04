#! /usr/bin/env python3

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re
import sys
import datetime
import serial.tools.list_ports as list_ports
import tempfile

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


def do_param_set_cmd(port_url, baudrate, param_name, param_value):
    ser = serial.serial_for_url(url=port_url, baudrate=baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=3, xonxoff=False, rtscts=False, dsrdtr=False, inter_byte_timeout=1)

    timeout_start = time.monotonic()
    timeout = 30  # 30 seconds

    ser.write("\n\n\n".encode("ascii"))

    # wait for nsh prompt
    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')

        if len(serial_line) > 0:
            if "nsh>" in serial_line:
                break
        else:
            if time.monotonic() > timeout_start + timeout:
                print("Error, timeout waiting for prompt")
                sys.exit(1)

            ser.write("\n".encode("ascii"))


    # clear
    ser.reset_input_buffer()

    # run command
    timeout_start = time.monotonic()
    timeout = 10  # 10 seconds

    cmd = "param set " + param_name + " " + param_value

    # write command (param set) and wait for command echo
    print("Running command: \'{0}\'".format(cmd))
    serial_cmd = '{0}\n'.format(cmd)

    ser.write(serial_cmd.encode("ascii"))

    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')

        if len(serial_line) > 0:
            if cmd in serial_line:
                break

            print_line(serial_line)

        else:
            if time.monotonic() > timeout_start + timeout:
                print("Error, timeout waiting for command echo")
                break

    # clear
    ser.reset_input_buffer()

    # verify param value
    cmd = "param show " + param_name
    print("Running command: \'{0}\'".format(cmd))
    serial_cmd = '{0}\n'.format(cmd)
    ser.write(serial_cmd.encode("ascii"))

    param_show_response = param_name + " ["

    timeout_start = time.monotonic()
    timeout = 3  # 3 seconds

    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')

        if len(serial_line) > 0:
            print_line(serial_line)

            if param_show_response in serial_line:
                current_param_value = serial_line.split(":")[-1].strip()

                if (current_param_value == param_value):
                    sys.exit(0)
                else:
                    sys.exit(1)
            else:
                if time.monotonic() > timeout_start + timeout:
                    if "nsh>" in serial_line:
                        sys.exit(1) # error, command didn't complete successfully
                    elif "NuttShell (NSH)" in serial_line:
                        sys.exit(1) # error, command didn't complete successfully
        else:
            if time.monotonic() > timeout_start + timeout:
                print("Error, timeout")
                sys.exit(-1)


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
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="serial port baud rate (default=57600)", default=57600)
    parser.add_argument("--name", "-p", dest="param_name", help="Parameter name")
    parser.add_argument("--value", "-v", dest="param_value", help="Parameter value")
    args = parser.parse_args()

    tmp_file = "{0}/pyserial_spy_file.txt".format(tempfile.gettempdir())
    port_url = "spy://{0}?file={1}".format(args.device, tmp_file)

    print("pyserial url: {0}".format(port_url))

    do_param_set_cmd(port_url, args.baudrate, args.param_name, args.param_value)

if __name__ == "__main__":
   main()
