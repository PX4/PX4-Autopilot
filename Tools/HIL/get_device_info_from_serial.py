#!/usr/bin/python

import serial
import json
from argparse import ArgumentParser
from pprint import pprint

def get_device_info_from_serial(device, baudrate):
    ser = serial.Serial(device, baudrate, timeout=1,parity=serial.PARITY_NONE) 
    ser.write('\n')
    ser.write('ver all\n')
    output_buffer = ser.read(1000)

    output_json = json.loads('{"device":"' + device + '"}')

    for line in output_buffer.split("\n"):
        if "HW arch: " in line:
            output_json['HW'] = line.replace("HW arch: ", "").replace('\r','').replace('\n','').replace('\t','').replace(' ','')
        if "UID: " in line and "MFG" not in line:
            output_json['UID'] = line.replace("UID: ", "").replace('\r','').replace('\n','').replace('\t','').replace(' ','')

    return output_json

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default = None, help='')
    parser.add_argument('--baudrate', "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()
    device_info = get_device_info_from_serial(args.device, args.baudrate)
    pprint(device_info)

if __name__ == '__main__':
    main()