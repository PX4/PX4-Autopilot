#!/usr/bin/python

from __future__ import print_function
import sys, select
import json
from pprint import pprint

try:
    from pymavlink import mavutil
    import serial
    import time
except:
    print("Failed to import pymavlink.")
    print("You may need to install it with 'pip install pymavlink pyserial'")
    print("")
    raise
from argparse import ArgumentParser

class MavlinkSerialPort():
    '''an object that looks like a serial port, but
    transmits using mavlink SERIAL_CONTROL packets'''
    def __init__(self, portname, baudrate, devnum=0, debug=0):
        self.baudrate = 0
        self._debug = debug
        self.buf = ''
        self.port = devnum
        self.debug("Connecting with MAVLink to %s ..." % portname)
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)
        self.mav.wait_heartbeat()
        self.debug("HEARTBEAT OK\n")
        self.debug("Locked serial device\n")

    def debug(self, s, level=1):
        '''write some debug text'''
        if self._debug >= level:
            print(s)

    def write(self, b):
        '''write some bytes'''
        self.debug("sending '%s' (0x%02x) of len %u\n" % (b, ord(b[0]), len(b)), 2)
        while len(b) > 0:
            n = len(b)
            if n > 70:
                n = 70
            buf = [ord(x) for x in b[:n]]
            buf.extend([0]*(70-len(buf)))
            self.mav.mav.serial_control_send(self.port,
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                                             0,
                                             0,
                                             n,
                                             buf)
            b = b[n:]

    def close(self):
        self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0]*70)

    def _recv(self):
        '''read some bytes into self.buf'''
        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                type='SERIAL_CONTROL', blocking=True,
                                timeout=0.03)
        if m is not None:
            if self._debug > 2:
                print(m)
            data = m.data[:m.count]
            self.buf += ''.join(str(chr(x)) for x in data)

    def read(self, n):
        '''read some bytes'''
        if len(self.buf) == 0:
            self._recv()
        if len(self.buf) > 0:
            if n > len(self.buf):
                n = len(self.buf)
            ret = self.buf[:n]
            self.buf = self.buf[n:]
            if self._debug >= 2:
                for b in ret:
                    self.debug("read 0x%x" % ord(b), 2)
            return ret
        return ''

def get_device_info_from_mavlink(device, baudrate):
    mav_serialport = MavlinkSerialPort(device, baudrate, devnum=10)
    mav_serialport.write('\n\n\n') # make sure the shell is started
    mav_serialport.write('ver all\n')
    mav_serialport.write('ver all\n') # fixme: ask twice because sometimes not everything from the first time could be retrieved

    output_buffer = ""
    start_time = time.time() 

    while True:
        data = mav_serialport.read(4000)
        output_buffer = output_buffer + data

        if time.time() - start_time > 0.5:
            break

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
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()

    device_info = get_device_info_from_mavlink(args.device, args.baudrate)
    pprint(device_info)

if __name__ == '__main__':
    main()