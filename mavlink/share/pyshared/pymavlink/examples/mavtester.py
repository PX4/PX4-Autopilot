#!/usr/bin/env python

'''
test mavlink messages
'''

import sys, struct, time, os
from curses import ascii

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

import mavlink, mavtest, mavutil

from optparse import OptionParser
parser = OptionParser("mavtester.py [options]")

parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=115200)
parser.add_option("--device", dest="device", default=None, help="serial device")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
(opts, args) = parser.parse_args()

if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

# create a mavlink serial instance
master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate, source_system=opts.SOURCE_SYSTEM)

# wait for the heartbeat msg to find the system ID
wait_heartbeat(master)

print("Sending all message types")
mavtest.generate_outputs(master.mav)

