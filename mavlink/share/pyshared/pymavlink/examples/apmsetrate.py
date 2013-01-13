#!/usr/bin/env python

'''
set stream rate on an APM 
'''

import sys, struct, time, os

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

from optparse import OptionParser
parser = OptionParser("apmsetrate.py [options]")

parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=115200)
parser.add_option("--device", dest="device", default=None, help="serial device")
parser.add_option("--rate", dest="rate", default=4, type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
parser.add_option("--showmessages", dest="showmessages", action='store_true',
                  help="show incoming messages", default=False)
parser.add_option("--mav10", action='store_true', default=False, help="Use MAVLink protocol 1.0")
(opts, args) = parser.parse_args()

if opts.mav10:
    os.environ['MAVLINK10'] = '1'
    import mavlink10 as mavlink
else:
    import mavlink
import mavutil

if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

def show_messages(m):
    '''show incoming mavlink messages'''
    while True:
        msg = m.recv_match(blocking=True)
        if not msg:
            return
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        else:
            print(msg)                    
                
# create a mavlink serial instance
master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)

# wait for the heartbeat msg to find the system ID
wait_heartbeat(master)

print("Sending all stream request for rate %u" % opts.rate)
for i in range(0, 3):
    master.mav.request_data_stream_send(master.target_system, master.target_component,
                                        mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)
if opts.showmessages:
    show_messages(master)
