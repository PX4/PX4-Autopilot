#!/usr/bin/env python

'''
show changes in flight modes
'''

import sys, time, os

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

from optparse import OptionParser
parser = OptionParser("flightmodes.py [options]")

(opts, args) = parser.parse_args()

import mavutil

if len(args) < 1:
    print("Usage: flightmodes.py [options] <LOGFILE...>")
    sys.exit(1)

def flight_modes(logfile):
    '''show flight modes for a log file'''
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    mode = -1
    nav_mode = -1

    filesize = os.path.getsize(filename)

    while True:
        m = mlog.recv_match(type=['SYS_STATUS','HEARTBEAT'],
                            condition='MAV.flightmode!="%s"' % mlog.flightmode)
        if m is None:
            return
        pct = (100.0 * mlog.f.tell()) / filesize
        print('%s MAV.flightmode=%-12s (MAV.timestamp=%u %u%%)' % (
            time.asctime(time.localtime(m._timestamp)),
            mlog.flightmode,
            m._timestamp, pct))

for filename in args:
    flight_modes(filename)


