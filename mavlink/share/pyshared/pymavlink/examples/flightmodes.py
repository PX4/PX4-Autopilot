#!/usr/bin/env python

'''
show changes in flight modes
'''

import sys, time, os

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

from optparse import OptionParser
parser = OptionParser("flightmodes.py [options]")
parser.add_option("--mav10", action='store_true', default=False, help="Use MAVLink protocol 1.0")

(opts, args) = parser.parse_args()

if opts.mav10:
    os.environ['MAVLINK10'] = '1'
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
        m = mlog.recv_match(type='SYS_STATUS',
                            condition='SYS_STATUS.mode != %u or SYS_STATUS.nav_mode != %u' % (mode, nav_mode))
        if m is None:
            return
        mode = m.mode
        nav_mode = m.nav_mode
        pct = (100.0 * mlog.f.tell()) / filesize
        print('%s MAV.flightmode=%-12s mode=%u nav_mode=%u (MAV.timestamp=%u %u%%)' % (
            time.asctime(time.localtime(m._timestamp)),
            mlog.flightmode,
            mode, nav_mode, m._timestamp, pct))

for filename in args:
    flight_modes(filename)


