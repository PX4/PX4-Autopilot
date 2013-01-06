#!/usr/bin/env python

'''
work out total flight time for a mavlink log
'''

import sys, time, os

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

from optparse import OptionParser
parser = OptionParser("flighttime.py [options]")
parser.add_option("--condition", default=None, help="condition for packets")
parser.add_option("--groundspeed", type='float', default=3.0, help="groundspeed threshold")

(opts, args) = parser.parse_args()

import mavutil

if len(args) < 1:
    print("Usage: flighttime.py [options] <LOGFILE...>")
    sys.exit(1)

def flight_time(logfile):
    '''work out flight time for a log file'''
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    in_air = False
    start_time = 0.0
    total_time = 0.0
    t = None

    while True:
        m = mlog.recv_match(type='VFR_HUD', condition=opts.condition)
        if m is None:
            if in_air:
                total_time += time.mktime(t) - start_time
            if total_time > 0:
                print("Flight time : %u:%02u" % (int(total_time)/60, int(total_time)%60))
            return total_time
        t = time.localtime(m._timestamp)
        if m.groundspeed > opts.groundspeed and not in_air:
            print("In air at %s (groundspeed %.1f)" % (time.asctime(t), m.groundspeed))
            in_air = True
            start_time = time.mktime(t)
        elif m.groundspeed < opts.groundspeed and in_air:
            print("On ground at %s (groundspeed %.1f  time=%.1f seconds)" % (
                time.asctime(t), m.groundspeed, time.mktime(t) - start_time))
            in_air = False
            total_time += time.mktime(t) - start_time
    return total_time

total = 0.0
for filename in args:
    total += flight_time(filename)

print("Total time in air: %u:%02u" % (int(total)/60, int(total)%60))
