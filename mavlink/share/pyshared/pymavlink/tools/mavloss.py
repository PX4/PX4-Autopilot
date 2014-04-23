#!/usr/bin/env python

'''
show MAVLink packet loss
'''

import sys, time, os

from optparse import OptionParser
parser = OptionParser("sigloss.py [options]")
parser.add_option("--no-timestamps",dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_option("--planner",dest="planner", action='store_true', help="use planner file format")
parser.add_option("--robust",dest="robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_option("--condition", default=None, help="condition for packets")

(opts, args) = parser.parse_args()

from pymavlink import mavutil

if len(args) < 1:
    print("Usage: mavloss.py [options] <LOGFILE...>")
    sys.exit(1)

def mavloss(logfile):
    '''work out signal loss times for a log file'''
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename,
                                      planner_format=opts.planner,
                                      notimestamps=opts.notimestamps,
                                      robust_parsing=opts.robust)

    m = mlog.recv_match(condition=opts.condition)

    while True:
        m = mlog.recv_match()
        if m is None:
            break
    print("%u packets, %u lost %.1f%%" % (
            mlog.mav_count, mlog.mav_loss, mlog.packet_loss()))


total = 0.0
for filename in args:
    mavloss(filename)
