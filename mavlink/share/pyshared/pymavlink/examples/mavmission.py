#!/usr/bin/env python

'''
extract mavlink mission from log
'''

import sys, time, os

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

from optparse import OptionParser
parser = OptionParser("mavmission.py [options]")
parser.add_option("--output", default='mission.txt', help="output file")

(opts, args) = parser.parse_args()

import mavutil, mavwp

if len(args) < 1:
    print("Usage: mavmission.py [options] <LOGFILE...>")
    sys.exit(1)

parms = {}

def mavmission(logfile):
    '''extract mavlink mission'''
    mlog = mavutil.mavlink_connection(filename)

    wp = mavwp.MAVWPLoader()

    while True:
        if mlog.mavlink10():
            m = mlog.recv_match(type='MISSION_ITEM')
        else:
            m = mlog.recv_match(type='WAYPOINT')
        if m is None:
            break
        wp.set(m, m.seq)
    wp.save(opts.output)
    print("Saved %u waypoints to %s" % (wp.count(), opts.output))


total = 0.0
for filename in args:
    mavmission(filename)
