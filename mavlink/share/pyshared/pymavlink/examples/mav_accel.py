#!/usr/bin/env python

'''
show accel calibration for a set of logs
'''

import sys, time, os

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

from optparse import OptionParser
parser = OptionParser("mav_accel.py [options]")
parser.add_option("--no-timestamps",dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_option("--planner",dest="planner", action='store_true', help="use planner file format")
parser.add_option("--robust",dest="robust", action='store_true', help="Enable robust parsing (skip over bad data)")

(opts, args) = parser.parse_args()

import mavutil

if len(args) < 1:
    print("Usage: mav_accel.py [options] <LOGFILE...>")
    sys.exit(1)

def process(logfile):
    '''display accel cal for a log file'''
    mlog = mavutil.mavlink_connection(filename,
                                      planner_format=opts.planner,
                                      notimestamps=opts.notimestamps,
                                      robust_parsing=opts.robust)

    m = mlog.recv_match(type='SENSOR_OFFSETS')
    if m is not None:
        z_sensor = (m.accel_cal_z - 9.805) * (4096/9.81)
        print("accel cal %5.2f %5.2f %5.2f %6u  %s" % (
            m.accel_cal_x, m.accel_cal_y, m.accel_cal_z,
            z_sensor,
            logfile))


total = 0.0
for filename in args:
    process(filename)
