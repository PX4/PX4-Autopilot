#!/usr/bin/env python

'''
extract one mode type from a log
'''

import sys, time, os, struct

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

from optparse import OptionParser
parser = OptionParser("mavextract.py [options]")

parser.add_option("--no-timestamps",dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_option("--robust",dest="robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_option("--condition",dest="condition", default=None, help="select packets by condition")
parser.add_option("--mode",  default='auto', help="mode to extract")
(opts, args) = parser.parse_args()

import mavutil

if len(args) < 1:
    print("Usage: mavextract.py [options] <LOGFILE>")
    sys.exit(1)

def process(filename):
    '''process one logfile'''
    print("Processing %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=opts.notimestamps,
                                      robust_parsing=opts.robust)

    output = None
    count = 1
    dirname = os.path.dirname(filename)

    while True:
        m = mlog.recv_match(condition=opts.condition)
        if m is None:
            break

        if mlog.flightmode.upper() == opts.mode.upper():
            if output is None:
                path = os.path.join(dirname, "%s%u.log" % (opts.mode, count))
                count += 1
                print("Creating %s" % path)
                output = mavutil.mavlogfile(path, write=True)
        else:
            if output is not None:
                output.close()
                output = None
            
        if output and m.get_type() != 'BAD_DATA':
            timestamp = getattr(m, '_timestamp', None)
            output.write(struct.pack('>Q', timestamp*1.0e6))
            output.write(m.get_msgbuf())

for filename in args:
    process(filename)

