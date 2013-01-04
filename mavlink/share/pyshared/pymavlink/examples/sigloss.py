#!/usr/bin/env python

'''
show times when signal is lost
'''

import sys, time, os

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

from optparse import OptionParser
parser = OptionParser("sigloss.py [options]")
parser.add_option("--no-timestamps",dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_option("--planner",dest="planner", action='store_true', help="use planner file format")
parser.add_option("--robust",dest="robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_option("--deltat", type='float', default=1.0, help="loss threshold in seconds")
parser.add_option("--condition",dest="condition", default=None, help="select packets by condition")
parser.add_option("--types",  default=None, help="types of messages (comma separated)")

(opts, args) = parser.parse_args()

import mavutil

if len(args) < 1:
    print("Usage: sigloss.py [options] <LOGFILE...>")
    sys.exit(1)

def sigloss(logfile):
    '''work out signal loss times for a log file'''
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename,
                                      planner_format=opts.planner,
                                      notimestamps=opts.notimestamps,
                                      robust_parsing=opts.robust)

    last_t = 0

    types = opts.types
    if types is not None:
        types = types.split(',')

    while True:
        m = mlog.recv_match(condition=opts.condition)
        if m is None:
            return
        if types is not None and m.get_type() not in types:
            continue
        if opts.notimestamps:
            if not 'usec' in m._fieldnames:
                continue
            t = m.usec / 1.0e6
        else:
            t = m._timestamp
        if last_t != 0:
            if t - last_t > opts.deltat:
                print("Sig lost for %.1fs at %s" % (t-last_t, time.asctime(time.localtime(t))))
        last_t = t

total = 0.0
for filename in args:
    sigloss(filename)
