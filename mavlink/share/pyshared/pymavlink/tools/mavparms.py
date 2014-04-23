#!/usr/bin/env python

'''
extract mavlink parameter values
'''

import sys, time, os

from optparse import OptionParser
parser = OptionParser("mavparms.py [options]")
parser.add_option("-c", "--changes", dest="changesOnly", default=False, action="store_true", help="Show only changes to parameters.")

(opts, args) = parser.parse_args()

from pymavlink import mavutil

if len(args) < 1:
    print("Usage: mavparms.py [options] <LOGFILE...>")
    sys.exit(1)

parms = {}

def mavparms(logfile):
    '''extract mavlink parameters'''
    mlog = mavutil.mavlink_connection(filename)

    while True:
        m = mlog.recv_match(type=['PARAM_VALUE', 'PARM'])
        if m is None:
            return
        if m.get_type() == 'PARAM_VALUE':
            pname = str(m.param_id).strip()
            value = m.param_value
        else:
            pname = m.Name
            value = m.Value
        if len(pname) > 0:
            if opts.changesOnly is True and pname in parms and parms[pname] != value:
                print("%s %-15s %.6f -> %.6f" % (time.asctime(time.localtime(m._timestamp)), pname, parms[pname], value))
            
            parms[pname] = value

total = 0.0
for filename in args:
    mavparms(filename)

if (opts.changesOnly is False):
    keys = parms.keys()
    keys.sort()
    for p in keys:
        print("%-15s %.6f" % (p, parms[p]))
